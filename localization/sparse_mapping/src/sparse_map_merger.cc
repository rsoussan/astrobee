/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/ransac_estimate_affine_3d.h>
#include <sparse_mapping/sparse_map_merger.h>

#include <Eigen/Geometry>
#include <glog/logging.h>

namespace sparse_mapping {
SparseMapMerger::SparseMapMerger(const SparseMap& map_a, const SparseMap& map_b, const SparseMapMergerParams& params) {
  Initialize(map_a, map_b, params);
}

SparseMapMerger::SparseMapMerger(const std::string& map_a_filename, const std::string& map_b_filename,
                                 const SparseMapMergerParams& params) {
  const SparseMap map_a(map_a_filename);
  const SparseMap map_b(map_b_filename);
  Initialize(map_a, map_b, params);
}

void SparseMapMerger::Initialize(const SparseMap& map_a, const SparseMap& map_b, const SparseMapMergerParams& params) {
  map_a_.reset(new SparseMap(map_a));
  map_b_.reset(new SparseMap(map_b));
  // Initialize merged map using map_a, map_b will be merged into it later
  merged_map_.reset(new SparseMap(map_a));
  params_ = params;
  if (!CompatableMaps()) LOG(FATAL) << "Incompatable Maps.";
}

bool SparseMapMerger::CompatableMaps() const {
  if (map_a_->camera_params() != map_b_->camera_params())
    LOG(FATAL) << "Input maps don't have the same camera parameters.";
  if (map_a_->DetectorName() != map_b_->DetectorName())
    LOG(FATAL) << "Input maps use different detectors.";
  if (map_a_->GetHistogramEqualization() != map_b_->GetHistogramEqualization())
    LOG(FATAL) << "Input maps use different histogram equalization.";
}

void SparseMapMerger::MergeMaps() {
  auto matching_tracks = MatchingTracks(*map_a_, *map_b_, *merged_map_);
  const auto map_a_T_map_b = EstimateRelativePoseAndPruneOutlierMatches(*map_a_, *map_b_, matching_tracks);
  LOG(INFO) << "map_a_T_map_b: " << std::endl << map_a_T_map_b.matrix();
  map_b_->Transform(map_a_T_map_b);
  merged_map_->AddPoses(map_b_->cid_to_cam_T_global());
  MergeTracks(matching_tracks);
  merged_map_->InitializeCidFidPidMap();
  merged_map_->BuildImageDatabase();
  if (params_.bundle_adjust_result) BundleAdjust(matching_tracks);
}

std::vector<MatchCandidates> SparseMapMerger::DatabaseMatchCandidates(const SparseMap& map_a, const SparseMap& map_b,
                                                                      const int max_query_matches) const {
  const int num_map_a_cids = map_a.NumCids();
  std::vector<MatchCandidates> database_match_candidates;
  for (int cid = 0; cid < map_b.NumCids(); ++cid) {
    MatchCandidates match_candidates;
    // Offset by num_map_a_cids since map_b cid indices start at num_map_a_cids in the merged map
    match_candidates.cid = num_map_a_cids + cid;
    const auto db_match_candidate_cids =
      map_a.image_database().Query(map_b.descriptors(cid), max_query_matches);
    for (const auto candidate_cid : db_match_candidate_cids) {
      match_candidates.candidate_cids.emplace_back(candidate_cid);
    }
    database_match_candidates.emplace_back(match_candidates);
  }
  return database_match_candidates;
}

void SparseMapMerger::IdentifyTrack(const std::map<int, int>& track, const int index, const SparseMap& map_a,
                                    const SparseMap& map_b, MatchingTracks& matching_tracks) const {
  std::unordered_map<int, int> a_pid_matches;
  std::vector<int> b_pids;
  // Aggregate a_pids that a b_fid matches to
  for (const auto& cid_fid : cid_fids) {
    const int cid = cid_fid.first;
    const int fid = cid_fid.second;
    // Since merged map contains a cids + b cids, any a cid should have a value less than num a cids
    if (cid < map_a.NumCams() && map_a.ContainsPid(cid, fid)) {
      const int a_pid = map_a.Pid(cid, fid);
      a_pid_matches[a_pid]++;
    } else {
      // Since merged map contains a cids + b cids, any b cid should be present in map b with value -= num a cids
      const int b_cid = cid - map_a.NumCids();
      if (map_b.ContainsPid(b_cid, fid)) b_pids.emplace_back(map_b.Pid(b_cid, fid));
    }
  }
  // Check whether to combine a b_pid to an a_pid if the track contains each of these
  if (b_pids.size() > 0) {
    const auto best_a_pid = BestMatch(a_pid_matches);
    if (best_a_pid) {
      matching_tracks.track_labels[index] = TrackLabel::kCombine;
      // Choose the first b_pid
      // TODO(rsoussan): Better way to disambiguate two matching b pids? Use longest or shortest one?
      // TODO(rsoussan): Add option to add all matches? Assume ransac/optimization will prune outlier?
      matching_tracks.a_b_pid_correspondences.emplace_back({*best_a_pid, b_pid[0]});
      return;
    } else {
      matching_tracks.track_labels[index] = TrackLabel::kBOnly;
      matching_tracks.non_matching_b_pids.insert(matching_tracks.non_matching_b_pids.end(), b_pids.begin(),
                                                 b_pids.end());
      return;
    }
  } else {  // Otherwise, check whether to create a new track or extend a b_fid to an existing a_pid
    if (a_pid_matches.empty()) {
      if (cid_fids.size() >= params_.min_num_features_for_new_track)
        matching_tracks.track_labels[index] = TrackLabel::kNew;
      return;
    } else {
      if (a_pid_matches.size() <= params_.max_num_matching_pids_for_extended_track)
        matching_tracks.track_labels[index] = TrackLabel::kExtend;
      return;
    }
  }
}

boost::optional<int> SparseMapMerger::BestMatch(const std::map<int, int>& pid_match_counts) const {
  int total_pid_matches = 0;
  int best_match_count = -1;
  int best_pid;
  // Find best match
  for (const auto& pid_match_count_pair : pid_match_counts) {
    const int pid = pid_match_count_pair.first;
    const int count = pid_match_count_pair.second;
    total_pid_matches += count;
    if (count > best_match_count) {
      best_pid = pid;
      best_match_count = count;
    }
  }
  // Make sure it's valid
  const double match_ratio = best_match_count / static_cast<double>(total_pid_matches);
  // TODO(rsoussan): Check what ratio of points in best_pid match b_pid??
  if (best_match_count >= params_.min_shared_track_features_for_merged_tracks &&
      match_ratio >= params_.min_match_ratio_for_merged_tracks) {
    return best_pid;
  }
  return boost::none;
}

MatchingTracks SparseMapMerger::MatchingTracks(const SparseMap& map_a, const SparseMap& map_b, SparseMap& merged_map) {
  const auto match_candidates = DatabaseMatchCandidates(map_a, map_b, params_.max_db_query_image_match_candidates);
  MatchingTracks matching_tracks;
  merged_map.MatchImagesAndBuildTracks(match_candidates, matching_tracks.pid_to_cid_fid);

  matching_tracks.track_labels = std::vector<TrackLabel>(matching_tracks.pid_to_cid_fid.size(), TrackLabel::kInvalid);
  for (int i = 0; i < pid_to_cid_fid.size(); ++i) {
    const auto& cid_fids = pid_to_cid_fid[i];
    IdentifyTrack(cid_fids, i, map_a, map_b, matching_tracks);
  }
  return matching_tracks;
}

double SparseMapMerger::InlierThreshold(const std::vector<Eigen::Vector3d>& points) const {
  const int num_pts = points.size();
  if (num_pts <= 0) LOG(FATAL) << "Empty set of points.\n";

  // TODO(rsoussan): Is bounding here really necessary?
  const int low_index = std::min(num_pts - 1, std::round(num_pts*params_.inlier_threshold_low_index_percent));
  const int high_index = std::min(num_pts -1, std::round(num_pts*params_.inlier_threshold_high_index_percent));

  Eigen::Vector3d scaled_bounds;
  for (int i = 0; i < 3; ++i) {
    std::vector<double> vals;
    vals.reserve(num_pts);
    for (const auto& point : points) {
      vals.emplace_back(point[i]);
    }
    std::sort(vals.begin(), vals.end());
    const double low_val = vals[low_index];
    const double high_val = vals[high_index];
    scaled_bounds[i] = params_.inlier_threshold_scale_factor*(high_val - low_val);
  }

  // TODO(rsoussan): Use norm or some better distance metric than averaged indices?
  const double inlier_threshold = scaled_bounds.sum()/3.0;
  return inlier_threshld;
}

Eigen::Affine3d SparseMapMerger::EstimateRelativePoseAndPruneOutlierMatches(const SparseMap& map_a,
                                                                            const SparseMap& map_b,
                                                                            MatchingTracks& matching_tracks) const {
  std::vector<Eigen::Vector3d> a_points;
  std::vector<Eigen::Vector3d> b_points;
  for (const auto& a_b_pid_correspondence : matching_tracks.a_b_pid_correspondences) {
    const int a_pid = a_b_pid_correspondence.first;
    const int b_pid = a_b_pid_correspondence.second;
    a_points.emplace_back(map_a.Point(a_pid));
    b_points.emplace_back(map_b.Point(b_pid));
  }
  const double inlier_threshold = InlierThreshold(a_points);
  const int min_num_output_inliers = a_points.size()* params_.ransac_min_num_ouput_inliers_percent;
  RansacEstimateAffine3d ransac_affine3d(num_iterations,
           inlier_threshold, min_num_output_inliers,
           params_.ransac_reduce_min_num_output_inliers_if_no_fit, params_.ransac_increase_threshold_if_no_fit);
  const auto map_a_T_map_b = ransac_affine3d(b_points, a_points);

  // Remove outliers from correspondences
  const auto inlier_indices = ransac_affine3d.inlier_indices(map_a_T_map_b, b_points, a_points);
  std::vector<bool> indices_to_remove(matching_tracks.a_b_pid_correspondences.size(), true);
  for (const auto inlier_index : inlier_indices) {
    indices_to_remove[inlier_index] = false;
  }
  lc::RemoveElements(indices_to_remove, matching_tracks.a_b_pid_correspondences);

  return map_a_T_map_b;
}

void SparseMapMerger::MergeTracks(const MatchingTracks& matching_tracks) {
  CombineTracks(matching_tracks.a_b_pid_correspondences);
  ExtendTracks(matching_tracks);
  AddRemainingMapBTracks(matching_tracks);
  if (params_.add_new_tracks) AddNewTracks(matching_tracks);
}

void SparseMapMerger::CombineTracks(const std::vector<std::pair<int, int>>& a_b_pid_correspondences) {
  for (const auto& a_b_pid_correspondence : a_b_pid_correspondences) {
    const int a_pid = a_b_pid_correspondence.first;
    const int b_pid = a_b_pid_correspondence.second;
    const int global_t_b_point = map_b_->Point(b_pid);
    const auto& b_cid_to_fid = map_b_->CidToFid(b_pid);
    map_a_->MergeTrack(a_pid, global_t_b_point, b_cid_to_fid);
  }
}

void SparseMapMerger::ExtendTracks(const MatchingTracks& matching_tracks) {
  for (i = 0; i < matching_tracks.pid_to_cid_fid.size(); ++i) {
    if (matching_tracks.track_labels[i] == TrackLabel::kExtend) {
      const auto& b_cid_to_fid = map_b_->CidToFid(b_pid);
      map_a_->ExtendTrack(a_pid, b_cid_to_fid);
    }
  }
}

void SparseMapMerger::AddRemainingMapBTracks(const std::vector<int>& non_matching_b_pids) {
  for (const auto b_pid : non_matching_b_pids) {
    const int global_t_b_point = map_b_->Point(b_pid);
    const auto& b_cid_to_fid = map_b_->CidToFid(b_pid);
    map_a_->AddTrack(global_t_b_point, b_cid_to_fid);
  }
}


void SparseMapMerger::AddNewTracks(const MatchingTracks& matching_tracks) {
      // TODO(rsoussan): add intrinsics creation in constructor, store as member variable???
      const double focal_length = map_a_->params().camera.GetFocalLength();
      Eigen::Matrix3d intrinsics;
  intrinsics << focal_length, 0, 0,
    0, focal_length, 0,
    0, 0, 1;
  for (int i = 0; i < matching_tracks.pid_to_cid_fid.size(); ++i) {
    if (matching_tracks.track_label[i] == TrackLabel::KNewTrack) {
     const auto& cid_to_fid = matching_tracks.pid_to_cid_fid[i];
      std::vector<Eigen::Affine3d> poses;
      Keypoints keypoints;
      for (const auto& cid_fid_pair : cid_to_fid) {
        const int cid = cid_fid_pair.first;
        const int fid = cid_fid_pair.second;
        poses.emplace_back(merged_map_->CamTGlobal(cid));
        keypoints.emplace_back(merged_map_->keypoint(cid, fid));
      }
      const auto global_t_point = Triangulate(intrinsics, poses, keypoints);
      merged_map_->AddTrack(global_t_b_point, cid_to_fid);
    }
  }
}

void SparseMapMerger::BundleAdjust(const std::vector<std::pair<int, int>>& a_b_pid_correspondences) {
  switch (params_.optimization_strategy) {
    case kOptimizeAllPosesAndPoints:
      params_.bundle_adjustment.optimize_camera_range = false;
      params_.bundle_adjustment.fix_all_cameras = false;
      break;
  case kOptimizeMergedPosesAndPoints:
  params_.bundle_adjustment.fix_all_cameras = false;
  params_.bundle_adjustment.optimize_camera_range = true;
  // Map b cameras in merged map start after the last map a camera
  params_.bundle_adjustment.first_optimized_camera = map_a_->NumCids();
  params_.bundle_adjustment.last_optimized_camera = merged_map_->NumCids() - 1;
      break;
  case kOptimizeMergedAndUpdatedPosesAndPoints:
  params_.bundle_adjustment.fix_all_cameras = false;
  params_.bundle_adjustment.optimize_camera_range = false;
  FillUnmodifiedCamerasAndPoints(a_b_pid_correspondences, params_.bundle_adjustment.fixed_cameras,
                                 params_.bundle_adjustment.fixed_points);
  break;
  }
  merged_map_->IterativelyBundleAdjust(params_.bundle_adjustment, params_.num_bundle_adjustment_iterations);
}

void SparseMapMerger::FillUnmodifiedCamerasAndPoints(const std::vector<std::pair<int, int>>& a_b_pid_correspondences,
                                                     std::unorderd_map<int>& fixed_cameras,
                                                     std::unordered_map<int>& fixed_points) const {
  std::unordered_set<int> modified_a_pids;
  std::unordered_set<int> modified_a_cids;
  // a_b_pid_correspondences contain list of feature tracks that were merged, so
  // each a pid here has been modified
  for (const auto& a_b_pid_correspondence : a_b_pid_correspondences) {
    const int a_pid = a_b_pid_correspondences.first;
    modified_a_pids.emplace_back(a_pid);
    for (const auto& cid_fid : CidToFid(a_pid)) {
      modified_a_cids.emplace(cid_fid.first);
    }
  }

  // Mark as fixed points and cameras that have not been modified
  for (int a_pid = 0; a_pid < map_a_->NumPoints(); ++a_pid) {
    if (modified_a_pids.count(a_pid) == 0) fixed_points.emplace(a_pid);
  }

  for (int a_cid = 0; a_cid < map_a_->NumCids(); ++a_cid) {
    if (modified_a_cids.count(a_cid) == 0) fixed_cameras.emplace(a_cid);
  }
}
}  // namespace sparse_mapping
