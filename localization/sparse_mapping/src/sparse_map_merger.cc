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
  const auto relative_transform = EstimateRelativePoseAndPruneOutlierMatches(*map_a_, *map_b_, matching_tracks);
}

std::vector<MatchCandidates> SparseMapMerger::DatabaseMatchCandidates(const SparseMap& map_a, const SparseMap& map_b,
                                                                      const int max_query_matches) const {
  const int num_map_a_cids = map_a.NumCIDs();
  std::vector<MatchCandidates> database_match_candidates;
  for (int cid = 0; cid < map_b.NumCIDs(); ++cid) {
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
                                    const SparseMap& map_b, std::vector<TrackLabel>& track_labels,
                                    std::vector<std::pair<int, int>>& a_b_pid_correspondences) const {
  std::unordered_map<int, int> a_pid_matches;
  boost::optional<int> b_pid;
  // Aggregate a_pids that a b_fid matches to
  for (const auto& cid_fid : cid_fids) {
    const int cid = cid_fid.first;
    const int fid = cid_fid.second;
    // Since merged map contains a cids + b cids, any a cid should have a value less than num a cids
    if (cid < map_a.NumCams() && map_a.ContainsPid(cid, fid)) {
      const int a_pid = map_a.Pid(cid, fid);
      a_pid_matches[a_pid]++;
    } else {  // Each cid_fid track contains at most one b_pid since the map_b images were individually matched to map_a
              // images
      // Since merged map contains a cids + b cids, any b cid should be present in map b with value -= num a cids
      const int b_cid = cid - map_a.NumCIDs();
      if (map_b.ContainsPid(b_cid, fid)) b_pid = map_b.Pid(b_cid, fid);
    }
  }
  // Check whether to merge a b_pid to an a_pid if the track contains each of these
  if (b_pid) {
    const auto best_a_pid = BestMatch(a_pid_matches);
    if (best_a_pid) {
      track_labels[index] = TrackLabel::kMerge;
      a_b_pid_correspondences.emplace_back({*best_a_pid, *b_pid});
      return;
    } else {
      track_labels[index] = TrackLabel::kInvalid;
      return;
    }
  } else {  // Otherwise, check whether to create a new track or append a b_fid to an existing a_pid
    if (a_pid_matches.empty()) {
      if (cid_fids.size() >= params_.min_num_features_for_new_track) track_labels[index] = TrackLabel::kNew;
      return;
    } else {
      if (a_pid_matches.size() <= params_.max_num_matching_pids_for_appended_track)
        track_labels[index] = TrackLabel::kAppend;
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

  // Identify tracks (merge, new, append, invalid) and update tracks to merge if necessary
  matching_tracks.track_labels = std::vector<TrackLabel>(matching_tracks.pid_to_cid_fid.size(), TrackLabel::kInvalid);
  for (int i = 0; i < pid_to_cid_fid.size(); ++i) {
    const auto& cid_fids = pid_to_cid_fid[i];
    IdentifyTrack(cid_fids, i, map_a, map_b, track_labels, matching_tracks.a_b_pid_correspondences);
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

void SparseMapMerger::EstimateRelativePoseAndPruneOutlierMatches(const SparseMap& map_a, const SparseMap& map_b,
                                                                 MatchingTracks& matching_tracks) {
  std::vector<Eigen::Vector3d> a_points;
  std::vector<Eigen::Vector3d> b_points;
  for (const auto& a_b_pid_correspondence : matching_tracks.a_b_pid_correspondences) {
    const int a_pid = a_b_pid_correspondence.first;
    const int b_pid = a_b_pid_correspondence.second;
    a_points.emplace_back(map_a.Point(a_pid));
    b_points.emplace_back(map_b.Point(b_pid));
  }
  const double inlier_threshold = InlierThreshold(a_points);

  // Estimate the transform from B_vec to A_vec using RANSAC.
  // A lot of outliers are possible.
  // TODO(rsoussan): add these as params!
  int  num_iterations = 1000;
  int  min_num_output_inliers = A_vec.size()/2;
  bool reduce_min_num_output_inliers_if_no_fit = true;  // If too many outliers
  bool increase_threshold_if_no_fit = true;  // Coz our threshold was done by a heuristic
  // TODO(rsoussan): update this to use ransacestiamteaffine3d!!
  RansacEstimateAffine3d ransac_affine3d(num_iterations,
           inlier_threshold, min_num_output_inliers,
           reduce_min_num_output_inliers_if_no_fit, increase_threshold_if_no_fit);
  // TODO(rsoussan): b_T_a or a_T_b???
  const auto map_b_T_map_a = ransac_affine3d(B_vec, A_vec);
  std::vector<size_t> inlier_indices = ransac_affine3d.inlier_indices(map_b_T_map_a, B_vec, A_vec);
  std::set<int> inlier_set;
  for (size_t it = 0; it < inlier_indices.size(); it++) {
    inlier_set.insert(inlier_indices[it]);
  }

  // Remove from A2B and B2A the outliers
  // TODO(rsoussan): Update this to remove ab correspondences from matching tracks!!
  std::map<int, int> A2B_orig = A2B;
  point_count = 0;
  for (auto it = A2B_orig.begin(); it != A2B_orig.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    if (inlier_set.find(point_count) == inlier_set.end()) {
      auto iter_a = A2B.find(pid_a);
      A2B.erase(iter_a);
      auto iter_b = B2A.find(pid_b);
      B2A.erase(iter_b);
    }
    point_count++;
  }

  // LOG(INFO) does not do well with Eiegn.
  std::cout << "Affine transform from second map to first map:\n";
  std::cout << "Matrix:\n"      << map_b_T_map_a.linear()       << "\n";
  std::cout << "Translation:\n" << map_b_T_map_a.translation()  << "\n";

  // Bring the B map into the coordinate system of the A map
  // TODO(rsoussan): Clean up transform code??? Is this in sparse_map??
  B.Transform(map_b_T_map_a);
}
}  // namespace sparse_mapping
