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
  MergeTracks(matching_tracks);
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
                                    const SparseMap& map_b, MatchingTracks& matching_tracks) const {
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
      matching_tracks.track_labels[index] = TrackLabel::kMerge;
      matching_tracks.a_b_pid_correspondences.emplace_back({*best_a_pid, *b_pid});
      return;
    } else {
      matching_tracks.track_labels[index] = TrackLabel::kBOnly;
      matching_tracks.non_matching_b_pids.emplace_back(*b_pid);
      return;
    }
  } else {  // Otherwise, check whether to create a new track or append a b_fid to an existing a_pid
    if (a_pid_matches.empty()) {
      if (cid_fids.size() >= params_.min_num_features_for_new_track)
        matching_tracks.track_labels[index] = TrackLabel::kNew;
      return;
    } else {
      if (a_pid_matches.size() <= params_.max_num_matching_pids_for_appended_track)
        matching_tracks.track_labels[index] = TrackLabel::kAppend;
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
  AddTracksToMerge(matching_tracks.a_b_pid_correspondences);
  AddTracksToAppend(matching_tracks);
  AddRemainingMapBTracks(matching_tracks);
  if (params_.add_new_tracks) AddNewTracks(matching_tracks);
}

void SparseMapMerger::AddTracksToMerge(const std::vector<std::pair<int, int>>& a_b_pid_correspondences) {
  for (const auto& a_b_pid_correspondence : a_b_pid_correspondences) {
    const int a_pid = a_b_pid_correspondence.first;
    const int b_pid = a_b_pid_correspondence.second;
    const int global_t_b_point = map_b_->Point(b_pid);
    const auto& b_cid_to_fid = map_b_->CidToFid(b_pid);
    map_a_->MergeTrack(a_pid, global_t_b_point, b_cid_to_fid);
  }
}

void SparseMapMerger::AddTracksToAppend(const MatchingTracks& matching_tracks) {
  for (i = 0; i < matching_tracks.pid_to_cid_fid.size(); ++i) {
    if (matching_tracks.track_labels[i] == TrackLabel::kAppend) {
      const auto& b_cid_to_fid = map_b_->CidToFid(b_pid);
      map_a_->AppendTrack(a_pid, b_cid_to_fid);
    }
  }
}

void SparseMapMerger::AddRemaingMapBTracks(const std::vector<int>& non_matching_b_pids) {
  for (const auto b_pid : non_matching_b_pids) {
    const int global_t_b_point = map_b_->Point(b_pid);
    const auto& b_cid_to_fid = map_b_->CidToFid(b_pid);
    map_a_->AddTrack(global_t_b_point, b_cid_to_fid);
  }
}


void SparseMapMerger::AddNewTracks(const MatchingTracks& matching_tracks) {
  for (int i = 0; i < matching_tracks.pid_to_cid_fid.size(); ++i) {
    if (matching_tracks.track_label[i] == TrackLabel::KNewTrack) {
      const auto& cid_to_fid = matching_tracks.pid_to_cid_fid[i];
      // TODO(rsoussan): triangulate point!
      // TODO(rsoussan): make sure all cam poses have been added to map a from map b first!!
      map_a_->AddTrack(global_t_b_point, cid_to_fid);
    }
  }
}

void SparseMapMerger::MergeTracks(const MatchingTracks& matching_tracks) {
  // We will use this to add new tracks taking advantage
  // of all the matching between the two image sets.
  std::vector<std::map<int, int> > merged_pid_to_cid_fid;
  if (!FLAGS_skip_adding_new_matches_on_merging)
    merged_pid_to_cid_fid = C.pid_to_cid_fid_;  // save it before wiping it below

  // Start creating the merged tracks
  C.pid_to_cid_fid_.clear();
  C.pid_to_xyz_ = A.pid_to_xyz_;  // Will later modify it by averaging/appending from B

  int num_tracks_in_A_only = 0, num_tracks_in_A_and_B = 0, num_tracks_in_B_only = 0;

  // Add to C.pid_to_cid_fid_ the tracks in A.pid_to_cid_fid_, and
  // merge the corresponding track from B.pid_to_cid_fid_ if available.
  for (size_t pid_a = 0; pid_a < A.pid_to_cid_fid_.size(); pid_a++) {
    auto cid_fid_c = A.pid_to_cid_fid_[pid_a];  // make a copy

    if (A2B.find(pid_a) != A2B.end()) {  // Can merge from B
      int pid_b = A2B[pid_a];

      if (pid_b >= static_cast<int>(B.pid_to_cid_fid_.size()) )
        LOG(FATAL) << "Book-keeping error in track merging.";

      // Append the B track to the C track. Add num_acid as we want
      // a track in C.
      auto & cid_fid_b = B.pid_to_cid_fid_[pid_b];  // alias
      for (auto it = cid_fid_b.begin(); it != cid_fid_b.end(); it++)
        cid_fid_c[it->first + num_acid] = it->second;

      // Merged map xyz will be the average of xyz's from both maps
      C.pid_to_xyz_[pid_a] = (A.pid_to_xyz_[pid_a] + B.pid_to_xyz_[pid_b])/2.0;

      num_tracks_in_A_and_B++;
    } else {
      num_tracks_in_A_only++;
    }

    // Add the current track, whether it is wholly in A or also paritially in B
    C.pid_to_cid_fid_.push_back(cid_fid_c);
  }

  // Now add the tracks that are purely in B.
  for (size_t pid_b = 0; pid_b < B.pid_to_cid_fid_.size(); pid_b++) {
    if (B2A.find(pid_b) != B2A.end()) {
      continue;  // Track partially in A, done already
    }

    num_tracks_in_B_only++;

    // Add this track, and add num_acid to be in C's indexing scheme
    std::map<int, int> cid_fid_c;
    auto & cid_fid_b = B.pid_to_cid_fid_[pid_b];  // alias
    for (auto it = cid_fid_b.begin(); it != cid_fid_b.end(); it++)
      cid_fid_c[it->first + num_acid] = it->second;

    C.pid_to_cid_fid_.push_back(cid_fid_c);
    C.pid_to_xyz_.push_back(B.pid_to_xyz_[pid_b]);
  }

  // Append the cameras from B. By now A and B are in same coordinate system.
  C.cid_to_cam_t_global_ = A.cid_to_cam_t_global_;
  for (int cid = 0; cid < num_bcid; cid++)
    C.cid_to_cam_t_global_.push_back(B.cid_to_cam_t_global_[cid]);

  // C.Save(output_map + ".merged.map");

  LOG(INFO) << "Number of tracks merged from both maps:    " << num_tracks_in_A_and_B;
  LOG(INFO) << "Number of tracks from the first map only:  " << num_tracks_in_A_only;
  LOG(INFO) << "Number of tracks from the second map only: " << num_tracks_in_B_only;

  // If a few images show up in both and in B, so far they show up in C twice,
  // with different cid value. Fix that.
  // Also keep the images sorted.
  std::vector<std::string> sorted = C.cid_to_filename_;
  std::sort(sorted.begin(), sorted.end());
  int num_out_cams = 0;
  std::map<std::string, int> image2cid;  // the new index of each image after rm repetitions
  for (size_t cid = 0; cid < sorted.size(); cid++) {
    std::string img = sorted[cid];
    if (image2cid.find(img) == image2cid.end()) {
      image2cid[img] = num_out_cams;
      num_out_cams++;
    }
  }

  // The index of the cid after removing the repetitions
  std::map<int, int> cid2cid;
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    cid2cid[cid] = image2cid[ C.cid_to_filename_[cid] ];
  }

  // Remove repetitions.
  TransformMap(cid2cid, &C);

  if (!FLAGS_skip_adding_new_matches_on_merging) {
    // Modify merged_pid_to_cid_fid as well after identifying identical images
    bool rm_tracks_of_len_one = true;
    TransformTracks(cid2cid, rm_tracks_of_len_one, &merged_pid_to_cid_fid);
  }

  // Add the new tracks that were identified during matching of images of A to B.
  if (!FLAGS_skip_adding_new_matches_on_merging) {
    // Form merged_cid_fid_to_pid
    int num_cid = C.cid_to_filename_.size();
    std::vector<std::map<int, int> > merged_cid_fid_to_pid;
    InitializeCidFidToPid(num_cid, merged_pid_to_cid_fid, &merged_cid_fid_to_pid);

    LOG(INFO) << "Number of tracks found as result of matching images between the maps: "
              << merged_pid_to_cid_fid.size();

    std::vector<std::map<int, int> > new_pid_to_cid_fid;
    std::set<int> new_pid_set;
    // See which tracks obtained during merging are new
    for (size_t cid = 0; cid < merged_cid_fid_to_pid.size(); cid++) {
      for (auto it = merged_cid_fid_to_pid[cid].begin();
           it != merged_cid_fid_to_pid[cid].end(); it++) {
        if (cid >= C.cid_fid_to_pid_.size()) continue;  // out of range
        int fid = it->first;
        if (C.cid_fid_to_pid_[cid].find(fid) != C.cid_fid_to_pid_[cid].end())
          continue;  // not new
        int new_pid = it->second;
        if (new_pid_set.find(new_pid) != new_pid_set.end()) continue;  // inserted already

        // Add this new track
        new_pid_to_cid_fid.push_back(merged_pid_to_cid_fid[new_pid]);
        new_pid_set.insert(new_pid);  // mark it as inserted
      }
    }

    // Triangulate to find the xyz coordinates of the new tracks
    std::vector<Eigen::Vector3d> new_pid_to_xyz;
    std::vector<std::map<int, int> > new_cid_fid_to_pid;
    bool rm_invalid_xyz = true;  // don't remove anything, as cameras are pretty unreliable now
    sparse_mapping::Triangulate(rm_invalid_xyz,
                                C.camera_params_.GetFocalLength(),
                                C.cid_to_cam_t_global_,
                                C.cid_to_keypoint_map_,
                                &new_pid_to_cid_fid,
                                &new_pid_to_xyz,
                                &new_cid_fid_to_pid);

    LOG(INFO) << "Of those, number of tracks that are new and will be added to the merged map: "
              << new_pid_to_cid_fid.size();

    // Append the new tracks to the merged map
    for (size_t pid = 0; pid < new_pid_to_cid_fid.size(); pid++) {
      C.pid_to_cid_fid_.push_back(new_pid_to_cid_fid[pid]);
      C.pid_to_xyz_.push_back(new_pid_to_xyz[pid]);
    }

    // Recreate cid_fid_to_pid_ from pid_to_cid_fid_.
    C.InitializeCidFidToPid();
  }

  LOG(INFO) << "Total number of tracks in the merged map: " << C.pid_to_xyz_.size();

  return;
}

}  // namespace sparse_mapping
}  // namespace sparse_mapping
