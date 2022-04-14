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

#include <sparse_mapping/map_merging_utilities.h>
#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/ransac.h>
#include <sparse_mapping/sparse_map.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>

#include <set>
#include <vector>

/*DEFINE_int32(num_subsequent_images, std::numeric_limits<int32_t>::max()/2,  // avoid overflow
             "When no vocabulary tree is provided, match every image against this "
             "many subsequent images.");
DEFINE_bool(skip_adding_new_matches_on_merging, false,
            "When merging maps, do not take advantage of performed matching to add new tracks.");
*/
namespace {
// Check if the two arrays share elements
bool haveSharedElements(std::vector<std::string> const& A, std::vector<std::string> const& B) {
  std::set<std::string> setA;
  for (size_t it = 0; it < A.size(); it++) setA.insert(A[it]);

  for (size_t it = 0; it < B.size(); it++)
    if (setA.find(B[it]) != setA.end())
      return true;

  return false;
}

// Given a set of points in 3D, heuristically estimate what it means
// for two points to be "not far" from each other. The logic is to
// find a bounding box of an inner cluster and multiply that by 0.2.
double estimateCloseDistance(std::vector<Eigen::Vector3d> const& vec) {
  Eigen::Vector3d range;
  int num_pts = vec.size();
  if (num_pts <= 0)
    LOG(FATAL) << "Empty set of points.\n";  // to avoid a segfault

  std::vector<double> vals(num_pts);
  for (int it = 0; it < range.size(); it++) {  // iterate in each coordinate
    // Sort all values in given coordinate
    for (int p = 0; p < num_pts; p++)
      vals[p] = vec[p][it];
    std::sort(vals.begin(), vals.end());

    // Find some percentiles
    int min_p = round(num_pts*0.25);
    int max_p = round(num_pts*0.75);
    if (min_p >= num_pts) min_p = num_pts - 1;
    if (max_p >= num_pts) max_p = num_pts - 1;
    double min_val = vals[min_p], max_val = vals[max_p];
    range[it] = 0.2*(max_val - min_val);
  }

  // Find the average of all ranges
  double range_val = 0.0;
  for (int it = 0; it < range.size(); it++)
    range_val += range[it];
  range_val /= range.size();

  return range_val;
}
}  // namespace




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
  merged_map_->AddImagesAndFeatures(*map_b_);
  std::map<int, int> A2B, B2A;
  findMatchingTracks(&A, &B, &C, output_map,
                       num_image_overlaps_at_endpoints,
                       A2B, B2A);  // outputs

  // Put the xyz points corresponding to the tracks to merge in vectors.
  std::vector<Eigen::Vector3d> A_vec(A2B.size()), B_vec(A2B.size());
  int point_count = 0;
  for (auto it = A2B.begin(); it != A2B.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    A_vec[point_count] = A.pid_to_xyz_[pid_a];
    B_vec[point_count] = B.pid_to_xyz_[pid_b];
    point_count++;
  }
  double inlier_threshold = estimateCloseDistance(A_vec);

  // Estimate the transform from B_vec to A_vec using RANSAC.
  // A lot of outliers are possible.
  int  num_iterations = 1000;
  int  min_num_output_inliers = A_vec.size()/2;
  bool reduce_min_num_output_inliers_if_no_fit = true;  // If too many outliers
  bool increase_threshold_if_no_fit = true;  // Coz our threshold was done by a heuristic
  RandomSampleConsensus < TranslationRotationScaleFittingFunctor, TransformError>
    ransac(TranslationRotationScaleFittingFunctor(), TransformError(), num_iterations,
           inlier_threshold, min_num_output_inliers,
           reduce_min_num_output_inliers_if_no_fit, increase_threshold_if_no_fit);
  Eigen::Affine3d B2A_trans = ransac(B_vec, A_vec);
  std::vector<size_t> inlier_indices = ransac.inlier_indices(B2A_trans, B_vec, A_vec);
  std::set<int> inlier_set;
  for (size_t it = 0; it < inlier_indices.size(); it++) {
    inlier_set.insert(inlier_indices[it]);
  }

  // Remove from A2B and B2A the outliers
  std::map<int, int> A2B_orig = A2B;
  point_count = 0;
  for (auto it = A2B_orig.begin(); it != A2B_orig.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    if (inlier_set.find(point_count) == inlier_set.end()) {
      auto iter_a = A2B.find(pid_a);
      if (iter_a == A2B.end())
        LOG(FATAL) << "Bookkeeping error 1 in merging maps.";
      A2B.erase(iter_a);

      auto iter_b = B2A.find(pid_b);
      if (iter_b == B2A.end())
        LOG(FATAL) << "Bookkeeping error 2 in merging maps.";
      B2A.erase(iter_b);
    }
    point_count++;
  }

  // LOG(INFO) does not do well with Eiegn.
  std::cout << "Affine transform from second map to first map:\n";
  std::cout << "Matrix:\n"      << B2A_trans.linear()       << "\n";
  std::cout << "Translation:\n" << B2A_trans.translation()  << "\n";

  // Bring the B map into the coordinate system of the A map
  B.Transform(B2A_trans);

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

// Load two maps, merge the second one onto the first one, and save the result.
void AppendMapFile(std::string const& mapOut, std::string const& mapIn,
                   int num_image_overlaps_at_endpoints,
                   double outlier_factor, bool bundle_adjust,
                   bool fix_first_map) {
  if (!bundle_adjust && fix_first_map) LOG(FATAL) << "Cannot fix first map if no bundle adjustment happens.";

  LOG(INFO) << "Appending " << mapIn << " to " << mapOut << std::endl;

  sparse_mapping::SparseMap A(mapOut);
  sparse_mapping::SparseMap B(mapIn);

  // Sanity check before we do a lot of work
  if (fix_first_map && haveSharedElements(A.cid_to_filename_, B.cid_to_filename_))
    LOG(FATAL) << "Cannot fix the first map if it shares cameras with the second map.";

  // C starts as A, as SparseMap lacks an empty constructor
  sparse_mapping::SparseMap C(mapOut);

  // Merge
  sparse_mapping::MergeMaps(&A, &B,
                            num_image_overlaps_at_endpoints,
                            outlier_factor,
                            mapOut,
                            &C);

  // Bundle-adjust the merged map
  if (bundle_adjust) {
    bool fix_all_cameras = false;

    std::set<int> fixed_cameras;

    // Find in the list of images of the merged map the ones from the first map
    if (fix_first_map) {
      std::set<std::string> setA;
      for (size_t it = 0; it < A.cid_to_filename_.size(); it++) setA.insert(A.cid_to_filename_[it]);

      for (size_t it = 0; it < C.cid_to_filename_.size(); it++) {
        if (setA.find(C.cid_to_filename_[it]) != setA.end()) fixed_cameras.insert(it);
      }
    }

    sparse_mapping::BundleAdjust(fix_all_cameras, &C, fixed_cameras);
  }

  C.Save(mapOut);
}

// Given a transform from cid2cid from some cid values to some others,
// apply the same transform to the tracks. This may make the tracks
// shorter if cid2cid maps different inputs to the same output.
// New tracks of length 1 can be excluded if desired.
// TODO(rsoussan): How does this make some tracks shorter???
void TransformTracks(std::map<int, int> const& cid2cid,
                     bool rm_tracks_of_len_one,
                     std::vector<std::map<int, int> > * pid_to_cid_fid) {
  std::vector<std::map<int, int> > pid_to_cid_fid2;
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    auto & cid_fid = (*pid_to_cid_fid)[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      int fid = it->second;
      auto cid_it = cid2cid.find(cid);
      if (cid_it == cid2cid.end()) continue;
      int out_cid = cid_it->second;
      cid_fid2[out_cid] = fid;
    }

    bool will_skip = rm_tracks_of_len_one && (cid_fid2.size() <= 1);
    if (!will_skip)
      pid_to_cid_fid2.push_back(cid_fid2);
  }
  *pid_to_cid_fid = pid_to_cid_fid2;
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
                                    std::vector<std::pair<int, int>>& a_b_pid_tracks_to_merge) const {
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
      a_b_pid_tracks_to_merge.emplace_back({*best_a_pid, *b_pid});
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

void SparseMapMerger::MatchingTracks(const SparseMap& map_a, const SparseMap& map_b, SparseMap& merged_map) {
  const auto match_candidates = DatabaseMatchCandidates(map_a, map_b, params_.max_db_query_image_match_candidates);
  // TODO(rsoussan): rename this?
  std::vector<std::map<int, int> > pid_to_cid_fid;
  merged_map.MatchImagesAndBuildTracks(match_candidates, pid_to_cid_fid);

  // Identify tracks (merge, new, append, invalid) and update tracks to merge if necessary
  std::vector<TrackLabel> track_labels(pid_to_cid_fid.size(), TrackLabel::kInvalid);
  std::vector<std::pair<int, int>> a_b_pid_tracks_to_merge;
  for (int i = 0; i < pid_to_cid_fid.size(); ++i) {
    const auto& cid_fids = pid_to_cid_fid[i];
    IdentifyTrack(cid_fids, i, map_a, map_b, track_labels, a_b_pid_tracks_to_merge);
  }
}
// Given a sparse map in C_out, and a map cid2cid from camera (image)
// indices to new indices, convert the map from being relative to old
// indices to relative to the new indices. If cid2cid maps two input
// indices to the same output index, reconcile the camera positions
// for those indices.
void TransformMap(std::map<int, int> & cid2cid,
                  sparse_mapping::SparseMap * C_out) {
  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & C = *C_out;

  // The total number of output cameras
  int num_out_cams = 0;
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    num_out_cams = std::max(num_out_cams, cid2cid[cid]);
  }
  num_out_cams++;  // move past the last

  // Each blob will be original cids that end up being a single cid
  // after identifying repeat images.
  std::vector< std::set<int> > blobs(num_out_cams);
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    blobs[cid2cid[cid]].insert(cid);
  }

  // To merge cid_to_cam_t_global_, find the average rotation and translation
  // from the two maps.
  std::vector<Eigen::Affine3d > cid_to_cam_t_global2(num_out_cams);
  for (size_t c = 0; c < blobs.size(); c++) {
    if (blobs[c].size() == 1) {
      cid_to_cam_t_global2[c] = C.cid_to_cam_t_global_[*blobs[c].begin()];
    } else {
      int num = blobs[c].size();

      // All cams to merge get equal weight
      std::vector<double> W(num, 1.0/num);

      // TODO(oalexan1): Something more clever could be done. If an
      // image in one map has few tracks going through it, or those
      // tracks are short, this instance could be given less weight.

      std::vector< Eigen::Quaternion<double> >Q(num);
      cid_to_cam_t_global2[c].translation() << 0.0, 0.0, 0.0;
      int pos = -1;
      for (auto it = blobs[c].begin(); it != blobs[c].end() ; it++) {
        pos++;
        int cid = *it;
        Q[pos] = Eigen::Quaternion<double> (C.cid_to_cam_t_global_[cid].linear());

        cid_to_cam_t_global2[c].translation()
          += W[pos]*C.cid_to_cam_t_global_[cid].translation();
      }
      Eigen::Quaternion<double> S = sparse_mapping::slerp_n(W, Q);
      cid_to_cam_t_global2[c].linear() = S.toRotationMatrix();
    }
  }

  // We really count during merging that if two maps have an image in
  // common, the same keypoint map is computed for that image in both
  // maps. Otherwise the book-keeping of cid_fid becomes a disaster.
  for (size_t c = 0; c < blobs.size(); c++) {
    int num = blobs[c].size();
    if (num <= 1) continue;
    int cid0 = *blobs[c].begin();
    for (auto it = blobs[c].begin(); it != blobs[c].end() ; it++) {
      int cid = *it;
      if (C.cid_to_keypoint_map_[cid0] != C.cid_to_keypoint_map_[cid]) {
        LOG(FATAL) << "The two input maps do not have the same features for same images. "
                   << "Cannot merge them. Consider rebuilding them.";
      }
    }
  }

  // Further removal of repetitions.
  std::vector<std::string> cid_to_filename2(num_out_cams);
  std::vector<Eigen::Matrix2Xd > cid_to_keypoint_map2(num_out_cams);
  std::vector<cv::Mat> cid_to_descriptor_map2(num_out_cams);
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    int cid2 = cid2cid[cid];
    cid_to_filename2[cid2]             = C.cid_to_filename_[cid];
    cid_to_keypoint_map2[cid2]         = C.cid_to_keypoint_map_[cid];
    cid_to_descriptor_map2[cid2]       = C.cid_to_descriptor_map_[cid];
  }

  // Modify the tracks after identifying identical images
  // We would rather keep tracks of length one (which we will filter out
  // some time later) than ruin the bookkeeping.
  bool rm_tracks_of_len_one = false;
  TransformTracks(cid2cid, rm_tracks_of_len_one, &C.pid_to_cid_fid_);

  if (C.pid_to_xyz_.size() != C.pid_to_cid_fid_.size()) {
    LOG(FATAL) << "Book-keeping failure in merging maps, "
               << "pid_to_xyz_ and pid_to_cid_fid_ must have the same size.";
  }

  // The new lists for the unique images
  C.cid_to_filename_       = cid_to_filename2;
  C.cid_to_keypoint_map_   = cid_to_keypoint_map2;
  C.cid_to_cam_t_global_   = cid_to_cam_t_global2;
  C.cid_to_descriptor_map_ = cid_to_descriptor_map2;

  // Note: after this step, it is possible some tracks are now
  // duplicate.  We don't bother removing them, it would be a pain,
  // since sometimes two tracks may differ in one or more values and
  // those are tricky to reconcile.

  // Recreate cid_fid_to_pid_ from pid_to_cid_fid_. This must happen
  // after the merging is complete but before using the new map.
  C.InitializeCidFidToPid();

  // C.Save(output_map + ".reduced.map");
}

// Merge two maps. See merge_maps.cc. The merged map needs to be
// bundle-adjusted. We need to have write-access to A and B to be able
// to initialize some auxiliary structures in these maps.
// TODO(oalexan1): Modularize this code (some was done already).
void MergeMaps(sparse_mapping::SparseMap * A_in,
               sparse_mapping::SparseMap * B_in,
               int num_image_overlaps_at_endpoints,
               double outlier_factor,
               std::string const& output_map,
               sparse_mapping::SparseMap * C_out) {
  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & A = *A_in;
  sparse_mapping::SparseMap & B = *B_in;
  sparse_mapping::SparseMap & C = *C_out;

  // Basic sanity checks (not exhaustive)
  if ( !(A.GetCameraParameters() == B.GetCameraParameters()) )
    LOG(FATAL) << "The input maps don't have the same camera parameters.";
  if ( !(A.detector_ == B.detector_) )
    LOG(FATAL) << "The input maps don't have the same detector and/or descriptor.";

  sparse_mapping::HistogramEqualizationCheck(A.GetHistogramEqualization(),
                                             B.GetHistogramEqualization());

  // Wipe things that we won't merge (or not yet)
  C.ClearImageDatabase();
  // TODO(rsoussan): make function to do this
  C.pid_to_cid_fid_.clear();
  C.pid_to_xyz_.clear();
  C.cid_fid_to_pid_.clear();
  C.cid_to_cid_.clear();
  C.user_cid_to_keypoint_map_.clear();
  C.user_pid_to_cid_fid_.clear();
  C.user_pid_to_xyz_.clear();

  // Merge things that make sense to merge and are easy to do
  // TODO(rsoussan): Add merge function to sparse map database that does this! (AA)
  int num_acid = A.cid_to_filename_.size();
  int num_bcid = B.cid_to_filename_.size();
  int num_ccid = num_acid + num_bcid;
  C.cid_to_filename_      .resize(num_ccid);
  C.cid_to_keypoint_map_  .resize(num_ccid);
  C.cid_to_cam_t_global_  .resize(num_ccid);
  C.cid_to_descriptor_map_.resize(num_ccid);
  for (int cid = 0; cid < num_bcid; cid++) {
    // C.cid_to_filename_ already contains A.cid_to_filename_, etc.
    int c = num_acid + cid;
    C.cid_to_filename_[c]       = B.cid_to_filename_[cid];
    C.cid_to_keypoint_map_[c]   = B.cid_to_keypoint_map_[cid];
    C.cid_to_descriptor_map_[c] = B.cid_to_descriptor_map_[cid];
    // We will have to deal with cid_to_cam_t_global_ later
  }

  // Create cid_fid_to_pid_ for both maps, to be able to go from cid_fid to pid.
  A.InitializeCidFidToPid();
  B.InitializeCidFidToPid();

  std::map<int, int> A2B, B2A;
  findMatchingTracks(&A, &B, &C, output_map,
                       num_image_overlaps_at_endpoints,
                       A2B, B2A);  // outputs

  // Put the xyz points corresponding to the tracks to merge in vectors.
  std::vector<Eigen::Vector3d> A_vec(A2B.size()), B_vec(A2B.size());
  int point_count = 0;
  for (auto it = A2B.begin(); it != A2B.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    A_vec[point_count] = A.pid_to_xyz_[pid_a];
    B_vec[point_count] = B.pid_to_xyz_[pid_b];
    point_count++;
  }
  double inlier_threshold = estimateCloseDistance(A_vec);

  // Estimate the transform from B_vec to A_vec using RANSAC.
  // A lot of outliers are possible.
  int  num_iterations = 1000;
  int  min_num_output_inliers = A_vec.size()/2;
  bool reduce_min_num_output_inliers_if_no_fit = true;  // If too many outliers
  bool increase_threshold_if_no_fit = true;  // Coz our threshold was done by a heuristic
  RandomSampleConsensus < TranslationRotationScaleFittingFunctor, TransformError>
    ransac(TranslationRotationScaleFittingFunctor(), TransformError(), num_iterations,
           inlier_threshold, min_num_output_inliers,
           reduce_min_num_output_inliers_if_no_fit, increase_threshold_if_no_fit);
  Eigen::Affine3d B2A_trans = ransac(B_vec, A_vec);
  std::vector<size_t> inlier_indices = ransac.inlier_indices(B2A_trans, B_vec, A_vec);
  std::set<int> inlier_set;
  for (size_t it = 0; it < inlier_indices.size(); it++) {
    inlier_set.insert(inlier_indices[it]);
  }

  // Remove from A2B and B2A the outliers
  std::map<int, int> A2B_orig = A2B;
  point_count = 0;
  for (auto it = A2B_orig.begin(); it != A2B_orig.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    if (inlier_set.find(point_count) == inlier_set.end()) {
      auto iter_a = A2B.find(pid_a);
      if (iter_a == A2B.end())
        LOG(FATAL) << "Bookkeeping error 1 in merging maps.";
      A2B.erase(iter_a);

      auto iter_b = B2A.find(pid_b);
      if (iter_b == B2A.end())
        LOG(FATAL) << "Bookkeeping error 2 in merging maps.";
      B2A.erase(iter_b);
    }
    point_count++;
  }

  // LOG(INFO) does not do well with Eiegn.
  std::cout << "Affine transform from second map to first map:\n";
  std::cout << "Matrix:\n"      << B2A_trans.linear()       << "\n";
  std::cout << "Translation:\n" << B2A_trans.translation()  << "\n";

  // Bring the B map into the coordinate system of the A map
  B.Transform(B2A_trans);

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
