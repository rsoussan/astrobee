/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <camera/camera_params.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/utilities.h>

#include <Eigen/Geometry>

namespace sparse_mapping {

SparseMap::SparseMap(const std::vector<std::string>& cid_to_filename, const SparseMapParams& params)
    : params_(params), cid_to_filename_(cid_to_filename) {
    ResizeFeatureMaps();
}

SparseMap::SparseMap(const std::vector<Eigen::Affine3d>& cid_to_cam_T_global,
                     const std::vector<std::string>& cid_to_filename, const SparseMapParams& params)
    : params_(params), cid_to_filename_(cid_to_filename), cid_to_cam_T_global_(cid_to_cam_T_global) {
  ResizeFeatureMaps();
}

void SparseMap::DetectFeatures() {
  ff_common::ThreadPool pool;
  const int num_cameras = num_cameras();
  for (int cid = 0; cid < num_cameras; ++cid) {
    ff_common::PrintProgressBar(stdout, static_cast<float>(cid) / static_cast<float>(num_cameras - 1));
    pool.AddTask(&SparseMap::DetectFeaturesFromFile, this,
                 std::cref(filename(cid)),
                 std::ref(descriptors(cid)),
                 std::ref(keypoints(cid)));
  }
  pool.Join();

  // TODO(rsoussan): Remove this? Is this necessary? Initializes new pid for every feature detected??
  // Create temporary pid_to_cid_fid_, it will contain all the raw
  // features we found so far, without matches (matching and outlier
  // removal will later reduce the number of features, so this is
  // useful for comparison).
  pid_to_cid_fid_.clear();
  for (int cid = 0; cid < num_cameras; ++cid) {
    for (int fid = 0; fid < num_features(cid); ++fid) {
      std::map<int, int> cid_fid;
      cid_fid[cid] = fid;
      pid_to_cid_fid_.emplace_back(cid_fid);
    }
  }
  // Allocate space for landmarks
  const int num_points = pid_to_cid_fid_.size();
  pid_to_xyz_.resize(num_points);
  InitializeCidFidToPid();
}

void SparseMap::DetectFeaturesFromFile(const std::string& filename,
                                       cv::Mat& descriptors,
                                       Eigen::Matrix2Xd& keypoints) {
  const auto image = LoadImage(filename);
  if (params_.detector_name == "surf") {
    vision_common::SurfDynamicDetector surf_detector(params_.surf_detector);
    DetectFeatures(image, params_.histogram_equalization, surf_detector, descriptors, keypoints);
  } else if (params_.detector_name == "brisk") {
    vision_common::BriskDynamicDetector brisk_detector(params_.brisk_detector);
    DetectFeatures(image, params_.histogram_equalization, brisk_detector, descriptors, keypoints);
  } else {
    LOG(FATAL) << "Invalid detector: " << params_.detector_name;
  }
}

CIDPairAffineMap SparseMap::MatchFeatures(const bool remove_invalid_traingulated_points) {
  ff_common::ThreadPool thread_pool;
  std::mutex match_mutex;
  openMVG::matching::PairWiseMatches match_map;
  CIDPairAffineMap relative_affines;
  for (int cid = 0; cid < num_cameras(); ++cid) {
    ff_common::PrintProgressBar(stdout, static_cast<float>(cid)
                             / static_cast <float>(num_features() - 1));
    // Find sequential matches
    for (int sequential_cid = cid + 1;
         sequential_cid < num_cameras() && cid - sequential_cid <= params_.max_sequential_image_match_candidates;
         ++sequential_cid) {
      thread_pool.AddTask(&sparse_map::MatchImages, this, cid, candidate_cid, std::ref(relative_affines),
                          std::ref(match_map), std::ref(match_mutex));
    }
    // Find other matches
    const auto db_match_candidate_cids =
      image_database().Query(descriptors(cid), params_.max_db_query_image_match_candidates);
    for (const auto candidate_cid : db_match_candidate_cids) {
      // Don't check candidate cids that were already checked as sequential candidates
      if (std::abs(candidate_cid - cid) <= params_.max_sequential_image_match_candidates) continue;
      thread_pool.AddTask(&sparse_map::MatchImages, this, cid, candidate_cid, std::ref(relative_affines),
                          std::ref(match_map), std::ref(match_mutex));
    }
  }
  thread_pool.Join();

  LOG(INFO) << "Number of affines found: " << relative_affines.size();

  openMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);
  trackBuilder.Filter();
  // Each entry is a sequence of imageId and featureIndex:
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  openMVG::tracks::STLMAPTracks map_tracks;
  trackBuilder.ExportToSTL(map_tracks);

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

  // Add tracks to database
  const int num_tracks = map_tracks.size();
  pid_to_cid_fid_.clear();
  pid_to_cid_fid_.resize(num_tracks);
  int pid = 0;
  for (const auto& map_track : map_tracks) {
    for (const auto& cid_fid_pair : map_tracks) {
      const int cid = cid_fid_pair.first;
      const int fid = cid_fid_pair.second;
      pid_to_cid_fid_[pid][cid] = fid;
    }
    ++pid;
  }

  // TODO(rsoussan): Remove this?
/* // Initial cameras based on the affines (won't be used later,
  // just for visualization purposes).
  int num_images = s->cid_to_filename_.size();
  (s->cid_to_cam_t_global_).resize(num_images);
  (s->cid_to_cam_t_global_)[0].setIdentity();
  for (int cid = 1; cid < num_images; cid++) {
    std::pair<int, int> P(cid-1, cid);
    if (relative_affines.find(P) != relative_affines.end())
      (s->cid_to_cam_t_global_)[cid] = relative_affines[P]*(s->cid_to_cam_t_global_)[cid-1];
    else
      (s->cid_to_cam_t_global_)[cid] = (s->cid_to_cam_t_global_)[cid-1];  // no choice
  }


  // Triangulate. The results should be quite inaccurate, we'll redo this
  // later. This step is mostly for consistency.
  Triangulate(remove_invalid_triangulated_points,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));*/

  return relative_affines;
}

void MatchImages(const int cid_a, const int cid_b, CIDPairAffineMap& relative_affines,
                 openMVG::matching::PairWiseMatches& match_map, std::mutex& match_mutex) const {
  std::vector<cv::DMatch> inlier_matches;
  const auto relative_pose =
    MatchImages(keypoints(cid_a), keypoints(cid_b), descriptors(cid_a), descriptors(cid_b), params_.camera,
                params_.max_num_image_pair_feature_matches, params_.min_num_inliers_for_valid_match, inlier_matches);
  if (!relative_pose) {
    LOG(DEBUG) << "Failed to match cid " << cid_a << " and cid " << cid_b;
    return;
  }

  std::vector<openMVG::matching::IndMatch> mvg_matches;
  for (const auto& match : inlier_matches)
    mvg_matches.push_back(openMVG::matching::IndMatch(match.queryIdx, match.trainIdx));
  match_mutex->lock();
  match_map[std::make_pair(cid_a, cid_b)] = mvg_matches;
  relative_affines.insert({std::make_pair(cid_a, cid_b), *relative_pose});
  match_mutex->unlock();
}

// delete all the features that do not match to a landmark but are still around!
void SparseMap::PruneMap(void) {
  for (unsigned int cid = 0; cid < cid_fid_to_pid_.size(); cid++) {
    std::vector<int> deleted_features;
    for (int fid = 0; fid < cid_to_descriptor_map_[cid].rows; fid++) {
      // delete if no matching landmark!
      if (cid_fid_to_pid_[cid].count(fid) == 0) {
        deleted_features.emplace_back(fid);
      }
    }
    if (deleted_features.size() == 0)
      continue;
    // create new descriptor map
    cv::Mat next_descriptor_map;
    next_descriptor_map.create(cid_to_descriptor_map_[cid].rows - deleted_features.size(),
                               cid_to_descriptor_map_[cid].cols, cid_to_descriptor_map_[cid].depth());
    int new_fid = 0;
    for (int fid = 0; fid < cid_to_descriptor_map_[cid].rows; fid++) {
      // delete if no matching landmark!
      if (cid_fid_to_pid_[cid].count(fid) == 0) {
        continue;
      } else {
        cid_to_descriptor_map_[cid].row(fid).copyTo(next_descriptor_map.row(new_fid));
        // fix indexing
        if (new_fid < fid) {
          int pid = cid_fid_to_pid_[cid][fid];
          // in localization mode this is empty
          if (pid_to_cid_fid_.size() > 0)
            pid_to_cid_fid_[pid][cid] = new_fid;
          cid_fid_to_pid_[cid][new_fid] = pid;
          cid_fid_to_pid_[cid].erase(fid);
        }
        new_fid++;
      }
    }
    cid_to_descriptor_map_[cid] = next_descriptor_map;

    // clean up other stuff
    for (int i = static_cast<int>(deleted_features.size() - 1); i >= 0; i--) {
      int fid = deleted_features[i];
      // these may not always exist if localizing
      if (cid_to_keypoint_map_.size() > 0) {
        int rows = cid_to_keypoint_map_[cid].rows();  // must be equal to 2
        int cols = cid_to_keypoint_map_[cid].cols();
        // TODO(oalexan1): Copying blocks like this repeatedly is
        // expensive.  It is simpler to just shift columns left one by
        // one, as done above.
        if (fid < cols - 1)
          cid_to_keypoint_map_[cid].block(0, fid, rows, cols - 1 - fid) =
            cid_to_keypoint_map_[cid].block(0, fid + 1, rows, cols - 1 - fid);
        cid_to_keypoint_map_[cid].conservativeResize(rows, cols - 1);
      }
    }
  }

  // This is not strictly necessary as all book-keeping was already done
  InitializeCidFidToPid();
}

// TODO(oalexan1): This very naive code can use serious performance
// improvements.  Each time we add a new camera we triangulate all
// points. We bundle-adjust the last several cameras, but while seeing
// (and keeping fixed) all the earlier cameras. It is sufficient to
// both triangulate and see during bundle adjustment only the several
// most similar cameras. Fixing these would need careful testing for
// both map quality and run-time before and after the fix.
void SparseMap::IncrementalBundleAdjust(const CIDPairAffineMap& relative_affines) {
  for (int latest_cid = 1; latest_cid < num_cameras(); ++latest_cid) {
    std::vector<Eigen::Affine3d > incremental_cid_to_cam_t_global;
    incremental_cid_to_cam_t_global.reserve(latest_cid + 1);
    for (int previous_cid = 0; previous_cid < latest_cid; ++previous_cid)
      incremental_cid_to_cam_t_global.emplace_back(cam_T_global(previous_cid));

    const int previous_cid = latest_cid -1;
    const std::pair<int, int> latest_to_previous_cid_pair(previous_cid, latest_cid);
    // Initialize latest cid pose. If relative pose not available wrt to previous cid,
    // initialize to previous cid pose.
    const Eigen::Affine3d latest_cid_T_previous_cid = relative_affines.count(latest_to_previous_cid_pair) > 0
                                                        ? relative_affines(latest_to_previous_cid_pair)
                                                        : Eigen::Affine3d::Identity();
    const auto& previous_cid_T_global = incremental_cid_to_cam_t_global[previous_cid];
      incremental_cid_to_cam_t_global.emplace_back(latest_cid_T_previous_cid*previous_cid_T_global);

    // Fill incremental tracks up to latest cid
    std::vector<std::map<int, int> > incremental_pid_to_cid_fid;
    for (int pid = 0; pid < num_points(); ++pid) {
      const auto& feature_track = feature_track(pid);
      std::map<int, int> incremental_track;
      for (const auto& cid_to_fid : feature_track) {
        const int cid = cid_to_fid.first;
        const int fid = cid_to_fid.second;
        if (cid <= latest_cid)
          incremental_track[cid] = fid;
      }

      // Add long enough tracks
      if ((latest_cid == 1 && track.size() > 1) || track.size() > params_.min_feature_track_length)
        incremental_pid_to_cid_fid.push_back(incremental_track);
    }

    // Triangulate incrementally added tracks
    std::vector<Eigen::Vector3d> incremental_pid_to_xyz;
    std::vector<std::map<int, int> > incremental_cid_fid_to_pid;
  // TODO(rsoussan): what happens when invalid points are removed??
    Triangulate(true,
                                params_.camera.GetFocalLength(),
                                incremental_cid_to_cam_t_global,
                                cid_to_keypoint_map_,
                                &incremental_pid_to_cid_fid,
                                &incremental_pid_to_xyz,
                                &incremental_cid_fid_to_pid);


    const int oldest_cid_to_optimize = OldestCidToOptimize(latest_cid);
    LOG(INFO) << "Optimizing cameras from " << oldest_cid_to_optimize << " to " << latest_cid << " (total: "
        << latest_cid-oldest_cid_to_optimize+1 << ")";

    ceres::Solver::Summary summary;
    ceres::LossFunction* cauchy_loss = new ceres::CauchyLoss(params_.cauchy_loss);
    // TODO(rsoussan): What are user_ maps? used elsewhere? make local?
    BundleAdjust(incremental_pid_to_cid_fid, s->cid_to_keypoint_map_,
                                 s->camera_params_.GetFocalLength(),
                                 &incremental_cid_to_cam_t_global, &incremental_pid_to_xyz,
                                 s->user_pid_to_cid_fid_,
                                 s->user_cid_to_keypoint_map_,
                                 &(s->user_pid_to_xyz_),
                                 cauchy_loss, params_.incremental_ba_options, &summary,
                                 oldest_cid_to_optimize, latest_cid);

    // Update sparse map with latest incremental cam_T_globals
    for (int cid = 0; cid <= latest_cid; ++c)
      cam_T_global(cid) = incremental_cid_to_cam_t_global[cid];
  }

  Triangulate(true);
}

void Triangulate(const bool remove_invalid_xyz) {
  Triangulate(remove_invalid_xyz,
                              params_.camera.GetFocalLength(),
                              cid_to_cam_t_global_,
                              cid_to_keypoint_map_,
                              &pid_to_cid_fid_,
                              &pid_to_xyz_,
                              &cid_fid_to_pid_);
}

int OldestCidToOptimize(const int latest_cid) const {
    // If cid+1 is divisible by 2^k, do at least 2^k cameras, ending
    // with camera cid.  E.g., if current camera index is 23 = 3*8-1, do at
    // least 8 cameras, so cameras 16, ..., 23. This way, we will try
    // to occasionally do more than just several close cameras.
    int val = latest_cid+1;
    int offset = 1;
    while (val % 2 == 0) {
      val /= 2;
      offset *= 2;
    }
    offset = std::min(offset, params_.max_num_cams_to_incrementally_optimize);

    int oldest_cid_to_optimize = latest_cid-offset+1;
    oldest_cid_to_optimize =
      std::min(latest_cid - params_.min_num_cams_to_incrementally_optimize + 1, oldest_cid_to_optimize);
    if (oldest_cid_to_optimize < 0) oldest_cid_to_optimize = 0;
    return oldest_cid_to_optimize;
}

void ClearImageDatabase() {
  image_database_.reset();
}

void SparseMap::BuildSurfImageDatabase() {
  BuildImageDatabase<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64>();
}

void SparseMap::BuildBriskImageDatabase() {
  BuildImageDatabase<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk>();
}
}  // namespace sparse_mapping
