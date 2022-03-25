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

void SparseMap::MatchFeatures(const bool remove_invalid_traingulated_points) {
  // TODO(rsoussan): Make struct for this?
  sparse_mapping::CIDPairAffineMap relative_affines;

  ff_common::ThreadPool thread_pool;
  std::mutex match_mutex;
  openMVG::matching::PairWiseMatches match_map;
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

  LOG(INFO) << "Number of affines found:        " << relative_affines.size() << "\n";

  // Initial cameras based on the affines (won't be used later,
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

  // Build tracks using the interface tracksbuilder
  openMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
  trackBuilder.Filter();          // Filter: Remove tracks that have conflict
  // trackBuilder.ExportToStream(std::cout);
  openMVG::tracks::STLMAPTracks map_tracks;
  // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  trackBuilder.ExportToSTL(map_tracks);

  // TODO(oalexan1): Print how many pairwise matches were there before
  // and after filtering tracks.

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

  // TODO(rsoussan): Make ths a function in sparse_map_database, test!
  size_t num_elems = map_tracks.size();
  // Populate back the filtered tracks.
  (s->pid_to_cid_fid_).clear();
  (s->pid_to_cid_fid_).resize(num_elems);
  size_t curr_id = 0;
  for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
    for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
      (s->pid_to_cid_fid_)[curr_id][itr2->first] = itr2->second;
    }
    curr_id++;
  }

  // Triangulate. The results should be quite inaccurate, we'll redo this
  // later. This step is mostly for consistency.
  sparse_mapping::Triangulate(remove_invalid_triangulated_points,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));
}

void MatchImages(const int cid_a, const int cid_b, sparse_mapping::CIDPairAffineMap& relative_affines,
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
