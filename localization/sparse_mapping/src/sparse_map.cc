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
  /*if (cid_to_filename.size() != cid_to_cam_T_global.size())
    LOG(FATAL) << "Expecting as many images as cameras";
  for (int cid = 0; cid < static_cast<int>(cid_to_cam_T_global.size()); ++cid) {
    // TODO(rsoussan): Is this check necessary?
    if (cid_to_cam_T_global[cid].linear() == Eigen::Matrix3d::Zero())
      continue;
    cid_to_cam_T_global_.emplace_back(cid_to_cam_T_global[cid]);
    cid_to_filename_.emplace_back(cid_to_filename[cid]);
  }*/
}

void SparseMap::DetectFeatures() {
  ff_common::ThreadPool pool;
  size_t num_files = cid_to_filename_.size();
  for (size_t cid = 0; cid < num_files; cid++) {
    ff_common::PrintProgressBar(stdout, static_cast<float>(cid) / static_cast<float>(num_files - 1));

    pool.AddTask(&SparseMap::DetectFeaturesFromFile, this,
                 std::ref(cid_to_filename_[cid]),
                 &cid_to_descriptor_map_[cid],
                 &cid_to_keypoint_map_[cid]);
  }
  pool.Join();

  // Create temporary pid_to_cid_fid_, it will contain all the raw
  // features we found so far, without matches (matching and outlier
  // removal will later reduce the number of features, so this is
  // useful for comparison).
  pid_to_cid_fid_.clear();
  for (size_t cid = 0; cid < cid_to_filename_.size(); cid++) {
    for (int fid = 0; fid < cid_to_keypoint_map_[cid].cols(); fid++) {
      std::map<int, int> cid_fid;
      cid_fid[cid] = fid;
      pid_to_cid_fid_.emplace_back(cid_fid);
    }
  }
  // Allocate space for landmarks
  pid_to_xyz_.resize(pid_to_cid_fid_.size());

  InitializeCidFidToPid();
}

void SparseMap::DetectFeaturesFromFile(const std::string& filename,
                                       cv::Mat* descriptors,
                                       Eigen::Matrix2Xd* keypoints) {
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
