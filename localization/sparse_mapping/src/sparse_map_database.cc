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

#include <sparse_mapping/sparse_map_database.h>
namespace sparse_mapping {
void SparseMapDatabase::ResizeFeatureMaps() {
  const int num_frames = GetNumFrames();
  cid_to_keypoint_map_.resize(num_frames);
  cid_to_descriptor_map_.resize(num_frames);
}

void SparseMapDatabase::InitializeCidFidToPid() {
  sparse_mapping::InitializeCidFidToPid(cid_to_filename_.size(),
                                        pid_to_cid_fid_,
                                        &cid_fid_to_pid_);
}

  std::vector<cv::Mat> SparseMapDatabase::GetCidFeatures(const int cid) const {
      std::vector<cv::Mat> features;
      const int num_features = map->GetFrameKeypoints(cid).outerSize();
      for (int i = 0; i < num_features; ++i) {
        features.emplace_back(map->GetDescriptor(cid, i));
      }
  }

  std::vector<std::vector<cv::Mat>> SparseMapDatabase::GetAllCidFeatures() const {
      std::vector<std::vector<cv::Mat>> all_features;
      const int num_images = map->GetNumFrames();
      for (int cid = 0; cid < num_images; ++cid) {
        all_features.emplace_back(GetCidFeatures(cid));
      }
  }

int SparseMapDatabase::NumFeatures() const {
  int num_features = 0;
  for (int cid = 0; cid < map.GetNumFrames(); ++cid) {
    total_features += map.GetFrameKeypoints(cid).outerSize();
  }
  return num_features;
}
}  // namespace sparse_mapping
