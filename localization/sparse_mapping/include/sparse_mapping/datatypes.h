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

#ifndef SPARSE_MAPPING_DATATYPES_H_
#define SPARSE_MAPPING_DATATYPES_H_

#include <opencv2/core/core.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sparse_mapping {
// Cids and Pids are assumed to always start at 0 and increase by one up to num cids/pids,
// therefore cid and pid maps can use a vector as a container.
using Cid = int;
using Fid = int;
using Pid = int;
using Keypoint = Eigen::Vector2d;
using Keypoints = std::vector<Keypoints>;
using Descriptor = cv::Mat;
using Descriptors = std::vector<Descriptor>;
using CidKeypointsMap = std::vector<Cid, Keypoints>;
using CidDescriptorsMap = std::vector<Cid, Descriptors>;
using CidFilenameMap = std::vector<std::string>;
using CidPoseMap = std::vector<Eigen::Affine3d>;
using FeatureTrack = std::unordered_map<Cid, Fid>;
using PidFeatureTrackMap = std::vector<FeatureTrack>;
using PidPointMap = std::vector<Eigen::Vector3d>;
// Useful for inverse lookup of points given feature ids
using FidPidMap = std::unordered_map<Fid, Pid>;
using CidFidPidMap = std::vector<FidPidMap>;
using CIDPairAffineMap = std::map<std::pair<int, int>, Eigen::Affine3d, std::less<std::pair<int, int> >,
                                  Eigen::aligned_allocator<std::pair<std::pair<int, int> const, Eigen::Affine3d> > >;
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_DATATYPES_H_
