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

#include <localization_common/logger.h>l
#include <sparse_map_matcher/sparse_map_matcher_wrapper.h>

namespace sparse_map_matcher {
SparseMapMatcherWrapper::SparseMapMatcherWrapper() {
  config_reader::ConfigReader config;
  config.AddFile("cameras.config");
  config.AddFile("sparse_map_matcher.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  SparseMapMatcherParams params;
  LoadSparseMapMatcherParams(config, params);
  matcher_.reset(std::make_unique<SparseMapMatcher>(params));
}

void SparseMapMatcherWrapper::ImageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
  // TODO(rsoussan): convert msg to cv::Mat!!!
  // TODO(rsoussan): move vl_msg creation to wrapper!!
  const auto vl_msg = matcher_.ImageCallback(image_msg);
  if (vl_msg) vl_publisher_.publish(*vl_msg);
}
}  // namespace sparse_map_matcher
