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

#include <sparse_mapping/utilities.h>

namespace sparse_mapping {
// From pid_to_cid_fid, create cid_fid_to_pid for lookup.
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid) {
  cid_fid_to_pid->clear();
  cid_fid_to_pid->resize(num_cid, std::map<int, int>());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid[pid]) {
      (*cid_fid_to_pid)[cid_fid.first][cid_fid.second] = pid;
    }
  }
}

cv::Mat LoadImage(const std::string& filename) {
  const cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Found empty image in file: " << filename;
  return image;
}

void DetectFeatures(const cv::Mat& image, const bool histogram_equalization,
                                interest_point::Detector& detector,
                               cv::Mat* descriptors,
                               Eigen::Matrix2Xd* keypoints) {
  cv::Mat hist_image;
  if (histogram_equalization) {
    cv::equalizeHist(image, hist_image);
  }
  const auto& input_image = histogram_equalization ? hist_image : image;

  std::vector<cv::KeyPoint> storage;
  detector.Detect(input_image, &storage, descriptors);

  keypoints->resize(2, storage.size());
  Eigen::Vector2d output;
  for (int i = 0; i < static_cast<int>(storage.size()); ++i) {
    camera_params_.Convert<camera::DISTORTED_C, camera::UNDISTORTED_C>
      (Eigen::Vector2d(storage[i].pt.x, storage[i].pt.y), &output);
    keypoints->col(i) = output;
  }
}
}  // namespace sparse_mapping
