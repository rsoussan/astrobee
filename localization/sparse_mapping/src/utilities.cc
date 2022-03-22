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

#include <sparse_mapping/utilities.h>
namespace sparse_mapping {
// Logic for implementing if two histogram equalization flags are compatible.
// This flag can be either 0 (false), 1 (true), or 2 (unknown). Be tolerant
// of unknown values, but intolerant when true and false are mixed.
void HistogramEqualizationCheck(int histogram_equalization1,
                                                int histogram_equalization2) {
  if ( (histogram_equalization1 == 0 && histogram_equalization2 == 1) ||
       (histogram_equalization1 == 1 && histogram_equalization2 == 0) )
    LOG(FATAL) << "Incompatible values of histogram equalization detected.";
}

bool IsBinaryDescriptor(std::string const& descriptor) {
  if (descriptor == "OPENSIFT" || descriptor == "SIFT" || descriptor == "SURF")
    return false;
  return true;
}

std::string CvMatTypeStr(cv::Mat const& Mat) {
  int type = Mat.type();
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  std::string r;
  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void ListToListMap(std::vector<std::string> const& big_list,
                                   std::vector<std::string> const& small_list,
                                   std::map<int, int> * map) {
  // Given a big list, and a smaller subset of it, for each index i in
  // the small list find the index j in the big list so that
  // small_list[i] equals big_list[j].  Define the map as map[j] = i.
  (*map).clear();

  std::map<std::string, int> str2int;
  for (size_t i = 0; i < big_list.size(); i++)
    str2int[big_list[i]] = i;

  for (size_t i = 0; i < small_list.size(); i++) {
    std::map<std::string, int>::iterator it = str2int.find(small_list[i]);
    if (it == str2int.end())
      LOG(FATAL) << "Could not query image: " << small_list[i];

    (*map)[it->second] = i;
  }
}
}  // namespace sparse_mapping
