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

#ifndef SPARSE_MAPPING_FEATURE_VOCABULARY_PARAMS_H_
#define SPARSE_MAPPING_FEATURE_VOCABULARY_PARAMS_H_

#include <sparse_map.pb.h>

// TODO(rsoussan): avoid this? check dbow2 templated voc and see if theres a virtual dtor there
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic push
#include <DBoW2/DBoW2.h>      // BoW db that works with both float and binary descriptors
#pragma GCC diagnostic pop

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <vector>
#include <string>

namespace sparse_mapping {
struct FeatureVocabularyParams {
  int branching_factor = 10;
  int depth_levels = 5;
  DBoW2::WeightingType weighting = DBoW2::TF_IDF;
  DBoW2::ScoringType scoring = DBoW2::L1_NORM;
};
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_FEATURE_VOCABULARY_PARAMS_H_
