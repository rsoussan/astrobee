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

#ifndef SPARSE_MAPPING_TEMPLATED_FEATURE_VOCABULARY_H_
#define SPARSE_MAPPING_TEMPLATED_FEATURE_VOCABULARY_H_

#include <sparse_map/feature_set.h>
#include <sparse_map/feature_vocabulary_params.h>

// TODO(rsoussan): avoid this? check dbow2 templated voc and see if theres a virtual dtor there
#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic push
#include <DBoW2/DBoW2.h>      // BoW db that works with both float and binary descriptors
#pragma GCC diagnostic pop

#include <vector>
#include <string>

namespace sparse_mapping {
template<class TDescriptor, class F>
class TemplatedFeatureVocabulary : public DBoW2::TemplatedVocabulary<TDescriptor, F> {
 public:
  // Initialize empty vocabulary
  explicit TemplatedFeatureVocabulary(const FeatureVocabularyParams& params);
  // Create vocabulary
  TemplatedFeatureVocabulary(const FeatureSets& feature_sets, const FeatureVocabularyParams& params);
};

// Implementation
template <class TDescriptor, class F>
TemplatedFeatureVocabulary<TDescriptor, F>::TemplatedFeatureVocabulary(const FeatureVocabularyParams& params)
    : DBoW2::TemplatedVocabulary<TDescriptor, F>(params.branching_factor, params.depth_levels, params.weighting,
                                                 params.scoring) {}

template <class TDescriptor, class F>
TemplatedFeatureVocabulary<TDescriptor, F>::TemplatedFeatureVocabulary(const FeatureSets& feature_sets,
                                                                       const FeatureVocabularyParams& params)
    : DBoW2::TemplatedVocabulary<TDescriptor, F>(params.branching_factor, params.depth_levels, params.weighting,
                                                 params.scoring) {
  create(feature_sets);
}
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_TEMPLATED_FEATURE_VOCABULARY_H_
