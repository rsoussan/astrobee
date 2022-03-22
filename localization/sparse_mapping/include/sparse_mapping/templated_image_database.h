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

#ifndef SPARSE_MAPPING_TEMPLATED_IMAGE_DATABASE_H_
#define SPARSE_MAPPING_TEMPLATED_IMAGE_DATABASE_H_

#include <sparse_map/feature_set.h>
#include <sparse_map/image_database.h>
#include <sparse_map/image_database_params.h>
#include <sparse_map/templated_feature_vocabulary.h>

#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic push
#include <DBoW2/DBoW2.h>
#pragma GCC diagnostic pop

#include <vector>
#include <string>

namespace sparse_mapping {

template<class TDescriptor, class F>
class TemplatedImageDatabase : public DBoW2::TemplatedDatabase<TDescriptor, F>, public ImageDatabase{
 public:
  TemplatedImageDatabase(const TemplatedFeatureVocabulary<TDescriptor, F>& voc, const ImageDatabaseParams& params);
  TemplatedImageDatabase(const FeatureSets& feature_sets, const ImageDatabaseParams& params);
  std::vector<int> Query(const cv::Mat& features, const int max_results) const override;
  // Return the cids of the images which are most similar to the current image in sorted order
  // beginning with the best matching cids
  std::vector<int> Query(const FeatureSet& features, const int max_results) const override;
};

// Implementation
template <class TDescriptor, class F>
TemplatedImageDatabase<TDescriptor, F>::TemplatedImageDatabase(
  TemplatedFeatureVocabulary<TDescriptor, F> const& vocabulary, const ImageDatabaseParams& params)
    : DBoW2::TemplatedDatabase<TDescriptor, F>(vocabulary, params.use_direct_index, params.direct_index_levels) {}

template <class TDescriptor, class F>
TemplatedImageDatabase<TDescriptor, F>::TemplatedImageDatabase(const FeatureSets& feature_sets,
                                                               const ImageDatabaseParams& params)
    : DBoW2::TemplatedDatabase<TDescriptor, F>(params.use_direct_index, params.direct_index_levels) {
  const TemplatedFeatureVocabulary<TDescriptor, F> vocabulary(feature_sets, params.vocabulary);
  setVocabulary(vocabulary);
  for (const auto& feature_set : feature_sets) {
    add(feature_set);
  }
}

template <class TDescriptor, class F>
std::vector<int> TemplatedImageDatabase<TDescriptor, F>::Query(const cv::Mat& features,
                                                               const int max_results) {
    FeatureSet feature_set;
    for (int row = 0; row < features.rows; ++row) {
      feature_set.push_back(features.row(row);
    }
    return Query(feature_set, max_results);
}

template <class TDescriptor, class F>
std::vector<int> TemplatedImageDatabase<TDescriptor, F>::Query(const FeatureSet& features,
                                                               const int max_results) {
  std::vector<int> matching_cids;
  DBoW2::QueryResults results;
  this->query(features, results, max_results);
  for (const auto& result : results) {
    matching_cids.push_back(result.Id);
  }
  return matching_cids;
}
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_TEMPLATED_IMAGE_DATABASE_H_
