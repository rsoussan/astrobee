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

#include <sparse_map.pb.h>
#include <sparse_map/feature_set.h>
#include <sparse_map/feature_vocabulary_params.h>

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

// Enables more efficient file storage for the vocabulary class, which otherwise
// uses an ASCII format that is very large and slow to load.
template<class TDescriptor, class F>
class TemplatedFeatureVocabulary : public DBoW2::TemplatedVocabulary<TDescriptor, F> {
 public:
  // Create empty vocab
  explicit TemplatedFeatureVocabulary(const FeatureVocabularyParams& params);
  // Create vocab and add features
  TemplatedFeatureVocabulary(const FeatureSets& feature_sets, const FeatureVocabularyParams& params);
  // Load vocab from a file
  explicit TemplatedFeatureVocabulary(google::protobuf::io::ZeroCopyInputStream* input);
  void SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const;
  void LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input);
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

template <class TDescriptor, class F>
TemplatedFeatureVocabulary<TDescriptor, F>::TemplatedFeatureVocabulary(google::protobuf::io::ZeroCopyInputStream* input)
    : DBoW2::TemplatedVocabulary<TDescriptor, F>() {
  LoadProtobuf(input);
}

template<class TDescriptor, class F>
void TemplatedFeatureVocabulary<TDescriptor, F>::LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input) {
  this->m_words.clear();
  this->m_nodes.clear();

  sparse_mapping_protobuf::DBoWVocab vocab;
  if (!ReadProtobufFrom(input, &vocab)) {
    LOG(FATAL) << "Failed to parse vocab file.";
  }

  this->m_k = vocab.k();
  this->m_L = vocab.l();
  this->m_scoring = (DBoW2::ScoringType)vocab.scoring_type();
  this->m_weighting = (DBoW2::WeightingType)vocab.weighting_type();
  int num_nodes = vocab.num_nodes();
  int num_words = vocab.num_words();

  this->createScoringObject();

  this->m_nodes.resize(num_nodes + 1);  // +1 to include root
  this->m_nodes[0].id = 0;

  for (int i = 0; i < num_nodes; ++i) {
    sparse_mapping_protobuf::DBoWNode node;
    if (!ReadProtobufFrom(input, &node)) {
      LOG(FATAL) << "Failed to parse node file.";
    }
    DBoW2::NodeId nid = node.node_id();
    DBoW2::NodeId pid = node.parent_id();
    DBoW2::WordValue weight = (DBoW2::WordValue)node.weight();
    std::string d = node.feature();

    this->m_nodes[nid].id = nid;
    this->m_nodes[nid].parent = pid;
    this->m_nodes[nid].weight = weight;
    this->m_nodes[pid].children.push_back(nid);

    F::fromBytes(this->m_nodes[nid].descriptor, d);
  }

  // words
  this->m_words.resize(num_words);
  for (int i = 0; i < num_words; ++i) {
    sparse_mapping_protobuf::DBoWWord word;
    if (!ReadProtobufFrom(input, &word)) {
      LOG(FATAL) << "Failed to parse word file.";
    }
    DBoW2::NodeId wid = word.word_id();
    DBoW2::NodeId nid = word.node_id();

    this->m_nodes[nid].word_id = wid;
    this->m_words[wid] = &this->m_nodes[nid];
  }
}

template <class TDescriptor, class F>
void TemplatedFeatureVocabulary<TDescriptor, F>::SaveProtobuf(
  google::protobuf::io::ZeroCopyOutputStream* output) const {
  sparse_mapping_protobuf::DBoWVocab vocab;

  vocab.set_k(this->m_k);
  vocab.set_l(this->m_L);
  vocab.set_scoring_type(this->m_scoring);
  vocab.set_weighting_type(this->m_weighting);
  vocab.set_num_nodes(this->m_nodes.size() - 1);  // -1 to exclude root node
  vocab.set_num_words(this->m_words.size());
  if (!WriteProtobufTo(vocab, output)) {
    LOG(FATAL) << "Failed to write vocab to file.";
  }

  std::vector<DBoW2::NodeId> parents, children;
  std::vector<DBoW2::NodeId>::const_iterator pit;
  parents.push_back(0);  // root
  while (!parents.empty()) {
    DBoW2::NodeId pid = parents.back();
    parents.pop_back();

    typename DBoW2::TemplatedVocabulary<TDescriptor, F>::Node const& parent = this->m_nodes[pid];
    children = parent.children;
    for (pit = children.begin(); pit != children.end(); pit++) {
      typename DBoW2::TemplatedVocabulary<TDescriptor, F>::Node const& child = this->m_nodes[*pit];

      sparse_mapping_protobuf::DBoWNode node;
      node.set_node_id(child.id);
      node.set_parent_id(pid);
      node.set_weight(child.weight);
      node.set_feature(F::toBytes(child.descriptor));
      if (!WriteProtobufTo(node, output)) {
        LOG(FATAL) << "Failed to write db node to file.";
      }

      // add to parent list
      if (!child.isLeaf()) {
        parents.push_back(*pit);
      }
    }
  }

  typename std::vector<typename DBoW2::TemplatedVocabulary<TDescriptor, F>::Node*>::const_iterator wit;
  for (wit = this->m_words.begin(); wit != this->m_words.end(); wit++) {
    sparse_mapping_protobuf::DBoWWord word;
    typename DBoW2::WordId id = wit - this->m_words.begin();
    word.set_word_id(id);
    word.set_node_id((*wit)->id);
    if (!WriteProtobufTo(word, output)) {
      LOG(FATAL) << "Failed to write word to file.";
    }
  }
}
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_TEMPLATED_FEATURE_VOCABULARY_H_
