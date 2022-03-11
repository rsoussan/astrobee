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

#ifndef SPARSE_MAPPING_PROTOBUF_DATABASE_H_
#define SPARSE_MAPPING_PROTOBUF_DATABASE_H_

#include <sparse_map.pb.h>

#pragma GCC diagnostic ignored "-Wdelete-non-virtual-dtor"
#pragma GCC diagnostic push
#include <DBoW2/DBoW2.h>      // BoW db that works with both float and binary descriptors
#pragma GCC diagnostic pop

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <vector>
#include <string>

namespace sparse_mapping {

template<class TDescriptor, class F>
class ProtobufDatabase : public DBoW2::TemplatedDatabase<TDescriptor, F> {
 public:
  explicit ProtobufDatabase(google::protobuf::io::ZeroCopyInputStream* input)
     : DBoW2::TemplatedDatabase<TDescriptor, F>() {LoadProtobuf(input);}
  ProtobufDatabase(ProtobufVocabulary<TDescriptor, F> const& voc, bool flag, int val) :
     DBoW2::TemplatedDatabase<TDescriptor, F>(voc, flag, val) {}
  void SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const;
  void LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input);
};

// TODO(rsoussan): Change this to binary database? change fbrief to brisk or doesn't matter?
// make typedef binary_descriptor and make comment that brief works for any binary descriptor?
typedef ProtobufDatabase<DBoW2::FBrief::TDescriptor, DBoW2::FBrief> BriefDatabase;

// Implementation
template<class TDescriptor, class F>
void ProtobufDatabase<TDescriptor, F>::LoadProtobuf(google::protobuf::io::ZeroCopyInputStream* input) {
  ProtobufVocabulary<TDescriptor, F>* voc = new ProtobufVocabulary<TDescriptor, F>();
  voc->LoadProtobuf(input);
  this->m_voc = voc;

  sparse_mapping_protobuf::DBoWDB db;

  if (!ReadProtobufFrom(input, &db)) {
    LOG(FATAL) << "Failed to parse db file.";
  }

  this->clear();  // resizes inverted file

  this->m_nentries = db.num_entries();
  this->m_use_di = 0;
  this->m_dilevels = 0;

  for (int i = 0; i < db.num_inverted_index(); ++i) {
    sparse_mapping_protobuf::DBoWInvertedIndexEntry entry;
    if (!ReadProtobufFrom(input, &entry)) {
      LOG(FATAL) << "Failed to parse index entry.";
    }
    DBoW2::WordId wid = entry.word_id();
    DBoW2::EntryId eid = entry.entry_id();
    DBoW2::WordValue v = entry.weight();

    this->m_ifile[wid].push_back(typename DBoW2::TemplatedDatabase<TDescriptor, F>::IFPair(eid, v));
  }
}

template<class TDescriptor, class F>
void ProtobufDatabase<TDescriptor, F>::SaveProtobuf(google::protobuf::io::ZeroCopyOutputStream* output) const {
  (dynamic_cast<ProtobufVocabulary<TDescriptor, F>* >(this->m_voc))->SaveProtobuf(output);

  sparse_mapping_protobuf::DBoWDB db;

  db.set_num_entries(this->m_nentries);

  int num_inverted_index = 0;
  typename DBoW2::TemplatedDatabase<TDescriptor, F>::InvertedFile::const_iterator iit;
  for (iit = this->m_ifile.begin(); iit != this->m_ifile.end(); ++iit)
    num_inverted_index += (*iit).size();
  db.set_num_inverted_index(num_inverted_index);
  if (!WriteProtobufTo(db, output)) {
    LOG(FATAL) << "Failed to write db to file.";
  }
  typename DBoW2::TemplatedDatabase<TDescriptor, F>::IFRow::const_iterator irit;
  int word_id = 0;
  for (iit = this->m_ifile.begin(); iit != this->m_ifile.end(); ++iit) {
    for (irit = iit->begin(); irit != iit->end(); ++irit) {
      sparse_mapping_protobuf::DBoWInvertedIndexEntry index;
      index.set_word_id(word_id);
      index.set_entry_id(irit->entry_id);
      index.set_weight(irit->word_weight);
      if (!WriteProtobufTo(index, output)) {
        LOG(FATAL) << "Failed to write db index entry to file.";
      }
    }
    word_id++;
  }
}
}  // namespace sparse_mapping
#endif  // SPARSE_MAPPING_PROTOBUF_DATABASE_H_
