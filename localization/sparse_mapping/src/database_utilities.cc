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


#include <sparse_mapping/database_utilities.h>

namespace sparse_mapping {
void BuildDB(std::string const& map_file,
                             std::string const& descriptor,
                             int depth, int branching_factor, int restarts) {
  SparseMap map(map_file);

  // replace any existing database
  ResetDB(&map.vocab_db_);

  int total_features = 0;
  for (size_t cid = 0; cid < map.GetNumFrames(); cid++)
    total_features += map.GetFrameKeypoints(cid).outerSize();
  while (pow(branching_factor, depth) < total_features) {
    depth++;
    LOG(WARNING) << "Database not large enough, increasing depth.";
  }
  LOG(INFO) << "Total database capacity is " << pow(branching_factor, depth)
            << ", total features to insert are " << total_features << ".";

  BuildDBforDBoW2(&map, descriptor, depth, branching_factor, restarts);

  map.Save(map_file);
}

void ResetDB(VocabDB* db) {
  if (db->binary_db != NULL) {
    delete db->binary_db;
    db->binary_db = NULL;
  }
}

void MatDescrToVec(cv::Mat const& mat, std::vector<float> * vec) {
  // Go from a row matrix of float descriptors to a vector of
  // descriptors.
  if (mat.rows != 1)
    LOG(FATAL) << "Expecting a single-row matrix.\n";

  (*vec).reserve(mat.cols);
  (*vec).clear();
  for (int c = 0; c < mat.cols; c++) {
    float val = static_cast<float>(mat.at<uchar>(0, c));
    (*vec).push_back(val);
  }
}

void MatDescrToVec(cv::Mat const& mat, DBoW2::BriefDescriptor * brief) {
  // Go from a row matrix of binary descriptors to a vector of
  // descriptors, extracting the bits from each byte along the way.
  if (mat.rows != 1)
    LOG(FATAL) << "Expecting a single-row matrix.\n";

  brief->Initialize(mat.cols);

  for (int c = 0; c < mat.cols; c++)
    brief->desc[c] = mat.at<uchar>(0, c);
}

// Query the database. Return the indices of the images
// which are most similar to the current image. Return
// at most num_similar such indices.
void QueryDB(std::string const& descriptor, VocabDB * vocab_db,
             int num_similar, cv::Mat const& descriptors,
             std::vector<int> * indices) {
  indices->clear();

  if (vocab_db->binary_db != NULL) {
    assert(IsBinaryDescriptor(descriptor));
    BinaryDB & db = *(vocab_db->binary_db);  // shorten

    std::vector<DBoW2::BriefDescriptor> descriptors_vec;
    for (int r = 0; r < descriptors.rows; r++) {
      DBoW2::BriefDescriptor descriptor;
      MatDescrToVec(descriptors.row(r), &descriptor);
      descriptors_vec.push_back(descriptor);
    }

    DBoW2::QueryResults ret;
    db.query(descriptors_vec, ret, num_similar);

    for (size_t j = 0; j < ret.size(); j++) {
      indices->push_back(ret[j].Id);
    }
  } else {
    // no database specified
    return;
  }

  return;
}

void BuildDBforDBoW2(SparseMap* map, std::string const& descriptor,
                     int depth, int branching_factor,
                     int restarts) {
  int num_frames = map->GetNumFrames();

  const DBoW2::WeightingType weight = DBoW2::TF_IDF;
  const DBoW2::ScoringType score = DBoW2::L1_NORM;
  int num_features = 0;

  if (!IsBinaryDescriptor(descriptor)) {
    LOG(ERROR) << "Using unsupported vocabulary database type.";
  } else {
    // Binary descriptors. For each image, copy them from a CV matrix
    // to a vector of vectors. Also extract individual bits from
    // each byte.
    std::vector<std::vector<DBoW2::FBrief::TDescriptor > > features;
    for (int cid = 0; cid < num_frames; cid++) {
      int num_keys = map->GetFrameKeypoints(cid).outerSize();
      num_features += num_keys;
      std::vector<DBoW2::FBrief::TDescriptor> descriptors;
      for (int i = 0; i < num_keys; i++) {
        cv::Mat row = map->GetDescriptor(cid, i);
        DBoW2::FBrief::TDescriptor descriptor;
        MatDescrToVec(row, &descriptor);
        descriptors.push_back(descriptor);
      }
      features.push_back(descriptors);
    }
    BinaryVocabulary voc(branching_factor, depth, weight, score);
    voc.create(features);

    BinaryDB* db = new BinaryDB(voc, false, 0);
    for (size_t i = 0; i < features.size(); i++)
      db->add(features[i]);

    map->vocab_db_.binary_db = db;
  }
}
}  // namespace sparse_mapping
