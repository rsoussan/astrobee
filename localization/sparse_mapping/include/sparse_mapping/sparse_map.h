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

#ifndef SPARSE_MAPPING_SPARSE_MAP_H_
#define SPARSE_MAPPING_SPARSE_MAP_H_

#include <ff_common/eigen_vectors.h>
#include <sparse_mapping/image_database.h>
#include <sparse_mapping/sparse_map_database.h>
#include <sparse_mapping/sparse_map_params.h>
#include <sparse_mapping/sparse_mapping.h>
#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <map>
#include <string>
#include <vector>

namespace sparse_mapping {
/**
 * A class representing a sparse map, which consists of a collection
 * of keyframes and detected features. To localize, an image's features
 * are matched to the keyframes in the map. They keyframe features have known
 * positions and the camera pose can be estimated with ransac.
 **/
class SparseMap : public SparseMapDatabase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * Creates a sparse map containing only a list of image files.
  **/
  SparseMap(const std::vector<std::string> & cid_to_filename,
            const SparseMapParams& params);

  /**
   * Creates a sparse map containing only a list of image files and camera poses.
  **/
  SparseMap(const std::vector<Eigen::Affine3d>& cid_to_cam_T_global,
            const std::vector<std::string> & cid_to_filename,
            const SparseMapParams& params);


  void BuildDatabase(const FeatureSets& feature_sets);

  /**
   * Detect features in given images
   **/
  void DetectFeatures();

  int GetHistogramEqualization() const {return params_.histogram_equalization;}
  /**
   * Return the parameters of the camera used to construct the map.
   **/
  const camera::CameraParameters& camera_params() const {return params_.camera;}

  void SetCameraParameters(const camera::CameraParameters& camera_params) {params_.camera = camera_params;}

  void DetectFeaturesFromFile(const std::string& filename,
                              cv::Mat& descriptors,
                              Eigen::Matrix2Xd& keypoints);

  CIDPairAffineMap MatchFeatures(const bool remove_invalid_traingulated_points);

  void IncrementalBundleAdjust();

  // delete feature descriptors with no matching landmark
  void PruneMap();

  std::string GetDetectorName() { return params_.detector.name; }

  const ImageDatabase& image_database() const { return *image_database_; }

  void ClearImageDatabase();

  template<class TDescriptor, class F>
  void BuildImageDatabase();

  void BuildSurfImageDatabase();

  void BuildBriskImageDatabase();

  SparseMapParams& params() { return params_; }

  const SparseMapParams& params() const { return params_; }

  // Protobuf Functions
  void Save(const std::string& protobuf_file) const;
  void Load(const std::string& protobuf_file, bool localization = false);

 private:
  void MatchImages(const int cid_a, const int cid_b, sparse_mapping::CIDPairAffineMap& relative_affines,
                   openMVG::matching::PairWiseMatches& match_map, std::mutex& match_mutex) const;

  int OldestCidToOptimize(const int latest_cid) const;

  std::unique_ptr<ImageDatabase> image_database_;
  SparseMapParams params_;

  // I found out the hard way that sparse maps cannot be copied
  // correctly, hence prohibit this. The only good way seems to be to
  // load a copy from disk. (oalexan1)
  SparseMap();
  SparseMap(SparseMap &);
  SparseMap& operator=(const SparseMap&);
};

// Implementation
template<class TDescriptor, class F>
void SparseMap::BuildImageDatabase() {
  const int total_features = NumFeatures();
  while (std::pow(params_.image_database.vocabulary.branching_factor, params_.image_database.vocabulary.depth) <
         total_features) {
    ++params_.image_database.vocabulary.depth;
    LOG(WARNING) << "Database not large enough, increasing depth.";
  }
  LOG(INFO) << "Total database capacity is "
            << std::pow(params_.image_database.vocabulary.branching_factor, params_.image_database.vocabulary.depth)
            << ", total features to insert are " << total_features << ".";

  const auto feature_sets = GetAllFeatures();
  image_database_.reset(new TemplatedImageDatabase<TDescriptor, F>(feature_sets, params_.image_database));
}
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_H_
