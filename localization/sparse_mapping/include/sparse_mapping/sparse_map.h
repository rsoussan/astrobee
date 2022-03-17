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
#include <interest_point/matching.h>
#include <sparse_mapping/image_database.h>
#include <sparse_mapping/params.h>
#include <sparse_mapping/sparse_map_database.h>
#include <sparse_mapping/sparse_mapping.h>
#include <camera/camera_model.h>
#include <camera/camera_params.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <string>
#include <vector>
#include <utility>
#include <limits>

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
   * Constructs a new sparse map from a list of image files and their
   * associate keypoint and descriptor files. If use_cached_features
   * is set to false, it reads the image files and performs feature
   * detection instead. Does not perform bundle adjustment.
   **/
  SparseMap(const std::vector<std::string> & filenames,
            const std::string & detector,
            const camera::CameraParameters & params);

  /**
   * Constructs a new sparse map from a protobuf file, with specified
   * vocabulary tree and optional parameters.
   **/
  SparseMap(const std::string & protobuf_file,
            bool localization = false);

  /**
     Form a sparse map with given cameras/images, and no features
  **/
  SparseMap(const std::vector<Eigen::Affine3d>& cid_to_cam_t,
            const std::vector<std::string> & filenames,
            const std::string & detector,
            const camera::CameraParameters & params);


  SparseMap(bool bundler_format, std::string const& filename, std::vector<std::string> const& files);

  void SetParams(const std::string& detector, const camera::CameraParameters& camera_params);

  void BuildDatabase(const FeatureSets& feature_sets);

  /**
   * Detect features in given images
   **/
  void DetectFeatures();

  /**
   * Save the map to a protobuf file.
   **/
  void Save(const std::string & protobuf_file) const;

  int GetHistogramEqualization() const {return params_.histogram_equalization;}
  /**
   * Return the parameters of the camera used to construct the map.
   **/
  const camera::CameraParameters& camera_params() const {return params_.camera;}

  void SetCameraParameters(const camera::CameraParameters& camera_params) {params_.camera = camera_params;}

  // Load map. If localization is true, load only the parts of the map
  // needed for localization.
  void Load(const std::string & protobuf_file, bool localization = false);

  void DetectFeaturesFromFile(const std::string& filename,
                              cv::Mat* descriptors,
                              Eigen::Matrix2Xd* keypoints);

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

 private:
  std::unique_ptr<ImageDatabase> image_database_;
  SparseMapParams params_;

  // I found out the hard way that sparse maps cannot be copied
  // correctly, hence prohibit this. The only good way seems to be to
  // load a copy from disk. (oalexan1)
  SparseMap();
  SparseMap(SparseMap &);
  SparseMap& operator=(const SparseMap&);

  // Reorder the images in the map and the rest of the data accordingly
  void reorderMap(std::map<int, int> const& old_cid_to_new_cid);
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
