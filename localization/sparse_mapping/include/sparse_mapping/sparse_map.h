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

#include <camera/camera_params.h>
#include <ff_common/eigen_vectors.h>
#include <sparse_mapping/bundle_adjustment_params.h>
#include <sparse_mapping/image_database.h>
#include <sparse_mapping/remove_invalid_points_and_detections_params.h>
#include <sparse_mapping/sparse_map_database.h>
#include <sparse_mapping/sparse_map_params.h>

#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sparse_mapping {
struct MatchCandidates {
  int cid;
  std::vector<int> candidate_cids;
};

// Contains functions to build a sparse map from images, consisting of finding features,
// building feature tracks, and performing bundle adjustment.
class SparseMap : public SparseMapDatabase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * Creates a sparse map containing only a list of image files.
   **/
  SparseMap(const CidFilenameMap& cid_to_filename, const SparseMapParams& params);

  /**
   * Creates a sparse map containing only a list of image files and camera poses.
   **/
  // TODO(rsoussan): How to use this in practice? remove this?
  SparseMap(const CidPoseMap& cid_to_cam_T_global, const CidFilenameMap& cid_to_filename,
            const SparseMapParams& params);

  void BuildMap();

  /**
   * Detect features in given images
   **/
  void DetectImageFeatures();

  int GetHistogramEqualization() const { return params_.histogram_equalization; }
  /**
   * Return the parameters of the camera used to construct the map.
   **/
  const camera::CameraParameters& camera_params() const { return params_.camera; }

  // TODO(rsoussan): remove this?
  void SetCameraParameters(const camera::CameraParameters& camera_params) { params_.camera = camera_params; }

  void DetectImageFeaturesFromFile(const std::string& filename, Descriptors& descriptors, Keypoints& keypoints);

  std::vector<MatchCandidates> SequentialMatchCandidates() const;

  std::vector<MatchCandidates> DatabaseMatchCandidates(const bool avoid_sequential_cids) const;

  CIDPairAffineMap MatchImagesAndBuildTracks();

  CIDPairAffineMap MatchImagesAndBuildTracks(const std::vector<MatchCandidates>& match_candidates_vec,
                                             PidFeatureTrackMap& pid_to_feature_track) const;

  void IncrementallyBundleAdjust();

  void IterativelyBundleAdjust(const BundleAdjustmentParams& params, const int num_iterations);

  // Assumes poses are initialized
  ceres::Solver::Summary BundleAdjust(const BundleAdjustmentParams& params,
                                      const std::vector<Eigen::Matrix2Xd>& cid_to_keypoints,
                                      PidPoseMap* cid_to_cam_T_global, PidFeatureTrackMap* pid_to_feature_track,
                                      PidPointMap* pid_to_global_t_point, CidFidPidMap* cid_to_fid_to_pid = nullptr);

  void UndistortAndAddControlPoints(std::vector<ControlPoint>& control_points);

  void RegisterUsingControlPoints();

  // delete feature descriptors with no matching landmark
  void PruneMap();

  const std::string& DetectorName() { return params_.detector.name; }

  const ImageDatabase& image_database() const { return *image_database_; }

  void ClearImageDatabase();

  void BuildImageDatabase();

  // TODO(rsoussan): Why is this needed??
  SparseMapParams& params() { return params_; }

  const SparseMapParams& params() const { return params_; }

  // Protobuf Functions
  void Save(const std::string& protobuf_file) const;
  void Load(const std::string& protobuf_file, bool localization = false);

 private:
  void MatchImages(const int cid_a, const int cid_b, sparse_mapping::CIDPairAffineMap& relative_affines,
                   openMVG::matching::PairWiseMatches& match_map, std::mutex& match_mutex) const;

  int OldestCidToOptimize(const int latest_cid) const;

  void TriangulateAllPoints(const bool remove_invalid_points = true, const bool initialize_cid_fid_pid_map = true,
                            const CidKeypointsMap& cid_to_keypoints = cid_to_keypoints(),
                            const PidFeatureTrackMap& pid_to_feature_track = pid_to_feature_track(),
                            PidPointMap& pid_to_global_t_point = pid_to_global_t_point());

  void AddCostsToBundleAdjustmentProblem(const BundleAdjustmentParams& params,
                                         const Eigen::Vector2d& zero_principal_points,
                                         const Eigen::VectorXd& zero_distortion, const Eigen::Vector2d& focal_lengths,
                                         const CidKeypointsMap& cid_to_keypoints,
                                         const PidFeatureTrackMap& pid_to_feature_track,
                                         PidPointMap& pid_to_global_t_point, ceres::LossFunction* loss_function,
                                         ceres::Problem& problem, const bool fix_all_points = false) const;

  // Remove points that don't project at valid camera pixels,
  // points behind the camera, and matches having large reprojection error.
  void RemoveInvalidPointsAndDetections(const RemoveInvalidPointsAndDetectionsParams& params);

  double ReprojectionError(const std::pair<int, int>& cid_fid, const Eigen::Matrix3d& intrinsics);

  std::vector<Eigen::Vector3d> TriangulatedControlPoints() const;

  void PrintControlPointErrors(const std::vector<Eigen::Vector3d>& triangulated_pid_to_global_t_point) const;

  template <class TDescriptor, class F>
  void BuildTemplatedImageDatabase();

  void BuildSurfImageDatabase();

  void BuildBriskImageDatabase();

  std::unique_ptr<ImageDatabase> image_database_;
  SparseMapParams params_;

  /*// I found out the hard way that sparse maps cannot be copied
  // correctly, hence prohibit this. The only good way seems to be to
  // load a copy from disk. (oalexan1)
  SparseMap();
  SparseMap(SparseMap&);
  SparseMap& operator=(const SparseMap&);*/
};

// Implementation
template <class TDescriptor, class F>
void SparseMap::BuildTemplatedImageDatabase() {
  const int total_features = NumFeatures();
  while (std::pow(params_.image_database.vocabulary.branching_factor, params_.image_database.vocabulary.depth) <
         total_features) {
    ++params_.image_database.vocabulary.depth;
    LOG(WARNING) << "Database not large enough, increasing depth.";
  }
  LOG(INFO) << "Total database capacity is "
            << std::pow(params_.image_database.vocabulary.branching_factor, params_.image_database.vocabulary.depth)
            << ", total features to insert are " << total_features << ".";

  const auto feature_sets = AllFeatures();
  image_database_.reset(new TemplatedImageDatabase<TDescriptor, F>(feature_sets, params_.image_database));
}
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_H_
