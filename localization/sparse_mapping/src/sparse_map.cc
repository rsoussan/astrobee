/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <sparse_mapping/remove_invalid_points_and_detections_stats.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/utilities.h>

#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic push
#include <openMVG/multiview/conditioning.hpp>
#include <openMVG/multiview/projection.hpp>
#include <openMVG/multiview/triangulation.hpp>
#pragma GCC diagnostic pop

#include <Eigen/Geometry>

namespace {
bool FixedCamera(const BundleAdjustmentParams& params, const Cid cid) {
  if (params.fix_all_cameras || params.fixed_cameras.count(cid) > 0) return true;
  const bool in_optimize_range =
    params.optimize_camera_range && (cid >= params.first_optimized_camera && cid <= params.last_optimized_camera);
  if (!in_optimize_range) return true;
  return false;
}

bool FixedPoint(const BundleAdjustmentParams& params, const Pid pid, const FeatureTrack& feature_track) {
  if (params.fixed_points.count(pid) > 0) return true;
  // Points which project into cameras that are not fixed are also not fixed
  for (const auto& cid_fid_pair : feature_track) {
    const int cid = cid_fid.first;
    if (!FixedCamera(cid)) return false;
  }
  return true;
}

bool ValidProjection(const Eigen::Vector2d& centered_projected_point, const Eigen::Vector2d& image_half_size) {
  if (centered_projected_point.x() < -1.0 * half_size.x() || centered_projected_point.x() >= half_size.x() ||
      centered_projected_point.y() < -1.0 * half_size.y() || centered_projected_point.y() >= half_size.y())
    return false;
  return true;
}
}  // namespace

namespace sparse_mapping {
namespace fc = ff_common;
namespace oc = optimization_common;

SparseMap::SparseMap(const CidFilenameMap& cid_to_filename, const SparseMapParams& params)
    : params_(params), cid_to_filename_(cid_to_filename) {
  ResizeFeatureMaps();
}

SparseMap::SparseMap(const CidPoseMap& cid_to_cam_T_global, const CidFilenameMap& cid_to_filename,
                     const SparseMapParams& params)
    : params_(params), cid_to_filename_(cid_to_filename), cid_to_cam_T_global_(cid_to_cam_T_global) {
  ResizeFeatureMaps();
}

SparseMap::SparseMap(const CidFilenameMap& cid_to_filename, const CidKeypointsMap& cid_to_keypoints,
                     const CidDescriptorsMap& cid_to_descriptor, const SparseMapParams& params)
    : cid_to_filename_(cid_to_filename),
      cid_to_keypoints_(cid_to_keypoints),
      cid_to_descriptors_(cid_to_descriptors),
      params_(params) {}

void SparseMap::BuildMap() {
  LogInfo("Detecting image features...");
  DetectImageFeatures();
  LogInfo("Matching images and building tracks...");
  const auto relative_affines = MatchImagesAndBuildTracks();
  LogInfo("Performing incremental bundle adjustment...");
  IncrementallyBundleAdjust(relative_affines);
  LogInfo("Performing iterative bundle adjustment...");
  IterativelyBundleAdjust(params_.iterative_bundle_adjustment, params_.num_bundle_adjustment_iterations);
  LogInfo("Building " << DetectorName() << " image database...");
  BuildImageDatabase();
}

void SparseMap::DetectImageFeatures() {
  fc::ThreadPool pool;
  const int num_cameras = NumCameras();
  for (int cid = 0; cid < num_cameras; ++cid) {
    fc::PrintProgressBar(stdout, static_cast<float>(cid) / static_cast<float>(num_cameras - 1));
    pool.AddTask(&SparseMap::DetectImageFeaturesFromFile, this, std::cref(filename(cid)), std::ref(descriptors(cid)),
                 std::ref(keypoints(cid)));
  }
  pool.Join();
}

void SparseMap::DetectImageFeaturesFromFile(const std::string& filename, Descriptors& descriptors,
                                            Keypoints& keypoints) {
  const auto image = LoadImage(filename);
  if (params_.detector_name == "surf") {
    vision_common::SurfDynamicDetector surf_detector(params_.surf_detector);
    DetectFeatures(image, params_.histogram_equalization, surf_detector, descriptors, keypoints);
  } else if (params_.detector_name == "brisk") {
    vision_common::BriskDynamicDetector brisk_detector(params_.brisk_detector);
    DetectFeatures(image, params_.histogram_equalization, brisk_detector, descriptors, keypoints);
  } else {
    LOG(FATAL) << "Invalid detector: " << params_.detector_name;
  }
}

std::vector<MatchCandidates> SparseMap::SequentialMatchCandidates() const {
  std::vector<MatchCandidates> sequential_match_candidates;
  for (int cid = 0; cid < NumCameras(); ++cid) {
    MatchCandidates match_candidates;
    match_candidates.cid = cid;
    for (int sequential_cid = cid + 1;
         sequential_cid < NumCameras() && cid - sequential_cid <= params_.max_sequential_image_match_candidates;
         ++sequential_cid) {
      match_candidates.candidate_cids.emplace_back(sequential_cid);
    }
    sequential_match_candidates.emplace_back(match_candidate);
  }
  return sequential_match_candidates;
}

std::vector<MatchCandidates> SparseMap::DatabaseMatchCandidates(const bool avoid_sequential_cids) const {
  std::vector<MatchCandidates> database_match_candidates;
  for (int cid = 0; cid < NumCameras(); ++cid) {
    MatchCandidates match_candidates;
    match_candidates.cid = cid;
    const auto db_match_candidate_cids =
      image_database().Query(descriptors(cid), params_.max_db_query_image_match_candidates);
    for (const auto candidate_cid : db_match_candidate_cids) {
      if (avoid_sequential_cids && std::abs(candidate_cid - cid) <= params_.max_sequential_image_match_candidates)
        continue;
      match_candidates.candidate_cids.emplace_back(candidate_cid);
    }
    database_match_candidates.emplace_back(match_candidates);
  }
  return database_match_candidates;
}

CIDPairAffineMap SparseMap::MatchImagesAndBuildTracks() {
  const auto sequential_match_candidates = SequentialMatchCandidates();
  const auto database_match_candidates = DatabaseMatchCandidates(true);
  std::vector<MatchCandidates> all_match_candidates;
  all_match_candidates.reserve(seqeuntial_match_candidates.size() + database_match_candidates.size());
  all_match_candidates.insert(all_match_candidates.end(), seqeuntial_match_candidates.begin(),
                              seqeuntial_match_candidates.end());
  all_match_candidates.insert(all_match_candidates.end(), database_match_candidates.begin(),
                              database_match_candidates.end());
  const auto relative_affines = MatchImagesAndBuildTracks(all_match_candidates, pid_to_feature_track_);
  InitializeCidFidPidMap();
  return relative_affines;
}

CIDPairAffineMap SparseMap::MatchImagesAndBuildTracks(const std::vector<MatchCandidates>& match_candidates_vec,
                                                      PidFeatureTrackMap& pid_to_feature_track) const {
  fc::ThreadPool thread_pool;
  std::mutex match_mutex;
  openMVG::matching::PairWiseMatches match_map;
  CIDPairAffineMap relative_affines;
  int i = 0;
  for (const auto& match_candidates : match_candidates_vec) {
    fc::PrintProgressBar(stdout, static_cast<float>(++i) / static_cast<float>(match_candidates_vec.size()));
    const int cid = match_candidates.cid;
    for (const auto& candidate_cid : match_candidates.candidate_cids) {
      thread_pool.AddTask(&sparse_map::MatchImages, this, cid, candidate_cid, std::ref(relative_affines),
                          std::ref(match_map), std::ref(match_mutex));
    }
  }
  thread_pool.Join();

  LOG(INFO) << "Number of affines found: " << relative_affines.size();

  openMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);
  trackBuilder.Filter();
  // Each entry is a sequence of imageId and featureIndex:
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  openMVG::tracks::STLMAPTracks map_tracks;
  trackBuilder.ExportToSTL(map_tracks);

  if (map_tracks.empty()) LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

  // Add tracks to database
  const int num_tracks = map_tracks.size();
  pid_to_feature_track.clear();
  pid_to_feature_track.resize(num_tracks);
  int pid = 0;
  for (const auto& map_track : map_tracks) {
    for (const auto& cid_fid_pair : map_tracks) {
      const int cid = cid_fid_pair.first;
      const int fid = cid_fid_pair.second;
      pid_to_feature_track[pid][cid] = fid;
    }
    ++pid;
  }

  return relative_affines;
}

void MatchImages(const int cid_a, const int cid_b, CIDPairAffineMap& relative_affines,
                 openMVG::matching::PairWiseMatches& match_map, std::mutex& match_mutex) const {
  std::vector<cv::DMatch> inlier_matches;
  const auto relative_pose =
    MatchImages(keypoints(cid_a), keypoints(cid_b), descriptors(cid_a), descriptors(cid_b), params_.camera,
                params_.max_num_image_pair_feature_matches, params_.min_num_inliers_for_valid_match, inlier_matches);
  if (!relative_pose) {
    LOG(DEBUG) << "Failed to match cid " << cid_a << " and cid " << cid_b;
    return;
  }

  std::vector<openMVG::matching::IndMatch> mvg_matches;
  for (const auto& match : inlier_matches)
    mvg_matches.push_back(openMVG::matching::IndMatch(match.queryIdx, match.trainIdx));
  match_mutex->lock();
  match_map[std::make_pair(cid_a, cid_b)] = mvg_matches;
  relative_affines.insert({std::make_pair(cid_a, cid_b), *relative_pose});
  match_mutex->unlock();
}

// TODO(rsoussan): Only triangulate newly added points in between bundle adjustment iterations,
// only bundle adjust cameras and points that have been modified (ala isam2)
void SparseMap::IncrementallyBundleAdjust(const CIDPairAffineMap& relative_affines) {
  // Initialize with same filename, keypoints, descriptors, and params as full map
  SparseMap incremental_map(cid_to_filename(), cid_to_keypoints(), cid_to_descriptors(), params());
  // Initialize first pose at identity
  const Eigen::Affine3d first_cam_T_global = Eigen::Affine3d::Identity();
  incremental_map.AddPose(first_cam_T_global);
  // Start with second camera, update all cameras before and including this, move to next camera and repeat
  for (int latest_cid = 1; latest_cid < NumCids(); ++latest_cid) {
    const int previous_cid = latest_cid - 1;
    const std::pair<int, int> latest_to_previous_cid_pair(previous_cid, latest_cid);
    const Eigen::Affine3d latest_cam_T_previous_cam = relative_affines.count(latest_to_previous_cid_pair) > 0
                                                        ? relative_affines(latest_to_previous_cid_pair)
                                                        : Eigen::Affine3d::Identity();
    const auto& previous_cam_T_global = incremental_map.cam_T_global(previous_cid);
    const Eigen::Affine3d latest_cam_T_global = latest_cam_T_previous_cam * previous_cam_T_global;
    incremental_map.AddPose(latest_cam_T_global);

    // Build tracks up to latest cid
    PidFeatureTrackMap incremental_pid_to_feature_track;
    for (int pid = 0; pid < NumPoints(); ++pid) {
      const auto& feature_track = feature_track(pid);
      FeatureTrack incremental_track;
      for (const auto& cid_fid : feature_track) {
        const int cid = cid_fid.first;
        const int fid = cid_fid.second;
        if (cid <= latest_cid) incremental_track[cid] = fid;
      }

      // Only add long enough tracks
      if ((latest_cid == 1 && track.size() > 1) || track.size() > params_.min_feature_track_length)
        incremental_map.AddFeatureTrack(incremental_track);
    }

    // Initialize points for incremental tracks
    // TODO(rsoussan): Do we really want to remove invalid points here?
    incremental_map.TriangulateAllPoints(true, false);

    const int oldest_cid_to_optimize = OldestCidToOptimize(latest_cid);
    // TODO(rsoussan): Add fcn to set range for params?
    params.incremental_bundle_adjustment.first_optimized_camera = oldest_cid_to_optimize;
    params.incremental_bundle_adjustment.last_optimized_camera = latest_cid;
    LOG(INFO) << "Optimizing cameras from " << oldest_cid_to_optimize << " to " << latest_cid
              << " (total: " << latest_cid - oldest_cid_to_optimize + 1 << ")";

    // TODO(rsoussan): Add warning if ba failed?
    incremental_map.BundleAdjust(params.incremental_bundle_adjustment);
  }

  SetPoses(incremental_map.cid_to_cam_T_global());
  // Initialize map points using bundle adjusted camera poses
  // TODO(rsoussan): Is this better than using bundle adjusted points from incremental map? test both?
  TriangulateAllPoints();
}

void SparseMap::TriangulateAllPoints(const bool remove_invalid_points, const bool initialize_cid_fid_pid_map,
                                     const CidKeypointsMap& cid_to_keypoints,
                                     const PidFeatureTrackMap& pid_to_feature_track,
                                     PidPointMap& pid_to_global_t_point) {
  const double focal_length = params().camera.GetFocalLength();
  Eigen::Matrix3d intrinsics;
  intrinsics << focal_length, 0, 0, 0, focal_length, 0, 0, 0, 1;

  std::vector<openMVG::Mat34> projection_matrices(NumPoses());
  for (int cid = 0; cid < NumPoses(); ++cid) {
    openMVG::P_From_KRt(intrinsics, cam_T_global(cid).linear(), cam_T_global(cid).translation(),
                        &projection_matrices[cid]);
  }

  const int num_points = pid_to_feature_track.size();
  pid_to_global_t_point.resize(num_points);
  // Iterate in reverse so invalid feature tracks can be removed without affecting the order of earlier feature tracks
  for (int pid = num_points - 1; pid >= 0; --pid) {
    openMVG::Triangulation triangulation;
    for (const auto& cid_fid : pid_to_feature_track[pid]) {
      triangulation.add(projection_matrices[cid_fid.first], cid_to_keypoints[cid_fid.first][cid_fid.second]);
    }
    const Eigen::Vector3d solution = triangulation.compute();
    if (remove_invalid_points && (std::isnan(solution[0]) || triangulation.minDepth() < 0)) {
      pid_to_global_t_point.erase(pid_to_global_t_point.begin() + pid);
      pid_to_feature_track.erase(pid_to_feature_track.begin() + pid);
    } else {
      pid_to_global_t_point[pid] = solution;
    }
  }

  if (remove_invalid_points && initialize_cid_fid_pid_map) InitializeCidFidPidMap();
}

int SparseMap::OldestCidToOptimize(const int latest_cid) const {
  // If cid+1 is divisible by 2^k, do at least 2^k cameras, ending
  // with camera cid.  E.g., if current camera index is 23 = 3*8-1, do at
  // least 8 cameras, so cameras 16, ..., 23. This way, we will try
  // to occasionally do more than just several close cameras.
  int val = latest_cid + 1;
  int offset = 1;
  while (val % 2 == 0) {
    val /= 2;
    offset *= 2;
  }
  offset = std::min(offset, params_.max_num_cams_to_incrementally_optimize);

  int oldest_cid_to_optimize = latest_cid - offset + 1;
  oldest_cid_to_optimize =
    std::min(latest_cid - params_.min_num_cams_to_incrementally_optimize + 1, oldest_cid_to_optimize);
  if (oldest_cid_to_optimize < 0) oldest_cid_to_optimize = 0;
  return oldest_cid_to_optimize;
}

void SparseMap::IterativelyBundleAdjust(const BundleAdjustmentParams& params, const int num_iterations) {
  for (int i = 0; i < num_iterations; ++i) {
    LOG(INFO) << "Beginning bundle adjustment, pass: " << i << ".\n";
    const auto summary = BundleAdjust(params);
    const int num_used_observations = NumUsedFeatures();
    LOG(INFO) << summary.FullReport() << "\n";
    LOG(INFO) << "Starting average reprojection error: " << summary.initial_cost / num_used_features();
    LOG(INFO) << "Final average reprojection error:    " << summary.final_cost / num_used_features();
  }
}

ceres::Solver::Summary SparseMap::BundleAdjust(const BundleAdjustmentParams& params) {
  std::vector<Eigen::Matrix<double, 7, 1>> cam_T_global_data_vec;
  cam_T_global_data_vec.reserve(NumPoses());
  for (const auto& cam_T_global : cid_to_cam_T_global()) {
    cam_T_global_data_vec.emplace_back(oc::VectorFromAffine3d(cam_T_global));
  }

  ceres::Problem problem;
  // Centered, undistored camera
  const Eigen::Vector2d zero_principal_points(Eigen::Vector2d::Zero());
  const Eigen::VectorXd zero_distortion(1);
  const Eigen::Vector2d focal_lengths = params().camera.GetFocalVector();
  oc::AddConstantParameterBlock(2, zero_principal_points.data(), problem);
  oc::AddConstantParameterBlock(1, zero_distortion.data(), problem);
  oc::AddConstantParameterBlock(2, focal_lengths.data(), problem);

  // Add detected points without fixing point positions
  AddCostsToBundleAdjustmentProblem(params, zero_principal_points, zero_distortion, focal_lengths, cid_to_keypoints(),
                                    pid_to_feature_track(), pid_to_global_t_point(), params().loss_function, problem);
  // Add control points with no loss function and fixed point positions
  AddCostsToBundleAdjustmentProblem(params, zero_principal_points, zero_distortion, focal_lengths,
                                    control_point_cid_to_keypoints(), control_point_pid_to_feature_track(),
                                    control_point_pid_to_global_t_point(), nullptr, problem, true);

  ceres::Solver::Summary summary;
  ceres::Solve(params.options, &problem, &summary);

  for (int cid = 0; cid < NumPoses(); ++cid) {
    cam_T_global(cid) = oc::Affine3d(cam_T_global_data_vec[cid]);
  }

  if (params.remove_invalid_points_and_detections) {
    RemoveInvalidPointsAndDetections(params.remove_invalid_points_and_detections_params);
  }

  return summary;
}

void SparseMap::AddCostsToBundleAdjustmentProblem(
  const BundleAdjustmentParams& params, const Eigen::Vector2d& zero_principal_points,
  const Eigen::VectorXd& zero_distortion, const Eigen::Vector2d& focal_lengths, const CidKeypointsMap& cid_to_keypoints,
  const PidFeatureTrackMap& pid_to_feature_track, PidPointMap& pid_to_global_t_point,
  ceres::LossFunction* loss_function, ceres::Problem& problem, const bool fix_all_points) const {
  for (int pid = 0; pid < static_cast<int>(pid_to_global_t_point.size()); ++pid) {
    const auto& feature_track = pid_to_feature_track[pid];
    if (feature_track.size() < 2) LOG(FATAL) << "Found a track of size < 2.";

    auto& global_t_point = pid_to_global_t_point[pid];
    const bool fixed_point = fix_all_points || FixedPoint(params, pid, feature_track);
    oc::AddParameterBlock(3, global_t_point.data(), problem, fixed_point);
    for (const auto& cid_fid_pair : feature_track) {
      const int cid = cid_fid_pair.first;
      const int fid = cid_fid_pair.second;
      const auto& image_point = cid_to_keypoints[cid][fid];
      auto& cam_T_global_data = cam_T_global_data_vec[cid];

      const bool fixed_camera = FixedCamera(params, cid);
      oc::AddAffine3ParameterBlock(cam_T_global_data.data(), problem, fixed_camera);
      if (!params.optimize_scale) {
        // TODO(rsoussan): Optimize for scale??? test!! switch to subset manifold?? Can you add two local
        // parameterizations to one param block??
        ceres::SubsetParameterization* constant_scale_parameterization = new ceres::SubsetParameterization(7, {6});
        problem.SetParameterization(cam_T_global_data.data(), constant_scale_parameterization);
      }

      oc::ReprojectionError<vc::IdentityDistorter, oc::AffineFunctor>::AddCostFunction(
        image_point, global_t_point, cam_T_global_data, const_cast<Eigen::Vector2d&>(focal_lengths),
        const_cast<Eigen::Vector2d&>(zero_principal_points), const_cast<Eigen::VectorXd&>(zero_distortion), problem,
        loss_function);
    }
  }
}

void SparseMap::RemoveInvalidPointsAndDetections(const RemoveInvalidPointsAndDetectionsParams& params) {
  std::vector<double> pid_reprojection_errors;
  std::vector<Eigen::Vector3d> global_t_cams;
  global_t_cams.reserve(NumPoses());
  for (const auto& cam_T_global : cid_to_cam_T_global()) {
    global_t_cams.emplace_back(cam_T_global.inverse().translation());
  }

  RemoveInvalidPointsAndDetectionsStats stats;
  stats.num_points = NumPoints();
  std::vector<bool> invalid_point(NumPoints(), false);
  const Eigen::Vector2d half_size = params().camera.GetUndistortedHalfSize();
  const Eigen::Matrix3d intrinsics = params().camera.GetIntrinsicMatrix<camera::UNDISTORTED_C>();
  for (int pid = 0; pid < NumPoints(); ++pid) {
    bool small_angle = false, behind_cam = false, invalid_reprojection = false;

    // Check camera angles
    const auto& feature_track = feature_track(pid);
    const auto& global_t_point = global_t_point(pid);
    const double max_angle_between_camera_rays =
      MaxAngleBetweenCameraRays(feature_track, global_t_point, global_t_cams);
    if (max_angle_between_camera_rays < params.min_max_angle_between_camera_rays) {
      small_angle = true;
      invalid_point[pid] = true;
    }

    for (const auto cid_fid : feature_track) {
      const int cid = cid_fid.first;
      const auto& cam_T_global = cam_T_global(cid);
      const Eigen::Vector3d cam_t_point = cam_T_global * global_t_point;
      // Check if point is behind any camera
      if (cam_t_point.z() <= 0) {
        behind_cam = true;
        invalid_point[pid] = true;
      }

      // Check projection
      const double reprojection_error = ReprojectionError(cid_fid, intrinsics);
      pid_reprojection_errors.emplace_back(reprojection_error);
      const bool valid_projection = ValidProjection(projected_point, half_size);
      if (!valid_projection) {
        invalid_reprojection = true;
        invalid_point[pid] = true;
      }
    }
    stats.small_angle += static_cast<int>(small_angle);
    stats.behind_cam += static_cast<int>(behind_cam);
    stats.invalid_reprojection += static_cast<int>(invalid_reprojection);
  }
  RemovePoints(invalid_point);

  std::vector<bool> invalid_point_detection_count(NumPoints(), false);
  // Remove high reprojection error feature detections
  const double reprojection_error_threshold = ReprojectionErrorThreshold(pid_reprojection_errors, params);
  LOG(INFO) << "Filtering features with reprojection error higher than: " << reprojection_error_threshold << " pixels";
  for (int pid = 0; pid < NumPoints(); ++pid) {
    auto& feauture_track = feature_track(pid);
    const auto& global_t_point = global_t_point(pid);
    for (auto cid_fid_it = feature_track.begin(); cid_fid_it != feature_track.end();) {
      ++stats.num_features;
      const double reprojection_error = ReprojectionError(*cid_fid_it, intrinsics);
      if (reprojection_error >= params.max_reprojection_error) {
        cid_fid_it = feature_track.erase(cid_fid_it);
        ++stats.big_reproj_err;
      } else {
        ++cid_fid_it;
      }
    }
    // Remove point if less than 2 valid feature detections remain
    const int num_detections = FeatureTrackLength(pid);
    if (num_detections < 2) {
      invalid_point_detection_count[pid] = true;
    }
  }
  RemovePoints(invalid_point_detection_count);
  InitializeCidFidPidMap();

  if (params.print_stats) stats.Print();
}

double SparseMap::ReprojectionError(const std::pair<int, int>& cid_fid, const Eigen::Matrix3d& intrinsics) {
  const int cid = cid_fid.first;
  const int fid = cid_fid.second;
  const auto& cam_T_global = cam_T_global(cid);
  const Eigen::Vector3d cam_t_point = cam_T_global * global_t_point;
  const Eigen::Vector2d projected_point = vc::Project(cam_t_point, intrinsics);
  const auto& keypoint = keypoint(cid, fid);
  return (keypoint - projected_point).norm();
}

void SparseMap::UndistortAndAddControlPoints(std::vector<ControlPoint>& control_points) {
  // Undistort keypoints since map expects image detections in undistorted frame
  for (auto& control_point : control_points) {
    params().camera.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(control_point.keypoint_left,
                                                                      &(control_point.keypoint_left));
    params().camera.Convert<camera::DISTORTED, camera::UNDISTORTED_C>(control_point.keypoint_right,
                                                                      &(control_point.keypoint_right));
  }
  AddControlPoints(control_points);
}

std::vector<Eigen::Vector3d> SparseMap::TriangulatedControlPoints() const {
  std::vector<Eigen::Vector3d> triangulated_pid_to_global_t_point;
  TriangulateAllPoints(false, false, control_point_cid_to_keypoints(), control_point_pid_to_feature_track(),
                       triangulated_pid_to_global_t_point);

  return triangulated_pid_to_global_t_point;
}

void SparseMap::PrintControlPointErrors(const std::vector<Eigen::Vector3d>& triangulated_pid_to_global_t_point) const {
  double mean_error = 0;
  const int num_points = control_point_pid_to_global_t_point().size();
  std::cout << "Triangulated xyz -- Control Point xyz -- error diff -- error norm (meters)" << std::endl;
  for (int i = 0; i < num_points; ++i) {
    const auto& triangulated_global_t_point = triangulated_pid_to_global_t_point[i];
    const auto& control_point_global_t_point = control_point_pid_to_global_t_point()[i];
    mean_error += (triangulated_global_t_point - control_point_global_t_point).norm();
    std::cout << triangulated_global_t_point.matrix() << " -- " << control_point_global_t_point.matrix() << " -- "
              << (triangulated_global_t_point - control_point_global_t_point).matrix() << " -- "
              << (triangulated_global_t_point - control_point_global_t_point).norm() << std::endl;
  }
  mean_error /= num_points;
  std::cout << "Mean absolute error for control points: " << mean_error << " meters" << std::endl;
}

void SparseMap::RegisterUsingControlPoints() {
  const auto triangulated_global_t_points = TriangulatedControlPoints();
  std::cout << "Control Point errors before registration: " << std::endl;
  PrintControlPointErrors(triangulated_global_t_points);
  const auto registered_global_T_global =
    EstimateRelativeAffine3D(triangulated_global_t_points, control_point_pid_to_global_t_point());
  Transform(registered_global_T_global);

  std::vector<Eigen::Vector3d> triangulated_registered_global_t_points;
  const int num_points = triangulated_global_t_points.size();
  double mean_error = 0.0;
  for (int i = 0; i < num_points; ++i) {
    triangulated_registered_global_t_points.emplace_back(registered_global_T_global * triangulated_global_t_points[i]);
    mean_error += (triangulated_registered_global_t_points.back() - control_point_pid_to_global_t_point[i]).norm();
  }
  mean_error /= num_points;

  const double scale = std::pow(registered_global_T_global.linear().determinant(), 1.0 / 3.0);
  std::cout << "Transform to world coordinates." << std::endl;
  std::cout << "Rotation:\n" << world_transform.linear() / scale << std::endl;
  std::cout << "Scale:\n" << scale << std::endl;
  std::cout << "Translation:\n" << world_transform.translation().transpose() << std::endl;

  std::cout << "Control Point errors after registration: " << std::endl;
  PrintControlPointErrors(triangulated_registered_global_t_points);
}

void SparseMap::ClearImageDatabase() { image_database_.reset(); }

void SparseMap::BuildImageDatabase() {
  if (DetectorName() == "surf") {
    BuildSurfImageDatabase();
  } else if (DetectorName() == "brisk") {
    BuildBriskImageDatabase();
  } else {
    LOG(FATAL) << "Invalid detector name, cannot build image database.";
  }
}

void SparseMap::BuildSurfImageDatabase() { BuildTemplatedImageDatabase<DBoW2::FSurf64::TDescriptor, DBoW2::FSurf64>(); }

void SparseMap::BuildBriskImageDatabase() { BuildTemplatedImageDatabase<DBoW2::FBrisk::TDescriptor, DBoW2::FBrisk>(); }
}  // namespace sparse_mapping
