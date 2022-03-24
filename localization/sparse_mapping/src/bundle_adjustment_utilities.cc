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

#include <ff_common/init.h>
#include <ff_common/thread.h>
#include <ff_common/utils.h>
#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/ransac.h>
#include <sparse_mapping/sparse_mapping.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/utilities.h>
#include <sparse_mapping/vocab_tree.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
// Get rid of warning beyond our control
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic push
#include <openMVG/multiview/projection.hpp>
#include <openMVG/multiview/rotation_averaging_l1.hpp>
#include <openMVG/multiview/triangulation_nview.hpp>
#include <openMVG/numeric/numeric.h>
#include <openMVG/tracks/tracks.hpp>
#pragma GCC diagnostic pop

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/stat.h>

#include <set>
#include <thread>
#include <vector>
#include <mutex>
#include <functional>
#include <cstdio>

DEFINE_int32(min_valid, 20,
              "Minimum number of valid inlier matches required to keep matches for given image pair.");
DEFINE_int32(max_pairwise_matches, 5000,
             "Maximum number of matches to keep in any image pair.");
DEFINE_int32(num_subsequent_images, std::numeric_limits<int32_t>::max()/2,  // avoid overflow
             "When no vocabulary tree is provided, match every image against this "
             "many subsequent images.");
DEFINE_int32(match_all_rate, -1,  // avoid overflow
             "If nonnegative, match one of every match_all_rate images to every other image.");
DEFINE_bool(skip_filtering, false,
            "Skip filtering of outliers after bundle adjustment.");
DEFINE_bool(skip_adding_new_matches_on_merging, false,
            "When merging maps, do not take advantage of performed matching to add new tracks.");
DEFINE_bool(fast_merge, false,
            "When merging maps that have shared images, use those and skip doing additional matches among the images.");
DEFINE_double(reproj_thresh, 5.0,
              "Filter points with re-projection error higher than this.");

// bundle adjustment phase parameters
DEFINE_int32(max_num_iterations, 1000,
             "Maximum number of iterations for bundle adjustment solver.");
DEFINE_int32(num_ba_passes, 5,
             "How many times to run bundle adjustment, removing outliers each time.");
DEFINE_string(cost_function, "Cauchy",
              "Choose a bundle adjustment cost function from: Cauchy, PseudoHuber, Huber, L1, L2.");
DEFINE_double(cost_function_threshold, 2.0,
              "Threshold to use with some cost functions, e.g., Cauchy.");
DEFINE_int32(first_ba_index, 0,
             "Vary only cameras starting with this index during bundle adjustment.");
DEFINE_int32(last_ba_index, std::numeric_limits<int>::max(),
             "Vary only cameras ending with this index during bundle adjustment.");

DEFINE_bool(silent_matching, false,
            "Do not print a lot of verbose info when matching.");

namespace sparse_mapping {
// Two minor and local utility functions
std::string print_vec(double a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f", a);
  return std::string(st);
}
std::string print_vec(Eigen::Vector3d a) {
  char st[256];
  snprintf(st, sizeof(st), "%7.4f %7.4f %7.4f", a[0], a[1], a[2]);
  return std::string(st);
}

  // Read matches from disk into OpenMVG's format
int ReadMatches(std::string const& matches_file,
                openMVG::matching::PairWiseMatches * match_map) {
  (*match_map).clear();
  std::ifstream imfile(matches_file.c_str());
  if (!imfile.good())
    LOG(FATAL) << "Could not read: " << matches_file << ". The matching step needs to be redone.";

  LOG(INFO) << "Reading: " << matches_file;
  std::string line;
  int num_matches = 0;
  while (std::getline(imfile, line)) {
    // replace '_' with ' '
    char * ptr = const_cast<char*>(line.c_str());
    for (size_t it = 0; it < line.size(); it++)
      if (ptr[it] == '_') ptr[it] = ' ';
    int i0, mi0, j0, mj0;
    if (sscanf(line.c_str(), "%d %d %d %d\n", &i0, &mi0, &j0, &mj0) != 4) continue;
    size_t i = i0, mi = mi0, j = j0, mj = mj0;
    std::pair<size_t, size_t> P(i, j);
    if ((*match_map).find(P) == (*match_map).end())
      (*match_map)[P] = std::vector<openMVG::matching::IndMatch>();
    openMVG::matching::IndMatch M(mi, mj);
    (*match_map)[P].push_back(M);
    num_matches++;
  }
  return num_matches;
}

void WriteMatches(openMVG::matching::PairWiseMatches const& match_map,
                  std::string const& matches_file) {
  // Save the matches to disk in the format: cidi_fidi cidj_fidj
  LOG(INFO) << "Writing: " << matches_file;
  std::ofstream mfile(matches_file.c_str());
  for (openMVG::matching::PairWiseMatches::const_iterator iter = match_map.begin();
       iter != match_map.end(); ++iter) {
    const size_t & I = iter->first.first;
    const size_t & J = iter->first.second;
    const std::vector<openMVG::matching::IndMatch> & matchVec = iter->second;
    // We have correspondences between I and J image indices
    for (size_t k = 0; k < matchVec.size(); ++k) {
      mfile << I << "_" << matchVec[k].i_ << " "
            << J << "_" << matchVec[k].j_ << std::endl;
    }
  }
  mfile.close();
}

void BuildMapPerformMatching(openMVG::matching::PairWiseMatches * match_map,
                             std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                             std::vector<cv::Mat> const& cid_to_descriptor_map,
                             camera::CameraParameters const& camera_params,
                             CIDPairAffineMap * relative_affines,
                             std::mutex * match_mutex,
                             int i /*query cid index*/, int j /*train cid index*/,
                             bool compute_rays_angle, double * rays_angle) {
  Eigen::Matrix2Xd const& keypoints1 = cid_to_keypoint_map[i];
  Eigen::Matrix2Xd const& keypoints2 = cid_to_keypoint_map[j];

  std::vector<cv::DMatch> matches, inlier_matches;
  FindMatches(cid_to_descriptor_map[i],
                              cid_to_descriptor_map[j],
                              &matches);

  // Do a check and verify that we meet our minimum before the
  // essential matrix fitting.
  if (static_cast<int32_t>(matches.size()) < FLAGS_min_valid) {
    if (!FLAGS_silent_matching) LOG(INFO) << i << " " << j << " | Failed to find enough matches " << matches.size();
    return;
  }

  bool compute_inliers_only = false;
  BuildMapFindEssentialAndInliers(keypoints1, keypoints2, matches,
                                  camera_params, compute_inliers_only,
                                  i, j,
                                  match_mutex,
                                  relative_affines,
                                  &inlier_matches,
                                  compute_rays_angle, rays_angle);

  if (static_cast<int32_t>(inlier_matches.size()) < FLAGS_min_valid) {
    if (!FLAGS_silent_matching)
      LOG(INFO) << i << " " << j << " | Failed to find enough inlier matches "
                << inlier_matches.size();
    return;
  }

  if (!FLAGS_silent_matching) LOG(INFO) << i << " " << j << " success " << inlier_matches.size();

  std::vector<openMVG::matching::IndMatch> mvg_matches;
  for (std::vector<cv::DMatch>::value_type const& match : inlier_matches)
    mvg_matches.push_back(openMVG::matching::IndMatch(match.queryIdx, match.trainIdx));
  match_mutex->lock();
  (*match_map)[ std::make_pair(i, j) ] = mvg_matches;
  match_mutex->unlock();
}


/**
 * Create the initial map by feature matching and essential affine computation.
 **/
void MatchFeatures(const std::string & essential_file,
                   const std::string & matches_file,
                   sparse_mapping::SparseMap * s) {
  sparse_mapping::CIDPairAffineMap relative_affines;

  // Iterate through the cid pairings
  ff_common::ThreadPool thread_pool;
  std::mutex match_mutex;

  openMVG::matching::PairWiseMatches match_map;
  for (size_t cid = 0; cid < s->cid_to_keypoint_map_.size(); cid++) {
    // Query the db for similar images
    ff_common::PrintProgressBar(stdout, static_cast<float>(cid)
                             / static_cast <float>(s->cid_to_keypoint_map_.size() - 1));
    std::vector<int> indices, queried_indices;
    queried_indices = s->image_database().Query(s->cid_to_descriptor_map_[cid], s->num_similar_);

    if (!queried_indices.empty()) {
      // always include the next three images
      if (cid + 1 < s->cid_to_filename_.size())
        indices.push_back(static_cast<int>(cid) + 1);
      if (cid + 2 < s->cid_to_filename_.size())
        indices.push_back(static_cast<int>(cid) + 2);
      if (cid + 3 < s->cid_to_filename_.size())
        indices.push_back(static_cast<int>(cid) + 3);
      // Managed to find images similar to the current one in the
      // database
      for (size_t j = 0; j < queried_indices.size(); j++) {
        if (static_cast<int>(cid) + 3 < queried_indices[j]) {
          // Keep only subsequent images
          indices.push_back(queried_indices[j]);
        }
      }
      std::sort(indices.begin(), indices.end());

      // Print what is going on. May need to remove this later.
      if (!FLAGS_silent_matching) LOG(INFO) << "Matching image " << s->cid_to_filename_[cid] << " with: ";
      for (size_t j = 0; j < indices.size(); j++) {
        // Keep only subsequent images
        if (!FLAGS_silent_matching) LOG(INFO) << s->cid_to_filename_[indices[j]];
      }
      if (!FLAGS_silent_matching) LOG(INFO) << "\n\n";
    } else {
      // No matches in the db, or no db was provided.
      if ( s->cid_to_cid_.find(cid) != s->cid_to_cid_.end() ) {
        // See if perhaps we know which images to match to from a
        // previous map
        std::set<int> & matches = s->cid_to_cid_.find(cid)->second;
        for (auto it = matches.begin(); it != matches.end() ; it++) {
          indices.push_back(*it);
        }
      } else {
        // No way out, try matching brute force to subsequent images
        int subsequent = FLAGS_num_subsequent_images;
        if (FLAGS_match_all_rate > 0 && cid % FLAGS_match_all_rate == 0)
          subsequent = static_cast<int>(s->cid_to_keypoint_map_.size());
        int end = std::min(static_cast<int>(cid) + subsequent + 1,
                           static_cast<int>(s->cid_to_keypoint_map_.size()));
        for (int j = cid + 1; j < end; j++) {
          // Use subsequent images
          indices.push_back(j);
        }
      }
    }

    bool compute_rays_angle = false;
    double rays_angle;
    for (size_t j = 0; j < indices.size(); j++) {
      // Need the check below for loop closing to pass in unit tests
      if (s->cid_to_filename_[cid] != s->cid_to_filename_[indices[j]]) {
        thread_pool.AddTask(&sparse_mapping::BuildMapPerformMatching,
                            &match_map,
                            s->cid_to_keypoint_map_,
                            s->cid_to_descriptor_map_,
                            std::cref(s->camera_params_),
                            &relative_affines,
                            &match_mutex,
                            cid, indices[j],
                            compute_rays_angle, &rays_angle);
      }
    }
  }
  thread_pool.Join();

  LOG(INFO) << "Number of affines found:        " << relative_affines.size() << "\n";

  // Write the solution
  sparse_mapping::WriteAffineCSV(relative_affines, essential_file);

  WriteMatches(match_map, matches_file);

  // Initial cameras based on the affines (won't be used later,
  // just for visualization purposes).
  int num_images = s->cid_to_filename_.size();
  (s->cid_to_cam_t_global_).resize(num_images);
  (s->cid_to_cam_t_global_)[0].setIdentity();
  for (int cid = 1; cid < num_images; cid++) {
    std::pair<int, int> P(cid-1, cid);
    if (relative_affines.find(P) != relative_affines.end())
      (s->cid_to_cam_t_global_)[cid] = relative_affines[P]*(s->cid_to_cam_t_global_)[cid-1];
    else
      (s->cid_to_cam_t_global_)[cid] = (s->cid_to_cam_t_global_)[cid-1];  // no choice
  }
}

void BuildTracks(bool rm_invalid_xyz,
                 const std::string & matches_file,
                 sparse_mapping::SparseMap * s) {
  openMVG::matching::PairWiseMatches match_map;
  ReadMatches(matches_file, &match_map);

  // Build tracks using the interface tracksbuilder
  openMVG::tracks::TracksBuilder trackBuilder;
  trackBuilder.Build(match_map);  // Build:  Efficient fusion of correspondences
  trackBuilder.Filter();          // Filter: Remove tracks that have conflict
  // trackBuilder.ExportToStream(std::cout);
  openMVG::tracks::STLMAPTracks map_tracks;
  // Export tracks as a map (each entry is a sequence of imageId and featureIndex):
  //  {TrackIndex => {(imageIndex, featureIndex), ... ,(imageIndex, featureIndex)}
  trackBuilder.ExportToSTL(map_tracks);

  // TODO(oalexan1): Print how many pairwise matches were there before
  // and after filtering tracks.

  if (map_tracks.empty())
    LOG(FATAL) << "No tracks left after filtering. Perhaps images are too dis-similar?\n";

  size_t num_elems = map_tracks.size();
  // Populate back the filtered tracks.
  (s->pid_to_cid_fid_).clear();
  (s->pid_to_cid_fid_).resize(num_elems);
  size_t curr_id = 0;
  for (auto itr = map_tracks.begin(); itr != map_tracks.end(); itr++) {
    for (auto itr2 = (itr->second).begin(); itr2 != (itr->second).end(); itr2++) {
      (s->pid_to_cid_fid_)[curr_id][itr2->first] = itr2->second;
    }
    curr_id++;
  }

  // Triangulate. The results should be quite inaccurate, we'll redo this
  // later. This step is mostly for consistency.
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));


  // Wipe file that is no longer needed
  try {
    std::remove(matches_file.c_str());
  }catch(...) {}

  // PrintTrackStats(s->pid_to_cid_fid_, "track building");
}

// TODO(oalexan1): This very naive code can use serious performance
// improvements.  Each time we add a new camera we triangulate all
// points. We bundle-adjust the last several cameras, but while seeing
// (and keeping fixed) all the earlier cameras. It is sufficient to
// both triangulate and see during bundle adjustment only the several
// most similar cameras. Fixing these would need careful testing for
// both map quality and run-time before and after the fix.
void IncrementalBA(std::string const& essential_file,
                   sparse_mapping::SparseMap * s) {
  // Do incremental bundle adjustment.

  // Optimize only the last several cameras, their number varies
  // between min_num_cams and max_num_cams.

  // TODO(oalexan1): Need to research how many previous cameras we
  // need for loop closure.
  int min_num_cams = 4;
  int max_num_cams = 128;

  // Read in all the affine R|t combinations between cameras
  sparse_mapping::CIDPairAffineMap relative_affines;
  sparse_mapping::ReadAffineCSV(essential_file,
                                &relative_affines);

  int num_images = s->cid_to_filename_.size();

  // Track and camera info up to the current cid
  std::vector<std::map<int, int> > pid_to_cid_fid_local;
  std::vector<Eigen::Affine3d > cid_to_cam_t_local;
  std::vector<Eigen::Vector3d> pid_to_xyz_local;
  std::vector<std::map<int, int> > cid_fid_to_pid_local;

  bool rm_invalid_xyz = true;

  for (int cid = 1; cid < num_images; cid++) {
    // The array of cameras so far including this one
    cid_to_cam_t_local.resize(cid + 1);
    for (int c = 0; c < cid; c++)
      cid_to_cam_t_local[c] = s->cid_to_cam_t_global_[c];

    // Add a new camera. Obtain it based on relative affines. Here we assume
    // the current camera is similar to the previous one.
    std::pair<int, int> P(cid-1, cid);
    if (relative_affines.find(P) != relative_affines.end())
      cid_to_cam_t_local[cid] = relative_affines[P]*cid_to_cam_t_local[cid-1];
    else
      cid_to_cam_t_local[cid] = cid_to_cam_t_local[cid-1];  // no choice

    // Restrict tracks to images up to cid.
    pid_to_cid_fid_local.clear();
    for (size_t p = 0; p < s->pid_to_cid_fid_.size(); p++) {
      std::map<int, int> & long_track = s->pid_to_cid_fid_[p];
      std::map<int, int> track;
      for (std::map<int, int>::iterator it = long_track.begin();
           it != long_track.end() ; it++) {
        if (it->first <= cid)
          track[it->first] = it->second;
      }

      // This is absolutely essential, using tracks of length >= 3
      // only greatly increases the reliability.
      if ( (cid == 1 && track.size() > 1) || track.size() > 2 )
        pid_to_cid_fid_local.push_back(track);
    }

    // Perform triangulation of all points. Multiview triangulation is
    // used.
    pid_to_xyz_local.clear();
    std::vector<std::map<int, int> > cid_fid_to_pid_local;
    sparse_mapping::Triangulate(rm_invalid_xyz,
                                s->camera_params_.GetFocalLength(),
                                cid_to_cam_t_local,
                                s->cid_to_keypoint_map_,
                                &pid_to_cid_fid_local,
                                &pid_to_xyz_local,
                                &cid_fid_to_pid_local);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    options.max_num_iterations = 500;
    options.logging_type = ceres::SILENT;
    options.num_threads = FLAGS_num_threads;
    ceres::Solver::Summary summary;
    ceres::LossFunction* loss = new ceres::CauchyLoss(0.5);

    // If cid+1 is divisible by 2^k, do at least 2^k cameras, ending
    // with camera cid.  E.g., if current camera index is 23 = 3*8-1, do at
    // least 8 cameras, so cameras 16, ..., 23. This way, we will try
    // to occasionally do more than just several close cameras.
    int val = cid+1;
    int offset = 1;
    while (val % 2 == 0) {
      val /= 2;
      offset *= 2;
    }
    offset = std::min(offset, max_num_cams);

    int start = cid-offset+1;
    start = std::min(cid-min_num_cams+1, start);
    if (start < 0) start = 0;

    LOG(INFO) << "Optimizing cameras from " << start << " to " << cid << " (total: "
        << cid-start+1 << ")";

    sparse_mapping::BundleAdjust(pid_to_cid_fid_local, s->cid_to_keypoint_map_,
                                 s->camera_params_.GetFocalLength(),
                                 &cid_to_cam_t_local, &pid_to_xyz_local,
                                 s->user_pid_to_cid_fid_,
                                 s->user_cid_to_keypoint_map_,
                                 &(s->user_pid_to_xyz_),
                                 loss, options, &summary,
                                 start, cid);

    // Copy back
    for (int c = 0; c <= cid; c++)
      s->cid_to_cam_t_global_[c] = cid_to_cam_t_local[c];
  }

  // Triangulate all points
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              s->camera_params_.GetFocalLength(),
                              s->cid_to_cam_t_global_,
                              s->cid_to_keypoint_map_,
                              &(s->pid_to_cid_fid_),
                              &(s->pid_to_xyz_),
                              &(s->cid_fid_to_pid_));

  // Wipe file that is no longer needed
  try {
    std::remove(essential_file.c_str());
  }catch(...) {}
}

void BundleAdjust(bool fix_all_cameras, sparse_mapping::SparseMap * map,
                  std::set<int> const& fixed_cameras) {
  for (int i = 0; i < FLAGS_num_ba_passes; i++) {
    LOG(INFO) << "Beginning bundle adjustment, pass: " << i << ".\n";

    // perform bundle adjustment
    ceres::Solver::Options options;
    // options.linear_solver_type = ceres::SPARSE_SCHUR; // Need to be building SuiteSparse
    options.linear_solver_type = ceres::ITERATIVE_SCHUR;
    // What should the preconditioner be?
    options.num_threads = FLAGS_num_threads;
    options.max_num_iterations = FLAGS_max_num_iterations;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::LossFunction* loss = sparse_mapping::GetLossFunction(FLAGS_cost_function,
                                                                FLAGS_cost_function_threshold);
    sparse_mapping::BundleAdjustment(map, loss, options, &summary,
                                     FLAGS_first_ba_index, FLAGS_last_ba_index,
                                     fix_all_cameras, fixed_cameras);

    LOG(INFO) << summary.FullReport() << "\n";
    LOG(INFO) << "Starting average reprojection error: "
              << summary.initial_cost / map->GetNumObservations();
    LOG(INFO) << "Final average reprojection error:    "
              << summary.final_cost / map->GetNumObservations();
  }
}

void BundleAdjustment(sparse_mapping::SparseMap * s,
                      ceres::LossFunction* loss,
                      const ceres::Solver::Options & options,
                      ceres::Solver::Summary* summary,
                      int first, int last, bool fix_all_cameras,
                      std::set<int> const& fixed_cameras) {
  sparse_mapping::BundleAdjust(s->pid_to_cid_fid_, s->cid_to_keypoint_map_,
                               s->camera_params_.GetFocalLength(), &(s->cid_to_cam_t_global_),
                               &(s->pid_to_xyz_),
                               s->user_pid_to_cid_fid_, s->user_cid_to_keypoint_map_,
                               &(s->user_pid_to_xyz_),
                               loss, options, summary, first, last, fix_all_cameras,
                               fixed_cameras);

  // First do BA, and only afterwards remove outliers.
  if (!FLAGS_skip_filtering) {
    FilterPID(FLAGS_reproj_thresh,  s->camera_params_, s->cid_to_cam_t_global_,
              s->cid_to_keypoint_map_, &(s->pid_to_cid_fid_), &(s->pid_to_xyz_));
    s->InitializeCidFidToPid();
  }

  // PrintTrackStats(s->pid_to_cid_fid_, "bundle adjustment and filtering");
}

// Check if the two arrays share elements
bool haveSharedElements(std::vector<std::string> const& A, std::vector<std::string> const& B) {
  std::set<std::string> setA;
  for (size_t it = 0; it < A.size(); it++) setA.insert(A[it]);

  for (size_t it = 0; it < B.size(); it++)
    if (setA.find(B[it]) != setA.end())
      return true;

  return false;
}

// Load two maps, merge the second one onto the first one, and save the result.
void AppendMapFile(std::string const& mapOut, std::string const& mapIn,
                   int num_image_overlaps_at_endpoints,
                   double outlier_factor, bool bundle_adjust,
                   bool fix_first_map) {
  if (!bundle_adjust && fix_first_map) LOG(FATAL) << "Cannot fix first map if no bundle adjustment happens.";

  LOG(INFO) << "Appending " << mapIn << " to " << mapOut << std::endl;

  sparse_mapping::SparseMap A(mapOut);
  sparse_mapping::SparseMap B(mapIn);

  // Sanity check before we do a lot of work
  if (fix_first_map && haveSharedElements(A.cid_to_filename_, B.cid_to_filename_))
    LOG(FATAL) << "Cannot fix the first map if it shares cameras with the second map.";

  // C starts as A, as SparseMap lacks an empty constructor
  sparse_mapping::SparseMap C(mapOut);

  // Merge
  sparse_mapping::MergeMaps(&A, &B,
                            num_image_overlaps_at_endpoints,
                            outlier_factor,
                            mapOut,
                            &C);

  // Bundle-adjust the merged map
  if (bundle_adjust) {
    bool fix_all_cameras = false;

    std::set<int> fixed_cameras;

    // Find in the list of images of the merged map the ones from the first map
    if (fix_first_map) {
      std::set<std::string> setA;
      for (size_t it = 0; it < A.cid_to_filename_.size(); it++) setA.insert(A.cid_to_filename_[it]);

      for (size_t it = 0; it < C.cid_to_filename_.size(); it++) {
        if (setA.find(C.cid_to_filename_[it]) != setA.end()) fixed_cameras.insert(it);
      }
    }

    sparse_mapping::BundleAdjust(fix_all_cameras, &C, fixed_cameras);
  }

  C.Save(mapOut);
}

// This fitting functor attempts to find a rotation + translation + scale transformation
// between two vectors of points.
struct TranslationRotationScaleFittingFunctor {
  typedef Eigen::Affine3d result_type;

  /// A transformation requires 3 inputs and 3 outputs to make a fit.
  size_t min_elements_needed_for_fit() const { return 3; }

  result_type operator() (std::vector<Eigen::Vector3d> const& in_vec,
                          std::vector<Eigen::Vector3d> const& out_vec) const {
    // check consistency
    if (in_vec.size() != out_vec.size())
      LOG(FATAL) << "There must be as many inputs as outputs to be "
                 << "able to compute a transform between them.\n";
    if (in_vec.size() < min_elements_needed_for_fit())
      LOG(FATAL) << "Cannot compute a transformation. Insufficient data.\n";

    Eigen::Matrix3Xd in_mat  = Eigen::MatrixXd(3, in_vec.size());
    Eigen::Matrix3Xd out_mat = Eigen::MatrixXd(3, in_vec.size());
    for (size_t it = 0; it < in_vec.size(); it++) {
      in_mat.col(it)  = in_vec[it];
      out_mat.col(it) = out_vec[it];
    }
    result_type out_trans;
    Find3DAffineTransform(in_mat, out_mat, &out_trans);
    return out_trans;
  }
};

// How well does the given transform do to map p1 to p2.
struct TransformError {
  double operator() (Eigen::Affine3d const& T, Eigen::Vector3d const& p1,
                     Eigen::Vector3d const& p2) const {
    return (T*p1 - p2).norm();
  }
};

// Given a set of points in 3D, heuristically estimate what it means
// for two points to be "not far" from each other. The logic is to
// find a bounding box of an inner cluster and multiply that by 0.2.
double estimateCloseDistance(std::vector<Eigen::Vector3d> const& vec) {
  Eigen::Vector3d range;
  int num_pts = vec.size();
  if (num_pts <= 0)
    LOG(FATAL) << "Empty set of points.\n";  // to avoid a segfault

  std::vector<double> vals(num_pts);
  for (int it = 0; it < range.size(); it++) {  // iterate in each coordinate
    // Sort all values in given coordinate
    for (int p = 0; p < num_pts; p++)
      vals[p] = vec[p][it];
    std::sort(vals.begin(), vals.end());

    // Find some percentiles
    int min_p = round(num_pts*0.25);
    int max_p = round(num_pts*0.75);
    if (min_p >= num_pts) min_p = num_pts - 1;
    if (max_p >= num_pts) max_p = num_pts - 1;
    double min_val = vals[min_p], max_val = vals[max_p];
    range[it] = 0.2*(max_val - min_val);
  }

  // Find the average of all ranges
  double range_val = 0.0;
  for (int it = 0; it < range.size(); it++)
    range_val += range[it];
  range_val /= range.size();

  return range_val;
}

// Given a transform from cid2cid from some cid values to some others,
// apply the same transform to the tracks. This may make the tracks
// shorter if cid2cid maps different inputs to the same output.
// New tracks of length 1 can be excluded if desired.
void TransformTracks(std::map<int, int> const& cid2cid,
                     bool rm_tracks_of_len_one,
                     std::vector<std::map<int, int> > * pid_to_cid_fid) {
  std::vector<std::map<int, int> > pid_to_cid_fid2;
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    auto & cid_fid = (*pid_to_cid_fid)[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      int fid = it->second;
      auto cid_it = cid2cid.find(cid);
      if (cid_it == cid2cid.end()) continue;
      int out_cid = cid_it->second;
      cid_fid2[out_cid] = fid;
    }

    bool will_skip = rm_tracks_of_len_one && (cid_fid2.size() <= 1);
    if (!will_skip)
      pid_to_cid_fid2.push_back(cid_fid2);
  }
  *pid_to_cid_fid = pid_to_cid_fid2;
}

// As result of matching some images in A to some images in B, we must
// now merge some tracks in A with some tracks in B, as those tracks
// correspond physically to the same point in space. A track in
// C.pid_to_cid_fid_ tells us which track in A.pid_to_cid_fid_ is tied
// with which track in B.pid_to_cid_fid_. If it turns out one track in
// A should be merged with multiple tracks in B or vice-versa, select
// just one candidate from each map, based on who got most votes. Note
// that here it is easier to work with A.cid_fid_to_pid_ rather than
// A.pid_to_cid_fid_.
void FindPidCorrespondences(std::vector<std::map<int, int> > const& A_cid_fid_to_pid,
                            std::vector<std::map<int, int> > const& B_cid_fid_to_pid,
                            std::vector<std::map<int, int> > const& C_pid_to_cid_fid,
                            int num_acid,  // How many images are in A
                            std::map<int, int> * A2B, std::map<int, int> * B2A) {
  A2B->clear();
  B2A->clear();

  std::map<int, std::map<int, int> > VoteMap;
  for (int pid = 0; pid < static_cast<int>(C_pid_to_cid_fid.size()); pid++) {
    // This track has some cid indices from A (those < num_acid)
    // and some from B (those >= num_acid). Ignore all other combinations.
    auto const& cid_fid_c = C_pid_to_cid_fid[pid];  // alias
    for (auto it_a = cid_fid_c.begin(); it_a != cid_fid_c.end(); it_a++) {
      for (auto it_b = it_a; it_b != cid_fid_c.end(); it_b++) {
        int cid_a = it_a->first, fid_a = it_a->second;
        int cid_b = it_b->first, fid_b = it_b->second;
        if (cid_a >= num_acid) continue;
        if (cid_b <  num_acid) continue;

        // Subtract num_acid from cid_b so it becomes a cid in B.
        cid_b -= num_acid;

        auto it_fida = A_cid_fid_to_pid[cid_a].find(fid_a);
        if (it_fida == A_cid_fid_to_pid[cid_a].end()) continue;

        auto it_fidb = B_cid_fid_to_pid[cid_b].find(fid_b);
        if (it_fidb == B_cid_fid_to_pid[cid_b].end()) continue;

        int pid_a = it_fida->second;
        int pid_b = it_fidb->second;

        VoteMap[pid_a][pid_b]++;
      }
    }
  }

  // For each pid in A, keep the pid in B with most votes
  std::map<int, std::map<int, int> > B2A_Version0;  // still not fully one-to-one
  for (auto it_a = VoteMap.begin(); it_a != VoteMap.end(); it_a++) {
    auto & M = it_a->second;  // all pid_b corresp to given pid_a with their votes
    int pid_a = it_a->first;
    int best_pid_b = -1;
    int max_vote = -1;
    for (auto it_b = M.begin(); it_b != M.end(); it_b++) {
      int pid_b = it_b->first;
      int vote = it_b->second;
      if (vote > max_vote) {
        best_pid_b = pid_b;
        max_vote = vote;
      }
    }
    B2A_Version0[best_pid_b][pid_a] = max_vote;
  }

  // And vice-versa
  for (auto it_b = B2A_Version0.begin(); it_b != B2A_Version0.end(); it_b++) {
    int pid_b = it_b->first;
    auto & M = it_b->second;
    int best_pid_a = -1;
    int max_vote = -1;
    for (auto it_a = M.begin(); it_a != M.end(); it_a++) {
      int pid_a = it_a->first;
      int vote = it_a->second;
      if (vote > max_vote) {
        best_pid_a = pid_a;
        max_vote = vote;
      }
    }

    (*A2B)[best_pid_a] = pid_b;  // track from A and track from B
    (*B2A)[pid_b] = best_pid_a;  // track from B and track from A
  }
}


// Determine which tracks from map A to merge with
// which tracks from map B. For that, find which images in map A have
// matches in map B.  We will look only at images close to the
// endpoints of both maps, per num_image_overlaps_at_endpoints.
void findMatchingTracks(sparse_mapping::SparseMap * A_in,
                        sparse_mapping::SparseMap * B_in,
                        sparse_mapping::SparseMap * C_out,
                        std::string const& output_map,
                        int num_image_overlaps_at_endpoints,
                        std::map<int, int> & A2B,  // output
                        std::map<int, int> & B2A   // output
                        ) {
  // Wipe the outputs
  A2B.clear();
  B2A.clear();

  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & A = *A_in;
  sparse_mapping::SparseMap & B = *B_in;
  sparse_mapping::SparseMap & C = *C_out;

  int num_acid = A.cid_to_filename_.size();
  int num_bcid = B.cid_to_filename_.size();

  std::set<int> A_search, B_search;  // use sets to avoid duplicates
  int num = num_image_overlaps_at_endpoints;

  // Images in A to search for matches in B
  for (int cid = 0; cid < num; cid++)
    if (cid < num_acid) A_search.insert(cid);
  for (int cid = num_acid-num; cid < num_acid; cid++)
    if (cid >= 0) A_search.insert(cid);

  // Images in B to search for matches in A. Add num_acid since we will
  // match A and B inside of C.
  for (int cid = 0; cid < num; cid++)
    if (cid < num_bcid) B_search.insert(num_acid + cid);
  for (int cid = num_bcid-num; cid < num_bcid; cid++)
    if (cid >= 0) B_search.insert(num_acid + cid);

  // Combine these into cid_to_cid_ and run the matching process.
  C.cid_to_cid_.clear();
  for (auto it1 = A_search.begin(); it1 != A_search.end() ; it1++) {
    for (auto it2 = B_search.begin(); it2 != B_search.end(); it2++) {
      if (*it1 == *it2)
        LOG(FATAL) << "Book-keeping failure in map merging.";
      C.cid_to_cid_[*it1].insert(*it2);
    }
  }

  // Match features. Must set num_subsequent_images to not try to
  // match images in same map, that was done when each map was built.
  google::SetCommandLineOption("num_subsequent_images", "0");
  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(output_map),
                                sparse_mapping::MatchesFile(output_map), &C);

  // Assemble the matches into tracks, this will populate C.pid_to_cid_fid_.
  bool rm_invalid_xyz = false;  // nothing is valid yet
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(output_map), &C);

  // This is not needed any longer
  C.cid_to_cid_.clear();

  // Wipe file that is no longer needed
  try {
    std::remove(sparse_mapping::EssentialFile(output_map).c_str());
  }catch(...) {}

  // For each track in A, find the map to its corresponding track in B
  // using the information in C.pid_to_cid_fid_.
  FindPidCorrespondences(A.cid_fid_to_pid_, B.cid_fid_to_pid_,  C.pid_to_cid_fid_,
                         num_acid, &A2B, &B2A);
}

// If two maps share images, can match tracks between the maps
// just based on that, which is fast.
void findTracksForSharedImages(sparse_mapping::SparseMap * A_in,
                               sparse_mapping::SparseMap * B_in,
                               std::map<int, int> & A2B,  // output
                               std::map<int, int> & B2A   // output
                               ) {
  // Wipe the outputs
  A2B.clear();
  B2A.clear();

  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & A = *A_in;
  sparse_mapping::SparseMap & B = *B_in;

  size_t num_acid = A.cid_to_filename_.size();
  size_t num_bcid = B.cid_to_filename_.size();

  // Map from file name to cid
  std::map<std::string, int> A_file_to_cid, B_file_to_cid;
  for (size_t cid = 0; cid < num_acid; cid++)
    A_file_to_cid[A.cid_to_filename_[cid]] = cid;
  for (size_t cid = 0; cid < num_bcid; cid++)
    B_file_to_cid[B.cid_to_filename_[cid]] = cid;

  // Iterate through A's cid_fid_to_pid_ and find matches in B.
  int num_shared_cid = 0;
  for (size_t cid_a = 0; cid_a < A.cid_fid_to_pid_.size(); cid_a++) {
    std::string filename = A.cid_to_filename_[cid_a];
    auto it = B_file_to_cid.find(filename);
    if (it == B_file_to_cid.end())
      continue;

    num_shared_cid++;

    // The corresponding camera id in the second map
    size_t cid_b = it->second;

    if (A.cid_to_keypoint_map_[cid_a] != B.cid_to_keypoint_map_[cid_b])
      LOG(FATAL) << "The input maps don't have the same features. They need to be rebuilt.";

    auto a_fid_to_pid = A.cid_fid_to_pid_[cid_a];
    auto b_fid_to_pid = B.cid_fid_to_pid_[cid_b];

    // Find tracks corresponding to same cid_fid
    for (auto it_a = a_fid_to_pid.begin(); it_a != a_fid_to_pid.end(); it_a++) {
      int pid_a = it_a->second;
      int fid = it_a->first;  // shared fid
      auto it_b = b_fid_to_pid.find(fid);
      if (it_b == b_fid_to_pid.end()) {
        // This fid is not in second image. This is fine. A feature in a current image
        // may match to features in one image but not in another.
        continue;
      }

      int pid_b = it_b->second;

      A2B[pid_a] = pid_b;
    }
  }

  // Now create B2A
  for (auto it = A2B.begin(); it != A2B.end(); it++) {
    B2A[it->second] = it->first;
  }

  // Just in case, recreate A2B, to avoid issues when the original
  // A2B mapped multiple A pids to same B pid.
  A2B.clear();
  for (auto it = B2A.begin(); it != B2A.end(); it++) {
    A2B[it->second] = it->first;
  }


  LOG(INFO) << "Number of shared images in the two maps: " << num_shared_cid << std::endl;
  LOG(INFO) << "Number of shared tracks: " << A2B.size() << std::endl;

  // Sanity check
  if (num_shared_cid <= 0 || A2B.size() <= 5)
    LOG(FATAL) << "Not enough shared images or features among the two maps. Run without the --fast option.";
}

// Given a sparse map in C_out, and a map cid2cid from camera (image)
// indices to new indices, convert the map from being relative to old
// indices to relative to the new indices. If cid2cid maps two input
// indices to the same output index, reconcile the camera positions
// for those indices.
void TransformMap(std::map<int, int> & cid2cid,
                  sparse_mapping::SparseMap * C_out) {
  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & C = *C_out;

  // The total number of output cameras
  int num_out_cams = 0;
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    num_out_cams = std::max(num_out_cams, cid2cid[cid]);
  }
  num_out_cams++;  // move past the last

  // Each blob will be original cids that end up being a single cid
  // after identifying repeat images.
  std::vector< std::set<int> > blobs(num_out_cams);
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    blobs[cid2cid[cid]].insert(cid);
  }

  // To merge cid_to_cam_t_global_, find the average rotation and translation
  // from the two maps.
  std::vector<Eigen::Affine3d > cid_to_cam_t_global2(num_out_cams);
  for (size_t c = 0; c < blobs.size(); c++) {
    if (blobs[c].size() == 1) {
      cid_to_cam_t_global2[c] = C.cid_to_cam_t_global_[*blobs[c].begin()];
    } else {
      int num = blobs[c].size();

      // All cams to merge get equal weight
      std::vector<double> W(num, 1.0/num);

      // TODO(oalexan1): Something more clever could be done. If an
      // image in one map has few tracks going through it, or those
      // tracks are short, this instance could be given less weight.

      std::vector< Eigen::Quaternion<double> >Q(num);
      cid_to_cam_t_global2[c].translation() << 0.0, 0.0, 0.0;
      int pos = -1;
      for (auto it = blobs[c].begin(); it != blobs[c].end() ; it++) {
        pos++;
        int cid = *it;
        Q[pos] = Eigen::Quaternion<double> (C.cid_to_cam_t_global_[cid].linear());

        cid_to_cam_t_global2[c].translation()
          += W[pos]*C.cid_to_cam_t_global_[cid].translation();
      }
      Eigen::Quaternion<double> S = sparse_mapping::slerp_n(W, Q);
      cid_to_cam_t_global2[c].linear() = S.toRotationMatrix();
    }
  }

  // We really count during merging that if two maps have an image in
  // common, the same keypoint map is computed for that image in both
  // maps. Otherwise the book-keeping of cid_fid becomes a disaster.
  for (size_t c = 0; c < blobs.size(); c++) {
    int num = blobs[c].size();
    if (num <= 1) continue;
    int cid0 = *blobs[c].begin();
    for (auto it = blobs[c].begin(); it != blobs[c].end() ; it++) {
      int cid = *it;
      if (C.cid_to_keypoint_map_[cid0] != C.cid_to_keypoint_map_[cid]) {
        LOG(FATAL) << "The two input maps do not have the same features for same images. "
                   << "Cannot merge them. Consider rebuilding them.";
      }
    }
  }

  // Further removal of repetitions.
  std::vector<std::string> cid_to_filename2(num_out_cams);
  std::vector<Eigen::Matrix2Xd > cid_to_keypoint_map2(num_out_cams);
  std::vector<cv::Mat> cid_to_descriptor_map2(num_out_cams);
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    int cid2 = cid2cid[cid];
    cid_to_filename2[cid2]             = C.cid_to_filename_[cid];
    cid_to_keypoint_map2[cid2]         = C.cid_to_keypoint_map_[cid];
    cid_to_descriptor_map2[cid2]       = C.cid_to_descriptor_map_[cid];
  }

  // Modify the tracks after identifying identical images
  // We would rather keep tracks of length one (which we will filter out
  // some time later) than ruin the bookkeeping.
  bool rm_tracks_of_len_one = false;
  TransformTracks(cid2cid, rm_tracks_of_len_one, &C.pid_to_cid_fid_);

  if (C.pid_to_xyz_.size() != C.pid_to_cid_fid_.size()) {
    LOG(FATAL) << "Book-keeping failure in merging maps, "
               << "pid_to_xyz_ and pid_to_cid_fid_ must have the same size.";
  }

  // The new lists for the unique images
  C.cid_to_filename_       = cid_to_filename2;
  C.cid_to_keypoint_map_   = cid_to_keypoint_map2;
  C.cid_to_cam_t_global_   = cid_to_cam_t_global2;
  C.cid_to_descriptor_map_ = cid_to_descriptor_map2;

  // Note: after this step, it is possible some tracks are now
  // duplicate.  We don't bother removing them, it would be a pain,
  // since sometimes two tracks may differ in one or more values and
  // those are tricky to reconcile.

  // Recreate cid_fid_to_pid_ from pid_to_cid_fid_. This must happen
  // after the merging is complete but before using the new map.
  C.InitializeCidFidToPid();

  // C.Save(output_map + ".reduced.map");
}

// Merge two maps. See merge_maps.cc. The merged map needs to be
// bundle-adjusted. We need to have write-access to A and B to be able
// to initialize some auxiliary structures in these maps.
// TODO(oalexan1): Modularize this code (some was done already).
void MergeMaps(sparse_mapping::SparseMap * A_in,
               sparse_mapping::SparseMap * B_in,
               int num_image_overlaps_at_endpoints,
               double outlier_factor,
               std::string const& output_map,
               sparse_mapping::SparseMap * C_out) {
  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & A = *A_in;
  sparse_mapping::SparseMap & B = *B_in;
  sparse_mapping::SparseMap & C = *C_out;

  // Basic sanity checks (not exhaustive)
  if ( !(A.GetCameraParameters() == B.GetCameraParameters()) )
    LOG(FATAL) << "The input maps don't have the same camera parameters.";
  if ( !(A.detector_ == B.detector_) )
    LOG(FATAL) << "The input maps don't have the same detector and/or descriptor.";

  sparse_mapping::HistogramEqualizationCheck(A.GetHistogramEqualization(),
                                             B.GetHistogramEqualization());

  // Wipe things that we won't merge (or not yet)
  C.ClearImageDatabase();
  // TODO(rsoussan): make function to do this
  C.pid_to_cid_fid_.clear();
  C.pid_to_xyz_.clear();
  C.cid_fid_to_pid_.clear();
  C.cid_to_cid_.clear();
  C.user_cid_to_keypoint_map_.clear();
  C.user_pid_to_cid_fid_.clear();
  C.user_pid_to_xyz_.clear();

  // Merge things that make sense to merge and are easy to do
  int num_acid = A.cid_to_filename_.size();
  int num_bcid = B.cid_to_filename_.size();
  int num_ccid = num_acid + num_bcid;
  C.cid_to_filename_      .resize(num_ccid);
  C.cid_to_keypoint_map_  .resize(num_ccid);
  C.cid_to_cam_t_global_  .resize(num_ccid);
  C.cid_to_descriptor_map_.resize(num_ccid);
  for (int cid = 0; cid < num_bcid; cid++) {
    // C.cid_to_filename_ already contains A.cid_to_filename_, etc.
    int c = num_acid + cid;
    C.cid_to_filename_[c]       = B.cid_to_filename_[cid];
    C.cid_to_keypoint_map_[c]   = B.cid_to_keypoint_map_[cid];
    C.cid_to_descriptor_map_[c] = B.cid_to_descriptor_map_[cid];
    // We will have to deal with cid_to_cam_t_global_ later
  }

  // Create cid_fid_to_pid_ for both maps, to be able to go from cid_fid to pid.
  A.InitializeCidFidToPid();
  B.InitializeCidFidToPid();

  std::map<int, int> A2B, B2A;
  if (!FLAGS_fast_merge)
    findMatchingTracks(&A, &B, &C, output_map,
                       num_image_overlaps_at_endpoints,
                       A2B, B2A);  // outputs
  else
    findTracksForSharedImages(&A, &B,
                              A2B, B2A);  // outputs

  // Put the xyz points corresponding to the tracks to merge in vectors.
  std::vector<Eigen::Vector3d> A_vec(A2B.size()), B_vec(A2B.size());
  int point_count = 0;
  for (auto it = A2B.begin(); it != A2B.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    A_vec[point_count] = A.pid_to_xyz_[pid_a];
    B_vec[point_count] = B.pid_to_xyz_[pid_b];
    point_count++;
  }
  double inlier_threshold = estimateCloseDistance(A_vec);

  // Estimate the transform from B_vec to A_vec using RANSAC.
  // A lot of outliers are possible.
  int  num_iterations = 1000;
  int  min_num_output_inliers = A_vec.size()/2;
  bool reduce_min_num_output_inliers_if_no_fit = true;  // If too many outliers
  bool increase_threshold_if_no_fit = true;  // Coz our threshold was done by a heuristic
  RandomSampleConsensus < TranslationRotationScaleFittingFunctor, TransformError>
    ransac(TranslationRotationScaleFittingFunctor(), TransformError(), num_iterations,
           inlier_threshold, min_num_output_inliers,
           reduce_min_num_output_inliers_if_no_fit, increase_threshold_if_no_fit);
  Eigen::Affine3d B2A_trans = ransac(B_vec, A_vec);
  std::vector<size_t> inlier_indices = ransac.inlier_indices(B2A_trans, B_vec, A_vec);
  std::set<int> inlier_set;
  for (size_t it = 0; it < inlier_indices.size(); it++) {
    inlier_set.insert(inlier_indices[it]);
  }

  // Remove from A2B and B2A the outliers
  std::map<int, int> A2B_orig = A2B;
  point_count = 0;
  for (auto it = A2B_orig.begin(); it != A2B_orig.end(); it++) {
    int pid_a = it->first;
    int pid_b = it->second;
    if (inlier_set.find(point_count) == inlier_set.end()) {
      auto iter_a = A2B.find(pid_a);
      if (iter_a == A2B.end())
        LOG(FATAL) << "Bookkeeping error 1 in merging maps.";
      A2B.erase(iter_a);

      auto iter_b = B2A.find(pid_b);
      if (iter_b == B2A.end())
        LOG(FATAL) << "Bookkeeping error 2 in merging maps.";
      B2A.erase(iter_b);
    }
    point_count++;
  }

  // LOG(INFO) does not do well with Eiegn.
  std::cout << "Affine transform from second map to first map:\n";
  std::cout << "Matrix:\n"      << B2A_trans.linear()       << "\n";
  std::cout << "Translation:\n" << B2A_trans.translation()  << "\n";

  // Bring the B map into the coordinate system of the A map
  B.ApplyTransform(B2A_trans);

  // We will use this to add new tracks taking advantage
  // of all the matching between the two image sets.
  std::vector<std::map<int, int> > merged_pid_to_cid_fid;
  if (!FLAGS_skip_adding_new_matches_on_merging)
    merged_pid_to_cid_fid = C.pid_to_cid_fid_;  // save it before wiping it below

  // Start creating the merged tracks
  C.pid_to_cid_fid_.clear();
  C.pid_to_xyz_ = A.pid_to_xyz_;  // Will later modify it by averaging/appending from B

  int num_tracks_in_A_only = 0, num_tracks_in_A_and_B = 0, num_tracks_in_B_only = 0;

  // Add to C.pid_to_cid_fid_ the tracks in A.pid_to_cid_fid_, and
  // merge the corresponding track from B.pid_to_cid_fid_ if available.
  for (size_t pid_a = 0; pid_a < A.pid_to_cid_fid_.size(); pid_a++) {
    auto cid_fid_c = A.pid_to_cid_fid_[pid_a];  // make a copy

    if (A2B.find(pid_a) != A2B.end()) {  // Can merge from B
      int pid_b = A2B[pid_a];

      if (pid_b >= static_cast<int>(B.pid_to_cid_fid_.size()) )
        LOG(FATAL) << "Book-keeping error in track merging.";

      // Append the B track to the C track. Add num_acid as we want
      // a track in C.
      auto & cid_fid_b = B.pid_to_cid_fid_[pid_b];  // alias
      for (auto it = cid_fid_b.begin(); it != cid_fid_b.end(); it++)
        cid_fid_c[it->first + num_acid] = it->second;

      // Merged map xyz will be the average of xyz's from both maps
      C.pid_to_xyz_[pid_a] = (A.pid_to_xyz_[pid_a] + B.pid_to_xyz_[pid_b])/2.0;

      num_tracks_in_A_and_B++;
    } else {
      num_tracks_in_A_only++;
    }

    // Add the current track, whether it is wholly in A or also paritially in B
    C.pid_to_cid_fid_.push_back(cid_fid_c);
  }

  // Now add the tracks that are purely in B.
  for (size_t pid_b = 0; pid_b < B.pid_to_cid_fid_.size(); pid_b++) {
    if (B2A.find(pid_b) != B2A.end()) {
      continue;  // Track partially in A, done already
    }

    num_tracks_in_B_only++;

    // Add this track, and add num_acid to be in C's indexing scheme
    std::map<int, int> cid_fid_c;
    auto & cid_fid_b = B.pid_to_cid_fid_[pid_b];  // alias
    for (auto it = cid_fid_b.begin(); it != cid_fid_b.end(); it++)
      cid_fid_c[it->first + num_acid] = it->second;

    C.pid_to_cid_fid_.push_back(cid_fid_c);
    C.pid_to_xyz_.push_back(B.pid_to_xyz_[pid_b]);
  }

  // Append the cameras from B. By now A and B are in same coordinate system.
  C.cid_to_cam_t_global_ = A.cid_to_cam_t_global_;
  for (int cid = 0; cid < num_bcid; cid++)
    C.cid_to_cam_t_global_.push_back(B.cid_to_cam_t_global_[cid]);

  // C.Save(output_map + ".merged.map");

  LOG(INFO) << "Number of tracks merged from both maps:    " << num_tracks_in_A_and_B;
  LOG(INFO) << "Number of tracks from the first map only:  " << num_tracks_in_A_only;
  LOG(INFO) << "Number of tracks from the second map only: " << num_tracks_in_B_only;

  // If a few images show up in both and in B, so far they show up in C twice,
  // with different cid value. Fix that.
  // Also keep the images sorted.
  std::vector<std::string> sorted = C.cid_to_filename_;
  std::sort(sorted.begin(), sorted.end());
  int num_out_cams = 0;
  std::map<std::string, int> image2cid;  // the new index of each image after rm repetitions
  for (size_t cid = 0; cid < sorted.size(); cid++) {
    std::string img = sorted[cid];
    if (image2cid.find(img) == image2cid.end()) {
      image2cid[img] = num_out_cams;
      num_out_cams++;
    }
  }

  // The index of the cid after removing the repetitions
  std::map<int, int> cid2cid;
  for (size_t cid = 0; cid < C.cid_to_filename_.size(); cid++) {
    cid2cid[cid] = image2cid[ C.cid_to_filename_[cid] ];
  }

  // Remove repetitions.
  TransformMap(cid2cid, &C);

  if (!FLAGS_skip_adding_new_matches_on_merging) {
    // Modify merged_pid_to_cid_fid as well after identifying identical images
    bool rm_tracks_of_len_one = true;
    TransformTracks(cid2cid, rm_tracks_of_len_one, &merged_pid_to_cid_fid);
  }

  // Add the new tracks that were identified during matching of images of A to B.
  if (!FLAGS_skip_adding_new_matches_on_merging) {
    // Form merged_cid_fid_to_pid
    int num_cid = C.cid_to_filename_.size();
    std::vector<std::map<int, int> > merged_cid_fid_to_pid;
    InitializeCidFidToPid(num_cid, merged_pid_to_cid_fid, &merged_cid_fid_to_pid);

    LOG(INFO) << "Number of tracks found as result of matching images between the maps: "
              << merged_pid_to_cid_fid.size();

    std::vector<std::map<int, int> > new_pid_to_cid_fid;
    std::set<int> new_pid_set;
    // See which tracks obtained during merging are new
    for (size_t cid = 0; cid < merged_cid_fid_to_pid.size(); cid++) {
      for (auto it = merged_cid_fid_to_pid[cid].begin();
           it != merged_cid_fid_to_pid[cid].end(); it++) {
        if (cid >= C.cid_fid_to_pid_.size()) continue;  // out of range
        int fid = it->first;
        if (C.cid_fid_to_pid_[cid].find(fid) != C.cid_fid_to_pid_[cid].end())
          continue;  // not new
        int new_pid = it->second;
        if (new_pid_set.find(new_pid) != new_pid_set.end()) continue;  // inserted already

        // Add this new track
        new_pid_to_cid_fid.push_back(merged_pid_to_cid_fid[new_pid]);
        new_pid_set.insert(new_pid);  // mark it as inserted
      }
    }

    // Triangulate to find the xyz coordinates of the new tracks
    std::vector<Eigen::Vector3d> new_pid_to_xyz;
    std::vector<std::map<int, int> > new_cid_fid_to_pid;
    bool rm_invalid_xyz = true;  // don't remove anything, as cameras are pretty unreliable now
    sparse_mapping::Triangulate(rm_invalid_xyz,
                                C.camera_params_.GetFocalLength(),
                                C.cid_to_cam_t_global_,
                                C.cid_to_keypoint_map_,
                                &new_pid_to_cid_fid,
                                &new_pid_to_xyz,
                                &new_cid_fid_to_pid);

    LOG(INFO) << "Of those, number of tracks that are new and will be added to the merged map: "
              << new_pid_to_cid_fid.size();

    // Append the new tracks to the merged map
    for (size_t pid = 0; pid < new_pid_to_cid_fid.size(); pid++) {
      C.pid_to_cid_fid_.push_back(new_pid_to_cid_fid[pid]);
      C.pid_to_xyz_.push_back(new_pid_to_xyz[pid]);
    }

    // Recreate cid_fid_to_pid_ from pid_to_cid_fid_.
    C.InitializeCidFidToPid();
  }

  LOG(INFO) << "Total number of tracks in the merged map: " << C.pid_to_xyz_.size();

  return;
}

// Take a map. Form a map with only a subset of the images.
// Bundle adjustment will happen later.
void ExtractSubmap(std::vector<std::string> * keep_ptr,
                   sparse_mapping::SparseMap * map_ptr) {
  // Create aliases to not use pointers all the time.
  sparse_mapping::SparseMap & map = *map_ptr;
  std::vector<std::string> & keep = *keep_ptr;

  // Wipe things that we won't merge (or not yet)
  map.ClearImageDatabase();
  map.pid_to_xyz_.clear();
  map.cid_fid_to_pid_.clear();
  map.cid_to_cid_.clear();
  map.user_cid_to_keypoint_map_.clear();
  map.user_pid_to_cid_fid_.clear();
  map.user_pid_to_xyz_.clear();

  // Sanity check. The images to keep must exist in the original map.
  std::map<std::string, int> image2cid;
  for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++)
    image2cid[map.cid_to_filename_[cid]] = cid;
  for (size_t cid = 0; cid < keep.size(); cid++) {
    if (image2cid.find(keep[cid]) == image2cid.end())
      LOG(WARNING) << "Could not find in the input map the image: " << keep[cid];
  }

  // To extract the submap-in place, it is simpler to reorder the images
  // to extract to be in the same order as in the map
  {
    std::set<std::string> keep_set;
    for (size_t cid = 0; cid < keep.size(); cid++)
      keep_set.insert(keep[cid]);
    std::vector<std::string> keep2;
    for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++) {
      if (keep_set.find(map.cid_to_filename_[cid]) != keep_set.end()) {
        keep2.push_back(map.cid_to_filename_[cid]);
      }
    }
    keep = keep2;
  }

  // Map each image we keep to its index
  std::map<std::string, int> keep2cid;
  for (size_t cid = 0; cid < keep.size(); cid++)
    keep2cid[keep[cid]] = cid;

  // The map from the old cid to the new cid
  std::map<int, int> cid2cid;
  for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    auto it = keep2cid.find(map.cid_to_filename_[cid]);
    if (it == keep2cid.end()) continue;  // current image is not in the final submap
    cid2cid[cid] = it->second;
  }

  // Sanity checks. All the kept images must be represented in cid2cid,
  // and the values in cid2cid must be consecutive.
  if (cid2cid.size() != keep.size() || cid2cid.empty())
    LOG(FATAL) << "Cannot extract a submap. Check your inputs.";
  for (auto it = cid2cid.begin(); it != cid2cid.end(); it++) {
    auto it2 = it; it2++;
    if (it2 == cid2cid.end()) continue;
    if (it->second + 1 != it2->second || cid2cid.begin()->second != 0 )
      LOG(FATAL) << "Cannot extract a submap. Check if the images "
                 << "you want to keep are in the same order as in the original map.";
  }

  // Over-write the data in-place. Should be safe with the checks done above.
  int num_cid = keep.size();
  for (size_t cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    if (cid2cid.find(cid) == cid2cid.end()) continue;
    size_t new_cid = cid2cid[cid];
    map.cid_to_filename_[new_cid]             = map.cid_to_filename_[cid];
    map.cid_to_keypoint_map_[new_cid]         = map.cid_to_keypoint_map_[cid];
    map.cid_to_cam_t_global_[new_cid]         = map.cid_to_cam_t_global_[cid];
    map.cid_to_descriptor_map_[new_cid]       = map.cid_to_descriptor_map_[cid];
  }
  map.cid_to_filename_             .resize(num_cid);
  map.cid_to_keypoint_map_         .resize(num_cid);
  map.cid_to_cam_t_global_         .resize(num_cid);
  map.cid_to_descriptor_map_       .resize(num_cid);

  // Create new pid_to_cid_fid_.
  std::vector<std::map<int, int> > pid_to_cid_fid;
  std::vector<Eigen::Vector3d> pid_to_xyz;
  for (int pid = 0; pid < static_cast<int>(map.pid_to_cid_fid_.size()); pid++) {
    auto const& cid_fid = map.pid_to_cid_fid_[pid];  // alias
    std::map<int, int> cid_fid2;
    for (auto it = cid_fid.begin(); it != cid_fid.end(); it++) {
      int cid = it->first;
      if (cid2cid.find(cid) == cid2cid.end()) continue;  // not an image we want to keep
      cid_fid2[cid2cid[cid]] = it->second;
    }
    if (cid_fid2.size() <= 1) continue;  // tracks must have size at least 2
    pid_to_cid_fid.push_back(cid_fid2);
    pid_to_xyz.push_back(map.pid_to_xyz_[pid]);
  }
  map.pid_to_cid_fid_ = pid_to_cid_fid;
  map.pid_to_xyz_ = pid_to_xyz;

  // Recreate cid_fid_to_pid_ from pid_to_cid_fid_. This must happen
  // after the merging is complete but before using the new map.
  map.InitializeCidFidToPid();

  LOG(INFO) << "Number of images in the extracted map: " << map.cid_to_filename_.size();
  LOG(INFO) << "Number of tracks in the extracted map: " << map.pid_to_cid_fid_.size();
  // map.Save(output_map + ".extracted.map");

  return;
}

// Register a map to world coordinates from user-supplied data, or simply
// verify how well the map performs with this data.
double RegistrationOrVerification(std::vector<std::string> const& data_files,
                                bool verification,
                                sparse_mapping::SparseMap * map) {
  // Get the interest points in the images, and their positions in
  // the world coordinate system, as supplied by a user.
  // Parse and concatenate that information from multiple files.
  std::vector<std::string> images;
  Eigen::MatrixXd user_ip;
  Eigen::Matrix3Xd user_xyz;
  for (size_t file_id = 0; file_id < data_files.size(); file_id++) {
    std::string file = data_files[file_id];
    std::string ext = ff_common::file_extension(file);
    std::vector<std::string> curr_images;
    Eigen::MatrixXd curr_ip, curr_xyz;

    if (ext == "pto") {
      sparse_mapping::ParseHuginControlPoints(file, &curr_images, &curr_ip);

      int orig_num_img = images.size();

      // Append to the larger sets
      for (size_t it = 0; it < curr_images.size(); it++)
        images.push_back(curr_images[it]);

      // Append to the larger set
      int orig_num_ip = user_ip.cols();
      Eigen::MatrixXd merged_ip(curr_ip.rows(),
                                user_ip.cols() + curr_ip.cols());
      if (user_ip.cols() > 0)
        merged_ip << user_ip, curr_ip;
      else
        merged_ip << curr_ip;
      user_ip = merged_ip;
      for (int pid = orig_num_ip; pid < user_ip.cols(); pid++) {
        user_ip(0, pid) += orig_num_img;  // update the index of the left image
        user_ip(1, pid) += orig_num_img;  // update the index of the right image
      }
    } else if (ext == "txt") {
      sparse_mapping::ParseXYZ(file, &curr_xyz);

      // Append to the larger set
      Eigen::Matrix3Xd merged_xyz(curr_xyz.rows(),
                                 user_xyz.cols() + curr_xyz.cols());
      if (user_xyz.cols() > 0)
        merged_xyz << user_xyz, curr_xyz;
      else
        merged_xyz << curr_xyz;
      user_xyz = merged_xyz;
    }
  }

  int num_points = user_ip.cols();
  if (num_points != user_xyz.cols())
    LOG(FATAL) << "Could not parse an equal number of control "
               << "points and xyz coordinates. Their numbers are "
               << num_points << " vs " << user_xyz.cols() << ".\n";

  std::map<std::string, int> filename_to_cid;
  for (size_t cid = 0; cid < map->cid_to_filename_.size(); cid++)
    filename_to_cid[map->cid_to_filename_[cid]] = cid;

  // Wipe images that are missing from the map
  std::map<int, int> cid2cid;
  int good_cid = 0;
  for (size_t cid = 0; cid < images.size(); cid++) {
    std::string image = images[cid];
    if (filename_to_cid.find(image) == filename_to_cid.end()) {
      LOG(WARNING) << "Will ignore image missing from map: " << image;
      continue;
    }
    cid2cid[cid] = good_cid;
    images[good_cid] = images[cid];
    good_cid++;
  }
  images.resize(good_cid);

  // Remove points corresponding to images missing from map
  int good_pid = 0;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end()) {
      continue;
    }
    user_ip.col(good_pid) = user_ip.col(pid);
    user_xyz.col(good_pid) = user_xyz.col(pid);
    good_pid++;
  }
  user_ip.conservativeResize(Eigen::NoChange_t(), good_pid);
  user_xyz.conservativeResize(Eigen::NoChange_t(), good_pid);
  num_points = good_pid;
  for (int pid = 0; pid < num_points; pid++) {
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);
    if (cid2cid.find(id1) == cid2cid.end() || cid2cid.find(id2) == cid2cid.end())
      LOG(FATAL) << "Book-keeping failure in registration.";
    user_ip(0, pid) = cid2cid[id1];
    user_ip(1, pid) = cid2cid[id2];
  }

  // Iterate over the control points in the hugin file. Copy the
  // control points to the list of user keypoints, and create the
  // corresponding user_pid_to_cid_fid_.
  map->user_cid_to_keypoint_map_.resize(map->cid_to_filename_.size());
  map->user_pid_to_cid_fid_.resize(num_points);
  for (int pid = 0; pid < num_points; pid++) {
    // Left and right image indices
    int id1 = user_ip(0, pid);
    int id2 = user_ip(1, pid);

    // Sanity check
    if (id1 < 0 || id2 < 0 ||
        id1 >= static_cast<int>(images.size()) ||
        id2 >= static_cast<int>(images.size()) )
      LOG(FATAL) << "Invalid image indices in the hugin file: " << id1 << ' ' << id2;

    // Find the corresponding indices in the map where these keypoints will go to
    if (filename_to_cid.find(images[id1]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id1];
    if (filename_to_cid.find(images[id2]) == filename_to_cid.end())
      LOG(FATAL) << "File missing from map: " << images[id2];
    int cid1 = filename_to_cid[images[id1]];
    int cid2 = filename_to_cid[images[id2]];

    // Append to the keypoints for cid1
    Eigen::Matrix<double, 2, -1> &M1 = map->user_cid_to_keypoint_map_[cid1];  // alias
    Eigen::Matrix<double, 2, -1> N1(M1.rows(), M1.cols()+1);
    N1 << M1, user_ip.block(2, pid, 2, 1);  // left image pixel x and pixel y
    M1.swap(N1);

    // Append to the keypoints for cid2
    Eigen::Matrix<double, 2, -1> &M2 = map->user_cid_to_keypoint_map_[cid2];  // alias
    Eigen::Matrix<double, 2, -1> N2(M2.rows(), M2.cols()+1);
    N2 << M2, user_ip.block(4, pid, 2, 1);  // right image pixel x and pixel y
    M2.swap(N2);

    // The corresponding user_pid_to_cid_fid_
    map->user_pid_to_cid_fid_[pid][cid1] = map->user_cid_to_keypoint_map_[cid1].cols()-1;
    map->user_pid_to_cid_fid_[pid][cid2] = map->user_cid_to_keypoint_map_[cid2].cols()-1;
  }

  // Shift the keypoints. Undistort if necessary.
  Eigen::Vector2d output;
  for (size_t cid = 0; cid < map->user_cid_to_keypoint_map_.size(); cid++) {
    for (int i = 0; i < map->user_cid_to_keypoint_map_[cid].cols(); i++) {
      map->camera_params_.Convert<camera::DISTORTED, camera::UNDISTORTED_C>
        (map->user_cid_to_keypoint_map_[cid].col(i), &output);
      map->user_cid_to_keypoint_map_[cid].col(i) = output;
    }
  }

  // Initialize user_pid_to_xyz_
  map->user_pid_to_xyz_.resize(user_xyz.cols());
  for (int i = 0; i < user_xyz.cols(); i++)
    map->user_pid_to_xyz_[i] = user_xyz.col(i);

  // Triangulate to find the coordinates of the current points
  // in the virtual coordinate system
  std::vector<Eigen::Vector3d> pid_to_xyz;
  std::vector<std::map<int, int> > cid_fid_to_pid_local;
  bool rm_invalid_xyz = false;  // there should be nothing to remove hopefully
  sparse_mapping::Triangulate(rm_invalid_xyz,
                              map->camera_params_.GetFocalLength(),
                              map->cid_to_cam_t_global_,
                              map->user_cid_to_keypoint_map_,
                              &(map->user_pid_to_cid_fid_),
                              &pid_to_xyz,
                              &cid_fid_to_pid_local);

  double mean_err = 0;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_xyz[i];
    Eigen::Vector3d b = user_xyz.col(i);
    mean_err += (a-b).norm();
  }
  mean_err /= user_xyz.cols();

  if (verification) {
    std::cout << "Mean absolute error on verification: " << mean_err << " meters" << std::endl;
    std::cout << "computed xyz -- measured xyz -- error diff -- error norm (meters)" << std::endl;
  } else {
    std::cout << "Mean absolute error before registration: " << mean_err << " meters" << std::endl;
    std::cout << "Un-transformed computed xyz -- measured xyz -- error diff -- error norm (meters)" << std::endl;
  }

  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = pid_to_xyz[i];
    Eigen::Vector3d b = user_xyz.col(i);
    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a-b) << " -- "
              << print_vec((a - b).norm())
              << std::endl;
  }

  if (verification)
    return 0;

  // Find the transform from the computed map coordinate system
  // to the world coordinate system.
  int np = pid_to_xyz.size();
  Eigen::Matrix3Xd in(3, np);
  for (int i = 0; i < np; i++)
    in.col(i) = pid_to_xyz[i];
  Eigen::Affine3d world_transform;
  sparse_mapping::Find3DAffineTransform(in, user_xyz, &world_transform);

  // Transform the map to the world coordinate system
  sparse_mapping::TransformCamerasAndPoints(world_transform,
                                            &(map->cid_to_cam_t_global_),
                                            &(map->pid_to_xyz_));

  mean_err = 0.0;
  for (int i = 0; i < user_xyz.cols(); i++)
    mean_err += (world_transform*in.col(i) - user_xyz.col(i)).norm();
  mean_err /= user_xyz.cols();

  // We don't use LOG(INFO) below, as it does not play well with
  // Eigen.
  double scale = pow(world_transform.linear().determinant(), 1.0 / 3.0);
  std::cout << "Transform to world coordinates." << std::endl;
  std::cout << "Rotation:\n" << world_transform.linear() / scale << std::endl;
  std::cout << "Scale:\n" << scale << std::endl;
  std::cout << "Translation:\n" << world_transform.translation().transpose()
            << std::endl;

  std::cout << "Mean absolute error after registration and before final bundle adjustment: "
            << mean_err << " meters" << std::endl;

  std::cout << "Transformed computed xyz -- measured xyz -- error diff - error norm (meters)" << std::endl;
  for (int i = 0; i < user_xyz.cols(); i++) {
    Eigen::Vector3d a = world_transform*in.col(i);
    Eigen::Vector3d b = user_xyz.col(i);
    int id1 = user_ip(0, i);
    int id2 = user_ip(1, i);

    std::cout << print_vec(a) << " -- "
              << print_vec(b) << " -- "
              << print_vec(a - b) << " -- "
              << print_vec((a - b).norm()) << " -- "
              << images[id1] << ' '
              << images[id2] << std::endl;
  }
  return scale;
}

void PrintTrackStats(std::vector<std::map<int, int> >const& pid_to_cid_fid,
                       std::string const& step) {
  LOG(INFO) << "Track statistics after: " << step;

  double track_len = 0.0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    track_len += pid_to_cid_fid[pid].size();
  double avg_len = track_len / pid_to_cid_fid.size();

  LOG(INFO) << "Number of tracks (points in the control network): " << pid_to_cid_fid.size();
  LOG(INFO) << "Total length of all tracks: " << track_len;
  LOG(INFO) << "Average track length: " << avg_len;

  std::map<int, int> stats;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    stats[pid_to_cid_fid[pid].size()]++;
  for (std::map<int, int>::const_iterator it = stats.begin(); it != stats.end() ; it++) {
    LOG(INFO) << "Track length and their number: "
              << it->first << ' ' << it->second;
  }
}

// Filter the matches by a geometric constraint. Compute the essential matrix.
void BuildMapFindEssentialAndInliers(Eigen::Matrix2Xd const& keypoints1,
                                     Eigen::Matrix2Xd const& keypoints2,
                                     std::vector<cv::DMatch> const& matches,
                                     camera::CameraParameters const& camera_params,
                                     bool compute_inliers_only,
                                     size_t cam_a_idx, size_t cam_b_idx,
                                     std::mutex * match_mutex,
                                     CIDPairAffineMap * relative_affines,
                                     std::vector<cv::DMatch> * inlier_matches,
                                     bool compute_rays_angle,
                                     double * rays_angle) {
  // Initialize the outputs
  inlier_matches->clear();
  if (compute_rays_angle)
    *rays_angle = 0.0;

  int pt_count = matches.size();
  Eigen::MatrixXd observationsa(2, pt_count);
  Eigen::MatrixXd observationsb(2, pt_count);
  for (int i = 0; i < pt_count; i++) {
    observationsa.col(i) = keypoints1.col(matches[i].queryIdx);
    observationsb.col(i) = keypoints2.col(matches[i].trainIdx);
  }

  std::pair<size_t, size_t> image_size(camera_params.GetUndistortedSize()[0],
                                       camera_params.GetUndistortedSize()[1]);
  Eigen::Matrix3d k = camera_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();

  Eigen::Matrix3d e;
  // Calculate the essential matrix
  std::vector<size_t> vec_inliers;
  double error_max = std::numeric_limits<double>::max();
  double max_expected_error = 2.5;

  if (!RobustEssential(k, k, observationsa, observationsb,
                                       &e, &vec_inliers,
                                       image_size, image_size,
                                       &error_max,
                                       max_expected_error)) {
    VLOG(2) << cam_a_idx << " " << cam_b_idx
            << " | Estimation of essential matrix failed!\n";
    return;
  }

  if (vec_inliers.size() < static_cast<size_t>(FLAGS_min_valid)) {
    VLOG(2) << cam_a_idx << " " << cam_b_idx
            << " | Failed to get enough inliers " << vec_inliers.size();
    return;
  }

  if (compute_inliers_only) {
    // We only need to know which interest points are inliers and not the
    // R and T matrices.
    int num_inliers = vec_inliers.size();
    inlier_matches->clear();
    inlier_matches->reserve(num_inliers);
    std::vector<Eigen::Matrix2Xd> observations2(2, Eigen::Matrix2Xd(2, num_inliers));
    for (int i = 0; i < num_inliers; i++) {
      inlier_matches->push_back(matches[vec_inliers[i]]);
    }
    return;
  }

  // Estimate the best possible R & T from the found Essential Matrix
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  if (!EstimateRTFromE(k, k, observationsa, observationsb,
                                       e, vec_inliers,
                                       &r, &t)) {
    VLOG(2) << cam_a_idx << " " << cam_b_idx
            << " | Failed to extract RT from E";
    return;
  }

  VLOG(2) << cam_a_idx << " " << cam_b_idx << " | Inliers from E: "
          << vec_inliers.size() << " / " << observationsa.cols();

  // Get the observations corresponding to inliers
  // TODO(ZACK): We could reuse everything.
  int num_inliers = vec_inliers.size();
  std::vector<Eigen::Matrix2Xd> observations2(2, Eigen::Matrix2Xd(2, num_inliers));
  for (int i = 0; i < num_inliers; i++) {
    observations2[0].col(i) = observationsa.col(vec_inliers[i]);
    observations2[1].col(i) = observationsb.col(vec_inliers[i]);
  }

  // Refine the found T and R via bundle adjustment
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.max_num_iterations = 200;
  options.logging_type = ceres::SILENT;
  options.num_threads = FLAGS_num_threads;
  ceres::Solver::Summary summary;
  std::vector<Eigen::Affine3d> cameras(2);
  cameras[0].setIdentity();
  cameras[1].linear() = r;
  cameras[1].translation() = t;
  Eigen::Matrix3Xd pid_to_xyz(3, observations2[0].cols());
  double error;
  int num_pts_behind_camera = 0;
  for (ptrdiff_t i = 0; i < observations2[0].cols(); i++) {
    pid_to_xyz.col(i) =
      sparse_mapping::TriangulatePoint
      (Eigen::Vector3d(observations2[0](0, i), observations2[0](1, i),
                       camera_params.GetFocalLength()),
       Eigen::Vector3d(observations2[1](0, i), observations2[1](1, i),
                       camera_params.GetFocalLength()),
       r, t, &error);
    Eigen::Vector3d P = pid_to_xyz.col(i);
    Eigen::Vector3d Q = r*P + t;
    if (P[2] <= 0 || Q[2] <= 0) {
      num_pts_behind_camera++;
    }
  }
  VLOG(2) << "Pair " << cam_a_idx  << ' ' << cam_b_idx
          << ": number of points behind cameras: "
          << num_pts_behind_camera << "/" <<  observations2[0].cols()
          << " (" << round((100.0*num_pts_behind_camera) / observations2[0].cols())
          << "%)";

  sparse_mapping::BundleAdjustSmallSet(observations2, camera_params.GetFocalLength(), &cameras,
                                       &pid_to_xyz, new ceres::CauchyLoss(0.5), options,
                                       &summary);

  if (!summary.IsSolutionUsable()) {
    LOG(ERROR) << cam_a_idx << " " << cam_b_idx << " | Failed to refine RT with bundle adjustment";
    return;
  }
  VLOG(2) << summary.BriefReport();

  if (compute_rays_angle) {
    // Compute the median angle between rays.
    std::vector<double> angles;
    Eigen::Vector3d ctr0 = cameras[0].inverse().translation();
    Eigen::Vector3d ctr1 = cameras[1].inverse().translation();

    for (ptrdiff_t i = 0; i < observations2[0].cols(); i++) {
      Eigen::Vector3d P =
        sparse_mapping::TriangulatePoint
        (Eigen::Vector3d(observations2[0](0, i), observations2[0](1, i),
                         camera_params.GetFocalLength()),
         Eigen::Vector3d(observations2[1](0, i), observations2[1](1, i),
                         camera_params.GetFocalLength()),
         cameras[1].linear(), cameras[1].translation(), &error);

      Eigen::Vector3d X0 = ctr0 - P;
      Eigen::Vector3d X1 = ctr1 - P;
      double l0 = X0.norm(), l1 = X1.norm();
      double angle;
      // TODO(oalexan1): Integrate this code with the other angle computation
      // code.
      if (l0 == 0 || l1 == 0) {
        angle = 0.0;
      } else {
        double dot = X0.dot(X1)/l0/l1;
        dot = std::min(dot, 1.0);
        dot = std::max(-1.0, dot);
        angle = (180.0/M_PI)*acos(dot);
      }
      angles.push_back(angle);
    }
    // Median rays angle
    if (angles.size() >= static_cast<size_t>(2*FLAGS_min_valid))
      *rays_angle = angles[angles.size()/2];
  }

  // Give the solution
  Eigen::Affine3d result = cameras[1] * cameras[0].inverse();
  result.translation().normalize();

  // Must use a lock to protect this map shared among the threads
  CHECK(match_mutex) << "Forgot to provide the mutex lock.";
  CHECK(relative_affines) << "Forgot to provide relative_affines argument.";
  match_mutex->lock();
  relative_affines->insert(std::make_pair(std::make_pair(cam_a_idx, cam_b_idx),
                                        result));
  match_mutex->unlock();

  cv::Mat valid = cv::Mat::zeros(pt_count, 1, CV_8UC1);
  for (size_t i = 0; i < vec_inliers.size(); i++) {
    valid.at<uint8_t>(vec_inliers[i], 0) = 1;
  }

  // Count the number of inliers
  int32_t num_of_inliers =
    std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);

  // Keep about FLAGS_max_pairwise_matches inliers. This is to speed
  // up map generation so that we don't have to bother with a 1000
  // matches between consecutive images.
  if (FLAGS_max_pairwise_matches < num_of_inliers) {
    std::vector<double> dist;
    for (size_t query_index = 0; query_index < matches.size(); query_index++) {
      if (valid.at<uint8_t>(query_index, 0) > 0)
        dist.push_back(matches[query_index].distance);
    }
    std::sort(dist.begin(), dist.end());
    double max_dist = dist[FLAGS_max_pairwise_matches - 1];
    for (size_t query_index = 0; query_index < matches.size(); query_index++) {
      if (valid.at<uint8_t>(query_index, 0) > 0 &&
          matches[query_index].distance > max_dist) {
        valid.at<uint8_t>(query_index, 0) = 0;
      }
    }
    num_of_inliers
      = std::accumulate(valid.begin<uint8_t>(), valid.end<uint8_t>(), 0);
  }

  // Copy the inliers only
  inlier_matches->clear();
  inlier_matches->reserve(num_of_inliers);
  for (size_t m = 0; m < matches.size(); m++) {
    if (valid.at<uint8_t>(m, 0) > 0) {
      inlier_matches->push_back(matches[m]);
    }
  }
}

// TODO(rsoussan): Pass sparse map database instead of all these individual params?
void BundleAdjust(std::vector<std::map<int, int> > const& pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd> const& cid_to_keypoint_map, double focal_length,
                  std::vector<Eigen::Affine3d>* cid_to_cam_t_global, std::vector<Eigen::Vector3d>* pid_to_xyz,
                  std::vector<std::map<int, int> > const& user_pid_to_cid_fid,
                  std::vector<Eigen::Matrix2Xd> const& user_cid_to_keypoint_map,
                  std::vector<Eigen::Vector3d>* user_pid_to_xyz, ceres::LossFunction* loss,
                  ceres::Solver::Options const& options, ceres::Solver::Summary* summary, int first, int last,
                  bool fix_all_cameras, std::set<int> const& fixed_cameras) {
  // Perform bundle adjustment. Keep fixed all cameras with cid
  // not within [first, last] and all xyz points which project only
  // onto fixed cameras.

  // If provided, use user-set registration points in the second pass.

  // Allocate space for the angle axis representation of rotation
  std::vector<double> camera_aa_storage(3 * cid_to_cam_t_global->size());
  for (size_t cid = 0; cid < cid_to_cam_t_global->size(); cid++) {
    Eigen::Map<Eigen::Vector3d> aa_storage(camera_aa_storage.data() + 3 * cid);
    Eigen::Vector3d vec;
    camera::RotationToRodrigues(cid_to_cam_t_global->at(cid).linear(),
                               &vec);
    aa_storage = vec;
  }

  // Build problem
  ceres::Problem problem;

  // Ideally the block inside of the loop below must be a function call,
  // but the compiler does not handle that correctly with ceres.
  // So do this by changing where things are pointing.

  int num_passes = 1;
  if (!user_pid_to_xyz->empty()) num_passes = 2;  // A second pass using control points

  for (int pass = 0; pass < num_passes; pass++) {
    std::vector<std::map<int, int> > const * p_pid_to_cid_fid;
    std::vector<Eigen::Matrix2Xd >   const * p_cid_to_keypoint_map;
    std::vector<Eigen::Vector3d>           * p_pid_to_xyz;
    ceres::LossFunction * local_loss;
    if (pass == 0) {
      local_loss            = loss;  // outside-supplied loss
      p_pid_to_cid_fid      = &pid_to_cid_fid;
      p_cid_to_keypoint_map = &cid_to_keypoint_map;
      p_pid_to_xyz          = pid_to_xyz;
    } else {
      local_loss            = NULL;  // l2, as user-supplied data is reliable
      p_pid_to_cid_fid      = &user_pid_to_cid_fid;
      p_cid_to_keypoint_map = &user_cid_to_keypoint_map;
      p_pid_to_xyz          = user_pid_to_xyz;
    }

    for (size_t pid = 0; pid < p_pid_to_xyz->size(); pid++) {
      if ((*p_pid_to_cid_fid)[pid].size() < 2)
        LOG(FATAL) << "Found a track of size < 2.";

      // Don't vary points which project only into cameras which we don't vary.
      bool fix_pid = true;
      for (std::map<int, int>::value_type const& cid_fid : (*p_pid_to_cid_fid)[pid]) {
        if (cid_fid.first >= first && cid_fid.first <= last)
          fix_pid = false;
      }

      for (std::map<int, int>::value_type const& cid_fid : (*p_pid_to_cid_fid)[pid]) {
        ceres::CostFunction* cost_function =
          ReprojectionError::Create((*p_cid_to_keypoint_map)[cid_fid.first].col(cid_fid.second));

        problem.AddResidualBlock(cost_function,
                                 local_loss,
                                 &cid_to_cam_t_global->at(cid_fid.first).translation()[0],
                                 &camera_aa_storage[3 * cid_fid.first],
                                 &p_pid_to_xyz->at(pid)[0],
                                 &focal_length);

        if (fix_all_cameras || (cid_fid.first < first || cid_fid.first > last) ||
            fixed_cameras.find(cid_fid.first) != fixed_cameras.end()) {
          problem.SetParameterBlockConstant(&cid_to_cam_t_global->at(cid_fid.first).translation()[0]);
          problem.SetParameterBlockConstant(&camera_aa_storage[3 * cid_fid.first]);
        }
      }
      if (fix_pid || pass == 1) {
        // Fix pids which don't project in cameras that are floated.
        // Also, must not float points given by the user, those are measurements
        // we are supposed to reference ourselves against, and floating
        // them can make us lose the real world scale.
        problem.SetParameterBlockConstant(&p_pid_to_xyz->at(pid)[0]);
      }
    }
    problem.SetParameterBlockConstant(&focal_length);
  }

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  for (size_t cid = 0; cid < cid_to_cam_t_global->size(); cid++) {
    Eigen::Map<Eigen::Vector3d> aa_storage
      (camera_aa_storage.data() + 3 * cid);
    Eigen::Matrix3d r;
    camera::RodriguesToRotation(aa_storage, &r);
    cid_to_cam_t_global->at(cid).linear() = r;
  }
}

// This is a very specialized function
void BundleAdjustSmallSet(std::vector<Eigen::Matrix2Xd> const& features_n,
                          double focal_length,
                          std::vector<Eigen::Affine3d> * cam_t_global_n,
                          Eigen::Matrix3Xd * pid_to_xyz,
                          ceres::LossFunction * loss,
                          ceres::Solver::Options const& options,
                          ceres::Solver::Summary * summary) {
  CHECK(cam_t_global_n) << "Variable cam_t_global_n needs to be defined";
  CHECK(cam_t_global_n->size() == features_n.size())
    << "Variables features_n and cam_t_global_n need to agree on the number of cameras";
  CHECK(cam_t_global_n->size() > 1) << "Bundle adjust needs at least 2 or more cameras";
  CHECK(pid_to_xyz->cols() == features_n[0].cols())
    << "There should be an equal amount of XYZ points as there are feature observations";
  for (size_t i = 1; i < features_n.size(); i++) {
    CHECK(features_n[0].cols() == features_n[i].cols())
      << "The same amount of features should be seen in all cameras";
  }

  const size_t n_cameras = features_n.size();

  // Allocate space for the angle axis representation of rotation
  std::vector<Eigen::Vector3d> aa(n_cameras);
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RotationToRodrigues(cam_t_global_n->at(cid).linear(), &aa[cid]);
  }

  // Build the problem
  ceres::Problem problem;
  for (ptrdiff_t pid = 0; pid < pid_to_xyz->cols(); pid++) {
    for (size_t cid = 0; cid < n_cameras; cid++) {
      ceres::CostFunction* cost_function = ReprojectionError::Create(features_n[cid].col(pid));
      problem.AddResidualBlock(cost_function, loss,
                               &cam_t_global_n->at(cid).translation()[0],
                               &aa.at(cid)[0],
                               &pid_to_xyz->col(pid)[0],
                               &focal_length);
    }
  }
  problem.SetParameterBlockConstant(&focal_length);

  // Solve the problem
  ceres::Solve(options, &problem, summary);

  // Write the rotations back to the transform
  Eigen::Matrix3d r;
  for (size_t cid = 0; cid < n_cameras; cid++) {
    camera::RodriguesToRotation(aa[cid], &r);
    cam_t_global_n->at(cid).linear() = r;
  }
}

}  // namespace sparse_mapping
