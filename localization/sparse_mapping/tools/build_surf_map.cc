/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
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
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <config_reader/config_reader.h>
#include <camera/camera_params.h>
#include <sparse_mapping/sparse_map.h>
#include <sparse_mapping/estimate_pose_utilities.h>
#include <sparse_mapping/tensor.h>
#include <sparse_mapping/sparse_mapping.h>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <thread>

namespace lc = localization_common;

/*// outputs
DEFINE_string(output_map, "",
              "Output file containing the matches and control network.");

// parameters used in feature detection step only
DEFINE_int32(sample_rate, 1,
             "Add one of every n input frames to the map.");
DEFINE_bool(save_individual_maps, false,
            "Save separately the maps after detection, matching, track building, "
            "incremental bundle adjustment, and global bundle adjustment.");

// output map parameters
DEFINE_string(detector, "SURF",
              "Feature detector to use. Options are: SURF, ORGBRISK.");
DEFINE_string(rebuild_detector, "ORGBRISK",
              "Feature detector to use. Options are: SURF, ORGBRISK.");

// control options
DEFINE_bool(feature_detection, false,
            "Compute features for input images.");
DEFINE_bool(feature_matching, false,
            "Perform feature matching.");
DEFINE_bool(track_building, false,
              "Perform track building.");
DEFINE_bool(incremental_ba, false,
            "Perform incremental bundle adjustment.");
DEFINE_bool(loop_closure, false,
            "Take a map where images start repeating, and close the loop.");
DEFINE_bool(bundle_adjustment, false,
              "Perform bundle adjustment.");
DEFINE_bool(rebuild, false,
              "Rebuild the map with BRISK features.");
DEFINE_bool(rebuild_replace_camera, false,
              "During rebuilding replace the camera with the one from ASTROBEE_ROBOT.");
DEFINE_bool(vocab_db, false,
              "Build the map with a vocabulary database.");
DEFINE_bool(registration, false,
            "Register the map to world coordinates(requires control points and their xyz coordinates). "
            "This new data is used to redo the bundle adjustment");
DEFINE_bool(verification, false,
            "Verify how an already registered map performs on an independently "
            "acquired set of control points and 3D measurements.");
DEFINE_bool(registration_skip_bundle_adjustment, false,
            "Skip bundle adjustment during the registration step.");
DEFINE_bool(prune, false,
              "Prune the map (the vocab db is unchanged).");
DEFINE_bool(info, false,
              "Print some information on the existing map.");
DEFINE_bool(save_poses, false,
              "Save the camera to world transform and its inverse for each image in the map.");
DEFINE_bool(save_xyz, false,
              "Save the sparse set of points obtained by triangulating the interest point matches.");
DEFINE_int32(num_repeat_images, 0,
             "How many images from the beginning of the sequence to repeat at the end "
             "of the sequence, to help with loop closure (assuming first and last images "
             "are similar).");
DEFINE_bool(fix_cameras, false,
            "Keep the cameras fixed during bundle adjustment.");
DEFINE_bool(rebuild_refloat_cameras, false,
            "Optimize the cameras as well as part of rebuilding. Usually that is "
            "avoided when rebuilding with ORGBRISK, but could be useful with SURF.");

DEFINE_int32(db_restarts, 1, "Number of restarts when building the tree.");
DEFINE_int32(db_depth, 0, "Depth of the tree to build. Default: 4");
DEFINE_int32(db_branching_factor, 0, "Branching factor of the tree to build. "
             "Default: 10");
DEFINE_string(robot_camera, "nav_cam",
              "Which of bot's cameras to use for map-building. Anything except nav_cam is experimental.");

DEFINE_string(undistorted_camera_params, "",
              "Assume that the camera has no distortion and given intrinsics. "
              "Specify as: 'image_wid_x image_wid_y focal_length optical_center_x "
              "optical_center_y'.");
*/

// TODO(rsoussan): move this to sparse_map
//bool g_pruning_was_done = false;  // If we already pruned the map, don't prune it again

/*void IncrementalBA() {
  LOG(INFO) << "Beginning incremental bundle adjustment.";

  sparse_mapping::SparseMap map(FLAGS_output_map);

  sparse_mapping::IncrementalBA(sparse_mapping::EssentialFile(FLAGS_output_map), &map);

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".incremental.map");
}

void CloseLoop() {
  LOG(INFO) << "Beginning loop closure.";

  sparse_mapping::SparseMap map(FLAGS_output_map);

  sparse_mapping::CloseLoop(&map);
  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".closed.map");
}

void BundleAdjust() {
  LOG(INFO) << "Performing bundle adjustment.";
  sparse_mapping::SparseMap map(FLAGS_output_map);

  bool fix_cameras = FLAGS_fix_cameras;
  sparse_mapping::BundleAdjust(fix_cameras, &map);

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".bundle.map");
}

// rebuilds with a different descriptor and detector
// TODO(Rsoussan): move this to sparse_map, rename 
void Rebuild() {
  LOG(INFO) << "Rebuilding map with " << FLAGS_rebuild_detector << " detector.";
  sparse_mapping::SparseMap original(FLAGS_output_map);

  camera::CameraParameters params = original.GetCameraParameters();
  if (FLAGS_rebuild_replace_camera) {
    char * bot_ptr = getenv("ASTROBEE_ROBOT");
    if (bot_ptr == NULL)
      LOG(FATAL) << "Must set ASTROBEE_ROBOT.";
    LOG(INFO) << "Using camera for robot: " << bot_ptr << ".";
    config_reader::ConfigReader config;
    config.AddFile("cameras.config");
    if (!config.ReadFiles()) {
      LOG(ERROR) << "Failed to read config files.";
      exit(1);
      return;
    }
    params = camera::CameraParameters(&config, FLAGS_robot_camera.c_str());
  }

  std::vector<std::string> files(original.GetNumFrames());
  for (size_t i = 0; i < original.GetNumFrames(); i++) {
    files[i] = original.GetFrameFilename(i);
  }

  sparse_mapping::SparseMap map(files, FLAGS_rebuild_detector, params);

  LOG(INFO) << "Detecting features.";
  map.DetectFeatures();

  LOG(INFO) << "Matching features.";
  // Borrow from the original map which images should be matched with which.
  map.cid_to_cid_.clear();
  for (size_t p = 0; p < original.pid_to_cid_fid_.size(); p++) {
    std::map<int, int> const &track = original.pid_to_cid_fid_[p];
    for (std::map<int, int>::const_iterator it1 = track.begin();
         it1 != track.end() ; it1++) {
      for (std::map<int, int>::const_iterator it2 = it1;
           it2 != track.end() ; it2++) {
        if (it1->first != it2->first) {
          // Never match an image with itself
          map.cid_to_cid_[it1->first].insert(it2->first);
        }
      }
    }
  }

  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(FLAGS_output_map),
                                sparse_mapping::MatchesFile(FLAGS_output_map), &map);
  for (unsigned int i = 0; i < original.GetNumFrames(); i++)
    map.SetFrameGlobalTransform(i, original.GetFrameGlobalTransform(i));

  // Wipe file that is no longer needed
  try {
    std::remove(sparse_mapping::EssentialFile(FLAGS_output_map).c_str());
  }catch(...) {}

  LOG(INFO) << "Building tracks.";
  bool rm_invalid_xyz = true;  // by now cameras are good, so filter out bad stuff
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(FLAGS_output_map),
                              &map);

  // It is essential that during re-building we do not vary the
  // cameras. Those were usually computed with a lot of SURF features,
  // while rebuilding is usually done with many fewer ORGBRISK
  // features.
  bool fix_cameras = !FLAGS_rebuild_refloat_cameras;
  if (fix_cameras)
    LOG(INFO) << "Performing bundle adjustment with fixed cameras.";
  else
    LOG(INFO) << "Performing bundle adjustment while floating cameras.";

  sparse_mapping::BundleAdjust(fix_cameras, &map);

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + "." +
                                           FLAGS_rebuild_detector + ".map");
}

// Prune the map leaving the vocab db unchanged
void PruneMap() {
  if (g_pruning_was_done)
    return;

  sparse_mapping::SparseMap map(FLAGS_output_map);

  LOG(INFO) << "Pruning map. This step is irreversible.\n";
  map.PruneMap();
  g_pruning_was_done = true;

  map.Save(FLAGS_output_map);
}

void VocabDB() {
  LOG(INFO) << "Building vocabulary database.";
  int depth, branching_factor;
  if (FLAGS_db_depth != 0)
    depth = FLAGS_db_depth;
  else
    depth = 6;
  if (FLAGS_db_branching_factor != 0)
    branching_factor = FLAGS_db_branching_factor;
  else
    branching_factor = 10;

  sparse_mapping::SparseMap map(FLAGS_output_map);
  sparse_mapping::SparseMapParams params; 
  params.image_database.depth = depth;
  params.image_database.branching_factor = branching_factor;
  map.params() = params;
  // TODO(rsoussan): support brisk and surf
  map.BuildBriskImageDatabase();
  map.Save(FLAGS_output_map);

  // Pruning must always happen after the database is built, as the
  // full set of features (so without pruning) is necessary to later
  // effectively find similar images.
  PruneMap();
}

// Do either registration or verification
void RegistrationOrVerification(std::vector<std::string> const& data_files) {
  if (FLAGS_registration && FLAGS_verification)
    LOG(FATAL) << "Cannot perform both registration and verification in one step.";

  if (FLAGS_registration)
    LOG(INFO) << "Beginning registration to world coordinates.";

  sparse_mapping::SparseMap map(FLAGS_output_map);

  sparse_mapping::RegistrationOrVerification(data_files, FLAGS_verification, &map);

  if (FLAGS_verification)
    return;

  if (!FLAGS_registration_skip_bundle_adjustment) {
    LOG(INFO) << "Redoing bundle adjustment incorporating the user control points.";
    bool fix_cameras = false;
    BundleAdjust(fix_cameras, &map);
  }

  map.Save(FLAGS_output_map);
  if (FLAGS_save_individual_maps) map.Save(FLAGS_output_map + ".registered.map");
}
*/

/*
// TODO(Rsoussan): move this to sparse_map 
void MapInfo() {
  sparse_mapping::SparseMap map(FLAGS_output_map);

  std::cout << map.GetNumFrames()    << " cameras and "
            << map.GetNumLandmarks() << " points.\n";
  std::cout << "Histogram equalization: " << map.GetHistogramEqualization() << std::endl;

  std::cout << "Images in the map:\n";
  for (unsigned int cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    std::cout << cid << " " << map.cid_to_filename_[cid] << "\n";
  }
}

// TODO(Rsoussan): move this to file utils
void SavePoses() {
  sparse_mapping::SparseMap map(FLAGS_output_map);

  for (unsigned int cid = 0; cid < map.cid_to_filename_.size(); cid++) {
    std::string img_file = map.cid_to_filename_[cid];
    std::string ext = ff_common::file_extension(img_file);

    {
      // Save cam2world
      std::string cam_file = ff_common::ReplaceInStr(img_file,
                                                  "." + ext,
                                                  "_cam2world.txt");

      if (cam_file == img_file)
        LOG(FATAL) << "Failed to replace the image extension in: " << img_file;

      std::cout << "Writing: " << cam_file << "\n";
      std::ofstream ofs(cam_file.c_str());
      ofs.precision(17);
      ofs << map.cid_to_cam_t_global_[cid].inverse().matrix() << "\n";
      ofs.close();
    }

    {
      // Save world2cam
      std::string cam_file = ff_common::ReplaceInStr(img_file,
                                                  "." + ext,
                                                  "_world2cam.txt");
      if (cam_file == img_file)
        LOG(FATAL) << "Failed to replace image extension in: " << img_file;

      std::cout << "Writing: " << cam_file << "\n";
      std::ofstream ofs(cam_file.c_str());
      ofs.precision(17);
      ofs << map.cid_to_cam_t_global_[cid].matrix() << "\n";
      ofs.close();
    }
  }
}

// Save the xyz coordinates of triangulated interest point matches
// TODO(Rsoussan): move this to file utils
void SaveXYZ() {
  sparse_mapping::SparseMap map(FLAGS_output_map);
  std::string xyz_file = ff_common::ReplaceInStr(FLAGS_output_map,
                                                  ".map",
                                              "_xyz.txt");
  std::cout << "Writing: " << xyz_file << std::endl;
  std::ofstream ofs(xyz_file);
  ofs.precision(17);
  for (size_t it = 0; it < map.pid_to_xyz_.size(); it++)
    ofs << map.pid_to_xyz_[it][0] << ' '
        << map.pid_to_xyz_[it][1] << ' '
        << map.pid_to_xyz_[it][2] << std::endl;
  ofs.close();
}*/

int main(int argc, char** argv) {
  std::string robot_config_file;
  //std::string world;
  po::options_description desc(
    "Creates a surf map from a set of images. Detects features, matches features, builds feature tracks, performs
      incremental bundle adjustment, and performs loop closures.");
  desc.add_options()("help,h", "produce help message")
("map-filename", po::value<std::string>()->required(), "Map filename.")
("image-directory", po::value<std::string>()->required(), "Image directory.")
// TODO(rsoussan): Automate finding this path!! Add check if it isn't found!
("config-path,c", po::value<std::string>()->required(), "Config path")
(
    "robot-config-file,r", po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
    "Robot config file")
//("world,w", po::value<std::string>(&world)->default_value("iss"), "World name")

  po::positional_options_description p;
  p.add("map-filename", 1);
  p.add("image-directory", 1);
  p.add("config-path", 1);
  po::variables_map vm;
  try {
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    if (vm.count("help") || (argc <= 1)) {
      std::cout << desc << "\n";
      return 1;
    }
    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  const std::string map_filename = vm["map-filename"].as<std::string>();
  const std::string image_directory = vm["image-directory"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);
  // The world name doesn't matter for map creation, only included since it's required for config loading
  const std::string world = "iss";
  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);

  if (boost::filesystem::exists(map_filename)) {
    LogFatal("Map file " << map_filename << " already exists.");
  }
  if (!boost::filesystem::exists(image_direectory)) {
    LogFatal("Image directory " << image_directory << " not found.");
  }

  const std::vector<std::string> images = lc::GetImageNames(image_directory);
  // TODO(rsoussan): Load params!!!!
  SparseMapping::SparseMap map(images, params);

    LogInfo("Detecting Features...");
    map.DetectFeatures();
    LogInfo("Matching Features...");
    // TODO(rsoussan): what are these flags? how are they used?
  sparse_mapping::MatchFeatures(sparse_mapping::EssentialFile(FLAGS_output_map),
                                sparse_mapping::MatchesFile(FLAGS_output_map), &map);
    LogInfo("Building feature tracks....");
  // TODO(rsoussan): what is this??
  bool rm_invalid_xyz = false;  // we don't have valid cameras, so can't rm xyz
  sparse_mapping::BuildTracks(rm_invalid_xyz,
                              sparse_mapping::MatchesFile(FLAGS_output_map),
                              &map);
  map.Save(map_filename);

/*  if (FLAGS_incremental_ba) {
    IncrementalBA();
  }
  if (FLAGS_loop_closure) {
    CloseLoop();
  }
  if (FLAGS_bundle_adjustment) {
    BundleAdjust();
  }
  if (FLAGS_rebuild) {
    Rebuild();
  }
  if (FLAGS_vocab_db) {
    VocabDB();
  }


  // TODO(rsoussan): Move the following functions to seperate tools (expect for prune map)?
  if (FLAGS_registration || FLAGS_verification) {
    std::vector<std::string> data_files;
    for (int arg = 1; arg < argc; arg++) {
      data_files.push_back(argv[arg]);
    }
    RegistrationOrVerification(data_files);
  }

  if (FLAGS_prune)
    PruneMap();

  if (FLAGS_info)
    MapInfo();

  if (FLAGS_save_poses)
    SavePoses();

  if (FLAGS_save_xyz)
    SaveXYZ();

  google::protobuf::ShutdownProtobufLibrary();

  return 0;*/
}
