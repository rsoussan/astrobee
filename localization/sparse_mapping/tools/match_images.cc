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
#include <interest_point/matching.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <cv_bridge/cv_bridge.h>

namespace po = boost::program_options;
namespace lc = localization_common;

int main(int argc, char** argv) {
  std::string image_topic;
  std::string robot_config_file;
  std::string world;
  std::string config_path_prefix;
  po::options_description desc("Test image matching with BRISK features for a bagfile and set of map images.");
  desc.add_options()("help,h", "produce help message")("bagfile", po::value<std::string>()->required(),
                                                       "Input bagfile")(
    "map-images-directory", po::value<std::string>()->required(), "Map images directory")(
    "config-path,c", po::value<std::string>()->required(), "Config path")(
    "image-topic,i", po::value<std::string>(&image_topic)->default_value("mgt/img_sampler/nav_cam/image_record"),
    "Image topic")("robot-config-file,r",
                   po::value<std::string>(&robot_config_file)->default_value("config/robots/bumble.config"),
                   "Robot config file")("world,w", po::value<std::string>(&world)->default_value("iss"), "World name");
  po::positional_options_description p;
  p.add("bagfile", 1);
  p.add("map-images-directory", 1);
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

  const std::string input_bag = vm["bagfile"].as<std::string>();
  const std::string map_images_directory = vm["map-images-directory"].as<std::string>();
  const std::string config_path = vm["config-path"].as<std::string>();

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(input_bag)) {
    LogFatal("Bagfile " << input_bag << " not found.");
  }

  if (!boost::filesystem::exists(map_images_directory)) {
    LogFatal("Map images directory " << map_images_directory << " not found.");
  }

  // Set environment configs
  lc::SetEnvironmentConfigs(config_path, world, robot_config_file);
  config_reader::ConfigReader config;

  // TODO(rsoussan): optionally add histogram equaliation , load other params (AAAAA)
  // TODO(rsoussan): set detector params!
  interest_point::FeatureDetector detector("brisk");
  std::vector<cv::Mat> map_image_descriptors_vec;
  std::vector<std::string> map_image_names;
  for (const auto& file : boost::filesystem::recursive_directory_iterator(map_images_directory)) {
    if (boost::filesystem::is_regular_file(file) && file.path().extension() == ".jpg") {
      const std::string map_image_name = file.path().string();
      const cv::Mat map_image = cv::imread(map_image_name, cv::IMREAD_GRAYSCALE);
      std::vector<cv::KeyPoint> keypoints;
      cv::Mat descriptors;
      detector.Detect(map_image, &keypoints, &descriptors);
      map_image_descriptors_vec.emplace_back(descriptors);
      map_image_names.emplace_back(file.path().string());
    }
  }

  rosbag::View view(rosbag::Bag(input_bag), rosbag::TopicQuery({image_topic}));
  for (const rosbag::MessageInstance msg : view) {
    const sensor_msgs::ImageConstPtr& image_msg = msg.instantiate<sensor_msgs::Image>();
    cv_bridge::CvImagePtr cv_image;
    try {
      cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      LogError("cv_bridge exception: " << e.what());
      return 0;
    }
    const auto& image = cv_image->image;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    detector.Detect(image, &keypoints, &descriptors);
    for (const auto& map_image_descriptors : map_image_descriptors_vec) {
      std::vector<cv::DMatch> matches;
      interest_point::FindMatches(descriptors, map_image_descriptors, &matches);
      if (matches.size() != 0) {
        std::cout << "Found " << matches.size() << " matches." << std::endl;
      }
    }
  }
}
