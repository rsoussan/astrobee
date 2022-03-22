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
#include <localization_common/logger.h>

#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <vector>

namespace po = boost::program_options;

std::string Filename(const int seq, const double timestamp, const bool use_timestamp_as_image_name,
                     const std::string& non_timestamp_name_format) {
  const boost::format format =
    use_timestamp_as_image_name ? boost::format("%10.7f") % timestamp : boost::format(non_timestamp_name_format) % seq;
  return format.str();
}

void ExtractImages(const std::string& input_bagname, const std::string& output_directory,
                   const std::string& output_format, const std::string& image_topic, const double start_time = 0,
                   const double duration = 1e5, const bool use_timestamp_as_image_name = true) {
  rosbag::Bag input_bag(input_bagname, rosbag::bagmode::Read);
  rosbag::View start_view(input_bag);
  const ros::Time bag_start_time = start_view.getBeginTime() + ros::Duration(start_time);
  const ros::Time bag_end_time = bag_start_time + ros::Duration(duration);
  rosbag::View view(input_bag, rosbag::TopicQuery({image_topic}), bag_start_time, bag_end_time);
  if (view.size() == 0) {
    LogFatal("No images for topic " << image_topic << " in bagfile.");
  }
  LogInfo("Copying at most " << view.size() << " frames from the bag file.");

  for (const auto& msg : view) {
    cv::Mat image;
    // Try to extract normal image
    const auto image_msg = msg.instantiate<sensor_msgs::Image>();
    if (image_msg) {
      try {
        // TODO(rsoussan): This will convert all images to color, is that desired?
        image = cv_bridge::toCvShare(image_msg, "bgr8")->image;
      } catch (cv_bridge::Exception const& e) {
        try {
          image = cv_bridge::toCvShare(image_msg, "32FC1")->image;
        } catch (cv_bridge::Exception const& e) {
          LogError("Unable to convert " << image_msg->encoding.c_str() << " image to bgr8 or 32FC1");
          continue;
        }
      }
    } else {
      // Try to extract a compressed image
      const auto image_msg = msg.instantiate<sensor_msgs::CompressedImage>();
      if (image_msg) {
        try {
          image = cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR);
        } catch (cv_bridge::Exception const& e) {
          LogError("Unable to convert compressed image to bgr8.");
          continue;
        }
      }
    }

    if (!image.empty()) {
      const double timestamp = (image_msg->header.stamp).toSec();
      const auto filename = Filename(image_msg->header.seq, timestamp, use_timestamp_as_image_name, output_format);
      const std::string name = (boost::filesystem::path(output_directory) / (filename + ".jpg")).string();
      cv::imwrite(name, image);
    }
  }

  input_bag.close();
}

int main(int argc, char** argv) {
  std::string image_topic;
  std::string output_directory;
  std::string output_format;
  double start_time;
  double duration;
  bool use_timestamp_as_image_name;
  po::options_description desc("Extracts images from a bagfile and saves them as jpg files.");
  desc.add_options()("help,h", "produce help message")("bagfile", po::value<std::string>()->required(),
                                                       "Input bagfile containing the images to extract.")(
    "image-topic,i", po::value<std::string>(&image_topic)->default_value("/hw/cam_nav"), "Image topic.")(
    "output-directory,o", po::value<std::string>(&output_directory)->default_value(""),
    "Output directory for extracted image files.")("start-time,s", po::value<double>(&start_time)->default_value(0),
                                                   "Begin extracting images this many seconds into the bagfile.")(
    "duration,d", po::value<double>(&duration)->default_value(1e5),
    "Extract images for this many seconds of the bagfile. Default behavior extracts images from the entire bag.")(
    "use-sequence-value-as-filename,u", po::bool_switch(&use_timestamp_as_image_name)->default_value(true),
    "Use the sequence value from the message header as the saved image filename. Default behavior uses the timestamp "
    "as the filename.")("output-format,f", po::value<std::string>(&output_format)->default_value("%06i"),
                        "String format for output image files if timestamps are not used as filenames.");
  po::positional_options_description p;
  p.add("bagfile", 1);
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

  // Only pass program name to free flyer so that boost command line options
  // are ignored when parsing gflags.
  int ff_argc = 1;
  ff_common::InitFreeFlyerApplication(&ff_argc, &argv);

  if (!boost::filesystem::exists(input_bag)) {
    LogFatal("Bagfile " << input_bag << " not found.");
  }

  if (output_directory == "") {
    const auto output_directory_path =
      boost::filesystem::current_path() /
      boost::filesystem::path(boost::filesystem::path(input_bag).stem().string() + "_images");
    output_directory = output_directory_path.string();
  }

  if (boost::filesystem::exists(output_directory)) {
    LogFatal("Output directory " << output_directory << " already exists.");
  }
  boost::filesystem::create_directory(output_directory);

  ExtractImages(input_bag, output_directory, output_format, image_topic, start_time, duration,
                use_timestamp_as_image_name);
}
