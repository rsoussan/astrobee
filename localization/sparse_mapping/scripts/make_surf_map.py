#!/usr/bin/python
#
# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.
"""
Generates a surf map for a given input bagfile. 
Extracts images, removes low movement images, and builds 
the surf map using incremental bundle adjustment using the remaining
images.
"""

import argparse
import os
import shutil
import sys

import localization_common.utilities as lu


def make_surf_map(
    bagfile,
    surf_map_name,
    image_topic,
    world,
    robot_name,
):
    bag_images_dir = "bag_images_" + lu.basename(bagfile)
    os.mkdir(bag_images_dir)
    bag_images = os.path.abspath(bag_images_dir)
    extract_images_command = (
        "rosrun bag_processing extract_images "
        + bagfile
        + " -i " + image_topic + " -o "
        + bag_images
    )
    lu.run_command_and_save_output(extract_images_command, "extract_images.txt")

    remove_low_movement_images_command = (
        "rosrun sparse_mapping remove_low_movement_images " + bag_images 
    )
    lu.run_command_and_save_output(remove_low_movement_images_command, "remove_low_movement_images.txt")

    # Set environment variables
    home = os.path.expanduser("~")
    robot_config_file = os.path.join("config/robots", robot_name + ".config")
    astrobee_path = os.path.join(home, "astrobee/src/astrobee")
    os.environ["ASTROBEE_RESOURCE_DIR"] = os.path.join(astrobee_path, "resources")
    os.environ["ASTROBEE_CONFIG_DIR"] = os.path.join(astrobee_path, "config")
    os.environ["ASTROBEE_ROBOT"] = os.path.join(
        astrobee_path, robot_config_path
    )
    os.environ["ASTROBEE_WORLD"] = world

    # Build surf map
    build_map_command = (
        "rosrun sparse_mapping build_map "
        + all_bag_images
        + " -output_map "
        + surf_map_name 
        + " -feature_detection -histogram_equalization -feature_matching -track_building -incremental_ba -bundle_adjustment -num_subsequent_images 100"
    )
    lu.run_command_and_save_output(build_map_command, "build_surf_map.txt")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile to generate surf map for.")
    parser.add_argument("-o", "--output-directory", default=None, help="Output directory where images and map will be saved. Defaults to bag_name_map_creation")
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Image topic.",
    )
    parser.add_argument("-w", "--world", default="iss", help="World name (iss or granite).")
    parser.add_argument("-r", "--robot-name", default="bumble", help="Robot name.")
    parser.add_argument("-m", "--map-name", default=None, help="Output map name. Defaults to bag_name_surf.map")

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print("Bag file " + args.bagfile + " does not exist.")
        sys.exit()
    if not args.output_directory:
      args.output_directory = lu.basename(args.bagfile) + "_map_creation" 
    if os.path.isdir(args.output_directory):
        print("Output directory " + args.output_directory + " already exists.")
        sys.exit()
    if not args.map_name:
      args.map_name = lu.basename(args.bagfile) + "_surf.map" 

    bagfile = os.path.abspath(args.bagfile)
    os.mkdir(args.output_directory)
    os.chdir(args.output_directory)

    make_surf_map(
        bagfile,
        args.map_name,
        args.image_topic
        args.world,
        args.robot_name,
    )
