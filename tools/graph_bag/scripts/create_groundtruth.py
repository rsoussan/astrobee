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

import argparse
import shutil

import os
import sys


def create_groundtruth(bagfile, base_surf_map):
  os.mkdir("images")
  extract_images_command = "rosrun localization_node extract_image_bag " + bagfile + " -use_timestamp_as_image_name -image_topic /mgt/img_sampler/nav_cam/image_record -output_directory images"
  os.system(extract_images_command)
  select_images_command = "rosrun sparse_mapping select_images -density_factor 1.4 images/*.jpg"
  os.system(select_images_command)

  # Set environment variables
  home = os.path.expanduser('~')
  astrobee_path = home + '/astrobee/astrobee'
  os.environ['ASTROBEE_RESOURCE_DIR'] = astrobee_path + '/resources'
  os.environ['ASTROBEE_CONFIG_DIR'] = astrobee_path + '/config'
  os.environ['ASTROBEE_ROBOT'] = astrobee_path + '/config/robots/bumble.config'
  os.environ['ASTROBEE_WORLD'] = 'iss'

  # Build groundtruth
  build_map_command = 'rosrun sparse_mapping build_map images/*jpg -output_map groundtruth.map -feature_detection -feature_matching -track_building -incremental_ba -bundle_adjustment -histogram_equalization -num_subsequent_images 1'
  os.system(build_map_command)

  # Merge with base map
  merge_map_command = 'rosrun sparse_mapping merge_maps ' + base_surf_map + ' groundtruth.map -output_map groundtruth.surf.map -num_image_overlaps_at_endpoints 100000000 -skip_bundle_adjustment'
  os.system(merge_map_command)

  return
  ## Convert SURF to BRISK map
  shutil.copyfile("groundtruth.surf.map", "groundtruth.brisk.map")
  rebuild_map_command = 'rosrun sparse_mapping build_map -rebuild -histogram_equalization -output_map groundtruth.brisk.map'
  os.system(rebuild_map_command)

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile')
  parser.add_argument('base_surf_map')
  args = parser.parse_args()
  if not os.path.isfile(args.bagfile):
    print('Bag file ' + args.bagfile + ' does not exist.')
    sys.exit()
  if not os.path.isfile(args.base_surf_map):
    print('Base surf map ' + args.base_surf_map + ' does not exist.')
    sys.exit()

  create_groundtruth(args.bagfile, args.base_surf_map)
