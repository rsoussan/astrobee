#!/usr/bin/env python
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
import os
import re
import sys

import rosbag


# https://stackoverflow.com/a/4836734
def natural_sort(l):
  convert = lambda text: int(text) if text.isdigit() else text.lower()
  alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
  return sorted(l, key=alphanum_key)


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('input_bag_prefix')
  parser.add_argument('--merged-bag', default='')
  args = parser.parse_args()

  # Find bagfiles with bag prefix in current directory, fail if none found
  bag_names = [
    bag for bag in os.listdir('.')
    if os.path.isfile(bag) and bag.startswith(args.input_bag_prefix) and bag.endswith('.bag')
  ]
  if (len(bag_names) == 0):
    print('No bag files found')
    sys.exit()
  else:
    print('Found ' + str(len(bag_names)) + ' bag files.')

  merged_bag_name = ''
  if not args.merged_bag:
    merged_bag_name = 'merged_' + args.input_bag_prefix + '.bag'

  sorted_bag_names = natural_sort(bag_names)

  topics = [
    '/hw/imu', '/loc/of/features', '/loc/ml/features', '/loc/ar/features', '/mob/flight_mode', '/mgt/img_sampler/nav_cam/image_record'
  ]

  with rosbag.Bag(merged_bag_name, 'w') as merged_bag:
    for sorted_bag_name in sorted_bag_names:
      with rosbag.Bag(sorted_bag_name, 'r') as sorted_bag:
        for topic, msg, t in sorted_bag.read_messages(topics):
          merged_bag.write(topic, msg, t)
