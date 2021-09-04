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

import loc_states
import plot_helpers
import plot_results
import poses
import velocities
import rmse_utilities
import utilities

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.image as mpimg

import geometry_msgs
import math
import rosbag


def create_overlay(pdf, poses_a, poses_b = None):
  img = mpimg.imread('/home/rsoussan/paper_images/iss_module_birds_eye_trimmed.png')
  colors = ['r', 'b', 'g']
  scale = 100
  x_offset = -950
  y_offset = 1300
  scaled_xs = [scale*x + x_offset for x in poses_a.positions.xs]
  scaled_ys = [scale*y + y_offset for y in poses_a.positions.ys]
  plt.figure()
  plt.imshow(img)
  plt.plot(scaled_ys, scaled_xs, 'g', linewidth=1, linestyle='-') 
  #plt.axis('off')
  pdf.savefig()
  plt.close()


def load_loc_state_msgs(vec_of_loc_states, bag, bag_start_time):
  topics = [loc_states.topic for loc_states in vec_of_loc_states]
  for topic, msg, t in bag.read_messages(topics):
    for loc_states in vec_of_loc_states:
      if loc_states.topic == topic:
        loc_states.add_loc_state(msg, bag_start_time)
        break

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile_a')
  parser.add_argument('bagfile_b')
  args = parser.parse_args()
  if not os.path.isfile(args.bagfile_a):
    print('Bag file ' + args.bagfile_a + ' does not exist.')
    sys.exit()
  if not os.path.isfile(args.bagfile_b):
    print('Bag file ' + args.bagfile_b + ' does not exist.')
    sys.exit()

  bag_a = rosbag.Bag(args.bagfile_a)
  bag_b = rosbag.Bag(args.bagfile_b)
  graph_localization_states_a = loc_states.LocStates('Graph Localization', '/graph_loc/state') 
  graph_localization_states_b = loc_states.LocStates('Graph Localization', '/graph_loc/state') 
  load_loc_state_msgs([graph_localization_states_a], bag_a, 0)
  load_loc_state_msgs([graph_localization_states_b], bag_b, 0)

  bag_a.close()
  bag_b.close()

  with PdfPages("overlay.pdf") as pdf:
    create_overlay(pdf, graph_localization_states_a) #, graph_localization_states_b)
