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

import csv

import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib.image as mpimg
from scipy import ndimage

import geometry_msgs
import math
import rosbag

def plot_poses(plt, poses, color):
  scale = 105
  x_offset =  1300
  y_offset = 1270 
  scaled_xs = [scale*-1.0*x for x in poses.positions.xs]
  scaled_ys = [scale*y for y in poses.positions.ys]
  theta = 0#-10.0*math.pi/180.0
  # Apply rotation and swap axis
  x_scale = 1
  for i in range(len(scaled_xs)):
    x_init = scaled_ys[i]
    y_init = scaled_xs[i]
    scaled_xs[i] = x_scale*(math.cos(theta)*x_init - math.sin(theta)*y_init) + x_offset
    scaled_ys[i] = math.sin(theta)*x_init + math.cos(theta)*y_init + y_offset
  plt.plot(scaled_xs, scaled_ys, color, linewidth=1, linestyle='-') 
 

def create_overlay(pdf, poses_a_vec, poses_b_vec):
  img = mpimg.imread('/home/rsoussan/paper_images/iss_module_birds_eye_trimmed_rotated.png')
  colors = ['r', 'b', 'g']
  plt.figure()
  plt.imshow(img)
  for poses_a in poses_a_vec:
    plot_poses(plt, poses_a, 'r')
  for poses_b in poses_b_vec:
    plot_poses(plt, poses_b, 'g')
  #plt.gca().set_aspect('equal', adjustable='box')
  #plt.xlim(0, 1200)
  #plt.ylim(0, 1200)
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

def load_bags(bags_file, topic):
  poses_vec = []
  with open(bags_file) as param_csvfile:
    reader = csv.reader(param_csvfile, delimiter=' ')
    for row in reader:
      bag = rosbag.Bag(row[0])
      poses = loc_states.LocStates('Graph Localization', topic) 
      load_loc_state_msgs([poses], bag, 0)
      poses_vec.append(poses)
      bag.close()
  return poses_vec

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('graph_bags')
  parser.add_argument('ekf_bags')
  args = parser.parse_args()
  if not os.path.isfile(args.graph_bags):
    print('Bag file ' + args.graph_bags + ' does not exist.')
    sys.exit()
  if not os.path.isfile(args.ekf_bags):
    print('Bag file ' + args.ekf_bags + ' does not exist.')
    sys.exit()

  graph_vec = load_bags(args.graph_bags, '/graph_loc/state')
  ekf_vec = load_bags(args.ekf_bags, 'ekf_ekf_msg')
  with PdfPages("overlay.pdf") as pdf:
    create_overlay(pdf, graph_vec, ekf_vec) 
