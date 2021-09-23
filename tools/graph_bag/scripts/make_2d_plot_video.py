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
import poses
import velocities
import rmse_utilities
import utilities
import vector3d_plotter2d_video

import argparse
import os
import sys

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import geometry_msgs
import math
import rosbag

import csv


def add_graph_plots(pdf, sparse_mapping_poses, ekf_poses, graph_localization_states, filename):
  colors = ['r', 'b', 'g']
 # position_plotter = vector3d_plotter2d_video.Vector3dPlotter('Time (s)', 'Position (m)', 'AstroLoc Position',
 #                                                     True)
 # position_plotter.add_pose_position(sparse_mapping_poses, color='r', linestyle='-')
 #                                #    color='r', 
 #                                #    linestyle='None',
 #                                #    marker='o',
 #                                #    markeredgewidth=0.1,
 #                                #    markersize=1.5)
 # position_plotter.add_pose_position(graph_localization_states, color='b', linestyle=(0, (5, 3)))
 # position_plotter.plot(pdf)

  if ekf_poses.times:
    ekf_position_plotter = vector3d_plotter2d_video.Vector3dPlotter('Time (s)', 'Position (m)', 'Previous Localizer Position',
                                                      True)
    ekf_position_plotter.add_pose_position(sparse_mapping_poses, color='r', linestyle='-')
                                   #    color='r',
                                   #    linestyle='None',
                                   #    marker='o',
                                   #    markeredgewidth=0.1,
                                   #    markersize=1.5)
    ekf_position_plotter.add_pose_position(ekf_poses, color='b', linestyle=(0, (5, 3)))
    ekf_position_plotter.plot(pdf, filename)

def load_pose_msgs(vec_of_poses, bag, bag_start_time):
  topics = [poses.topic for poses in vec_of_poses]
  for topic, msg, t in bag.read_messages(topics):
    for poses in vec_of_poses:
      if poses.topic == topic:
        poses.add_msg(msg, msg.header.stamp, bag_start_time)
        break

def load_loc_state_msgs(vec_of_loc_states, bag, bag_start_time):
  topics = [loc_states.topic for loc_states in vec_of_loc_states]
  for topic, msg, t in bag.read_messages(topics):
    for loc_states in vec_of_loc_states:
      if loc_states.topic == topic:
        loc_states.add_loc_state(msg, bag_start_time)
        break


def has_topic(bag, topic):
  topics = bag.get_type_and_topic_info().topics
  return topic in topics


# Groundtruth bag must have the same start time as other bagfile, otherwise RMSE calculations will be flawed
def create_plots(bagfile,
                 output_pdf_file,
                 groundtruth_bagfile):
  bag = rosbag.Bag(bagfile)
  groundtruth_bag = rosbag.Bag(groundtruth_bagfile) if groundtruth_bagfile else bag
  bag_start_time = bag.get_start_time()

  graph_localization_states = loc_states.LocStates('AstroLoc', '/graph_loc/state')
  vec_of_loc_states = [graph_localization_states]
  load_loc_state_msgs(vec_of_loc_states, bag, bag_start_time)
  sparse_mapping_poses = poses.Poses('Ground truth', '/sparse_mapping/pose')
  groundtruth_vec_of_poses = [sparse_mapping_poses]
  load_pose_msgs(groundtruth_vec_of_poses, groundtruth_bag, bag_start_time)
  ekf_poses = poses.Poses('Previous Localizer', 'ekf_pose')
  vec_of_poses = [ekf_poses]
  # ekf times are using first ekf pose as start time
  # TODO(rsoussan): run ekf tool again, get start time for each bag!!!
  ekf_start_time = bag_start_time  
  print('t_bag: ' + str(bag_start_time))
  print('t_ekf: ' + str(ekf_start_time))
  load_pose_msgs(vec_of_poses, bag, bag_start_time - ekf_start_time)
  
  print("len times: " + str(len(ekf_poses.times)))
  bag.close()
  gt_val = 0
  for val in range(1, len(ekf_poses.times)): 
    if val % 6 != 0:
      continue
    loc_poses = poses.Poses('Previous Localizer', '')
    loc_poses.times = ekf_poses.times[0:val]
    loc_poses.positions.xs = ekf_poses.positions.xs[0:val]
    loc_poses.positions.ys = ekf_poses.positions.ys[0:val]
    loc_poses.positions.zs = ekf_poses.positions.zs[0:val]
    loc_poses.orientations.yaws = ekf_poses.orientations.yaws[0:val]
    loc_poses.orientations.pitches = ekf_poses.orientations.pitches[0:val]
    loc_poses.orientations.rolls = ekf_poses.orientations.rolls[0:val]
    gt_poses = poses.Poses('Ground truth', '')
    loc_time = loc_poses.times[val-1]
    loc_time_diff = loc_time - loc_poses.times[0]
    gt_time = sparse_mapping_poses.times[0] 
    gt_val = -1
    for time in sparse_mapping_poses.times:
      gt_time_diff = time - gt_time - 15 
      gt_val += 1
      if gt_time_diff > loc_time_diff:
        break
    gt_poses.times = sparse_mapping_poses.times[0:gt_val]
    gt_poses.positions.xs = sparse_mapping_poses.positions.xs[0:gt_val]
    gt_poses.positions.ys = sparse_mapping_poses.positions.ys[0:gt_val]
    gt_poses.positions.zs = sparse_mapping_poses.positions.zs[0:gt_val]
    gt_poses.orientations.yaws = sparse_mapping_poses.orientations.yaws[0:gt_val]
    gt_poses.orientations.pitches = sparse_mapping_poses.orientations.pitches[0:gt_val]
    gt_poses.orientations.rolls = sparse_mapping_poses.orientations.rolls[0:gt_val]
    filename = 'ekf_im_' + str(val).zfill(6) + '.png'
    #with PdfPages(filename) as pdf:
    pdf = 0
    add_graph_plots(pdf, gt_poses, loc_poses, loc_poses, filename)
  for gt_val2 in range(gt_val, len(sparse_mapping_poses.times)):
    val = len(ekf_poses.times) -1
    loc_poses = poses.Poses('Previous Localizer', '')
    loc_poses.times = ekf_poses.times[0:val]
    loc_poses.positions.xs = ekf_poses.positions.xs[0:val]
    loc_poses.positions.ys = ekf_poses.positions.ys[0:val]
    loc_poses.positions.zs = ekf_poses.positions.zs[0:val]
    loc_poses.orientations.yaws = ekf_poses.orientations.yaws[0:val]
    loc_poses.orientations.pitches = ekf_poses.orientations.pitches[0:val]
    loc_poses.orientations.rolls = ekf_poses.orientations.rolls[0:val]
    gt_poses = poses.Poses('Ground truth', '')
    gt_poses.times = sparse_mapping_poses.times[0:gt_val2]
    gt_poses.positions.xs = sparse_mapping_poses.positions.xs[0:gt_val2]
    gt_poses.positions.ys = sparse_mapping_poses.positions.ys[0:gt_val2]
    gt_poses.positions.zs = sparse_mapping_poses.positions.zs[0:gt_val2]
    gt_poses.orientations.yaws = sparse_mapping_poses.orientations.yaws[0:gt_val2]
    gt_poses.orientations.pitches = sparse_mapping_poses.orientations.pitches[0:gt_val2]
    gt_poses.orientations.rolls = sparse_mapping_poses.orientations.rolls[0:gt_val2]

    filename = 'ekf_im_' + str(val + gt_val2).zfill(6) + '.png'
    #with PdfPages(filename) as pdf:
    pdf = 0
    add_graph_plots(pdf, gt_poses, loc_poses, loc_poses, filename)
 
    
  os.system("ffmpeg -r 6 -i ekf_im_%*.png -vcodec mpeg4 -y movie.mp4")

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('bagfile')
  parser.add_argument('--output-file', default='output.pdf')
  parser.add_argument('-g', '--groundtruth-bagfile', default=None)
  args = parser.parse_args()
  if not os.path.isfile(args.bagfile):
    print('Bag file ' + args.bagfile + ' does not exist.')
    sys.exit()
  if args.groundtruth_bagfile and not os.path.isfile(args.groundtruth_bagfile):
    print('Groundtruth Bag file ' + args.groundtruth_bagfile + ' does not exist.')
    sys.exit()
  create_plots(args.bagfile, args.output_file, args.groundtruth_bagfile)
