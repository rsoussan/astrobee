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

import poses
import loc_states

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

import math
import rosbag
import geometry_msgs


def plot_vals(x_axis_vals,
              vec_of_y_axis_vals,
              labels,
              colors,
              linewidth=1,
              linestyle='-',
              marker=None,
              markeredgewidth=None,
              markersize=1):
  for index, _ in enumerate(vec_of_y_axis_vals):
    plt.plot(x_axis_vals,
             vec_of_y_axis_vals[index],
             colors[index],
             linestyle=linestyle,
             linewidth=linewidth,
             marker=marker,
             markeredgewidth=markeredgewidth,
             markersize=markersize,
             label=labels[index])


def plot_vector3ds(vector3ds,
                   times,
                   label,
                   colors=['r', 'g', 'b'],
                   linewidth=1,
                   linestyle='-',
                   marker=None,
                   markeredgewidth=None,
                   markersize=1):
  labels = [label + ' (X)', label + ' (Y)', label + ' (Z)']
  plot_vals(times, [vector3ds.xs, vector3ds.ys, vector3ds.zs], labels, colors, linewidth, linestyle, marker,
            markeredgewidth, markersize)


def plot_positions(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  plot_vector3ds(poses.positions,
                 poses.times,
                 'Pos.',
                 linewidth=linewidth,
                 linestyle=linestyle,
                 marker=marker,
                 markeredgewidth=markeredgewidth,
                 markersize=markersize)


def plot_orientations(poses, colors, linewidth=1, linestyle='-', marker=None, markeredgewidth=None, markersize=1):
  labels = [
    poses.pose_type + ' Orientation (Yaw)', poses.pose_type + ' Orientation (Roll)',
    poses.pose_type + 'Orienation (Pitch)'
  ]
  plot_vals(poses.times, [poses.orientations.yaws, poses.orientations.rolls, poses.orientations.pitches], labels,
            colors, linewidth, linestyle, marker, markeredgewidth, markersize)


def add_pose_plots(pdf, sparse_mapping_poses, graph_localization_poses, imu_augmented_graph_localization_poses):
  colors = ['r', 'b', 'g']
  plt.figure()
  plot_positions(sparse_mapping_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_positions(graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_orientations(sparse_mapping_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_orientations(graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Imu Augmented Loc vs. Loc
  plt.figure()
  plot_positions(graph_localization_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_positions(imu_augmented_graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Position (m)')
  plt.title('Position')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # orientations
  plt.figure()
  plot_orientations(graph_localization_poses, colors, marker='o', markeredgewidth=0.1, markersize=1.5)
  plot_orientations(imu_augmented_graph_localization_poses, colors, linewidth=0.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Orienation (deg)')
  plt.title('Orientation')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def plot_features(feature_counts, times, label, color, marker, markeredgewidth=0.1, markersize=1.5):
  plt.plot(times,
           feature_counts,
           color,
           marker=marker,
           markeredgewidth=markeredgewidth,
           markersize=markersize,
           label=label)


def add_feature_count_plots(pdf, graph_localization_states):
  plt.figure()
  plot_features(graph_localization_states.of_counts,
                graph_localization_states.times,
                'OF',
                'r',
                marker='x',
                markeredgewidth=0.1,
                markersize=1.5)
  plot_features(graph_localization_states.vl_counts,
                graph_localization_states.times,
                'VL',
                'b',
                marker='o',
                markeredgewidth=0.1,
                markersize=1.5)
  plt.xlabel('Time (s)')
  plt.ylabel('Feature Counts (num features in graph)')
  plt.title('Feature Counts')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def add_other_vector3d_plots(pdf, imu_augmented_graph_localization_states):
  # Acceleration
  plt.figure()
  plot_vector3ds(imu_augmented_graph_localization_states.accelerations, imu_augmented_graph_localization_states.times,
                 'Acc.')
  plt.xlabel('Time (s)')
  plt.ylabel('Acceleration (m/s^2)')
  plt.title('Acceleration')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Biases
  plt.figure()
  plot_vector3ds(imu_augmented_graph_localization_states.accelerometer_biases,
                 imu_augmented_graph_localization_states.times, 'Acc. Bias')
  plot_vector3ds(imu_augmented_graph_localization_states.gyro_biases, imu_augmented_graph_localization_states.times,
                 'Gyro Bias')
  plt.xlabel('Time (s)')
  plt.ylabel('Biases')
  plt.title('Biases')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Angular Velocity
  plt.figure()
  plot_vector3ds(imu_augmented_graph_localization_states.angular_velocities,
                 imu_augmented_graph_localization_states.times, 'Ang. Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Angular Velocities')
  plt.title('Angular Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()

  # Velocity
  plt.figure()
  plot_vector3ds(imu_augmented_graph_localization_states.velocities, imu_augmented_graph_localization_states.times,
                 'Vel.')
  plt.xlabel('Time (s)')
  plt.ylabel('Velocities')
  plt.title('Velocities')
  plt.legend(prop={'size': 6})
  pdf.savefig()
  plt.close()


def add_other_loc_plots(pdf, graph_localization_states, imu_augmented_graph_localization_states):
  add_feature_count_plots(pdf, graph_localization_states)
  add_other_vector3d_plots(pdf, imu_augmented_graph_localization_states)


def load_pose_msgs(vec_of_poses, bag):
  topics = [poses.topic for poses in vec_of_poses]
  for topic, msg, t in bag.read_messages(topics):
    for poses in vec_of_poses:
      if poses.topic == topic:
        poses.add_pose(msg.pose, msg.header.stamp)
        break


def load_loc_state_msgs(vec_of_loc_states, bag):
  topics = [loc_states.topic for loc_states in vec_of_loc_states]
  for topic, msg, t in bag.read_messages(topics):
    for loc_states in vec_of_loc_states:
      if loc_states.topic == topic:
        loc_states.add_loc_state(msg)
        break


def create_plots(bagfile, output_file):
  bag = rosbag.Bag(bagfile)
  sparse_mapping_poses = poses.Poses('Sparse Mapping', 'sparse_mapping_pose')
  vec_of_poses = [sparse_mapping_poses]
  load_pose_msgs(vec_of_poses, bag)

  graph_localization_states = loc_states.LocStates('Graph Localization', 'graph_loc/state')
  imu_augmented_graph_localization_states = loc_states.LocStates('Imu Augmented Graph Localization', 'gnc/ekf')
  vec_of_loc_states = [graph_localization_states, imu_augmented_graph_localization_states]
  load_loc_state_msgs(vec_of_loc_states, bag)

  bag.close()

  with PdfPages(output_file) as pdf:
    add_pose_plots(pdf, sparse_mapping_poses, graph_localization_states, imu_augmented_graph_localization_states)
    add_other_loc_plots(pdf, graph_localization_states, imu_augmented_graph_localization_states)
