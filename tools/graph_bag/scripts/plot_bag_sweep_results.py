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
import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd

import os
import sys


def save_rmse_results_to_csv(rmses, prefix='', rmses_2=None, label_1=None, label_2=None):
  mean_rmses_dataframe = pd.DataFrame()
  labels = []
  if label_1 and label_2 and rmses_2 is not None:
    labels.append(label_1)
    labels.append(label_2)
  if labels:
    mean_rmses_dataframe['Label'] = labels
  mean_rmses_list = []
  mean_rmses_list.append(rmses.mean())
  relative_rmses = []
  relative_change_in_rmses = []
  if rmses_2 is not None:
    mean_rmses_list.append(rmses_2.mean())
    relative_rmses.append(mean_rmses_list[0] / mean_rmses_list[1])
    relative_rmses.append(mean_rmses_list[1] / mean_rmses_list[0])
    relative_change_in_rmses.append(mean_rmses_list[0] / mean_rmses_list[1] - 1.0)
    relative_change_in_rmses.append(mean_rmses_list[1] / mean_rmses_list[0] - 1.0)
    mean_rmses_dataframe['rel_' + prefix + 'rmse_%'] = relative_rmses
    mean_rmses_dataframe['rel_' + prefix + 'rmse_delta_%'] = relative_change_in_rmses
  mean_rmses_dataframe['mean_' + prefix + 'rmse'] = mean_rmses_list
  mean_rmses_csv_file = 'mean_rmses.csv'
  mean_rmses_dataframe.to_csv(mean_rmses_csv_file, index=False, mode='a')
  return mean_rmses_list, labels, relative_rmses, relative_change_in_rmses


def rmse_plots(pdf,
               x_axis_vals,
               shortened_bag_names,
               rmses,
               integrated_rmses,
               orientation_rmses,
               prefix='',
               label_1='',
               rmses_2=None,
               integrated_rmses_2=None,
               orientation_rmses_2=None,
               label_2=''):
  plt.figure()
  plt.plot(x_axis_vals, rmses, 'b', label=label_1, linestyle='None', marker='o', markeredgewidth=0.1, markersize=10.5)
  if rmses_2 is not None:
    plt.plot(x_axis_vals,
             rmses_2,
             'r',
             label=label_2,
             linestyle='None',
             marker='o',
             markeredgewidth=0.1,
             markersize=10.5)
    plt.legend(prop={'size': 8}, bbox_to_anchor=(1.05, 1))
  plt.xticks(x_axis_vals, shortened_bag_names, fontsize=7, rotation=20)
  plt.ylabel(prefix + ' RMSE')
  plt.title(prefix + ' RMSE vs. Bag')
  x_range = x_axis_vals[len(x_axis_vals) - 1] - x_axis_vals[0]
  x_buffer = x_range * 0.1
  # Extend x axis on either side to make data more visible
  plt.xlim([x_axis_vals[0] - x_buffer, x_axis_vals[len(x_axis_vals) - 1] + x_buffer])
  plt.tight_layout()
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(x_axis_vals,
           orientation_rmses,
           'b',
           label=label_1,
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=10.5)
  if orientation_rmses_2 is not None:
    plt.plot(x_axis_vals,
             orientation_rmses_2,
             'r',
             label=label_2,
             linestyle='None',
             marker='o',
             markeredgewidth=0.1,
             markersize=10.5)
    plt.legend(prop={'size': 8}, bbox_to_anchor=(1.05, 1))
  plt.xticks(x_axis_vals, shortened_bag_names, fontsize=7, rotation=20)
  plt.ylabel(prefix + ' Orientation RMSE')
  plt.title(prefix + ' Orientation RMSE vs. Bag')
  x_range = x_axis_vals[len(x_axis_vals) - 1] - x_axis_vals[0]
  x_buffer = x_range * 0.1
  # Extend x axis on either side to make data more visible
  plt.xlim([x_axis_vals[0] - x_buffer, x_axis_vals[len(x_axis_vals) - 1] + x_buffer])
  plt.tight_layout()
  pdf.savefig()
  plt.close()

  plt.figure()
  plt.plot(x_axis_vals,
           integrated_rmses,
           'b',
           label=label_1,
           linestyle='None',
           marker='o',
           markeredgewidth=0.1,
           markersize=10.5)
  if integrated_rmses_2 is not None:
    plt.plot(x_axis_vals,
             integrated_rmses_2,
             'r',
             label=label_2,
             linestyle='None',
             marker='o',
             markeredgewidth=0.1,
             markersize=10.5)
    plt.legend(prop={'size': 8}, bbox_to_anchor=(1.05, 1))
  plt.xticks(x_axis_vals, shortened_bag_names, fontsize=7, rotation=20)
  plt.ylabel(prefix + ' Integrated RMSE')
  plt.title(prefix + ' Integrated RMSE vs. Bag')
  x_range = x_axis_vals[len(x_axis_vals) - 1] - x_axis_vals[0]
  x_buffer = x_range * 0.1
  # Extend x axis on either side to make data more visible
  plt.xlim([x_axis_vals[0] - x_buffer, x_axis_vals[len(x_axis_vals) - 1] + x_buffer])
  plt.tight_layout()
  pdf.savefig()
  plt.close()

  # Plot mean rmses
  mean_rmses, labels, relative_rmses, relative_change_in_rmses = save_rmse_results_to_csv(
    rmses, prefix, rmses_2, label_1, label_2)
  if (prefix):
    prefix += '_'
  mean_integrated_rmses, labels, relative_integrated_rmses, relative_change_in_integrated_rmses = save_rmse_results_to_csv(
    integrated_rmses, prefix + 'integrated_', integrated_rmses_2, label_1, label_2)
  mean_orientation_rmses, labels, relative_orientation_rmses, relative_change_in_orientation_rmses = save_rmse_results_to_csv(
    orientation_rmses, prefix + 'orientation_', orientation_rmses_2, label_1, label_2)

  mean_rmses_1_string = prefix + 'rmse: ' + str(mean_rmses[0])
  mean_integrated_rmses_1_string = prefix + 'integrated rmse: ' + str(mean_integrated_rmses[0])
  mean_orientation_rmses_1_string = prefix + 'orientation rmse: ' + str(mean_orientation_rmses[0])
  if labels:
    mean_rmses_1_string += ', label: ' + labels[0]
  plt.figure()
  plt.axis('off')
  plt.text(0.0, 1.0, mean_rmses_1_string)
  plt.text(0.0, 0.95, mean_orientation_rmses_1_string)
  plt.text(0.0, 0.9, mean_integrated_rmses_1_string)
  if len(mean_rmses) > 1:
    mean_rmses_2_string = prefix + 'rmse: ' + str(mean_rmses[1])
    mean_integrated_rmses_2_string = prefix + 'integrated rmse: ' + str(mean_integrated_rmses[1])
    mean_orientation_rmses_2_string = prefix + 'orientation rmse: ' + str(mean_orientation_rmses[1])
    if labels:
      mean_rmses_2_string += ', label: ' + labels[1]
      plt.text(0.0, 0.85, mean_rmses_2_string)
      plt.text(0.0, 0.8, mean_orientation_rmses_2_string)
      plt.text(0.0, 0.75, mean_integrated_rmses_2_string)
    relative_rmses_string = prefix + 'rel rmse %: ' + str(100 * relative_rmses[0])
    relative_integrated_rmses_string = prefix + 'rel integrated rmse %: ' + str(100 * relative_integrated_rmses[0])
    relative_orientation_rmses_string = prefix + 'rel orientation rmse %: ' + str(100 * relative_orientation_rmses[0])
    plt.text(0.0, 0.7, relative_rmses_string)
    plt.text(0.0, 0.65, relative_orientation_rmses_string)
    plt.text(0.0, 0.6, relative_integrated_rmses_string)
    relative_rmses_change_string = prefix + 'rel change in rmse %: ' + str(100 * relative_change_in_rmses[0])
    relative_orientation_rmses_change_string = prefix + 'rel change in orientation rmse %: ' + str(
      100 * relative_change_in_orientation_rmses[0])
    relative_integrated_rmses_change_string = prefix + 'rel change in integrated rmse %: ' + str(
      100 * relative_change_in_integrated_rmses[0])
    plt.text(0.0, 0.55, relative_rmses_change_string)
    plt.text(0.0, 0.5, relative_orientation_rmses_change_string)
    plt.text(0.0, 0.4, relative_integrated_rmses_change_string)
  pdf.savefig()


def create_plot(output_file, csv_file, label_1='', csv_file_2=None, label_2='', imu_augmented_2=True):
  dataframe = pd.read_csv(csv_file)
  dataframe.sort_values(by=['Bag'], inplace=True)
  # Graph rmses
  rmses = dataframe['rmse']
  integrated_rmses = dataframe['integrated_rmse']
  orientation_rmses = dataframe['orientation_rmse']
  # IMU augmented rmses
  imu_augmented_rmses = dataframe['imu_augmented_rmse']
  imu_augmented_integrated_rmses = dataframe['imu_augmented_integrated_rmse']
  imu_augmented_orientation_rmses = dataframe['imu_augmented_orientation_rmse']

  bag_names = dataframe['Bag'].tolist()
  max_name_length = 45
  shortened_bag_names = [
    bag_name[-1 * max_name_length:] if len(bag_name) > max_name_length else bag_name for bag_name in bag_names
  ]
  x_axis_vals = range(len(shortened_bag_names))
  rmses_2 = None
  integrated_rmses_2 = None
  orientation_rmses_2 = None
  imu_augmented_rmses_2 = None
  imu_augmented_integrated_rmses_2 = None
  imu_augmented_orientation_rmses_2 = None

  if (csv_file_2):
    dataframe_2 = pd.read_csv(csv_file_2)
    dataframe_2.sort_values(by=['Bag'], inplace=True)
    # Graph rmses
    rmses_2 = dataframe_2['rmse']
    integrated_rmses_2 = dataframe_2['integrated_rmse']
    orientation_rmses_2 = dataframe_2['orientation_rmse']
    if (imu_augmented_2):
      # IMU augmented rmses
      imu_augmented_rmses_2 = dataframe_2['imu_augmented_rmse']
      imu_augmented_integrated_rmses_2 = dataframe_2['imu_augmented_integrated_rmse']
      imu_augmented_orientation_rmses_2 = dataframe_2['imu_augmented_orientation_rmse']

    bag_names_2 = dataframe_2['Bag'].tolist()
    if bag_names != bag_names_2:
      print('Bag names for first and second csv file are not the same')
      exit()
  with PdfPages(output_file) as pdf:
    rmse_plots(pdf, x_axis_vals, shortened_bag_names, rmses, integrated_rmses, orientation_rmses, '', label_1, rmses_2,
               integrated_rmses_2, orientation_rmses_2, label_2)
    if imu_augmented_2:
      rmse_plots(pdf, x_axis_vals, shortened_bag_names, imu_augmented_rmses, imu_augmented_integrated_rmses,
                 imu_augmented_orientation_rmses, 'imu_augmented', label_1, imu_augmented_rmses_2,
                 imu_augmented_integrated_rmses_2, imu_augmented_orientation_rmses_2, label_2)
    else:
      rmse_plots(pdf, x_axis_vals, shortened_bag_names, imu_augmented_rmses, imu_augmented_integrated_rmses,
                 imu_augmented_orientation_rmses, 'imu_augmented', label_1, rmses_2, integrated_rmses_2,
                 orientation_rmses_2, label_2 + ' no imu aug')


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  # Combined csv results, where each row is the result from a bag file
  parser.add_argument('csv_file')
  parser.add_argument('--output-file', default='bag_sweep_results.pdf')
  parser.add_argument('--csv-file2', help='Optional second csv file to plot')
  parser.add_argument('--label1', default='', help='Optional label for first csv file')
  parser.add_argument('--label2', default='', help='Optional label for second csv file')
  parser.add_argument('--no-imu-augmented2', dest='imu_augmented2', action='store_false')
  args = parser.parse_args()
  create_plot(args.output_file, args.csv_file, args.label1, args.csv_file2, args.label2, args.imu_augmented2)
