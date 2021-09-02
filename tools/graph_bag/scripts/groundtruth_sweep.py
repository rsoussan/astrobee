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

import multiprocessing_helpers

import argparse
import csv
import itertools
import multiprocessing
import os
import sys


class GroundtruthParams(object):

  def __init__(self, bagfile, map_file):
    self.bagfile = bagfile
    self.map_file = map_file 


def load_params(param_file):
  groundtruth_params_list = []
  with open(param_file) as param_csvfile:
    reader = csv.reader(param_csvfile, delimiter=' ')
    for row in reader:
      groundtruth_params_list.append(
        GroundtruthParams(row[0], row[1])) 

  return groundtruth_params_list


def check_params(groundtruth_params_list):
  for params in groundtruth_params_list:
    if not os.path.isfile(params.bagfile):
      print('Bagfile ' + params.bagfile + ' does not exist.')
      sys.exit()
    if not os.path.isfile(params.map_file):
      print('Map file ' + params.map_file + ' does not exist.')
      sys.exit()


# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@multiprocessing_helpers.full_traceback
def run_groundtruth(params):
  bag_name = os.path.splitext(os.path.basename(params.bagfile))[0]
  os.mkdir(bag_name)
  os.chdir(bag_name)
  make_groundtruth_command = 'rosrun graph_bag create_groundtruth.py ' + params.bagfile + ' ' + params.map_file 
  print(make_groundtruth_command)
  os.system(make_groundtruth_command)


# Helper that unpacks arguments and calls original function
# Aides running jobs in parallel as pool only supports
# passing a single argument to workers
def run_groundtruth_helper(zipped_vals):
  return run_groundtruth(*zipped_vals)


def groundtruth_sweep(config_file):
  groundtruth_params_list = load_params(config_file)
  check_params(groundtruth_params_list)
  num_processes = 12 
  pool = multiprocessing.Pool(num_processes)
  # izip arguments so we can pass as one argument to pool worker
  pool.map(run_groundtruth_helper, itertools.izip(groundtruth_params_list))


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('config_file')
  args = parser.parse_args()
  if not os.path.isfile(args.config_file):
    print('Config file ' + args.config_file + ' does not exist.')
    sys.exit()
  groundtruth_sweep(args.config_file)
