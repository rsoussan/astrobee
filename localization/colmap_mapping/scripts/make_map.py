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
Extracts features, applies sequential mapping with loop closures, and creates a sparse map using colmap
given a set of sequential images. 
"""

import argparse
import os
import shutil
import subprocess
import sys

import localization_common.utilities as lu

#def extract_features(images_directory):

# TODO: unify these with loc analysis, move to loc utils! (added separator, test param sweep)
def check_and_fill_line(value_map, config_file_line, separator):
    line_strings = config_file_line.split(separator)
    # Overwrite val if config variable is in value map
    if len(line_strings) > 0 and line_strings[0] in value_map:
        return line_strings[0] + separator + str(value_map[line_strings[0]]) + "\n"
    return config_file_line


def fill_in_values(original_config, value_map, new_config, separator):
    original_config_file = open(original_config, "r")
    new_config_file = open(new_config, "w")
    for config_file_line in original_config_file:
        new_config_file.write(check_and_fill_line(value_map, config_file_line, separator))


def make_value_map(values, value_names):
    value_map = {}
    if len(values) != len(value_names):
        print("values and value_names not same length!")
        exit()

    for index, value_name in enumerate(value_names):
        value_map[value_name] = values[index]

    return value_map


def make_config(values, value_names, original_config, new_config, separator):
    value_map = make_value_map(values, value_names)
    fill_in_values(original_config, value_map, new_config, separator)

def create_project_file(images_directory, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_project.ini") 
   project_file = images_directory + "_project.ini"
   database_path = images_directory + ".db"
   value_names = ["database_path", "image_path"]
   values = [database_path, images_directory]
   make_config(values, value_names, base_project_file, project_file, "=")
   return project_file
  
def create_database(project_file):
    command = "colmap database_creator --project_path " + project_file
    subprocess.call(command, shell=True)
      

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "image_directory",
        help="Directory containing images. Images are assumed to be named in sequential order.",
    )
    parser.add_argument(
        "config_path",
        help="Full path to astrobee/src/astrobee directory location, e.g. ~/astrobee/src/astrobee.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.image_directory):
        print("Image directory " + args.image_directory + " does not exist.")
        sys.exit()

    image_directory = os.path.abspath(args.image_directory)
    project_file = create_project_file(image_directory, args.config_path)
    create_database(project_file)
    #extract_features(project_file)
    #match_features(project_file)
    #build_sparse_map(project_file)
