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
import os
import shutil
import subprocess
import sys

import localization_common.utilities as lu

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

def create_database(database_path):
    command = "colmap database_creator --database_path " + database_path
    lu.run_command_and_save_output(command, "database_creation.txt")

def merge_databases(database_a, database_b, merged_database):
    command = "colmap database_merger --database_path1 " + database_a + " --database_path2 " + database_b + " --merged_database_path " + merged_database
    lu.run_command_and_save_output(command, "database_merge.txt")

def extract_features(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_feature_extractor.ini") 
   project_file = image_directory + "_feature_extractor.ini"
   value_names = ["database_path", "image_path"]
   values = [database_path, image_directory]
   make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap feature_extractor --project_path " + project_file
   lu.run_command_and_save_output(command, "feature_extraction.txt")

def sequential_match_features(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_sequential_matcher.ini") 
   project_file = image_directory + "_sequential_matcher.ini"
   value_names = ["database_path", "image_path"]
   values = [database_path, image_directory]
   make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap sequential_matcher --project_path " + project_file
   lu.run_command_and_save_output(command, "sequential_matcher.txt")

def vocab_match_features(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_vocab_matcher.ini") 
   project_file = image_directory + "_vocab_matcher.ini"
   value_names = ["database_path", "image_path"]
   values = [database_path, image_directory]
   make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap vocab_tree_matcher --project_path " + project_file
   lu.run_command_and_save_output(command, "vocab_matcher.txt")

def build_sparse_map(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_mapper.ini") 
   project_file = image_directory + "_mapper.ini"
   results_directory = "mapping_results"
   os.mkdir(results_directory)
   value_names = ["database_path", "image_path", "output_path"]
   values = [database_path, image_directory, results_directory] 
   make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap mapper --project_path " + project_file
   lu.run_command_and_save_output(command, "mapper.txt")

def grow_map(merged_import_path, merged_image_directory, merged_database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_mapper.ini") 
   project_file = image_directory + "_merged_mapper.ini"
   value_names = ["database_path", "image_path", "output_path"]
   values = [database_path, image_directory, merged_import_path] 
   make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap mapper --project_path " + project_file
   lu.run_command_and_save_output(command, "merged_mapper.txt")

# TODO: move this to loc common!
def read_value(config_filename, value_name):
    config_file = open(config_filename, "r")
    value = None
    for config_file_line in config_file:
        line_strings = config_file_line.split("=")
        if len(line_strings) > 0 and line_strings[0] == value_name:
            # Remove trailing newline character if it exists
            value = line_strings[1].rstrip("\n")

    return value
