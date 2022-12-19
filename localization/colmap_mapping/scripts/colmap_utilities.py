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
import os
import shutil

import file_utilities as fu
import localization_common.utilities as lu

def create_database(database_path):
    command = "colmap database_creator --database_path " + database_path
    lu.run_command_and_save_output(command, "database_creation.txt")

def merge_databases(database_a, database_b, merged_database):
    command = "colmap database_merger --database_path1 " + database_a + " --database_path2 " + database_b + " --merged_database_path " + merged_database
    lu.run_command_and_save_output(command, "database_merge.txt")

def extract_features(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_feature_extractor.ini") 
   project_file = "feature_extractor.ini"
   value_names = ["database_path", "image_path"]
   values = [database_path, image_directory]
   fu.make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap feature_extractor --project_path " + project_file
   lu.run_command_and_save_output(command, "feature_extraction.txt")

def sequential_match_features(database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_sequential_matcher.ini") 
   project_file = "sequential_matcher.ini"
    # TODO: add vocab file here!!!
   value_names = ["database_path"]
   values = [database_path]
   fu.make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap sequential_matcher --project_path " + project_file
   lu.run_command_and_save_output(command, "sequential_matcher.txt")

def vocab_match_features(database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_vocab_matcher.ini") 
   project_file = "vocab_matcher.ini"
    # TODO: add vocab file here!!!
   value_names = ["database_path"]
   values = [database_path]
   fu.make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap vocab_tree_matcher --project_path " + project_file
   lu.run_command_and_save_output(command, "vocab_matcher.txt")

def build_sparse_map(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_mapper.ini") 
   project_file = "mapper.ini"
   results_directory = "map"
   os.mkdir(results_directory)
   value_names = ["database_path", "image_path", "output_path"]
   values = [database_path, image_directory, results_directory] 
   fu.make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap mapper --project_path " + project_file
   lu.run_command_and_save_output(command, "mapper.txt")
   # Colmap exports map files to a dirctory 0/, move to map 
   shutil.move("map/0", "tmp_map")
   shutil.rmtree("map")
   os.rename("tmp_map", "map")

def grow_map(merged_import_path, merged_image_directory, merged_database_path, output_map, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_mapper.ini") 
   project_file = "merged_mapper.ini"
   value_names = ["database_path", "image_path", "output_path", "input_path"]
   values = [merged_database_path, merged_image_directory, output_map, merged_import_path] 
   fu.make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap mapper --project_path " + project_file
   lu.run_command_and_save_output(command, "merged_mapper.txt")
