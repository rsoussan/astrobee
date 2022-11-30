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
Adds more images to an existing map file.
"""

import argparse
import os
import shutil
import subprocess
import sys

import utilities as ut

#TODO: move to utils?
def vocab_match_features(image_directory, database_path, config_path):
   base_project_file = os.path.join(os.path.dirname(config_path), "localization/colmap_mapping/files/base_vocab_matcher.ini") 
   project_file = image_directory + "_vocab_matcher.ini"
   value_names = ["database_path", "image_path"]
   values = [database_path, image_directory]
   make_config(values, value_names, base_project_file, project_file, "=")
   command = "colmap vocab_tree_matcher --project_path " + project_file
   lu.run_command_and_save_output(command, "vocab_matcher.txt")
      
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

    if os.path.isdir("mapping_results"):
        print("Directory mapping_results already exists.")
        sys.exit()

    image_directory = os.path.abspath(args.image_directory)
    database_path = image_directory + ".db"
    ut.create_database(database_path)
    ut.extract_features(image_directory, database_path, args.config_path)
    ut.sequential_match_features(image_directory, database_path, args.config_path)
    # TODO: add merge database to utils!
    vocab_match_features(image_directory, database_path, args.config_path)
    #TODO: add option to merge images!
    #TODO: pass merged images directory and merged database!
    # TODO: add grow map function that also takes existing map files!
    #build_sparse_map(image_directory, database_path, args.config_path)
