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
Adds more images to an existing map file. Extracts then sequentially matches features in new images before matching to existing map using vocab_tree matching. The new images are then added to the map using the reconstruction process.
"""

import argparse
import os
import shutil
import subprocess
import sys

import utilities as ut

def merged_database_name(database_path_a, database_path_b):
    database_a_name = os.path.splitext(os.path.basename(database_path_a))[0]
    database_b_name = os.path.splitext(os.path.basename(database_path_b))[0]
    merged_database_name = database_a_name + "." + database_b_name + ".db" 
    return merged_database_name 

def merge_image_directories(image_directory_a, image_directory_b):
    merged_directory = "merged_images"
    os.mkdir(merged_directory)
    merged_image_directory_a = os.path.join(merged_directory, os.path.basename(image_directory_a))  
    merged_image_directory_b = os.path.join(merged_directory, os.path.basename(image_directory_b))  
    os.symlink(image_directory_a, merged_image_directory_a)
    os.symlink(image_directory_b, merged_image_directory_b)
    return merged_directory
    
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "image_directory",
        help="Directory containing new images to add. Images are assumed to be named in sequential order.",
    )
    parser.add_argument(
        "output_directory",
        help="Output directory to save merged mapping results to.",
    )
    parser.add_argument(
        "mapper_ini_file",
        help="Mapper.ini file for existing map.",
    )
    parser.add_argument(
        "config_path",
        help="Full path to astrobee/src/astrobee directory location, e.g. ~/astrobee/src/astrobee.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.image_directory):
        print("Image directory " + args.image_directory + " does not exist.")
        sys.exit()

    if not os.path.isfile(args.mapper_ini_file):
        print("Mapper.ini file " + args.mapper_ini_file + " does not exist.")
        sys.exit()

    if os.path.isdir(args.output_directory):
        print("Output directory already exists.")
        sys.exit()

    database_path_a = ut.read_value(args.mapper_ini_file, "database_path")
    if not database_path_a:
        print("Failed to read database a path.")
        sys.exit()

    image_directory_a = ut.read_value(args.mapper_ini_file, "image_path")
    if not image_directory_a:
        print("Failed to read image a path.")
        sys.exit()


    import_path_a = ut.read_value(args.mapper_ini_file, "output_path")
    if not import_path_a:
        print("Failed to read import a path.")
        sys.exit()
    # Colmap saves to a directory "0" in the output directory
    import_path_a = os.path.join(import_path_a, "0")

    # Get sequential matches for new images
    image_directory_b = os.path.abspath(args.image_directory)
    database_path_b = image_directory_b + ".db"
    ut.create_database(database_path_b)
    ut.extract_features(image_directory_b, database_path_b, args.config_path)
    ut.sequential_match_features(image_directory_b, database_path_b, args.config_path)

    # Merge database and images with existing map, match new images to existing map
    merged_database = merged_database_name(database_path_a, database_path_b)
    ut.merge_databases(database_path_a, database_path_b, merged_database)
    merged_image_directory = merge_image_directories(image_directory_a, image_directory_b)
    ut.vocab_match_features(merged_image_directory, merged_database, args.config_path)

    # Grow map
    # TODO: put the following in a function! (C)
    ## Colmap saves results to a '0' directory
    #merged_import_path = os.path.join(args.output_directory, "merged_mapping_results")
    #merged_import_path_with_0 = os.path.join(merged_import_path, "0")
    #shutil.copytree(import_path_a, merged_import_path_with_0)
    ##ut.grow_map(merged_import_path_with_0, merged_image_directory, merged_database, args.config_path)
