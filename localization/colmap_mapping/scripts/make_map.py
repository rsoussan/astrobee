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

import colmap_utilities as cu 
import file_utilities as fu

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "image_directory",
        help="Directory containing images. Images are assumed to be named in sequential order and stored as /path/to/image_directory/sequence_number/*jpg. By default each sequence_number subdirectory will be used for mapping. If only certain sequences should be used, pass the sequence_numbers explicity using the --sequence_number option.",
    )
    parser.add_argument(
        "config_path",
        help="Full path to astrobee/src/astrobee directory location, e.g. ~/astrobee/src/astrobee.",
    )
    parser.add_argument(
        "-s",
        "--sequence_numbers",
        nargs="*",
        help="Sequence numbers to use for mapping. If none provided, each sequence in the image directory will be used.",
        default=[],
    )

    args = parser.parse_args()

    if not os.path.isdir(args.image_directory):
        print("Image directory " + args.image_directory + " does not exist.")
        sys.exit()
    image_directory_absolute_path = os.path.abspath(args.image_directory)
   
    if args.sequence_numbers:
        fu.check_sequence_numbers(args.image_directory, args.sequence_numbers)
    else:
        args.sequence_numbers = fu.get_sequence_numbers(args.image_directory)

    output_directory = fu.get_mapping_directory_name([args.image_directory], [args.sequence_numbers])

    if os.path.isdir(output_directory):
        print("Output directory " + output_directory + " already exists.")
        sys.exit()
    os.mkdir(output_directory)
    os.chdir(output_directory)

    # Setup directories necessary for mapping, maintain directory structure of image_directory/sequence_number/*jpg
    # TODO(rsoussan): Avoid copying images and use simlinks if issue in colmap fixed (doesn't find simlinks)
    tmp_parent_image_directory = os.path.basename(os.path.dirname(image_directory_absolute_path))
    os.mkdir(tmp_parent_image_directory)
    fu.copy_image_directories(tmp_parent_image_directory, [image_directory_absolute_path], [args.sequence_numbers])
    fu.save_image_directories_to_sequence_numbers([os.path.basename(args.image_directory)], [args.sequence_numbers], "image_sequences.txt")


    # Run mapping relative to parent_image_directory, so each project file saves the image path relative to parent_image_directory
    database_path = output_directory + ".db"
    cu.create_database(database_path)
    cu.extract_features(tmp_parent_image_directory, database_path, args.config_path)
    cu.sequential_match_features(database_path, args.config_path)
    cu.build_sparse_map(tmp_parent_image_directory, database_path, args.config_path)

    # Remove temporary directory used for map creation
    shutil.rmtree(tmp_parent_image_directory)
