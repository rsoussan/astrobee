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

import utilities as ut

# TODO: unify with prune_partioned_directories, move to loc common utils!!!
def subdirectories(directory):
    subdirectories = []
    try:
        _, subdirectories, _ = next(os.walk(directory))
    except:
        pass
    return subdirectories



if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "image_directory",
        help="Directory containing images. Images are assumed to be named in sequential order and stored as /path/to/image_directory/sequence_number/*jpg. By default each sequence_number subdirectory will be use for mapping. If only certain sequences should be used, pass the sequence_numbers explicity using the --sequence_number option.",
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
    
    if args.sequence_numbers:
        for sequence_number in args.sequence_numbers:
            if not os.path.isdir(os.path.join(args.image_directory, sequence_number)):
                print("Sequence number " + sequence_number + " subdirectory does not exist.")
                sys.exit()

    
    output_directory = os.path.basename(args.image_directory) 
    if not args.sequence_numbers:
        subdirs = subdirectories(args.image_directory)
        for subdirectory in subdirs:
            if subdirectory.isdigit():
                args.sequence_numbers.append(subdirectory)

    args.sequence_numbers.sort(key=int)
    for sequence_number in args.sequence_numbers:
        output_directory += "." + sequence_number

    if os.path.isdir(output_directory):
        print("Output directory " + output_directory + " already exists.")
        sys.exit()
    image_directory_absolute_path = os.path.abspath(args.image_directory)
    os.mkdir(output_directory)
    os.chdir(output_directory)
    # Setup directories necessary for mapping, maintain directory structure of image_directory/sequence_number/*jpg
    # TODO(rsoussan): Avoid copying images and use simlinks if issue in colmap fixed (doesn't find simlinks)
    tmp_parent_image_directory = os.path.basename(os.path.dirname(image_directory_absolute_path))
    tmp_image_directory = os.path.join(tmp_parent_image_directory, os.path.basename(args.image_directory))
    os.mkdir(tmp_parent_image_directory)
    os.mkdir(tmp_image_directory)
    for sequence_number in args.sequence_numbers:
        shutil.copytree(os.path.join(image_directory_absolute_path, sequence_number), os.path.join(tmp_image_directory, sequence_number))


    # Run mapping relative to parent_image_directory, so each project file saves the image path relative to parent_image_directory
    database_path = output_directory + ".db"
    ut.create_database(database_path)
    ut.extract_features(tmp_parent_image_directory, database_path, args.config_path)
    ut.sequential_match_features(database_path, args.config_path)
    ut.build_sparse_map(tmp_parent_image_directory, database_path, args.config_path)

    # Remove temporary directory used for map creation
    shutil.rmtree(tmp_parent_image_directory)
