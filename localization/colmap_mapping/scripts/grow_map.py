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
Adds more images to an existing map file. Extracts then sequentially matches features in new images before matching to existing map using vocab_tree matching. The new images are then combined with the existing map using a reconstruction process and a new map is saved.
"""

import argparse
import os
import shutil
import subprocess
import sys

import colmap_utilities as cu 
import file_utilities as fu

#def merged_database_name(database_path_a, database_path_b):
#    database_a_name = os.path.splitext(os.path.basename(database_path_a))[0]
#    database_b_name = os.path.splitext(os.path.basename(database_path_b))[0]
#    merged_database_name = database_a_name + "." + database_b_name + ".db" 
#    return merged_database_name 
#
#def merge_image_directories(image_directory_a, image_directory_b):
#    merged_directory = "merged_images"
#    os.mkdir(merged_directory)
#    merged_image_directory_a = os.path.join(merged_directory, os.path.basename(image_directory_a))  
#    merged_image_directory_b = os.path.join(merged_directory, os.path.basename(image_directory_b))  
#    os.symlink(image_directory_a, merged_image_directory_a)
#    os.symlink(image_directory_b, merged_image_directory_b)
#    return merged_directory
#
#def copy_import_path(output_directory, import_path_a): 
#    ## Colmap saves results to a '0' directory
#    merged_import_path = os.path.join(output_directory, "merged_mapping_results")
#    merged_import_path_with_0 = os.path.join(merged_import_path, "0")
#    shutil.copytree(import_path_a, merged_import_path_with_0)
#    return merged_import_path_with_0
 
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "image_directory",
        help="Directory containing images. Images are assumed to be named in sequential order and stored as /path/to/image_directory/sequence_number/*jpg. By default each sequence_number subdirectory will be used for mapping. If only certain sequences should be used, pass the sequence_numbers explicity using the --sequence_number option.",
    )
    parser.add_argument(
        "mapping_directory",
        help="Directory containing the existing map. The directory should contain the map database file and map directory, and is named as image_directory.a.b.c_mapping, where a/b/c are sequence numbers from the map's image_directory used for map creation.",
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

    if not os.path.isdir(args.mapping_directory):
        print("Mapping directory " + args.mapping_directory + " does not exist.")
        sys.exit()

    if args.sequence_numbers:
        fu.check_sequence_numbers(args.image_directory, args.sequence_numbers)
    else:
        args.sequence_numbers = fu.get_sequence_numbers(args.image_directory)

    # Get mapping files from mapping directory
    mapping_directory_basename = os.path.basename(args.mapping_directory)
    mapping_database_path = os.path.abspath(os.path.join(args.mapping_directory, mapping_directory_basename + ".db")) 
    if not os.path.isfile(mapping_database_path):
        print("Failed to find mapping database path.")
        sys.exit()

    mapping_import_path = os.path.abspath(os.path.join(args.mapping_directory, "map")) 
    if not os.path.isdir(mapping_import_path):
        print("Failed to find mapping import path.")
        sys.exit()

    mapping_image_sequences_file = os.path.abspath(os.path.join(args.mapping_directory, "image_sequences.txt"))
    if not os.path.isfile(mapping_image_sequences_file):
        print("Failed to find mapping images sequence file.")
        sys.exit()

    mapping_image_directories, mapping_sequence_numbers_list = fu.image_sequences(mapping_image_sequences_file)
    fu.check_multiple_sequence_numbers(mapping_image_directories, mapping_sequence_numbers_list)

    image_directories = mapping_image_directories[:]
    image_directories.append(args.image_directory)
    sequence_numbers_list = mapping_sequence_numbers_list[:]
    sequence_numbers_list.append(args.sequence_numbers)
    output_directory = fu.get_mapping_directory_name(image_directories, sequence_numbers_list)
    print(output_directory)
    sys.exit()

    # TODO: setup directory hierachy! copy/share code with make_maps.py!! (C)


    # TODO: put these back! (C)
#    # Get sequential matches for new images
#    image_directory_b = os.path.abspath(args.image_directory)
#    database_path_b = image_directory_b + ".db"
#    ut.create_database(database_path_b)
#    ut.extract_features(image_directory_b, database_path_b, args.config_path)
#    ut.sequential_match_features(database_path_b, args.config_path)
#
#    # Merge database and images with existing map, match new images to existing map
#    merged_database = merged_database_name(mapping_database_path, database_path_b)
#    ut.merge_databases(mapping_database_path, database_path_b, merged_database)
#    merged_image_directory = merge_image_directories(image_directory_a, image_directory_b)
#    ut.vocab_match_features(os.path.abspath(merged_database), args.config_path)
#
#    # Grow map
#    merged_import_path = copy_import_path(args.output_directory, mapping_import_path) 
#    ut.grow_map(merged_import_path, merged_image_directory, os.path.abspath(merged_database), args.config_path)

    # Remove temporary directory used for map creation
#    shutil.rmtree(tmp_parent_image_directory)
