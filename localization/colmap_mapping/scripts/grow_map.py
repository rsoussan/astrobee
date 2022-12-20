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
#TODO: why isn't database viewable?

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
    image_directory_absolute_path = os.path.abspath(args.image_directory)

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

    # Merge image directory and sequence number lists
    image_directories = mapping_image_directories[:]
    image_directories.append(args.image_directory)
    image_directory_absolute_paths = [os.path.abspath(image_directory) for image_directory in image_directories]
    sequence_numbers_list = mapping_sequence_numbers_list[:]
    sequence_numbers_list.append(args.sequence_numbers)
    output_directory = fu.get_mapping_directory_name(image_directories, sequence_numbers_list)

    # Move to output directory
    if os.path.isdir(output_directory):
         print("Output directory " + output_directory + " already exists.")
         sys.exit()
    os.mkdir(output_directory)
    os.chdir(output_directory)

    # Setup new mapping directory structure
    # Temporary, only to extract and match features for new images
    # Colmap checks for all images recursively, so need to avoid adding mapping image directories here
    tmp_parent_image_directory = fu.setup_mapping_images_directory_structure(output_directory, [image_directory_absolute_path], [args.sequence_numbers])

    # Get sequential matches for new images
    new_database_path = "tmp_new.db"
    cu.create_database(new_database_path)
    cu.extract_features(tmp_parent_image_directory, new_database_path, args.config_path)
    cu.sequential_match_features(new_database_path, args.config_path)

    # Remove temporary directory used for new image extraction/mapping
    shutil.rmtree(tmp_parent_image_directory)

    # Setup merged mapping directory structure
    tmp_parent_image_directory = fu.setup_mapping_images_directory_structure(output_directory, image_directory_absolute_paths, sequence_numbers_list)
    fu.save_image_directories_to_sequence_numbers(image_directories, sequence_numbers_list, "image_sequences.txt")

    # Merge database and images with existing map and match new images to existing map
    merged_basename = os.path.basename(output_directory)
    merged_database = merged_basename + ".db" 
    cu.merge_databases(mapping_database_path, new_database_path, merged_database)
    cu.vocab_match_features(os.path.abspath(merged_database), args.config_path)

    # Remove tmp new database
    os.remove(new_database_path)
    
    # Create location for new map
    output_map_directory = "map"
    os.mkdir(output_map_directory)
    
    # Grow map
    cu.grow_map(mapping_import_path, tmp_parent_image_directory, merged_database, output_map_directory, args.config_path)

    # Remove temporary image directories used for map creation
    shutil.rmtree(tmp_parent_image_directory)
