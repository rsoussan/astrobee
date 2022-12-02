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

import utilities as ut

def image_directories_to_sequences(mapping_directory_name):
    

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
        for sequence_number in args.sequence_numbers:
            if not os.path.isdir(os.path.join(args.image_directory, sequence_number)):
                print("Sequence number " + sequence_number + " subdirectory does not exist.")
                sys.exit()

    # Get mapping files from mapping directory
    mapping_directory_basename = os.path.basename(args.mapping_directory)
    database_path_a = os.path.join(args.mapping_directory, mapping_directory_basename + ".db") 
    print(database_path_a)
    if not os.path.isfile(database_path_a):
        print("Failed to find database path a.")
        sys.exit()

    import_path_a = os.path.join(args.mapping_directory, "map") 
    print(import_path_a)
    if not os.path.isdir(import_path_a):
        print("Failed to find import path a.")
        sys.exit()

    sys.exit() 

    # TODO: create this function!
    mapping_image_directories_to_sequences = image_directories_to_sequences(mapping_directory_basename)
    # TODO: create this function! unify with make_map.py!
    verify_sequences_exist(mapping_image_directories_to_sequences)


    # TODO: get image directory for each sequence using mapping dir name!!!
    # TODO: save mapping name with sequences, etc elsewhere???
    #image_directory_a = ... 
    #if not os.path.isfile(image_directory_a):
    #    print("Failed to find image directory a.")
    #    sys.exit()




#    output_directory = os.path.basename(args.image_directory) 
#    if not args.sequence_numbers:
#        # TODO: add function that does this, unify with make_map.py
#        subdirs = subdirectories(args.image_directory)
#        for subdirectory in subdirs:
#            if subdirectory.isdigit():
#                args.sequence_numbers.append(subdirectory)
#
#    # TODO: combine with make_map.py!
#    # Output directory should for example be image_directory.0.1.3_mapping, when runs 0, 1, and 3 are included 
#    args.sequence_numbers.sort(key=int)
#    for sequence_number in args.sequence_numbers:
#        output_directory += "." + sequence_number
#    output_directory += "_mapping"
#
#    # TODO: combine with make_map.py??
#    if os.path.isdir(output_directory):
#        print("Output directory " + output_directory + " already exists.")
#        sys.exit()
#    image_directory_absolute_path = os.path.abspath(args.image_directory)
#    os.mkdir(output_directory)
#    os.chdir(output_directory)

    # TODO: combine with make_map.py?
   # Setup directories necessary for mapping, maintain directory structure of image_directory/sequence_number/*jpg
    # TODO(rsoussan): Avoid copying images and use simlinks if issue in colmap fixed (doesn't find simlinks)
    tmp_parent_image_directory = os.path.basename(os.path.dirname(image_directory_absolute_path))
    tmp_image_directory = os.path.join(tmp_parent_image_directory, os.path.basename(args.image_directory))
    os.mkdir(tmp_parent_image_directory)
    os.mkdir(tmp_image_directory)
    for sequence_number in args.sequence_numbers:
        shutil.copytree(os.path.join(image_directory_absolute_path, sequence_number), os.path.join(tmp_image_directory, sequence_number))


    # TODO: make new directory using combined names! share code with make_map???
        # Add function to create name using a unordered map from image dirs to list of sorted sequences!

#    # Get sequential matches for new images
#    image_directory_b = os.path.abspath(args.image_directory)
#    database_path_b = image_directory_b + ".db"
#    ut.create_database(database_path_b)
#    ut.extract_features(image_directory_b, database_path_b, args.config_path)
#    ut.sequential_match_features(database_path_b, args.config_path)
#
#    # Merge database and images with existing map, match new images to existing map
#    merged_database = merged_database_name(database_path_a, database_path_b)
#    ut.merge_databases(database_path_a, database_path_b, merged_database)
#    merged_image_directory = merge_image_directories(image_directory_a, image_directory_b)
#    ut.vocab_match_features(os.path.abspath(merged_database), args.config_path)
#
#    # Grow map
#    merged_import_path = copy_import_path(args.output_directory, import_path_a) 
#    ut.grow_map(merged_import_path, merged_image_directory, os.path.abspath(merged_database), args.config_path)

    # Remove temporary directory used for map creation
#    shutil.rmtree(tmp_parent_image_directory)
