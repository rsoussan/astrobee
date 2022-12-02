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
View the sparse map using the colmap gui.
"""

import argparse
import os
import subprocess
import sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "mapping_directory",
        help="Mapping directory used to create the sparse map. Assumes a directory structure of /path/to/parent/map_dir/ containing a database file map_dir.db and map directory map/ and that image directories are contained in /path/to/parent.",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.mapping_directory):
        print("Mapping directory " + args.mapping_directory + " does not exist.")
        sys.exit()

    mapping_directory_absolute_path = os.path.abspath(args.mapping_directory)

    map_path = os.path.join(mapping_directory_absolute_path, "map") 

    database_path = os.path.join(mapping_directory_absolute_path, os.path.basename(args.mapping_directory) + ".db") 
 
    image_path = os.path.dirname(mapping_directory_absolute_path)
 
    command = 'colmap gui --import_path ' + map_path + ' --database_path ' + database_path + ' --image_path ' + image_path
    subprocess.call(command, shell=True)
