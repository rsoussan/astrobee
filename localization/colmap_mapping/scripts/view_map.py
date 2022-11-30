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

import utilities as ut

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "mapping_project_file",
        help="Mapping project file used to create the sparse map, typically ending in mapping.ini.",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.mapping_project_file):
        print("Mapping project file " + args.mapping_project_file + " does not exist.")
        sys.exit()
    

    import_path = ut.read_value(args.mapping_project_file, "output_path")
    if not import_path:
        print("Failed to read import path.")
        sys.exit()
    # Colmap saves to a directory "0" in the output directory
    import_path = os.path.join(import_path, "0")

    database_path = ut.read_value(args.mapping_project_file, "database_path")
    if not database_path:
        print("Failed to read database path.")
        sys.exit()
 
    image_path = ut.read_value(args.mapping_project_file, "image_path")
    if not image_path:
        print("Failed to read image path.")
        sys.exit()
 
    command = 'colmap gui --import_path ' + import_path + ' --database_path ' + database_path + ' --image_path ' + image_path
    subprocess.call(command, shell=True)
