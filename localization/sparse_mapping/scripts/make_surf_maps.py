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
Generate surf maps in parallel for each bagfile in a directory.
See make_surf_map.py for details.
"""

import argparse
import multiprocessing
import os
import sys

import localization_common.utilities as lu


class Params(object):
    def __init__(
        self,
        bagfile,
        image_topic,
        world,
        robot_name,
    ):
        self.bagfile = bagfile 
        self.image_topic = image_topic
        self.world = world
        self.robot_name = robot_name


# Add traceback so errors are forwarded, otherwise
# some errors are suppressed due to the multiprocessing
# library call
@lu.full_traceback
def make_surf_map(params):
    make_surf_map_command = (
        "rosrun sparse_mapping make_surf_map.py "
        + params.bagfile
        + " -w "
        + params.world
        + " -i "
        + params.image_topic
        + " -r "
        + params.robot_name
    )
    output_file = lu.basename(params.bagfile) + "_surf_map.txt"
    lu.run_command_and_save_output(make_surf_map_command, output_file)


def make_surf_maps(params_list, num_processes):
    pool = multiprocessing.Pool(num_processes)
    pool.map(make_surf_map, params_list)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--bagfiles",
        nargs="*",
        help="List of bagfiles to create maps for. If none provided, all bagfiles in the current directory are used.",
    )
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/mgt/img_sampler/nav_cam/image_record",
        help="Image topic.",
    )
    parser.add_argument("-w", "--world", default="iss", help="World name (iss or granite).")
    parser.add_argument("-r", "--robot-name", default="bumble", help="Robot name.")
    parser.add_argument("-m", "--map-name", default=None, help="Output map name. Defaults to bag_name_surf.map")
    parser.add_argument(
        "-p",
        "--num-processes",
        type=int,
        default=10,
        help="Number of concurrent processes to run, where each groundtruth creation job is assigned to one process.",
    )
    args = parser.parse_args()
    if args.bagfiles:
        for bagfile in args.bagfiles:
            if not os.path.isfile(args.bagfile):
                print(("Bag file " + args.bagfile + " does not exist."))
                sys.exit()
    bagfiles = args.bagfiles if args.bagfiles else glob.glob("*.bag")
    params_list = []
    for bagfile in bagfiles:
        params_list.append(Params(bagfile, args.image_topic, args.world, args.robot_name))
    make_surf_maps(params_list, args.num_processes)
