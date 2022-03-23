#!/usr/bin/env python
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
Slices a bagfile at selected points to create multiple smaller bagfiles, which combined 
span the original bagfile.
"""

import argparse
import os
import sys

import cv2
import rosbag
import rospy
import utilities
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def show_image_with_message(image, window, title, message, origin = (50, 450), font_size = 4, timeout = 1500):
    color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    splice_image = cv2.putText(color_image, message, origin, cv2.FONT_HERSHEY_SIMPLEX, font_size, (0, 0, 255), 10)  
    cv2.imshow(window, splice_image)
    cv2.setWindowTitle(window, title)
    key = cv2.waitKey(timeout)

def splice_bag(bagfile, splice_timestamps):
    print('Splicing!')

def select_splice_timestamps_and_splice_bag(bagfile, image_topic):
    splice_timestamps = []
    bridge = CvBridge()
    with rosbag.Bag(bagfile, "r") as bag:
        msg_tuples = []
        print ("Reading msgs...")
        for topic, msg, t in bag.read_messages([image_topic]):
            msg_tuples.append((msg, t))
        i = 0
        num_msgs = len(msg_tuples)
        window = "image"
        cv2.namedWindow(window)
        cv2.moveWindow(window, 40,30)  
        while i < num_msgs:
            msg = (msg_tuples[i])[0]
            timestamp = ((msg_tuples[i])[1]).to_sec()
            progress = i/float(num_msgs)*100
            msg_info_string = '{0:.2f}'.format(progress) + '%, Image ' + str(i) + '/' + str(num_msgs) + ', t: ' + str(timestamp)
            print(msg_info_string)
            try:
                image = bridge.imgmsg_to_cv2(msg, msg.encoding)
            except (CvBridgeError) as e:
                print(e)
            cv2.imshow(window, image)
            cv2.setWindowTitle(window, msg_info_string)
            key = cv2.waitKey(0) 
            print(str(key))
            if key == ord('n') or key == 83: # Right arrow key
                i += 1
            elif key == ord('p') or key == 81: # Left arrow key 
                i -= 1
            elif key == ord('q') or key == 27: # Escape key
                print('Manually closing program, no splice operation applied.')
                exit(0)
            elif key == ord('s'):
                print('Splice timestamp selected, t: ' + str(timestamp))
                splice_timestamps.append(timestamp)
                show_image_with_message(image, window, 'Splice t selected! ' + msg_info_string, 'Saving splice time') 
            elif key == 13: # Enter key
                if len(splice_timestamps) == 0:
                    message = 'No splice timestamps selected.'
                    print(message)
                    show_image_with_message(image, window, message, message, (40, 450), 2.4) 
                else:
                    print('Splicing bag using selected timestamps.')
                    show_image_with_message(image, window, 'Splicing', 'Splicing', (300, 450)) 
                    print(splice_timestamps)
                    splice_bag(bagfile, splice_timestamps)
                    return 

            if i < 0: 
                i = 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("bagfile", help="Input bagfile.")
    parser.add_argument(
        "-i",
        "--image-topic",
        default="/hw/cam_nav",
        help="Image topic name.",
    )

    args = parser.parse_args()
    if not os.path.isfile(args.bagfile):
        print(("Bag file " + args.bagfile + " does not exist."))
        sys.exit()

    select_splice_timestamps_and_splice_bag(args.bagfile, args.image_topic)
