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

#import environment
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped 
from ff_msgs.msg import GraphState

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
from numpy.linalg import norm
import scipy.spatial.transform


class EkfLog(object):
  def __init__(self, filename, start_time=float('-inf'), end_time=float('inf')):
    self.ekf = {'t': [], 'x': [], 'y': [], 'z': [], 'angle1': [], 'angle2': [], 'angle3': [], \
            'vx': [], 'vy': [], 'vz': [], 'ox': [], 'oy': [], 'oz': [], 'ax': [], 'ay': [], 'az': [], \
            'abx': [], 'aby': [], 'abz': [], 'gbx': [], 'gby': [], 'gbz': []}
    f = open(filename, 'r')
    for l in f:
      p = l.split(' ')
      if l.startswith('EKF '):
        t = float(p[1])
        if t < start_time or t > end_time:
          print('bad time!')
          continue
        self.ekf['t'].append(float(p[1]))
        self.ekf['x'].append(float(p[2]))
        self.ekf['y'].append(float(p[3]))
        self.ekf['z'].append(float(p[4]))
        self.ekf['angle1'].append(float(p[5]) * 180 / math.pi)
        self.ekf['angle2'].append(float(p[6]) * 180 / math.pi)
        self.ekf['angle3'].append(float(p[7]) * 180 / math.pi)
        self.ekf['vx'].append(float(p[8]))
        self.ekf['vy'].append(float(p[9]))
        self.ekf['vz'].append(float(p[10]))
        self.ekf['ox'].append(float(p[11]) * 180 / math.pi)
        self.ekf['oy'].append(float(p[12]) * 180 / math.pi)
        self.ekf['oz'].append(float(p[13]) * 180 / math.pi)
        self.ekf['ax'].append(float(p[14]))
        self.ekf['ay'].append(float(p[15]))
        self.ekf['az'].append(float(p[16]))
        self.ekf['abx'].append(float(p[17]))
        self.ekf['aby'].append(float(p[18]))
        self.ekf['abz'].append(float(p[19]))
        self.ekf['gbx'].append(float(p[20]) * 180 / math.pi)
        self.ekf['gby'].append(float(p[21]) * 180 / math.pi)
        self.ekf['gbz'].append(float(p[22]) * 180 / math.pi)


  def save_poses(self, bag):
    print(len(self.ekf['t']))
    for i in range(len(self.ekf['t'])):
      msg = PoseStamped() 
      msg.pose.position.x = self.ekf['x'][i]
      msg.pose.position.y = self.ekf['y'][i]
      msg.pose.position.z = self.ekf['z'][i]
      euler_angles = [self.ekf['angle1'][i], self.ekf['angle2'][i], self.ekf['angle3'][i]]
      quaternion = scipy.spatial.transform.Rotation.from_euler('ZYX', euler_angles, degrees=True).as_quat()
      msg.pose.orientation.x = quaternion[0] 
      msg.pose.orientation.y = quaternion[1]
      msg.pose.orientation.z = quaternion[2]
      msg.pose.orientation.w = quaternion[3]
      msg.header.stamp = rospy.Time.from_sec(self.ekf['t'][i])
      bag.write('ekf_pose', msg)

      ekf_msg = GraphState()
      ekf_msg.header.stamp = rospy.Time.from_sec(self.ekf['t'][i])
      ekf_msg.header.frame_id = 'world'
      ekf_msg.child_frame_id = 'body'
      ekf_msg.pose = msg.pose
      ekf_msg.velocity.x = self.ekf['vx'][i]
      ekf_msg.velocity.y = self.ekf['vy'][i]
      ekf_msg.velocity.z = self.ekf['vz'][i]
     # ekf_msg.omega.x = self.ekf['ox'][i]
     # ekf_msg.omega.y = self.ekf['oy'][i]
     # ekf_msg.omega.z = self.ekf['oz'][i]
     # ekf_msg.accel.x = self.ekf['ax'][i]
     # ekf_msg.accel.y = self.ekf['ay'][i]
     # ekf_msg.accel.z = self.ekf['az'][i]
      ekf_msg.accel_bias.x = self.ekf['abx'][i]
      ekf_msg.accel_bias.y = self.ekf['aby'][i]
      ekf_msg.accel_bias.z = self.ekf['abz'][i]
      ekf_msg.gyro_bias.x = self.ekf['gbx'][i]
      ekf_msg.gyro_bias.y = self.ekf['gby'][i]
      ekf_msg.gyro_bias.z = self.ekf['gbz'][i]
      bag.write('ekf_ekf_msg', ekf_msg)
    bag.close()
    

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('txtfile')
  args = parser.parse_args()
  if not os.path.isfile(args.txtfile):
    print('txtfile ' + args.txtfile + ' does not exist.')
    sys.exit()

  bagfile = os.path.splitext(args.txtfile)[0] + '_results.bag'
  bag = rosbag.Bag(bagfile, 'w')

  log = EkfLog(args.txtfile)
  log.save_poses(bag)
