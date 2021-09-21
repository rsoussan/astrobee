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

import poses
import vector3ds

import matplotlib
matplotlib.use('pdf')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

def unwrap_in_degrees(angles):
  print("unwrapping!")
  return np.rad2deg(np.unwrap(np.deg2rad(angles)))

class Vector3dPlotter():

  def __init__(self, xlabel, ylabel, title, individual_plots):
    self.xlabel = xlabel
    self.ylabel = ylabel
    self.title = title
    self.individual_plots = individual_plots
    self.y_vals_vec = []
    self.color_vec = []

  def add_pose_position(self,
                        pose,
                        color='r', 
                        linestyle='-',
                        linewidth=1,
                        marker=None,
                        markeredgewidth=None,
                        markersize=1):
    position_plotter = Vector3dYVals(pose.pose_type, pose.times, pose.positions.xs, pose.positions.ys,
                                     pose.positions.zs, ['', 'Pos. (Y)', 'Pos. (Z)'], ['r', 'g', 'b'], linestyle,
                                     linewidth, marker, markeredgewidth, markersize)
    self.add_y_vals(position_plotter)
    self.add_color(color)

  def add_pose_orientation(self,
                           pose,
                           colors=['r', 'b', 'g'],
                           linestyle='-',
                           linewidth=1,
                           marker=None,
                           markeredgewidth=None,
                           markersize=1):
    orientation_plotter = Vector3dYVals(pose.pose_type, pose.times, unwrap_in_degrees(pose.orientations.yaws), unwrap_in_degrees(pose.orientations.rolls),
                                        unwrap_in_degrees(pose.orientations.pitches),
                                        ['Orientation (Yaw)', 'Orientation (Roll)', 'Orientation (Pitch)'], colors,
                                        linestyle, linewidth, marker, markeredgewidth, markersize)
    self.add_y_vals(orientation_plotter)

  def add_y_vals(self, y_vals):
    self.y_vals_vec.append(y_vals)

  def add_color(self, color):
    self.color_vec.append(color)



  def set_axes_equal(self, ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

  def plot(self, pdf, filename, individual_plots=True):
    plt.figure()
    ax = plt.figure().add_subplot(111, projection='3d')
    spacing = 13
    font_size =15
    ax.set_xlabel('x (m)', labelpad=spacing, fontsize=font_size)
    ax.set_ylabel('y (m)', labelpad=spacing, fontsize=font_size)
    ax.axes.set_xlim3d(left=10.2, right=11.8) 
    ax.axes.set_ylim3d(bottom=-9, top=-7.2) 
    ax.axes.set_zlim3d(4.2, 5.6) 
    ax.zaxis.set_rotate_label(False)  # disable automatic rotation
    ax.set_zlabel('z (m)', labelpad=spacing, fontsize=font_size)
    #ax.axis('equal')
    color_index = 0
    for y_vals in self.y_vals_vec:
      y_vals.full_plot(ax, self.color_vec[color_index])
      color_index = color_index + 1
    i = 0
    for label in ax.get_xticklabels():
      i += 1
      if (i % 2 == 0):
        label.set_visible(False)
    i = 0
    for label in ax.get_yticklabels():
      i += 1
      if (i % 2 == 0):
        label.set_visible(False)
    i = 0
    for label in ax.get_zticklabels():
      i += 1
      if (i % 2 == 0):
        label.set_visible(False)
    ##ax.set_xticks(ax.get_xticks()[::2])
    ##plt.title(self.title)
    #label_size = 15
    #ax.tick_params(axis='x', labelsize=label_size)
    #ax.tick_params(axis='y', labelsize=label_size)
    #ax.tick_params(axis='z', labelsize=label_size)
    plt.legend(loc=2, prop={'size': 17})
    #self.set_axes_equal(ax)
    plt.savefig(filename, dpi=75)
    plt.close()

    #if individual_plots:
    #  self.plot_xs(pdf)
    #  self.plot_ys(pdf)
    #  self.plot_zs(pdf)

  def plot_xs(self, pdf):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.plot_x()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()

  def plot_ys(self, pdf):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.plot_y()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()

  def plot_zs(self, pdf):
    plt.figure()
    for y_vals in self.y_vals_vec:
      y_vals.plot_z()
    plt.xlabel(self.xlabel)
    plt.ylabel(self.ylabel)
    plt.title(self.title)
    plt.legend(prop={'size': 6})
    pdf.savefig()
    plt.close()


class Vector3dYVals():

  def __init__(self,
               name,
               x_axis_vals,
               x_vals,
               y_vals,
               z_vals,
               labels,
               colors=['r', 'b', 'g'],
               linestyle='-',
               linewidth=1,
               marker=None,
               markeredgewidth=None,
               markersize=1):
    self.x_vals = x_vals
    self.y_vals = y_vals
    self.z_vals = z_vals
    self.x_axis_vals = x_axis_vals
    self.name = name
    self.x_label = name + ' ' + labels[0]
    self.y_label = name + ' ' + labels[1]
    self.z_label = name + ' ' + labels[2]
    self.x_color = colors[0]
    self.y_color = colors[1]
    self.z_color = colors[2]
    self.linestyle = linestyle
    self.linewidth = linewidth
    self.marker = marker
    self.markeredgewidth = markeredgewidth
    self.markersize = markersize

  def full_plot(self, ax, color):
    self.plot_2d(ax, color)
 
  def plot_2d(self, ax, color):
    ax.plot(self.x_vals,
            self.y_vals,
            self.z_vals,
            label=self.x_label,
            color=color,
            linestyle=self.linestyle,
            marker=self.marker,
            markeredgewidth=self.markeredgewidth,
            markersize=self.markersize)
  
  def plot_x(self):
    plt.plot(self.x_axis_vals,
             self.x_vals,
             label=self.x_label,
             color=self.x_color,
             linestyle=self.linestyle,
             marker=self.marker,
             markeredgewidth=self.markeredgewidth,
             markersize=self.markersize)

  def plot_y(self):
    plt.plot(self.x_axis_vals,
             self.y_vals,
             label=self.y_label,
             color=self.y_color,
             linewidth=self.linewidth,
             linestyle=self.linestyle,
             marker=self.marker,
             markeredgewidth=self.markeredgewidth,
             markersize=self.markersize)

  def plot_z(self):
    plt.plot(self.x_axis_vals,
             self.z_vals,
             label=self.z_label,
             color=self.z_color,
             linestyle=self.linestyle,
             marker=self.marker,
             markeredgewidth=self.markeredgewidth,
             markersize=self.markersize)
