#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.0)
project(bag_processing)

if (USE_ROS)
  ## Compile as C++14, supported in ROS Kinetic and newer
  add_compile_options(-std=c++14)

  ## Find catkin macros and libraries
  find_package(catkin2 REQUIRED COMPONENTS
    ff_common
    localization_common
    rosbag
  )

  catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ff_common localization_common rosbag
  )

  ## Declare a C++ executable: extract_images
  add_executable(extract_images tools/extract_images.cc)
  add_dependencies(extract_images ${catkin_EXPORTED_TARGETS})
  target_include_directories(extract_images PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(extract_images
    glog ${catkin_LIBRARIES})

  # Install C++ executables
  install(TARGETS extract_images DESTINATION bin)
  install(CODE "execute_process(
    COMMAND ln -s ../../bin/extract_images share/${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}
    OUTPUT_QUIET
    ERROR_QUIET
    )")

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(test/test_fix_all.test)
  endif()
endif (USE_ROS)
