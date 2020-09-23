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

cmake_minimum_required(VERSION 2.8.3)

project(graph_bag)

# For gprof - remove otherwise
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# For google prof: 
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-as-needed -lprofiler -Wl,--as-needed")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")

find_package(Eigen3)
# Store these in variables so they are automatically replicated in GTSAMConfig.cmake and such.
set(BOOST_FIND_MINIMUM_VERSION 1.43)
set(BOOST_FIND_MINIMUM_COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

find_package(Boost ${BOOST_FIND_MINIMUM_VERSION} COMPONENTS ${BOOST_FIND_MINIMUM_COMPONENTS})

# Required components
if(NOT Boost_SERIALIZATION_LIBRARY OR NOT Boost_SYSTEM_LIBRARY OR NOT Boost_FILESYSTEM_LIBRARY OR
    NOT Boost_THREAD_LIBRARY OR NOT Boost_DATE_TIME_LIBRARY)
  message(FATAL_ERROR "Missing required Boost components >= v1.43, please install/upgrade Boost or configure your search paths.")
endif()

# Allow for not using the timer libraries on boost < 1.48 (GTSAM timing code falls back to old timer library)
set(GTSAM_BOOST_LIBRARIES
  Boost::serialization
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::date_time
  Boost::regex
)

find_package(Eigen3 REQUIRED)

# include ff_nodelet to get ff_util header files since these aren't exposed elsewhere
#create_library(TARGET ${PROJECT_NAME} 
#  LIBS ff_common ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} camera ff_nodelet graph_localizer imu_augmentor lk_optical_flow localization_common localization_measurements localization_node sparse_mapping msg_conversions 
#  INC ${catkin_INCLUDE_DIRS}
#)

#create_tool_targets(DIR tools
#  LIBS ${GTSAM_BOOST_LIBRARIES}# profiler
#  INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
#)
file(GLOB_RECURSE source_list "*.cc" "*.h" "*.cpp" "*.hpp" "*.c")
MESSAGE(STATUS "source list" ${source_list})
add_executable(gtsam_test ${source_list})
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(gtsam_test ${GTSAM_BOOST_LIBRARIES})
