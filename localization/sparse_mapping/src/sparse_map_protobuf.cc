/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http:  //  www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/sparse_map.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

#include <sparse_map.pb.h>

namespace sparse_mapping {
void SparseMap::Load(const std::string & protobuf_file, bool localization) {
  sparse_mapping_protobuf::Map map;
  int input_fd = open(protobuf_file.c_str(), O_RDONLY);
  if (input_fd < 0)
    LOG(FATAL) << "Failed to open map file: " << protobuf_file;

  google::protobuf::io::ZeroCopyInputStream* input =
    new google::protobuf::io::FileInputStream(input_fd);
  if (!ReadProtobufFrom(input, &map)) {
    LOG(FATAL) << "Failed to parse map file.";
  }

  detector_.Reset(map.detector_name());

  // Check that the maps is correctly formed
  assert(map.camera().focal_length_size() == 2);
  assert(map.camera().optical_offset_size() == 2);
  assert(map.camera().distorted_image_size_size() == 2);
  assert(map.camera().undistorted_image_size_size() == 2);
  typedef Eigen::Vector2d V2d;
  typedef Eigen::Vector2i V2i;
  camera_params_.SetFocalLength(V2d(map.camera().focal_length(0),
                                    map.camera().focal_length(1)));
  camera_params_.SetOpticalOffset(V2d(map.camera().optical_offset(0),
                                      map.camera().optical_offset(1)));
  camera_params_.SetDistortedSize(V2i(map.camera().distorted_image_size(0),
                                      map.camera().distorted_image_size(1)));
  camera_params_.SetUndistortedSize(V2i(map.camera().undistorted_image_size(0),
                                        map.camera().undistorted_image_size(1)));
  Eigen::VectorXd distortion(map.camera().distortion_size());
  for (int i = 0; i < map.camera().distortion_size(); i++) {
    distortion[i] = map.camera().distortion(i);
  }
  camera_params_.SetDistortion(distortion);

  int num_frames = map.num_frames();
  int num_landmarks = map.num_landmarks();

  cid_to_filename_.resize(num_frames);
  cid_to_descriptors_.resize(num_frames);
  if (!localization) {
    cid_to_keypoints_.resize(num_frames);
    cid_to_cam_T_global_.resize(num_frames);
  }

  // load each frame
  for (int cid = 0; cid < num_frames; cid++) {
    sparse_mapping_protobuf::Frame frame;
    if (!ReadProtobufFrom(input, &frame)) {
      LOG(FATAL) << "Failed to parse frame.";
    }
    if (frame.has_name())
      cid_to_filename_[cid] = frame.name();
    else
      cid_to_filename_[cid] = "";

    // load keypoints
    if (!localization)
      cid_to_keypoints_[cid].resize(Eigen::NoChange_t(), frame.feature_size());

    // Poke the first frame's first descriptor to see how long the
    // descriptor is.
    if (frame.feature_size()) {
      size_t descriptor_length = frame.feature(0).description().size() /
        cv::getElemSize(map.descriptor_depth());
      cid_to_descriptors_[cid].create(frame.feature_size(),  // rows
                                         descriptor_length,     // columns
                                         map.descriptor_depth());
    } else {
      cid_to_descriptors_[cid].create(0, 0, map.descriptor_depth());
    }

    for (int fid = 0; fid < frame.feature_size(); fid++) {
      sparse_mapping_protobuf::Feature feature = frame.feature(fid);

      // Copy the features
      if (!localization)
        cid_to_keypoints_[cid].col(fid) << feature.x(), feature.y();

      // Copy the descriptors
      memcpy(cid_to_descriptors_[cid].ptr<uint8_t>(fid),  // Destination
             feature.description().data(),                   // Source
             feature.description().size());                  // Length
    }

    // Load pose
    if (frame.has_pose() && !localization) {
      sparse_mapping_protobuf::Affine3d pose = frame.pose();
      cid_to_cam_T_global_[cid].translation()
        << pose.t0(), pose.t1(), pose.t2();

      cid_to_cam_T_global_[cid].linear() <<
        pose.r00(), pose.r01(), pose.r02(),
        pose.r10(), pose.r11(), pose.r12(),
        pose.r20(), pose.r21(), pose.r22();
    }
  }

  // if not, only feature detection step has been run... or something is wrong
  if (num_landmarks > 0) {
    pid_to_global_t_point_.resize(num_landmarks);

    if (!localization) {
      pid_to_feature_track_.resize(num_landmarks);
    } else {
      // Create directly cid_to_fid_to_pid
      cid_to_fid_to_pid_.clear();
      cid_to_fid_to_pid_.resize(cid_to_filename_.size(), std::map<int, int>());
    }

    for (int i = 0; i < num_landmarks; i++) {
      sparse_mapping_protobuf::Landmark l;
      if (!ReadProtobufFrom(input, &l)) {
        LOG(FATAL) << "Failed to parse landmark.";
      }
      Eigen::Vector3d pos(l.loc().x(), l.loc().y(), l.loc().z());
      pid_to_global_t_point_[i] = pos;
      for (int j = 0; j < l.match_size(); j++) {
        sparse_mapping_protobuf::Matching m = l.match(j);
        if (!localization)
          pid_to_feature_track_[i][m.camera_id()] = m.feature_id();
        else
          cid_to_fid_to_pid_[m.camera_id()][m.feature_id()] = i;
      }
    }

    // If in localization mode, we already initialized cid_to_fid_to_pid_ right above.
    if (!localization)
      InitializeCidFidPidMap();

  } else {
    LOG(WARNING) << "There appear to be no landmarks in map file.";
  }

  // TODO(rsoussan): Is this right?
  // TODO(rsoussan): Allow for brisk or surf here! add protobuf param?
  if (map.has_vocab_db())
    image_database_.reset(new BriskImageDatabase(input));

  histogram_equalization_ = map.histogram_equalization();

  assert(histogram_equalization_ == 0 ||
         histogram_equalization_ == 1 ||
         histogram_equalization_ == 2);

  // For backward compatibility with old maps, allow a map to have its
  // histogram_equalization flag unspecified, but it is best to avoid
  // that situation, and rebuild the map if necessary.
  if (histogram_equalization_ == 2)
    std::cout << "Warning: Unknown value of histogram_equalization! "
              << "It is strongly suggested to rebuild this map to avoid "
              << "poor quality results." << std::endl;

  delete input;
  close(input_fd);
}

void SparseMap::Save(const std::string & protobuf_file) const {
  // For backward compatibility with old maps, allow a map to have its
  // histogram_equalization flag unspecified, but it is best to avoid
  // that situation, and rebuild the map if necessary.
  if (histogram_equalization_ == 2)
    std::cout << "Warning: Unknown value of histogram_equalization! "
              << "It is strongly suggested to rebuild this map to avoid "
              << "poor quality results." << std::endl;

  sparse_mapping_protobuf::Map map;
  // map.set_detector_name(detector_.DetectorName());
  if (!cid_to_descriptors_.empty())
    map.set_descriptor_depth(cid_to_descriptors_[0].depth());
  else
    map.set_descriptor_depth(0);

  sparse_mapping_protobuf::CameraModel* camera = map.mutable_camera();
  camera->add_focal_length(camera_params_.GetFocalVector()[0]);
  camera->add_focal_length(camera_params_.GetFocalVector()[1]);
  camera->add_optical_offset(camera_params_.GetOpticalOffset()[0]);
  camera->add_optical_offset(camera_params_.GetOpticalOffset()[1]);
  camera->add_distorted_image_size(camera_params_.GetDistortedSize()[0]);
  camera->add_distorted_image_size(camera_params_.GetDistortedSize()[1]);
  camera->add_undistorted_image_size(camera_params_.GetUndistortedSize()[0]);
  camera->add_undistorted_image_size(camera_params_.GetUndistortedSize()[1]);
  for (int i = 0; i < camera_params_.GetDistortion().size(); i++) {
    camera->add_distortion(camera_params_.GetDistortion()[i]);
  }

  CHECK(cid_to_filename_.size() == cid_to_keypoints_.size())
    << "Number of CIDs in filenames and keypoint map do not match";
  CHECK(cid_to_filename_.size() == cid_to_descriptors_.size())
    << "Number of CIDs in filenames and descriptor map do not match";

  map.set_num_frames(cid_to_filename_.size());
  map.set_num_landmarks(pid_to_global_t_point_.size());

  // TODO(rsoussan): put this back? remove this?
  if (vocab_db_.binary_db != NULL)
    map.set_vocab_db(sparse_mapping_protobuf::Map::BINARYDB);

  map.set_histogram_equalization(histogram_equalization_);

  LOG(INFO) << "Writing: " << protobuf_file;
  int output_fd = open(protobuf_file.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (output_fd < 0) {
    LOG(FATAL) << "Failed to open protobuf writing file.";
  }
  google::protobuf::io::ZeroCopyOutputStream* output = new google::protobuf::io::FileOutputStream(output_fd);
  if (!WriteProtobufTo(map, output)) {
    LOG(FATAL) << "Failed to write map to file.";
  }

  // write the frames
  for (size_t cid = 0; cid < cid_to_filename_.size(); cid++) {
    sparse_mapping_protobuf::Frame frame;

    // set the filename if existing
    if (!cid_to_filename_[cid].empty()) {
      frame.set_name(cid_to_filename_[cid]);
    }

    // set the features, required
    for (int fid = 0; fid < cid_to_keypoints_[cid].cols(); fid++) {
      sparse_mapping_protobuf::Feature* f = frame.add_feature();
      f->set_x(cid_to_keypoints_[cid].col(fid).x());
      f->set_y(cid_to_keypoints_[cid].col(fid).y());
      f->set_description(cid_to_descriptors_[cid].ptr<uint8_t>(fid),
                         cid_to_descriptors_[cid].elemSize() *
                         cid_to_descriptors_[cid].cols);
    }

    // set the camera pose if available.
    if (cid < cid_to_cam_T_global_.size()) {
      sparse_mapping_protobuf::Affine3d* a = frame.mutable_pose();
      Eigen::Matrix4d c = cid_to_cam_T_global_[cid].matrix();
      a->set_r00(c(0, 0));
      a->set_r01(c(0, 1));
      a->set_r02(c(0, 2));
      a->set_r10(c(1, 0));
      a->set_r11(c(1, 1));
      a->set_r12(c(1, 2));
      a->set_r20(c(2, 0));
      a->set_r21(c(2, 1));
      a->set_r22(c(2, 2));
      a->set_t0(c(0, 3));
      a->set_t1(c(1, 3));
      a->set_t2(c(2, 3));
    }
    if (!WriteProtobufTo(frame, output)) {
      LOG(FATAL) << "Failed to write frame to file.";
    }
  }

  if (pid_to_global_t_point_.size() != pid_to_feature_track_.size()) {
    LOG(FATAL) << "Book-keeping failure, expecting the following "
               << "arrays to have the same size:\n"
               << "pid_to_global_t_point_.size() = " << pid_to_global_t_point_.size() << "\n"
               << "pid_to_feature_track_.size() = " << pid_to_feature_track_.size();
  }

  for (size_t i = 0; i < pid_to_global_t_point_.size(); i++) {
    sparse_mapping_protobuf::Landmark l;
    l.mutable_loc()->set_x(pid_to_global_t_point_[i].x());
    l.mutable_loc()->set_y(pid_to_global_t_point_[i].y());
    l.mutable_loc()->set_z(pid_to_global_t_point_[i].z());
    for (std::map<int, int >::const_iterator it =
          pid_to_feature_track_[i].begin(); it != pid_to_feature_track_[i].end(); it++) {
      sparse_mapping_protobuf::Matching* m = l.add_match();
      m->set_camera_id(it->first);
      m->set_feature_id(it->second);
    }

    if (!WriteProtobufTo(l, output))
      LOG(FATAL) << "Failed to write landmark to file.";
  }

  if (vocab_db_)
    vocab_db_->SaveProtobuf(output);

  delete output;
  close(output_fd);
}
}  // namespace sparse_mapping
