/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <sparse_mapping/math_utilities.h>
// TODO(rsoussan): Remove these flags
DEFINE_double(min_valid_angle, 1e-2,
              "If all rays converging to a triangulated point make angles "
              "less than this, in degrees, drop it.");

DEFINE_bool(verbose_parsing, false,
              "If true, be more verbose when parsing camera data.");


namespace sparse_mapping {
// Compute the n-weight slerp, analogous to the linear combination
// W[0]*Q[0] + ... + W[n-1]*Q[n-1]. This is experimental.
// We assume the sum of weights is 1.
Eigen::Quaternion<double> slerp_n(std::vector<double> const& W,
                                                  std::vector<Eigen::Quaternion<double> > const& Q) {
  if (W.size() != Q.size())
    LOG(FATAL) << "Expecting as many quaternions as weights.";

  if (Q.empty())
    LOG(FATAL) << "Expecting at least one quaternion and weight.";

  if (Q.size() == 1)
    return Q[0];

  if (Q.size() == 2) {
    if (!(std::abs(W[0] + W[1] - 1.0) < 1e-6 && W[0] >= 0 && W[1] >= 0))
      LOG(FATAL) << "Expecting the weights to be >= 0 and sum up to 1.";
    return Q[0].slerp(W[1], Q[1]);
  }

  // Call recursively this function with fewer terms
  double sum = W[0] + W[1];
  if (sum == 0) sum = 1.0;
  Eigen::Quaternion<double> q = Q[0].slerp(W[1]/sum, Q[1]);
  std::vector<double> W2 = W;
  std::vector<Eigen::Quaternion<double> > Q2 = Q;
  W2.erase(W2.begin());
  Q2.erase(Q2.begin());
  W2[0] = sum;
  Q2[0] = q;
  return slerp_n(W2, Q2);
}

  // Statistics for filtering
  struct FilterStats{
    int total;
    int small_angle;
    int behind_cam;
    int invalid_reproj;
    int big_reproj_err;
    int num_features;
    FilterStats():total(0), small_angle(0), behind_cam(0), invalid_reproj(0),
                  big_reproj_err(0), num_features(0) {}

    void PrintStats() {
      // Print the stats.
      LOG(INFO) << "Statistics of points to filter out.";
      LOG(INFO) << "Total: " << total;
      LOG(INFO) << "xyz points with small ray angles:     "
                << small_angle << " (" << (100.0*small_angle)/total << " %)";
      LOG(INFO) << "xyz points behind camera:             "
                << behind_cam << " (" << (100.0*behind_cam)/total << " %)";
      LOG(INFO) << "Reprojected outside of image:         "
                << invalid_reproj << " (" << (100.0*invalid_reproj)/total << " %)";
      LOG(INFO) << "Features with big reprojection error: "
                << big_reproj_err << " (" << (100.0*big_reproj_err)/num_features << " %)";
    }
  };

  Eigen::Vector3d TriangulatePoint(Eigen::Vector3d const& unnormalized_pt1, Eigen::Vector3d const& unnormalized_pt2,
                                   Eigen::Matrix3d const& cam2_r_cam1, Eigen::Vector3d const& cam2_t_cam1,
                                   double* error) {
    // The second camera's center in the coordinate system of the first
    // camera.
    Eigen::Vector3d p2 = -cam2_r_cam1.transpose() * cam2_t_cam1;

    // Calculate the two unit pointing vectors in the domain of cam1
    Eigen::Vector3d unit1 = unnormalized_pt1.normalized();
    Eigen::Vector3d unit2 = cam2_r_cam1.transpose() * unnormalized_pt2.normalized();

    Eigen::Vector3d v12 = unit1.cross(unit2);
    Eigen::Vector3d v1 = v12.cross(unit1);
    Eigen::Vector3d v2 = v12.cross(unit2);

    Eigen::Vector3d closestPoint1 = v2.dot(p2) / v2.dot(unit1) * unit1;
    Eigen::Vector3d closestPoint2 = p2 + v1.dot(-p2) / v1.dot(unit2) * unit2;
    *error = (closestPoint2 - closestPoint1).norm();

    return 0.5 * (closestPoint2 + closestPoint1);
  }

void DecomposeFMatIntoEMat(Eigen::Matrix3d const& fundamental,
                                           Eigen::Matrix3d const& intrinsics,
                                           Eigen::Matrix3d * essential) {
  Eigen::Matrix<double, 3, 3> messy_essential =
    intrinsics.transpose() * fundamental * intrinsics;
  Eigen::JacobiSVD<Eigen::Matrix3d>
    svd(messy_essential, Eigen::ComputeFullU | Eigen::ComputeFullV);
  *essential =
    svd.matrixU() * Eigen::Vector3d(1, 1, 0).asDiagonal() * svd.matrixV().transpose();
}

void DecomposeEMatIntoRT(Eigen::Matrix3d const& essential,
                                         Eigen::Matrix2Xd const& unnormalized_pts1,
                                         Eigen::Matrix2Xd const& unnormalized_pts2,
                                         std::vector<cv::DMatch> const& matches,
                                         double focal_length1,  // Camera 1
                                         double focal_length2,  // Camera 2
                                         Eigen::Matrix3d * cam2_r_cam1,
                                         Eigen::Vector3d * cam2_t_cam1) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(essential,
                                        Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d w;
  w << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  Eigen::Matrix3d R[2];
  Eigen::Vector3d t[2];
  R[0] = svd.matrixU() * w * svd.matrixV().transpose();
  R[1] = svd.matrixU() * w.transpose() * svd.matrixV().transpose();
  R[0] /= R[0].determinant();
  R[1] /= R[1].determinant();
  t[0] = svd.matrixU().col(2);
  t[1] = -t[0];

  // Test all possible combinations to determine which combination
  // is correct. (Or most correct in this case. Our fundamental
  // matrix that was fitted still has measurements with 5 px error.)
  int histogram[2][2] = {{0, 0}, {0, 0}};
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (std::vector<cv::DMatch>::value_type const& match : matches) {
        double error;

        Eigen::Vector2d px1 = unnormalized_pts1.col(match.queryIdx);
        Eigen::Vector2d px2 = unnormalized_pts2.col(match.trainIdx);

        Eigen::Vector3d pt =
          TriangulatePoint
          (Eigen::Vector3d(px1[0], px1[1], focal_length1),
           Eigen::Vector3d(px2[0], px2[1], focal_length2),
           R[i], t[j], &error);

        // Point described in the other camera's space
        Eigen::Vector3d pt_2 = R[i] * pt + t[j];

        if (pt[2] > 0 && pt_2[2] > 0) {
          // In front of the second camera. This must be a valid
          // solution.
          histogram[i][j]++;
        }
      }
    }
  }

  // Select the best solution for R and t
  int best_sol = std::max(std::max(histogram[0][0], histogram[0][1]),
                          std::max(histogram[1][0], histogram[1][1]));
  if (best_sol == histogram[0][0]) {
  } else if (best_sol == histogram[0][1]) {
    t[0] = t[1];
  } else if (best_sol == histogram[1][0]) {
    R[0] = R[1];
  } else if (best_sol == histogram[1][1]) {
    R[0] = R[1];
    t[0] = t[1];
  }

  // Write the output
  *cam2_r_cam1 = R[0];
  *cam2_t_cam1 = t[0];
}

// Apply a given transform to the specified xyz points, and adjust accordingly the cameras
// for consistency. We assume that the transform is of the form
// A(x) = scale * rotation * x + translation
void TransformCamerasAndPoints(Eigen::Affine3d const& A,
                                               std::vector<Eigen::Affine3d> *cid_to_cam_t,
                                               std::vector<Eigen::Vector3d> *xyz) {
  for (size_t pid = 0; pid < (*xyz).size(); pid++)
    (*xyz)[pid] = A * (*xyz)[pid];

  // Inverse of rotation component
  double scale = pow(A.linear().determinant(), 1.0/3.0);
  Eigen::MatrixXd Ainv = (A.linear()/scale).inverse();

  for (size_t cid = 0; cid < (*cid_to_cam_t).size(); cid++) {
    (*cid_to_cam_t)[cid].linear() = (*cid_to_cam_t)[cid].linear()*Ainv;
    (*cid_to_cam_t)[cid].translation() = scale*(*cid_to_cam_t)[cid].translation() -
      (*cid_to_cam_t)[cid].linear()*A.translation();
  }
}

// Get the median error value, and multiply it by factor.
double GetErrThresh(std::vector<double> const& errors, double factor) {
  std::vector<double> sorted_errors = errors;
  std::sort(sorted_errors.begin(), sorted_errors.end());

  int len = sorted_errors.size();
  if (len == 0) return 0;

  // The case when there are too few errors
  if (len <= 2) return factor*sorted_errors[len-1];

    return factor*sorted_errors[len/2];
}

// Find the maximum angle between n rays intersecting at given
// point. Must compute the camera centers in the global coordinate
// system before calling this function.
double ComputeRaysAngle(int pid,
                                        std::vector<std::map<int, int> > const& pid_to_cid_fid,
                                        std::vector<Eigen::Vector3d> const & cam_ctrs,
                                        std::vector<Eigen::Vector3d> const& pid_to_xyz) {
  double max_angle = 0;
  std::map<int, int> const& track = pid_to_cid_fid[pid];
  for (std::map<int, int>::const_iterator it1 = track.begin();
       it1 != track.end(); it1++) {
    int cid1 = it1->first;
    for (std::map<int, int>::const_iterator it2 = it1;
         it2 != track.end(); it2++) {
      if (it1 == it2) continue;

      int cid2 = it2->first;
      Eigen::Vector3d X1 = cam_ctrs[cid1] - pid_to_xyz[pid];
      Eigen::Vector3d X2 = cam_ctrs[cid2] - pid_to_xyz[pid];
      double l1 = X1.norm(), l2 = X2.norm();
      if (l1 == 0 || l2 == 0)
        continue;

      double dot = X1.dot(X2)/l1/l2;
      dot = std::min(dot, 1.0);
      dot = std::max(-1.0, dot);
      double angle = (180.0/M_PI)*acos(dot);
      max_angle = std::max(angle, max_angle);
    }
  }
  return max_angle;
}

void FilterPID(double reproj_thresh,
                               camera::CameraParameters const& camera_params,
                               std::vector<Eigen::Affine3d > const& cid_to_cam_t_global,
                               std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                               std::vector<std::map<int, int> > * pid_to_cid_fid,
                               std::vector<Eigen::Vector3d> * pid_to_xyz,
                               bool print_stats, double multiple_of_median) {
  // Remove points that don't project at valid camera pixels,
  // points behind the camera, and matches having large reprojection error.

  // Reprojection error at each match point.
  std::vector<double> errors;

  int num_cams = cid_to_cam_t_global.size();
  std::vector<Eigen::Vector3d> cam_ctrs(num_cams);
  for (int cid = 0; cid < num_cams; cid++) {
    cam_ctrs[cid] = cid_to_cam_t_global[cid].inverse().translation();
  }

  // Init the stats
  FilterStats s;
  s.total = (*pid_to_xyz).size();

  std::vector<bool> is_bad((*pid_to_xyz).size(), false);
  Eigen::Vector2d half_size = camera_params.GetUndistortedHalfSize();
  for (size_t pid = 0; pid < (*pid_to_xyz).size(); pid++) {
    bool small_angle = false, behind_cam = false, invalid_reproj = false;

    double max_angle
      = ComputeRaysAngle(pid, *pid_to_cid_fid,
                                         cam_ctrs,  *pid_to_xyz);
    if (max_angle < FLAGS_min_valid_angle) {
      small_angle = true;
      is_bad[pid] = true;
    }

    for (std::pair<int, int> cid_fid : (*pid_to_cid_fid)[pid]) {
      Eigen::Vector2d pix = (cid_to_cam_t_global[cid_fid.first] *
                             (*pid_to_xyz)[pid]).hnormalized() * camera_params.GetFocalLength();
      errors.push_back((cid_to_keypoint_map[cid_fid.first].col(cid_fid.second) - pix).norm());
      // Mark points which don't project at valid camera pixels
      // TODO(zmoratto) : This can probably be done with a Eigen Array reduction
      if (pix[0] < -half_size[0] || pix[0] >= half_size[0] || pix[1] < -half_size[1] || pix[1] >= half_size[1]) {
        invalid_reproj = true;
        is_bad[pid] = true;
      }

      // Mark points that are behind the camera
      Eigen::Vector3d P = cid_to_cam_t_global[cid_fid.first] * (*pid_to_xyz)[pid];
      if (P[2] <= 0) {
        behind_cam = true;
        is_bad[pid] = true;
      }
    }
    s.small_angle    += static_cast<int>(small_angle);
    s.behind_cam     += static_cast<int>(behind_cam);
    s.invalid_reproj += static_cast<int>(invalid_reproj);
  }

  for (size_t pid = (*pid_to_xyz).size() - 1; pid < (*pid_to_xyz).size(); pid--) {
    if (is_bad[pid]) {
      std::vector<std::map<int, int> >::iterator cid_fid_it = (*pid_to_cid_fid).begin();
      std::vector<Eigen::Vector3d>::iterator xyz_it = (*pid_to_xyz).begin();
      std::advance(cid_fid_it, pid);
      std::advance(xyz_it, pid);
      (*pid_to_cid_fid).erase(cid_fid_it);
      (*pid_to_xyz).erase(xyz_it);
    }
  }

  // Wipe all features who are further than the reprojection of the
  // corresponding 3D point than given threshold.
  double thresh = std::max(GetErrThresh(errors, multiple_of_median), reproj_thresh);
  LOG(INFO) << "Filtering features with reprojection error higher than: "
            << thresh << " pixels";
  for (size_t pid = (*pid_to_xyz).size() - 1; pid < (*pid_to_xyz).size(); pid--) {
    std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];
    std::map<int, int>::iterator itr = cid_fid.begin();
    while (itr != cid_fid.end()) {
      s.num_features++;
      Eigen::Vector2d pix = (cid_to_cam_t_global[itr->first] *
                             (*pid_to_xyz)[pid]).hnormalized() * camera_params.GetFocalLength();
      double err
        = (cid_to_keypoint_map[itr->first].col(itr->second) - pix).norm();

      if (err >= thresh) {
        std::map<int, int>::iterator toErase = itr;
        ++itr;
        cid_fid.erase(toErase);
        s.big_reproj_err++;
      } else {
        ++itr;
      }
    }

    // Wipe a 3D point altogether if it corresponds to less than 2 matches.
    int total = (*pid_to_cid_fid)[pid].size();
    if (total < 2) {
      std::vector<std::map<int, int> >::iterator cid_fid_it
        = (*pid_to_cid_fid).begin();
      std::vector<Eigen::Vector3d>::iterator xyz_it = (*pid_to_xyz).begin();
      std::advance(cid_fid_it, pid);
      std::advance(xyz_it, pid);
      (*pid_to_cid_fid).erase(cid_fid_it);
      (*pid_to_xyz).erase(xyz_it);
    }
  }

  if (print_stats)
    s.PrintStats();
}

// Given a data sequence having camera pose information for a set of
// timestamps, interpolate those poses at the timestamps given in
// out_time. We assume timestamps are always in increasing values.
void PoseInterpolation(std::vector<std::string> const& images,
                                       std::vector<double> const& out_time,
                                       std::map< std::string, std::vector<double> >
                                       const& data,
                                       std::vector<Eigen::Affine3d> * cid_to_cam_t,
                                       std::vector<std::string> * good_images) {
  if (images.size() != out_time.size())
    LOG(FATAL) << "Number of images is inconsistent with the number of "
               << "timestamps at which to find the camera poses";

  // Pull the timestamps and pose information from the CSV file.
  typedef std::vector<double> dvec;
  dvec const& time = data.find("field.header.stamp")->second;
  dvec const& posx = data.find("field.pose.position.x")->second;
  dvec const& posy = data.find("field.pose.position.y")->second;
  dvec const& posz = data.find("field.pose.position.z")->second;
  dvec const& qx   = data.find("field.pose.orientation.x")->second;
  dvec const& qy   = data.find("field.pose.orientation.y")->second;
  dvec const& qz   = data.find("field.pose.orientation.z")->second;
  dvec const& qw   = data.find("field.pose.orientation.w")->second;

  // Ensure all columns were parsed correctly.
  size_t n = time.size();
  if (n != posx.size() || n != posy.size() || n != posz.size() ||
      n != qx.size() || n != qy.size() || n != qz.size() || n != qw.size())
    LOG(FATAL) << "Could not parse all ground truth fields.\n";

  if (FLAGS_verbose_parsing) {
    // Useful debug code. Save the bot trajectory as measured,
    // before we interpolated it at times the images were aquired.
    std::string uninterp_traj = "uninterp_trajectory.txt";
    LOG(INFO) << "Writing: " << uninterp_traj << std::endl;
    std::ofstream ut(uninterp_traj.c_str());
    ut.precision(18);
    for (size_t i = 0; i < n; i++) {
      ut << posx[i] << ' ' << posy[i] << ' ' << posz[i] << std::endl;
    }
    ut.close();
  }

  int num_in = time.size();
  int num_out = out_time.size();
  (*cid_to_cam_t).reserve(num_out); (*cid_to_cam_t).clear();
  (*good_images).reserve(num_out);  (*good_images).clear();

  // Bracket the output time. We count on the fact that the arrays
  // time and time_out are both increasing.
  int prev_beg = -1, prev_end = -1;
  int pos = 0;
  for (int cid = 0; cid < num_out; cid++) {
    bool success = false;
    // Increase pos until time[pos] <= out_time[cid] <= time[pos+1]
    while (1) {
      if (pos + 1 >= num_in) break;  // Out of range

      if (time[pos] > out_time[cid]) {
        // No point in going further, the values in time from
        // now on will always be larger than out_time[cid].
        success = false;
        break;
      }

      if (time[pos] <= out_time[cid] && out_time[cid] <= time[pos+1]) {
        success = true;
        break;
      }
      pos++;
    }

    if (!success) {
      // Try to see if we can bracket out_time[cid]
      // between the next values of the time array.
      continue;
    }

    // Insist that each cid be bracketed by new pos and pos+1.
    // Otherwise we have the unfortunate situation that we use
    // the same very sparse pos values to interpolate successive
    // cids, when the measurements are very sparse, with the result
    // that the interpolated values are very inaccurate. We'd rather
    // not interpolate at all then.
    if (prev_beg != -1 && prev_end != -1) {
      if (prev_beg == pos || prev_end == pos+1) {
        success = false;
      }
    }
    prev_beg = pos;
    prev_end = pos+1;
    if (!success) continue;

    double t = 0.0;
    if (time[pos] != time[pos+1])
      t = (out_time[cid] - time[pos])/(time[pos+1] - time[pos]);
    Eigen::Vector3d va(posx[pos],   posy[pos],   posz[pos]);
    Eigen::Vector3d vb(posx[pos+1], posy[pos+1], posz[pos+1]);
    Eigen::Quaternion<double> qa(qw[pos], qx[pos], qy[pos], qz[pos]);
    Eigen::Quaternion<double> qb(qw[pos+1], qx[pos+1], qy[pos+1], qz[pos+1]);

    Eigen::Affine3d T;
    T.translation() = (1.0-t)*va + t*vb;
    T.linear() = qa.slerp(t, qb).toRotationMatrix();
    (*cid_to_cam_t).push_back(T);
    (*good_images).push_back(images[cid]);
  }

  if (FLAGS_verbose_parsing) {
    // Useful debug code. Save the camera positions after interpolating
    // at image timestamps.
    std::string interp_traj = "interp_trajectory.txt";
    LOG(INFO) << "Writing: " << interp_traj << std::endl;
    std::ofstream it(interp_traj.c_str());
    it.precision(18);
    for (size_t i = 0; i < (*cid_to_cam_t).size(); i++) {
      it << (*cid_to_cam_t)[i].translation().transpose() << std::endl;
    }
    it.close();
  }

  return;
}
}  // namespace sparse_mapping
