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

#include <sparse_mapping/file_utilities.h>
namespace {
  // A little helper function
  bool is_blank(std::string const& line) {
    return (line.find_first_not_of(" \t\n\v\f\r") == std::string::npos);
  }
}

namespace sparse_mapping {
// Writes the NVM control network format.
void WriteNVM(std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map,
                              std::vector<std::string> const& cid_to_filename,
                              std::vector<std::map<int, int> > const& pid_to_cid_fid,
                              std::vector<Eigen::Vector3d> const& pid_to_xyz,
                              std::vector<Eigen::Affine3d> const&
                              cid_to_cam_t_global,
                              double focal_length,
                              std::string const& output_filename) {
  std::fstream f(output_filename, std::ios::out);
  f << "NVM_V3\n";

  CHECK(cid_to_filename.size() == cid_to_keypoint_map.size())
    << "Unequal number of filenames and keypoints";
  CHECK(pid_to_cid_fid.size() == pid_to_xyz.size())
    << "Unequal number of pid_to_cid_fid and xyz measurements";
  CHECK(cid_to_filename.size() == cid_to_cam_t_global.size())
    << "Unequal number of filename and camera transforms";

  // Write camera information
  f << cid_to_filename.size() << std::endl;
  for (size_t cid = 0; cid < cid_to_filename.size(); cid++) {
    // Decompose cam_t_global so that we can write it into a WXY
    // quaternion and an XYZ camera position
    Eigen::Quaterniond q(cid_to_cam_t_global[cid].rotation());
    Eigen::Vector3d t(cid_to_cam_t_global[cid].translation());
    Eigen::Vector3d camera_center =
      - cid_to_cam_t_global[cid].rotation().inverse() * t;

    // The NVM format is a little crazy. When using a quaternion, we
    // write the camera center instead of the t from camera_t_global.
    f << cid_to_filename[cid] << " " << focal_length
      << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
      << camera_center[0] << " " << camera_center[1] << " "
      << camera_center[2] << " " << "0.9 0\n";
  }

  // Write the number of points
  f << pid_to_cid_fid.size() << std::endl;

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    f << pid_to_xyz[pid][0] << " " << pid_to_xyz[pid][1] << " "
      << pid_to_xyz[pid][2] << " 0 0 0 "
      << pid_to_cid_fid[pid].size();

    CHECK(pid_to_cid_fid[pid].size() > 1)
      << "PID " << pid << " has " << pid_to_cid_fid[pid].size() << " measurements";

    for (std::map<int, int>::const_iterator it = pid_to_cid_fid[pid].begin();
         it != pid_to_cid_fid[pid].end(); it++) {
      f << " " << it->first << " " << it->second << " "
        << cid_to_keypoint_map[it->first].col(it->second)[0] << " "
        << cid_to_keypoint_map[it->first].col(it->second)[1];
    }
    f << std::endl;
  }

  // Close the file
  f.flush();
  f.close();
}

// Reads the NVM control network format.
void ReadNVM(std::string const& input_filename,
                             std::vector<Eigen::Matrix2Xd > * cid_to_keypoint_map,
                             std::vector<std::string> * cid_to_filename,
                             std::vector<std::map<int, int> > * pid_to_cid_fid,
                             std::vector<Eigen::Vector3d> * pid_to_xyz,
                             std::vector<Eigen::Affine3d> *
                             cid_to_cam_t_global) {
  std::ifstream f(input_filename, std::ios::in);
  std::string token;
  std::getline(f, token);

  // Assert that we start with our NVM token
  if (token.compare(0, 6, "NVM_V3") != 0) {
    LOG(FATAL) << "File doesn't start with NVM token";
  }

  // Read the number of cameras
  ptrdiff_t number_of_cid;
  f >> number_of_cid;
  if (number_of_cid < 1) {
    LOG(FATAL) << "NVM file is missing cameras";
  }

  // Resize all our structures to support the number of cameras we now expect
  cid_to_keypoint_map->resize(number_of_cid);
  cid_to_filename->resize(number_of_cid);
  cid_to_cam_t_global->resize(number_of_cid);
  for (ptrdiff_t cid = 0; cid < number_of_cid; cid++) {
    // Clear keypoints from map. We'll read these in shortly
    cid_to_keypoint_map->at(cid).resize(Eigen::NoChange_t(), 2);

    // Read the line that contains camera information
    double focal, dist1, dist2;
    Eigen::Quaterniond q;
    Eigen::Vector3d c;
    f >> token >> focal;
    f >> q.w() >> q.x() >> q.y() >> q.z();
    f >> c[0] >> c[1] >> c[2] >> dist1 >> dist2;
    cid_to_filename->at(cid) = token;

    // Solve for t, which is part of the affine transform
    Eigen::Matrix3d r = q.matrix();
    cid_to_cam_t_global->at(cid).linear() = r;
    cid_to_cam_t_global->at(cid).translation() = -r * c;
  }

  // Read the number of points
  ptrdiff_t number_of_pid;
  f >> number_of_pid;
  if (number_of_pid < 1) {
    LOG(FATAL) << "The NVM file has no triangulated points.";
  }

  // Read the point
  pid_to_cid_fid->resize(number_of_pid);
  pid_to_xyz->resize(number_of_pid);
  Eigen::Vector3d xyz;
  Eigen::Vector3i color;
  Eigen::Vector2d pt;
  ptrdiff_t cid, fid;
  for (ptrdiff_t pid = 0; pid < number_of_pid; pid++) {
    pid_to_cid_fid->at(pid).clear();

    ptrdiff_t number_of_measures;
    f >> xyz[0] >> xyz[1] >> xyz[2] >>
      color[0] >> color[1] >> color[2] >> number_of_measures;
    pid_to_xyz->at(pid) = xyz;
    for (ptrdiff_t m = 0; m < number_of_measures; m++) {
      f >> cid >> fid >> pt[0] >> pt[1];

      pid_to_cid_fid->at(pid)[cid] = fid;

      if (cid_to_keypoint_map->at(cid).cols() <= fid) {
        cid_to_keypoint_map->at(cid).conservativeResize(Eigen::NoChange_t(), fid + 1);
      }
      cid_to_keypoint_map->at(cid).col(fid) = pt;
    }

    if (!f.good())
      LOG(FATAL) << "Unable to correctly read PID: " << pid;
  }
}

std::string ImageToFeatureFile(std::string const& image_file,
                                               std::string const& detector_name) {
  return std::string(image_file) + ".yaml.gz";
}

std::string DBImagesFile(std::string const& db_name) {
  return db_name + ".txt";
}

std::string MatchesFile(std::string const& map_file) {
  return map_file + ".matches.txt";
}

std::string EssentialFile(std::string const& map_file) {
  return map_file + ".essential.csv";
}

int ReadFeaturesSIFT(std::string const& filename,
                                     cv::Mat * descriptors,
                                     std::vector<cv::KeyPoint> * keypoints) {
  // Read SIFT keypoints and descriptors as written by Lowe's sift tool and opensift.

  std::ifstream f(filename);
  if (!f.good()) {
    LOG(ERROR) << "Could not read: " << filename;
    return 0;
  }

  std::string line;
  if (!std::getline(f, line)) {
    LOG(ERROR) << "Invalid file: " << filename;
    return 0;
  }

  int i, num, len;

  if (sscanf(line.c_str(), "%d %d", &num, &len) != 2) {
    LOG(ERROR) << "Invalid file: " << filename;
    return 0;
  }

  if (len != 128) {
    printf("Keypoint descriptor length invalid (should be 128).");
    return 0;
  }

  *descriptors = cv::Mat(num, 128, CV_32F);
  keypoints->resize(num);

  int16_t pbuf[128];
  for (i = 0; i < num; i++) {
    // Allocate memory for the keypoint.
    float x, y, scale, ori;

    if (!std::getline(f, line)) {
      LOG(ERROR) << "Invalid file: " << filename;
      return 0;
    }

    if (sscanf(line.c_str(), "%f %f %f %f\n", &y, &x, &scale, &ori) != 4) {
      printf("Invalid keypoint file format.");
      return 0;
    }

    (*keypoints)[i] = cv::KeyPoint(x, y, scale, ori);

    int16_t * p = pbuf;
    for (int iter = 0; iter < 7; iter++) {
      if (!std::getline(f, line)) {
        LOG(ERROR) << "Invalid file: " << filename;
        return 0;
      }

      if (iter < 6) {
        sscanf(line.c_str(),
               "%hu %hu %hu %hu %hu %hu %hu %hu %hu %hu "
               "%hu %hu %hu %hu %hu %hu %hu %hu %hu %hu",
               p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7, p+8, p+9,
               p+10, p+11, p+12, p+13, p+14,
               p+15, p+16, p+17, p+18, p+19);

        p += 20;
      } else {
        sscanf(line.c_str(),
               "%hu %hu %hu %hu %hu %hu %hu %hu",
               p+0, p+1, p+2, p+3, p+4, p+5, p+6, p+7);
        p += 8;
      }
    }

    for (int c = 0; c < 128; c++)
      (*descriptors).at<float>(i, c) = pbuf[c];
  }

  return num;
}

void WriteFeatures(std::string const& detector_name,
                                   std::vector<cv::KeyPoint> const& keypoints,
                                   cv::Mat const& descriptors,
                                   std::string const& output_filename) {
  LOG(INFO) << "Writing: " << output_filename;
  cv::FileStorage fs(output_filename,
                     cv::FileStorage::WRITE);
  cv::write(fs, "keypoints", keypoints);
  cv::write(fs, "descriptions", descriptors);
}

bool ReadFeatures(std::string const& input_filename,
                                  std::string const& detector_name,
                                  std::vector<cv::KeyPoint> * keypoints,
                                  cv::Mat * descriptors) {
  LOG(INFO) << "Reading: " << input_filename;

  // Test the file
  std::ifstream f(input_filename);
  if (!f.good()) {
    LOG(FATAL) << "Could not read: " << input_filename;
    return false;
  }

  // Read the yaml.gz file
  cv::FileStorage fs(input_filename, cv::FileStorage::READ);
  cv::FileNode fn = fs["keypoints"];
  cv::read(fn, *keypoints);
  fs["descriptions"] >> *descriptors;

  return true;
}

void MergePids(int repeat_index, int num_unique,
                               std::vector<std::map<int, int> > * pid_to_cid_fid) {
  // Consider a set of images, and the corresponding tracks, stored in
  // (*pid_to_cid_fid). By design, we have num_unique images,
  // and after these, the images up to index repeat_index are repeated
  // (we do this repetition to help us close the loop). So, our sequence
  // is in fact,
  // image[0], ..., image[repeat_index], ..., image[num_unique-1], ...
  // image[0], ..., image[repeat_index].

  // So, some tracks show up twice (sometimes the second instance is
  // longer than the first, since it sees images with indices <=
  // num_unique - 1). Merge the repeated tracks, by replacing each cid
  // >=num_unique in the tracks with cid%num_unique, and wipe the now
  // redundant tracks.

  int num_to_wipe = 0;

  std::set<int> pids_to_wipe;
  for (int cid1 = 0; cid1 <= repeat_index; cid1++) {
    // Images cid1 and cid2 are the same, so in the final
    // merge tracks cid2 will become cid1
    int cid2 = cid1 + num_unique;

    // For given cid1 and cid2, find all pids containing these cids.
    // Index by fid, which is the same for both images.
    std::map<int, int> fid_to_pid1, fid_to_pid2;

    for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
      // Ignore already merged and redundant pids which we will wipe
      if (pids_to_wipe.find(pid) != pids_to_wipe.end())
        continue;

      std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];

      std::map<int, int>::iterator it1 = cid_fid.find(cid1);
      if (it1 != cid_fid.end())  fid_to_pid1[it1->second] = pid;

      std::map<int, int>::iterator it2 = cid_fid.find(cid2);
      if (it2 != cid_fid.end())  fid_to_pid2[it2->second] = pid;
    }

    // Merge the tracks having the same fids
    for (std::map<int, int>::iterator fid_it1 = fid_to_pid1.begin();
         fid_it1 != fid_to_pid1.end(); fid_it1++) {
      std::map<int, int>::iterator it2 = fid_to_pid2.find(fid_it1->first);
      if (it2 == fid_to_pid2.end()) continue;

      // The indices of the pids to merge
      int pid1 = fid_it1->second, pid2 = it2->second;

      // The below is quite unlikely
      if (pid1 == pid2) continue;

      // Ignore already merged and redundant pids which we will wipe
      if (pids_to_wipe.find(pid1) != pids_to_wipe.end()) continue;
      if (pids_to_wipe.find(pid2) != pids_to_wipe.end()) continue;

      // Important, we are aliasing below
      std::map<int, int> & cid_fid1 = (*pid_to_cid_fid)[pid1];
      std::map<int, int> & cid_fid2 = (*pid_to_cid_fid)[pid2];

      for (std::map<int, int>::iterator it = cid_fid2.begin();
           it != cid_fid2.end() ; it++) {
        int good_cid = it->first;
        if (good_cid >= num_unique) good_cid = good_cid % num_unique;

        // Merge feature into first pid
        cid_fid1[good_cid] = it->second;
      }

      pids_to_wipe.insert(pid2);
      num_to_wipe++;
    }
  }

  // Any leftover cid >= num_unique must be shifted by num_unique
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    // Ignore already merged and redundant pids which we will wipe
    if (pids_to_wipe.find(pid) != pids_to_wipe.end())
      continue;

    std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];
    std::map<int, int>   cid_fid2;
    bool need_to_shift = false;
    for (std::map<int, int>::iterator it = cid_fid.begin();
         it != cid_fid.end() ; it++) {
      if (it->first >= num_unique) {
        need_to_shift = true;
      }

      int cid = it->first % num_unique;
      cid_fid2[cid] = it->second;
    }
    if (need_to_shift) {
      // Overwrite with the pid with the shifted cid
      (*pid_to_cid_fid)[pid] = cid_fid2;
    }
    // some pids only link to themselves, delete
    if ((*pid_to_cid_fid)[pid].size() < 2)
      pids_to_wipe.insert(pid);
  }

  // Wipe the pids which were merged
  std::vector<std::map<int, int> > pid_to_cid_fid2;
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    if (pids_to_wipe.find(pid) != pids_to_wipe.end()) continue;
    pid_to_cid_fid2.push_back((*pid_to_cid_fid)[pid]);
  }
  LOG(INFO) << "Number of pids before and after loop closure: "
            << (*pid_to_cid_fid).size() << ' ' << pid_to_cid_fid2.size();
  (*pid_to_cid_fid) = pid_to_cid_fid2;

  LOG(INFO) << "Number of removed pids: " << num_to_wipe;

  // Sanity check, there must be no cids >= num_unique by now
  for (size_t pid = 0; pid < (*pid_to_cid_fid).size(); pid++) {
    std::map<int, int> & cid_fid = (*pid_to_cid_fid)[pid];
    for (std::map<int, int>::iterator it = cid_fid.begin(); it != cid_fid.end(); it++) {
      if (it->first >= num_unique)
        LOG(FATAL) << "Must have fixed all cids by now.";
    }
  }
}

void PrintPidStats(std::vector<std::map<int, int> > const& pid_to_cid_fid) {
  std::map<int, int> cid_to_pid;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    std::map<int, int> const& cid_fid = pid_to_cid_fid[pid];
    for (std::map<int, int>::const_iterator it = cid_fid.begin(); it != cid_fid.end();
         it++) {
      cid_to_pid[it->first]++;
    }
  }
  LOG(INFO) << "cid and number of pids having fids in that cid";
  for (std::map<int, int>::iterator it = cid_to_pid.begin();
       it != cid_to_pid.end(); it++) {
    LOG(INFO) << "cid_fid " << it->first << ' ' << it->second;
  }
}

// Extract control points and the images they correspond 2 from
// a hugin project file
void ParseHuginControlPoints(std::string const& hugin_file,
                                             std::vector<std::string> * images,
                                             Eigen::MatrixXd * points) {
  // Initialize the outputs
  (*images).clear();
  *points = Eigen::MatrixXd(6, 1);

  std::ifstream hf(hugin_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseHuginControlPoints(): Could not open hugin file: " << hugin_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Parse for images
    if (line.find("i ") == 0) {
      size_t it = line.find("n\"");
      if (it == std::string::npos)
        LOG(FATAL) << "ParseHuginControlPoints(): Invalid line: " << line;
      it += 2;
      std::string image;
      while (it < line.size() && line[it] != '"') {
        image += line[it];
        it++;
      }
      (*images).push_back(image);
    }

    // Parse control points
    if (line.find("c ") == 0) {
      // First wipe all letters
      std::string orig_line = line;
      char * ptr = const_cast<char*>(line.c_str());
      for (size_t i = 0; i < line.size(); i++) {
        // Wipe some extra chars
        if ( (ptr[i] >= 'a' && ptr[i] <= 'z') ||
             (ptr[i] >= 'A' && ptr[i] <= 'Z') )
          ptr[i] = ' ';
      }

      // Out of a line like:
      // c n0 N1 x367 y240 X144.183010710425 Y243.04008545843 t0
      // we store the numbers, 0, 1, 367, 240, 144.183010710425 243.04008545843
      // as a column.
      // The stand for left image index, right image index,
      // left image x, left image y, right image x, right image y.
      double a, b, c, d, e, f;
      if (sscanf(ptr, "%lf %lf %lf %lf %lf %lf", &a, &b, &c, &d, &e, &f) != 6)
        LOG(FATAL) << "ParseHuginControlPoints(): Could not scan line: " << line;

      // The left and right images must be different
      if (a == b)
        LOG(FATAL) << "The left and right images must be distinct. "
                   << "Offending line in " << hugin_file << " is:\n"
                   << orig_line << "\n";

      num_points++;
      (*points).conservativeResize(Eigen::NoChange_t(), num_points);
      (*points).col(num_points-1) << a, b, c, d, e, f;
    }
  }
}

// Parse a file having on each line xyz coordinates
void ParseXYZ(std::string const& xyz_file,
                              Eigen::MatrixXd * xyz) {
  // Initialize the outputs
  *xyz = Eigen::MatrixXd(3, 1);

  std::ifstream hf(xyz_file.c_str());
  if (!hf.good())
    LOG(FATAL) << "ParseXYZ(): Could not open hugin file: " << xyz_file;

  int num_points = 0;
  std::string line;
  while (getline(hf, line)) {
    // Ignore lines starting with comments and empty lines
    if (line.find("#") == 0 || is_blank(line)) continue;

    // Apparently sometimes empty lines show up as if of length 1
    if (line.size() == 1)
      continue;

    // Replace commas with spaces
    char * ptr = const_cast<char*>(line.c_str());
    for (size_t c = 0; c < line.size(); c++)
      if (ptr[c] == ',') ptr[c] = ' ';
    double x, y, z;
    if (sscanf(line.c_str(), "%lf %lf %lf", &x, &y, &z) != 3)
      LOG(FATAL) << "ParseXYZ(): Could not scan line: '" << line << "'\n";

    num_points++;
    (*xyz).conservativeResize(Eigen::NoChange_t(), num_points);
    (*xyz).col(num_points-1) << x, y, z;
  }
}

// Parse a CSV file, with the first line having column names. Return
// the results as columns in an std::map, with the column name being the
// key. We assume all values are numbers (non-numbers are set to 0).
void ParseCSV(std::string const& csv_file,
                              std::map< std::string, std::vector<double> > *cols) {
  // Initialize the output
  (*cols).clear();

  std::ifstream cf(csv_file.c_str());
  if (!cf.good())
    LOG(FATAL) << "ParseCSV(): Could not open file: " << csv_file;

  int line_pos = -1;
  std::vector<std::string> col_names;
  std::string line;
  while (getline(cf, line)) {
    line_pos++;

    // Replace leading % or # in the first line.
    // Ignore other lines starting with this field
    char * ptr = const_cast<char*>(line.c_str());
    if (line.find("#") == 0 || line.find("%") == 0) {
      if (line_pos == 0)
        ptr[0] = ' ';
      else
        continue;
    }

    // Skip empty lines
    if (is_blank(line)) continue;

    // Replace commas with spaces
    for (size_t c = 0; c < line.size(); c++)
      if (ptr[c] == ',') ptr[c] = ' ';

    // Parse the fields in the line
    std::istringstream is(line);
    std::string val;
    int col_pos = -1;
    while (is >> val) {
      col_pos++;

      // Initialize columns
      if (line_pos == 0) {
        col_names.push_back(val);
        (*cols)[val] = std::vector<double>();
        continue;
      }

      // Convert value to double (invalid values will be 0).
      double v = atof(val.c_str());
      if (col_pos < static_cast<int>(col_names.size()))
        (*cols)[col_names[col_pos]].push_back(v);
    }
  }

  return;
}

// Write the BAL format.
bool WriteBAL(const std::string& filename,
                              camera::CameraParameters const& camera_params,
                              std::vector<std::map<int, int> > const& pid_to_cid_fid,
                              std::vector<Eigen::Vector3d> const& pid_to_xyz,
                              std::vector<Eigen::Affine3d> const& cid_to_cam_t_global,
                              std::vector<Eigen::Matrix2Xd > const& cid_to_keypoint_map) {
  std::ofstream os;
  os.open(filename.c_str());
  os.precision(20);
  if (!os.is_open()) {
    LOG(ERROR) << "WriteBAL: cannot open the file.";
    return false;
  }

  LOG(INFO) << "Writing: " << filename << std::endl;

  // Write the number of camera poses and 3D points
  size_t nrObservations = 0;
  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++)
    nrObservations += pid_to_cid_fid[pid].size();

  // Write observations
  os << cid_to_cam_t_global.size() << " " << pid_to_cid_fid.size() << " "
     << nrObservations << std::endl;
  os << std::endl;

  for (size_t pid = 0; pid < pid_to_xyz.size(); pid++) {
    std::map<int, int> const& track = pid_to_cid_fid[pid];

    for (std::map<int, int>::const_iterator it = track.begin(); it != track.end(); it++) {
      int cid = it->first;
      int fid = it->second;

      Eigen::Vector2d pt = cid_to_keypoint_map[cid].col(fid);
      os << cid /*camera id*/<< " " << pid /*point id*/<< " "
         << -pt[0] << ' ' << pt[1] << std::endl;  // need minus due to bal conventions
    }
  }
  os << std::endl;

  // One more place at which to deal with bal conversion. The camera plane
  // is on the negative z axis.
  Eigen::Matrix3d T; T << -1, 0, 0, 0, 1, 0, 0, 0, -1;

  // Write cameras
  double k1 = 0, k2 = 0;  // no distortion
  for (size_t i = 0; i < cid_to_cam_t_global.size(); i++) {
    Eigen::Vector3d vec;
    camera::RotationToRodrigues(T*(cid_to_cam_t_global.at(i).linear()),
                                &vec);

    os << vec.transpose() << std::endl;
    os << (T*(cid_to_cam_t_global.at(i).translation())).transpose() << std::endl;
    os << camera_params.GetFocalLength() << std::endl;
    os << k1 << std::endl;
    os << k2 << std::endl;
    os << std::endl;
  }

  // Write the points
  for (size_t j = 0; j < pid_to_xyz.size(); j++) {
    Eigen::Vector3d point = pid_to_xyz[j];
    os << point[0] << std::endl;
    os << point[1] << std::endl;
    os << point[2] << std::endl;
    os << std::endl;
  }

  os.close();
  return true;
}

// From pid_to_cid_fid, create cid_fid_to_pid for lookup.
// TODO(rsoussan): Move this to sparse_map_database!!
void InitializeCidFidToPid(int num_cid,
                           std::vector<std::map<int, int> > const& pid_to_cid_fid,
                           std::vector<std::map<int, int> > * cid_fid_to_pid) {
  cid_fid_to_pid->clear();
  cid_fid_to_pid->resize(num_cid, std::map<int, int>());

  for (size_t pid = 0; pid < pid_to_cid_fid.size(); pid++) {
    for (std::pair<int, int> const& cid_fid : pid_to_cid_fid[pid]) {
      (*cid_fid_to_pid)[cid_fid.first][cid_fid.second] = pid;
    }
  }
}

cv::Mat LoadImage(const std::string& filename) {
  const cv::Mat image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
  if (image.rows == 0 || image.cols == 0)
    LOG(FATAL) << "Found empty image in file: " << filename;
  return image;
}

template <class IterT>
void WriteCIDPairAffineIterator(IterT it,
                                IterT end,
                                std::ofstream* file) {
  Eigen::IOFormat fmt(Eigen::FullPrecision, 0, " ", "\n", "", "", "", "");
  while (it != end) {
    *file << it->first.first << " " << it->first.second << std::endl;
    *file << it->second.affine().format(fmt) << std::endl;
    it++;
  }
}

template <class IterT>
void ReadAffine(std::ifstream* file,
                IterT output_iter) {
  std::string line[4];
  std::getline(*file, line[0]);
  std::getline(*file, line[1]);
  std::getline(*file, line[2]);
  std::getline(*file, line[3]);
  if (line[0].empty())
    return;

  int i, j;
  Eigen::Matrix3d r;
  Eigen::Vector3d t;
  {
    std::stringstream ss(line[0]);
    ss >> i >> j;
  }

  for (int k = 0; k < 3; k++) {
    std::stringstream ss(line[k + 1]);
    ss >> r(k, 0) >> r(k, 1) >> r(k, 2) >> t[k];
  }

  Eigen::Affine3d affine;
  affine.linear() = r;
  affine.translation() = t;
  *output_iter = std::make_pair(std::make_pair(i, j),
                                affine);
}

// Use a back inserter with this if you haven't previously allocated enough space.
template <class IterT>
void PushBackCIDPairAffine(std::ifstream* file,
                           IterT output_iter,
                           IterT output_iter_end) {
  do {
    ReadAffine(file, output_iter);
    output_iter++;
  } while (file->good() && output_iter != output_iter_end);
}

template <class IterT>
void PushBackCIDPairAffine(std::ifstream* file,
                           IterT iter) {
  do {
    ReadAffine(file, iter);
    iter++;
  } while (file->good());
}

void WriteAffineCSV(CIDPairAffineMap const& relative_affines,
                    std::string const& output_filename) {
  LOG(INFO) << "Writing: " << output_filename;
  std::ofstream f(output_filename, std::ofstream::out);
  WriteCIDPairAffineIterator(relative_affines.begin(),
                             relative_affines.end(),
                             &f);
  f.close();
}
void WriteAffineCSV(CIDAffineTupleVec const& relative_affines,
                    std::string const& output_filename) {
  LOG(INFO) << "Writing: " << output_filename;
  std::ofstream f(output_filename, std::ofstream::out);
  for (CIDAffineTupleVec::value_type const& tuple : relative_affines) {
    f << "Tuple:" << std::endl;
    WriteCIDPairAffineIterator(tuple.begin(), tuple.end(), &f);
  }
  f.close();
}
void ReadAffineCSV(std::string const& input_filename,
                   CIDPairAffineMap* relative_affines) {
  LOG(INFO) << "Reading: " << input_filename;
  std::ifstream f(input_filename, std::ifstream::in);
  if (!f.good())
    LOG(FATAL) << "Could no read: " << input_filename << ". Must redo the matching step.";
  relative_affines->clear();
  PushBackCIDPairAffine(&f, std::inserter(*relative_affines, relative_affines->begin()));
  f.close();
}
void ReadAffineCSV(std::string const& input_filename,
                   CIDAffineTupleVec* relative_affines) {
  std::ifstream f(input_filename, std::ifstream::in);
  if (!f.good())
    LOG(FATAL) << "Could no read: " << input_filename << ". Must redo the matching step.";
  relative_affines->clear();
  std::string line;
  std::getline(f, line);
  while (!line.empty()) {
    relative_affines->push_back({});
    PushBackCIDPairAffine(&f, relative_affines->back().begin(),
                          relative_affines->back().end());
    std::getline(f, line);
  }
  f.close();
}
}  // namespace sparse_mapping
