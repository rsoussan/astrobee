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
std::string MatchesFile(std::string const& map_file) {
  return map_file + ".matches.txt";
}

std::string EssentialFile(std::string const& map_file) {
  return map_file + ".essential.csv";
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
