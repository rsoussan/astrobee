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

#ifndef SPARSE_MAPPING_SPARSE_MAP_MERGER_H_
#define SPARSE_MAPPING_SPARSE_MAP_MERGER_H_

#include <sparse_mapping/sparse_map_merger_params.h>
#include <sparse_mapping/SparseMap.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sparse_mapping {
enum class TrackLabel {
  kAppend,
  kInvalid,
  kMerge,
  kNew,
};

struct MatchingTracks {
  std::vector<std::map<int, int> > pid_to_cid_fid;
  std::vector<TrackLabel> track_labels;
  std::vector<std::pair<int, int>> a_b_pid_correspondences;
}

class SparseMapMerger {
 public:
  SparseMapMerger(const SparseMap& map_a, const SparseMap& map_b, const SparseMapMergerParams& params);

  SparseMapMerger(const std::string& map_a_filename, const std::string& map_b_filename,
                  const SparseMapMergerParams& params);


  void MergeMaps();

 private:
  void Initialize(const SparseMap& map_a, const SparseMap& map_b, const SparseMapMergerParams& params);
  bool CompatableMaps() const;
  void IdentifyTrack(const std::map<int, int>& track, const int index, const SparseMap& map_a, const SparseMap& map_b,
                     std::vector<TrackLabel>& track_labels,
                     std::vector<std::pair<int, int>>& a_b_pid_tracks_to_merge) const;

// Uses the range of the point axes to find an inlier threshold.
// Computes a low and high index and uses the scaled distance between sorted values
// for each point axis at these indices to calculate the threshold.
  double InlierThreshold(const std::vector<Eigen::Vector3d>& points) const;
  // Returns map_a_T_map_b
  Eigen::Affine3d EstimateRelativePoseAndPruneOutlierMatches(const SparseMap& map_a, const SparseMap& map_b,
                                                  MatchingTracks& matching_tracks) const;
  boost::optional<int> BestMatch(const std::map<int, int>& pid_match_counts) const;
  MatchingTracks MatchingTracks(const SparseMap& map_a, const SparseMap& map_b, SparseMap& merged_map);
  void AddTracksToMerge(const std::vector<std::pair<int, int>>& a_b_pid_correspondences);

  SparseMapMergerParams params_;
  std::unique_ptr<SparseMap> map_a_;
  std::unique_ptr<SparseMap> map_b_;
  std::unique_ptr<SparseMap> merged_map_;
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_MERGER_H_
