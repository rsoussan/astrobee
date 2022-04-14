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
#include <vector>

namespace sparse_mapping {
enum class TrackLabel {
  kAppend,
  kInvalid,
  kMerge,
  kNew,
};

class SparseMapMerger {
 public:
  SparseMapMerger(const SparseMap& map_a, const SparseMap& map_b, const SparseMapMergerParams& params);

  SparseMapMerger(const std::string& map_a_filename, const std::string& map_b_filename,
                  const SparseMapMergerParams& params);

  void Initialize(const SparseMap& map_a, const SparseMap& map_b, const SparseMapMergerParams& params);

  void MergeMaps();

 private:
  bool CompatableMaps() const;
  void IdentifyTrack(const std::map<int, int>& track, const int index, const SparseMap& map_a, const SparseMap& map_b,
                     std::vector<TrackLabel>& track_labels,
                     std::unordered_map<int, std::map<int, int>>& b_pid_to_a_pid_match_counts) const;
  void MatchingTracks(const SparseMap& map_a, const SparseMap& map_b, SparseMap& merged_map);

  SparseMapMergerParams params_;
  std::unique_ptr<SparseMap> map_a_;
  std::unique_ptr<SparseMap> map_b_;
  std::unique_ptr<SparseMap> merged_map_;
};
}  // namespace sparse_mapping

#endif  // SPARSE_MAPPING_SPARSE_MAP_MERGER_H_
