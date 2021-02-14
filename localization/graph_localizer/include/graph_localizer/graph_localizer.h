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

#ifndef GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
#define GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_

#include <graph_localizer/factor_to_add.h>
#include <graph_localizer/feature_tracker.h>
#include <graph_localizer/graph_action.h>
#include <graph_localizer/graph_localizer_params.h>
#include <graph_localizer/graph_stats.h>
#include <graph_localizer/graph_values.h>
#include <graph_localizer/key_info.h>
#include <graph_localizer/robust_smart_projection_pose_factor.h>
#include <graph_localizer/loc_factor_adder.h>
#include <graph_localizer/projection_factor_adder.h>
#include <graph_localizer/rotation_factor_adder.h>
#include <graph_localizer/smart_projection_cumulative_factor_adder.h>
#include <graph_localizer/standstill_factor_adder.h>
#include <imu_integration/latest_imu_integrator.h>
#include <imu_integration/fan_speed_mode.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_points_measurement.h>
#include <localization_measurements/matched_projections_measurement.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartFactorParams.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <boost/serialization/serialization.hpp>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
using Calibration = gtsam::Cal3_S2;
using Camera = gtsam::PinholePose<Calibration>;
using RobustSmartFactor = gtsam::RobustSmartProjectionPoseFactor<Calibration>;
using SharedRobustSmartFactor = boost::shared_ptr<RobustSmartFactor>;
using ProjectionFactor = gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3>;

class GraphLocalizer {
 public:
  explicit GraphLocalizer(const GraphLocalizerParams& params);
  // For Serialization Only
  GraphLocalizer() {}
  ~GraphLocalizer();
  void AddImuMeasurement(const localization_measurements::ImuMeasurement& imu_measurement);
  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;
  boost::optional<localization_common::CombinedNavState> GetCombinedNavState(
    const localization_common::Time time) const;
  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances() const;
  bool AddOpticalFlowMeasurement(
    const localization_measurements::FeaturePointsMeasurement& optical_flow_feature_points_measurement);
  bool TriangulateNewPoint(FactorsToAdd& factors_to_add);
  bool LocProjectionNoiseScaling(FactorsToAdd& factors_to_add);
  bool ARProjectionNoiseScaling(FactorsToAdd& factors_to_add);
  bool MapProjectionNoiseScaling(const LocFactorAdderParams& params, FactorsToAdd& factors_to_add);
  void CheckForStandstill();
  void AddARTagMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  void AddSparseMappingMeasurement(
    const localization_measurements::MatchedProjectionsMeasurement& matched_projections_measurement);
  // Attempts to remove most recent or oldest measurements to make and invalid smart factor valid
  // TODO(rsoussan): Move this to SmartProjectionFactorAdder or utilities!
  void SplitSmartFactorsIfNeeded(FactorsToAdd& factors_to_add);

  bool Update();
  const FeatureTrackMap& feature_tracks() const { return feature_tracker_->feature_tracks(); }

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBiases() const;

  boost::optional<localization_common::Time> LatestExtrapolatedPoseTime() const;

  int NumFeatures() const;

  int NumOFFactors(const bool check_valid = true) const;

  int NumSmartFactors(const bool check_valid = true) const;

  int NumProjectionFactors(const bool check_valid = true) const;

  const GraphValues& graph_values() const;

  const gtsam::NonlinearFactorGraph& factor_graph() const;

  void SaveGraphDotFile(const std::string& output_path = "graph.dot") const;

  bool standstill() const;

  const GraphLocalizerParams& params() const;

  template <typename FactorType>
  int NumFactors() const {
    int num_factors = 0;
    for (const auto& factor : graph_) {
      if (dynamic_cast<const FactorType*>(factor.get())) {
        ++num_factors;
      }
    }
    return num_factors;
  }

  void LogOnDestruction(const bool log_on_destruction);

  const GraphStats& graph_stats() const;

  void SetFanSpeedMode(const imu_integration::FanSpeedMode fan_speed_mode);

 private:
  gtsam::NonlinearFactorGraph MarginalFactors(const gtsam::NonlinearFactorGraph& old_factors,
                                              const gtsam::KeyVector& old_keys,
                                              const gtsam::GaussianFactorGraph::Eliminate& eliminate_function) const;

  // Removes Keys and Values outside of sliding window.
  // Removes any factors depending on removed values
  // Optionally adds marginalized factors encapsulating linearized error of removed factors
  // Optionally adds priors using marginalized covariances for new oldest states
  bool SlideWindow(const boost::optional<gtsam::Marginals>& marginals,
                   const localization_common::Time last_latest_time);

  void UpdatePointPriors(const gtsam::Marginals& marginals);

  void RemovePriors(const int key_index);

  // Integrates latest imu measurements up to timestamp and adds imu factor and
  // new combined nav state
  bool CreateAndAddLatestImuFactorAndCombinedNavState(const localization_common::Time timestamp);

  bool AddOrSplitImuFactorIfNeeded(const localization_common::Time timestamp);

  bool SplitOldImuFactorAndAddCombinedNavState(const localization_common::Time timestamp);

  void AddStartingPriors(const localization_common::CombinedNavState& global_N_body_start, const int key_index,
                         gtsam::NonlinearFactorGraph& graph);

  void AddPriors(const localization_common::CombinedNavState& global_N_body,
                 const localization_common::CombinedNavStateNoise& noise, const int key_index,
                 gtsam::NonlinearFactorGraph& graph);

  boost::optional<std::pair<localization_common::CombinedNavState, localization_common::CombinedNavStateCovariances>>
  LatestCombinedNavStateAndCovariances(const gtsam::Marginals& marginals) const;

  bool CreateAndAddImuFactorAndPredictedCombinedNavState(const localization_common::CombinedNavState& global_N_body,
                                                         const gtsam::PreintegratedCombinedMeasurements& pim);

  void BufferFactors(const std::vector<FactorsToAdd>& factors_to_add_vec);

  void BufferCumulativeFactors();

  void RemoveOldMeasurementsFromCumulativeFactors(const gtsam::KeyVector& old_keys);

  int AddBufferedFactors();

  bool DoGraphAction(FactorsToAdd& factors_to_add);

  bool Rekey(FactorToAdd& factor_to_add);

  bool ReadyToAddMeasurement(const localization_common::Time timestamp) const;

  bool MeasurementRecentEnough(const localization_common::Time timestamp) const;

  void RemoveOldBufferedFactors(const localization_common::Time oldest_allowed_timestamp);

  std::vector<localization_common::Time> TimestampsToAdd(const localization_common::Time timestamp,
                                                         const localization_common::Time last_added_timestamp);

  template <typename FactorType>
  void DeleteFactors() {
    int num_removed_factors = 0;
    for (auto factor_it = graph_.begin(); factor_it != graph_.end();) {
      if (dynamic_cast<FactorType*>(factor_it->get())) {
        factor_it = graph_.erase(factor_it);
        ++num_removed_factors;
        continue;
      }
      ++factor_it;
    }
    LogDebug("DeleteFactors: Num removed factors: " << num_removed_factors);
  }

  // TODO(rsoussan): make a static and dynamic key index?
  static int GenerateKeyIndex() {
    static int key_index = 0;
    return key_index++;
  }

  void PrintFactorDebugInfo() const;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(feature_tracker_);
    ar& BOOST_SERIALIZATION_NVP(graph_);
    ar& BOOST_SERIALIZATION_NVP(graph_values_);
  }

  std::shared_ptr<FeatureTracker> feature_tracker_;
  imu_integration::LatestImuIntegrator latest_imu_integrator_;
  std::shared_ptr<GraphValues> graph_values_;
  bool log_on_destruction_;
  GraphLocalizerParams params_;
  gtsam::LevenbergMarquardtParams levenberg_marquardt_params_;
  gtsam::TriangulationParameters projection_triangulation_params_;
  gtsam::SmartProjectionParams smart_projection_params_;
  gtsam::NonlinearFactorGraph graph_;
  boost::optional<gtsam::Marginals> marginals_;
  boost::optional<localization_measurements::FeaturePointsMeasurement> last_optical_flow_measurement_;
  std::multimap<localization_common::Time, FactorsToAdd> buffered_factors_to_add_;

  // Factor Adders
  std::unique_ptr<LocFactorAdder> ar_tag_loc_factor_adder_;
  std::unique_ptr<LocFactorAdder> loc_factor_adder_;
  std::unique_ptr<ProjectionFactorAdder> projection_factor_adder_;
  std::unique_ptr<RotationFactorAdder> rotation_factor_adder_;
  std::unique_ptr<SmartProjectionCumulativeFactorAdder> smart_projection_cumulative_factor_adder_;
  std::unique_ptr<StandstillFactorAdder> standstill_factor_adder_;

  gtsam::Marginals::Factorization marginals_factorization_;
  boost::optional<bool> standstill_;
  boost::optional<localization_common::Time> last_latest_time_;
  GraphStats graph_stats_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_LOCALIZER_H_
