#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file: st_graph_point.cc
 **/

#include "modules/planning/tasks/optimizers/path_time_heuristic/st_graph_point.h"

#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

std::uint32_t StGraphPoint::index_s() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::index_s";
 return index_s_; }

std::uint32_t StGraphPoint::index_t() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::index_t";
 return index_t_; }

const STPoint& StGraphPoint::point() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::point";
 return point_; }

const StGraphPoint* StGraphPoint::pre_point() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::pre_point";
 return pre_point_; }

double StGraphPoint::reference_cost() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::reference_cost";
 return reference_cost_; }

double StGraphPoint::obstacle_cost() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::obstacle_cost";
 return obstacle_cost_; }

double StGraphPoint::spatial_potential_cost() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::spatial_potential_cost";

  return spatial_potential_cost_;
}

double StGraphPoint::total_cost() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::total_cost";
 return total_cost_; }

void StGraphPoint::Init(const std::uint32_t index_t,
                        const std::uint32_t index_s, const STPoint& st_point) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::Init";

  index_t_ = index_t;
  index_s_ = index_s;
  point_ = st_point;
}

void StGraphPoint::SetReferenceCost(const double reference_cost) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::SetReferenceCost";

  reference_cost_ = reference_cost;
}

void StGraphPoint::SetObstacleCost(const double obs_cost) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::SetObstacleCost";

  obstacle_cost_ = obs_cost;
}

void StGraphPoint::SetSpatialPotentialCost(
    const double spatial_potential_cost) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::SetSpatialPotentialCost";

  spatial_potential_cost_ = spatial_potential_cost;
}

void StGraphPoint::SetTotalCost(const double total_cost) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::SetTotalCost";

  total_cost_ = total_cost;
}

void StGraphPoint::SetPrePoint(const StGraphPoint& pre_point) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::SetPrePoint";

  pre_point_ = &pre_point;
}

double StGraphPoint::GetOptimalSpeed() const {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::GetOptimalSpeed";
 return optimal_speed_; }

void StGraphPoint::SetOptimalSpeed(const double optimal_speed) {
    AINFO<<"(DMCZP) EnteringMethod: StGraphPoint::SetOptimalSpeed";

  optimal_speed_ = optimal_speed;
}

}  // namespace planning
}  // namespace apollo
