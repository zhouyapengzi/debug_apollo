#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tool/benchmark/lidar/eval/position_metric.h"

#include <algorithm>
#include <vector>

#include "absl/strings/str_cat.h"

namespace apollo {
namespace perception {
namespace benchmark {

void PositionMetric::cal_position_metric(const ObjectPtr& object,
                                         const PositionMetricOption& option) {
    AINFO<<"(DMCZP) EnteringMethod: PositionMetric::cal_position_metric";

  if (object->cloud->points.empty()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: PositionMetric::cal_position_metric";
  return;
  }
  std::vector<double> distance;
  Eigen::Vector2d center(0.0, 0.0);
  distance.reserve(object->cloud->size());
  for (auto& point : object->cloud->points) {
    distance.push_back(sqrt(point.x * point.x + point.y * point.y));
    center(0) += point.x;
    center(1) += point.y;
  }
  center /= static_cast<int>(object->cloud->size());
  std::sort(distance.begin(), distance.end());
  unsigned int sample1 = 0;
  unsigned int sample2 =
      static_cast<unsigned int>(0.2 * static_cast<int>(distance.size()));

  radial_distance = 0.5 * (distance[sample1] + distance[sample2]);
  horizontal_distance = fabs(center(1));
  vertical_distance = center(0);
  angle =
      acos(center.dot(Eigen::Vector2d(1, 0)) / center.norm()) * 180.0 / M_PI;
  is_in_roi =
      option.roi_is_main_lanes ? object->is_in_main_lanes : object->is_in_roi;
  is_valid = true;

   AINFO<<"(DMCZP) LeaveMethod: PositionMetric::cal_position_metric";
 }

double DistanceBasedRangeInterface::_s_distance = 60.0;
double DistanceBasedRangeInterface::_s_half_distance = 30.0;

void DistanceBasedRangeInterface::set_distance(double distance) {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeInterface::set_distance";

  _s_distance = distance;
  _s_half_distance = 0.5 * distance;

   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeInterface::set_distance";
 }

unsigned int DistanceBasedRangeInterface::get_index(
    const PositionMetric& position) const {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeInterface::get_index";

  // unsigned int index = static_cast<unsigned int>(position.radial_distance /
  // 30.0);
  // index = index > 2 ? 2 : index;
  // 
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_index";
  return index;
  unsigned int index =
      static_cast<unsigned int>(position.radial_distance / _s_half_distance);
  index = index > 2 ? 2 : index;
  
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_index";
  return index;

   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeInterface::get_index";
 }

unsigned int DistanceBasedRangeInterface::get_dim() const {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeInterface::get_dim";
 
  AINFO<<
   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeInterface::get_dim";
 "(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_dim";
  return 3; }

std::string DistanceBasedRangeInterface::get_element(unsigned int index) const {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeInterface::get_element";

  if (index >= get_dim()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_element";
  return "Total";
  } else {
    switch (index) {
      case 0:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_element";
  return absl::StrCat("0 - ", static_cast<int>(_s_half_distance), "m");
      case 1:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_element";
  return absl::StrCat(static_cast<int>(_s_half_distance), " - ",
                            static_cast<int>(_s_distance), "m");
      default:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeInterface::get_element";
  return absl::StrCat(static_cast<int>(_s_distance), " - 500m");
    }
  }

   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeInterface::get_element";
 }

unsigned int DistanceBasedRangeRadarInterface::get_index(
    const PositionMetric& position) const {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeRadarInterface::get_index";

  // unsigned int index = static_cast<unsigned int>(position.radial_distance /
  // 30.0);
  // index = index > 2 ? 2 : index;
  // 
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
  return index;
  unsigned int distance = static_cast<unsigned int>(position.radial_distance);
  if (distance < 30) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
  return 0;
  } else if (distance < 60) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
  return 1;
  } else if (distance < 120) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
  return 2;
  } else if (distance < 200) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
  return 3;
  } else {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
  return 4;
  }

   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeRadarInterface::get_index";
 }

unsigned int DistanceBasedRangeRadarInterface::get_dim() const {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeRadarInterface::get_dim";
 
  AINFO<<
   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeRadarInterface::get_dim";
 "(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_dim";
  return 5; }

std::string DistanceBasedRangeRadarInterface::get_element(
    unsigned int index) const {
    AINFO<<"(DMCZP) EnteringMethod: DistanceBasedRangeRadarInterface::get_element";

  if (index >= get_dim()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
  return "Total";
  } else {
    switch (index) {
      case 0:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
  return "  0 - 30m";
      case 1:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
  return " 30 - 60m";
      case 2:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
  return " 60 - 120m";
      case 3:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
  return " 120 - 200m";
      default:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
  return "200 - 500m";
    }
  }

   AINFO<<"(DMCZP) LeaveMethod: DistanceBasedRangeRadarInterface::get_element";
 }

double ViewBasedRangeInterface::_s_front_view_angle = 60.0;
double ViewBasedRangeInterface::_s_rear_view_angle = 60.0;
double ViewBasedRangeInterface::_s_front_view_distance = 60.0;
double ViewBasedRangeInterface::_s_rear_view_distance = 60.0;

void ViewBasedRangeInterface::set_front_view_angle(double angle) {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::set_front_view_angle";

  _s_front_view_angle = angle;

   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::set_front_view_angle";
 }

void ViewBasedRangeInterface::set_rear_view_angle(double angle) {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::set_rear_view_angle";

  _s_rear_view_angle = angle;

   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::set_rear_view_angle";
 }

void ViewBasedRangeInterface::set_front_view_distance(double distance) {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::set_front_view_distance";

  _s_front_view_distance = distance;

   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::set_front_view_distance";
 }

void ViewBasedRangeInterface::set_rear_view_distance(double distance) {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::set_rear_view_distance";

  _s_rear_view_distance = distance;

   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::set_rear_view_distance";
 }

unsigned int ViewBasedRangeInterface::get_index(
    const PositionMetric& position) const {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::get_index";

  if (position.radial_distance >= 60.0) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 3;
  }
  // if (position.angle <= _s_front_view_angle * 0.5) {
  //    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 0;
  //} else if (position.angle >= 180.0 - _s_rear_view_angle * 0.5) {
  //    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 1;
  //} else if (position.horizontal_distance <= 30.0) {
  //    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 2;
  //} else {
  //    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 3;
  //}
  if (position.angle <= _s_front_view_angle * 0.5 &&
      position.radial_distance <= _s_front_view_distance) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 0;
  } else if (position.angle >= 180.0 - _s_rear_view_angle * 0.5 &&
             position.radial_distance <= _s_rear_view_distance) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 1;
  } else if (position.horizontal_distance <= 30 &&
             position.radial_distance <= 60) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 2;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_index";
  return 3;

   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::get_index";
 }

unsigned int ViewBasedRangeInterface::get_dim() const {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::get_dim";
 
  AINFO<<
   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::get_dim";
 "(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_dim";
  return 4; }

std::string ViewBasedRangeInterface::get_element(unsigned int index) const {
    AINFO<<"(DMCZP) EnteringMethod: ViewBasedRangeInterface::get_element";

  if (index >= get_dim()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_element";
  return "Total";
  } else {
    switch (index) {
      case 0:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_element";
  return absl::StrCat("Front(", static_cast<int>(_s_front_view_angle),
                            "d", static_cast<int>(_s_front_view_distance),
                            "m)");
      case 1:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_element";
  return absl::StrCat("Rear(", static_cast<int>(_s_rear_view_angle), "d",
                            static_cast<int>(_s_rear_view_distance), "m)");
      case 2:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_element";
  return "Side(30m)";
      default:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: ViewBasedRangeInterface::get_element";
  return "Outside  ";
    }
  }

   AINFO<<"(DMCZP) LeaveMethod: ViewBasedRangeInterface::get_element";
 }

double BoxBasedRangeInterface::_s_front_box_distance = 120.0;
double BoxBasedRangeInterface::_s_rear_box_distance = 45.0;
double BoxBasedRangeInterface::_s_side_box_distance = 10.0;

void BoxBasedRangeInterface::set_front_box_distance(double distance) {
    AINFO<<"(DMCZP) EnteringMethod: BoxBasedRangeInterface::set_front_box_distance";

  _s_front_box_distance = distance;

   AINFO<<"(DMCZP) LeaveMethod: BoxBasedRangeInterface::set_front_box_distance";
 }

void BoxBasedRangeInterface::set_rear_box_distance(double distance) {
    AINFO<<"(DMCZP) EnteringMethod: BoxBasedRangeInterface::set_rear_box_distance";

  _s_rear_box_distance = distance;

   AINFO<<"(DMCZP) LeaveMethod: BoxBasedRangeInterface::set_rear_box_distance";
 }

void BoxBasedRangeInterface::set_side_box_distance(double distance) {
    AINFO<<"(DMCZP) EnteringMethod: BoxBasedRangeInterface::set_side_box_distance";

  _s_side_box_distance = distance;

   AINFO<<"(DMCZP) LeaveMethod: BoxBasedRangeInterface::set_side_box_distance";
 }

unsigned int BoxBasedRangeInterface::get_index(
    const PositionMetric& position) const {
    AINFO<<"(DMCZP) EnteringMethod: BoxBasedRangeInterface::get_index";

  if (position.vertical_distance <= _s_front_box_distance &&
      position.vertical_distance >= -_s_rear_box_distance &&
      position.horizontal_distance <= _s_side_box_distance) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: BoxBasedRangeInterface::get_index";
  return 0;
  } else {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: BoxBasedRangeInterface::get_index";
  return 1;
  }

   AINFO<<"(DMCZP) LeaveMethod: BoxBasedRangeInterface::get_index";
 }

unsigned int BoxBasedRangeInterface::get_dim() const {
    AINFO<<"(DMCZP) EnteringMethod: BoxBasedRangeInterface::get_dim";
 
  AINFO<<
   AINFO<<"(DMCZP) LeaveMethod: BoxBasedRangeInterface::get_dim";
 "(DMCZP) (return) LeaveMethod: BoxBasedRangeInterface::get_dim";
  return 2; }

std::string BoxBasedRangeInterface::get_element(unsigned int index) const {
    AINFO<<"(DMCZP) EnteringMethod: BoxBasedRangeInterface::get_element";

  if (index >= get_dim()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: BoxBasedRangeInterface::get_element";
  return "Total";
  } else {
    switch (index) {
      case 0:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: BoxBasedRangeInterface::get_element";
  return absl::StrCat("Box(F", static_cast<int>(_s_front_box_distance),
                            "R", static_cast<int>(_s_rear_box_distance), "S",
                            static_cast<int>(_s_side_box_distance), ")");
      case 1:
      default:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: BoxBasedRangeInterface::get_element";
  return "Outside-box";
    }
  }

   AINFO<<"(DMCZP) LeaveMethod: BoxBasedRangeInterface::get_element";
 }

bool RoiDistanceBasedRangeInterface::_s_ignore_roi_outside = false;

void RoiDistanceBasedRangeInterface::set_ignore_roi_outside(bool ignore) {
    AINFO<<"(DMCZP) EnteringMethod: RoiDistanceBasedRangeInterface::set_ignore_roi_outside";

  _s_ignore_roi_outside = ignore;

   AINFO<<"(DMCZP) LeaveMethod: RoiDistanceBasedRangeInterface::set_ignore_roi_outside";
 }

unsigned int RoiDistanceBasedRangeInterface::get_index(
    const PositionMetric& position) const {
    AINFO<<"(DMCZP) EnteringMethod: RoiDistanceBasedRangeInterface::get_index";

  unsigned int index =
      static_cast<unsigned int>(position.radial_distance / _s_half_distance);
  if (_s_ignore_roi_outside) {
    index = (index > 2 || !position.is_in_roi) ? 2 : index;
  } else {
    index = index > 1 ? 4 : index;
    if (index != 4) {
      index = position.is_in_roi ? index : index + 2;
    }
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_index";
  return index;

   AINFO<<"(DMCZP) LeaveMethod: RoiDistanceBasedRangeInterface::get_index";
 }

unsigned int RoiDistanceBasedRangeInterface::get_dim() const {
    AINFO<<"(DMCZP) EnteringMethod: RoiDistanceBasedRangeInterface::get_dim";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_dim";
  return _s_ignore_roi_outside ? 3 : 5;

   AINFO<<"(DMCZP) LeaveMethod: RoiDistanceBasedRangeInterface::get_dim";
 }

std::string RoiDistanceBasedRangeInterface::get_element(
    unsigned int index) const {
    AINFO<<"(DMCZP) EnteringMethod: RoiDistanceBasedRangeInterface::get_element";

  if (index >= get_dim()) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return "Total";
  }
  if (_s_ignore_roi_outside) {
    switch (index) {
      case 0:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat("I.0-", static_cast<int>(_s_half_distance), "m");
      case 1:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat("I.", static_cast<int>(_s_half_distance), "-",
                            static_cast<int>(_s_distance), "m");
      default:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return "Outside";
    }
  } else {
    switch (index) {
      case 0:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat("I.0-", static_cast<int>(_s_half_distance), "m");
      case 1:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat("I.", static_cast<int>(_s_half_distance), "-",
                            static_cast<int>(_s_distance), "m");
      case 2:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat("O.0-", static_cast<int>(_s_half_distance), "m");
      case 3:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat("O.", static_cast<int>(_s_half_distance), "-",
                            static_cast<int>(_s_distance), "m");
      default:
        
  AINFO<<"(DMCZP) (return) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
  return absl::StrCat(static_cast<int>(_s_distance), "-500m");
    }
  }

   AINFO<<"(DMCZP) LeaveMethod: RoiDistanceBasedRangeInterface::get_element";
 }

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
