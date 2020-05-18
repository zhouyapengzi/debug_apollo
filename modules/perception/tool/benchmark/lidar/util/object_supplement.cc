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
#include "modules/perception/tool/benchmark/lidar/util/object_supplement.h"

namespace apollo {
namespace perception {
namespace benchmark {

/**LidarSupplement implementation*/
LidarSupplement::LidarSupplement() {
    AINFO<<"(DMCZP) EnteringMethod: LidarSupplement::LidarSupplement";

   AINFO<<"(DMCZP) LeaveMethod: LidarSupplement::LidarSupplement";
 }

LidarSupplement::~LidarSupplement() {}

LidarSupplement::LidarSupplement(const LidarSupplement& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: LidarSupplement::LidarSupplement";

   AINFO<<"(DMCZP) LeaveMethod: LidarSupplement::LidarSupplement";
 }

LidarSupplement& LidarSupplement::operator=(const LidarSupplement& rhs) {
  return (*this);
}

void LidarSupplement::clone(const LidarSupplement& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: LidarSupplement::clone";

   AINFO<<"(DMCZP) LeaveMethod: LidarSupplement::clone";
 }

/**RadarSupplement implementation*/
RadarSupplement::RadarSupplement() {
    AINFO<<"(DMCZP) EnteringMethod: RadarSupplement::RadarSupplement";

   AINFO<<"(DMCZP) LeaveMethod: RadarSupplement::RadarSupplement";
 }

RadarSupplement::~RadarSupplement() {}

RadarSupplement::RadarSupplement(const RadarSupplement& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: RadarSupplement::RadarSupplement";

  range = rhs.range;
  angle = rhs.angle;
  relative_radial_velocity = rhs.relative_radial_velocity;
  relative_tangential_velocity = rhs.relative_tangential_velocity;

   AINFO<<"(DMCZP) LeaveMethod: RadarSupplement::RadarSupplement";
 }

RadarSupplement& RadarSupplement::operator=(const RadarSupplement& rhs) {
  range = rhs.range;
  angle = rhs.angle;
  relative_radial_velocity = rhs.relative_radial_velocity;
  relative_tangential_velocity = rhs.relative_tangential_velocity;
  return (*this);
}

void RadarSupplement::clone(const RadarSupplement& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: RadarSupplement::clone";

  range = rhs.range;
  angle = rhs.angle;
  relative_radial_velocity = rhs.relative_radial_velocity;
  relative_tangential_velocity = rhs.relative_tangential_velocity;

   AINFO<<"(DMCZP) LeaveMethod: RadarSupplement::clone";
 }

/**CameraSupplement implementation*/
CameraSupplement::CameraSupplement() {
    AINFO<<"(DMCZP) EnteringMethod: CameraSupplement::CameraSupplement";

   AINFO<<"(DMCZP) LeaveMethod: CameraSupplement::CameraSupplement";
 }

CameraSupplement::~CameraSupplement() {}

CameraSupplement::CameraSupplement(const CameraSupplement& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: CameraSupplement::CameraSupplement";

  upper_left = rhs.upper_left;
  lower_right = rhs.lower_right;

   AINFO<<"(DMCZP) LeaveMethod: CameraSupplement::CameraSupplement";
 }

CameraSupplement& CameraSupplement::operator=(const CameraSupplement& rhs) {
  upper_left = rhs.upper_left;
  lower_right = rhs.lower_right;
  return (*this);
}

void CameraSupplement::clone(const CameraSupplement& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: CameraSupplement::clone";

  upper_left = rhs.upper_left;
  lower_right = rhs.lower_right;

   AINFO<<"(DMCZP) LeaveMethod: CameraSupplement::clone";
 }

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
