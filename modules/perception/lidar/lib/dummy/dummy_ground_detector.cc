#include "cyber/common/log.h"
/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/lidar/lib/dummy/dummy_ground_detector.h"

namespace apollo {
namespace perception {
namespace lidar {

bool DummyGroundDetector::Init(const GroundDetectorInitOptions& options) {
    AINFO<<"(DMCZP) EnteringMethod: DummyGroundDetector::Init";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: DummyGroundDetector::Init";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: DummyGroundDetector::Init";
 }

bool DummyGroundDetector::Detect(const GroundDetectorOptions& options,
                                 LidarFrame* frame) {
    AINFO<<"(DMCZP) EnteringMethod: DummyGroundDetector::Detect";

  if (!frame) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DummyGroundDetector::Detect";
  return false;
  }
  if (!frame->cloud) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: DummyGroundDetector::Detect";
  return false;
  }
  frame->non_ground_indices.indices.resize(frame->cloud->size());
  std::iota(frame->non_ground_indices.indices.begin(),
            frame->non_ground_indices.indices.end(), 0);

  
  AINFO<<"(DMCZP) (return) LeaveMethod: DummyGroundDetector::Detect";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: DummyGroundDetector::Detect";
 }

PERCEPTION_REGISTER_GROUNDDETECTOR(DummyGroundDetector);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
