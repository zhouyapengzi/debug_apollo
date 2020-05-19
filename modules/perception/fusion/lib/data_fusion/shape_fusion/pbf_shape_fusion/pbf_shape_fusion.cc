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
#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

bool PbfShapeFusion::s_use_camera_3d_ = true;
float PbfShapeFusion::s_camera_radar_time_diff_th_ = 0.3f;

bool PbfShapeFusion::Init() {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::Init";
 
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfShapeFusion::Init";
  return true; }

void PbfShapeFusion::UpdateWithMeasurement(const SensorObjectPtr measurement,
                                           double target_timestamp) {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::UpdateWithMeasurement";

  // base::SensorType sensor_type = measurement->GetSensorType();
  SensorObjectConstPtr latest_lidar = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr latest_radar = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr latest_camera = track_ref_->GetLatestCameraObject();
  if (IsLidar(measurement)) {
    UpdateState(measurement);
  } else if (IsRadar(measurement)) {
    if (latest_lidar == nullptr) {
      if (latest_camera != nullptr) {
        if (std::fabs(measurement->GetTimestamp() -
                      latest_camera->GetTimestamp()) <
            s_camera_radar_time_diff_th_) {
          UpdateShape(latest_camera);
          UpdateCenter(measurement);
        } else {
          // nothing to do
        }
      } else {
        UpdateState(measurement);
      }
    } else {
      // nothing to do
    }
  } else if (IsCamera(measurement) && s_use_camera_3d_) {
    if (latest_lidar == nullptr) {
      UpdateState(measurement);
    } else {
      // nothing to do
    }
  } else {
    // nothing to do;
  }

   AINFO<<"(DMCZP) LeaveMethod: PbfShapeFusion::UpdateWithMeasurement";
 }
void PbfShapeFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                              double measurement_timestamp,
                                              double target_timestamp) {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::UpdateWithoutMeasurement";

   AINFO<<"(DMCZP) LeaveMethod: PbfShapeFusion::UpdateWithoutMeasurement";
 }

std::string PbfShapeFusion::Name() const {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::Name";
 
  AINFO<<"(DMCZP) (return) LeaveMethod: PbfShapeFusion::Name";
  return "PbfShapeFusion"; }

void PbfShapeFusion::UpdateState(const SensorObjectConstPtr& measurement) {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::UpdateState";

  UpdateShape(measurement);
  UpdateCenter(measurement);

   AINFO<<"(DMCZP) LeaveMethod: PbfShapeFusion::UpdateState";
 }

void PbfShapeFusion::UpdateShape(const SensorObjectConstPtr& measurement) {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::UpdateShape";

  base::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();
  base::ObjectConstPtr src_obj = measurement->GetBaseObject();

  dst_obj->size = src_obj->size;
  dst_obj->direction = src_obj->direction;
  dst_obj->theta = src_obj->theta;
  dst_obj->polygon = src_obj->polygon;

   AINFO<<"(DMCZP) LeaveMethod: PbfShapeFusion::UpdateShape";
 }

void PbfShapeFusion::UpdateCenter(const SensorObjectConstPtr& measurement) {
    AINFO<<"(DMCZP) EnteringMethod: PbfShapeFusion::UpdateCenter";

  base::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();
  base::ObjectConstPtr src_obj = measurement->GetBaseObject();

  dst_obj->center = src_obj->center;
  dst_obj->anchor_point = src_obj->anchor_point;

   AINFO<<"(DMCZP) LeaveMethod: PbfShapeFusion::UpdateCenter";
 }

// FUSION_REGISTER_SHAPEFUSION(PbfShapeFusion)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
