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
#include "modules/perception/lidar/lib/scene_manager/ground_service/ground_service.h"

#include <limits>

#include "cyber/common/file.h"
#include "modules/perception/common/i_lib/geometry/i_plane.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/scene_manager/ground_service/proto/ground_service_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

void GroundServiceContent::GetCopy(SceneServiceContent* content) const {
    AINFO<<"(DMCZP) EnteringMethod: GroundServiceContent::GetCopy";

  GroundServiceContent* ground_content =
      dynamic_cast<GroundServiceContent*>(content);
  if (ground_content == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::GetCopy";
  return;
  }
  ground_content->grid_ = grid_;
  ground_content->grid_center_ = grid_center_;
  ground_content->resolution_x_ = resolution_x_;
  ground_content->resolution_y_ = resolution_y_;
  ground_content->bound_x_min_ = bound_x_min_;
  ground_content->bound_y_min_ = bound_y_min_;
  ground_content->bound_x_max_ = bound_x_max_;
  ground_content->bound_y_max_ = bound_y_max_;
  ground_content->rows_ = rows_;
  ground_content->cols_ = cols_;
  ground_content->service_ready_ = service_ready_;

   AINFO<<"(DMCZP) LeaveMethod: GroundServiceContent::GetCopy";
 }

void GroundServiceContent::SetContent(const SceneServiceContent& content) {
    AINFO<<"(DMCZP) EnteringMethod: GroundServiceContent::SetContent";

  const GroundServiceContent* ground_content =
      dynamic_cast<const GroundServiceContent*>(&content);
  if (ground_content == nullptr) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::SetContent";
  return;
  }
  grid_ = ground_content->grid_;
  grid_center_ = ground_content->grid_center_;
  resolution_x_ = ground_content->resolution_x_;
  resolution_y_ = ground_content->resolution_y_;
  bound_x_min_ = ground_content->bound_x_min_;
  bound_y_min_ = ground_content->bound_y_min_;
  bound_x_max_ = ground_content->bound_x_max_;
  bound_y_max_ = ground_content->bound_y_max_;
  rows_ = ground_content->rows_;
  cols_ = ground_content->cols_;

  service_ready_ = true;

   AINFO<<"(DMCZP) LeaveMethod: GroundServiceContent::SetContent";
 }

uint32_t inline GetIndex(uint32_t r, uint32_t c, uint32_t cols) {
    AINFO<<"(DMCZP) EnteringMethod: GetIndex";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: GetIndex";
  return (r * cols + c);
 }

bool GroundServiceContent::PointToGrid(const Eigen::Vector3d& world_point,
                                       uint32_t* grid_index) const {
    AINFO<<"(DMCZP) EnteringMethod: GroundServiceContent::PointToGrid";

  double x = world_point(0) - grid_center_(0);
  double y = world_point(1) - grid_center_(1);
  if (x < bound_x_min_ || x > bound_x_max_ || y < bound_y_min_ ||
      y > bound_y_max_) {
    (*grid_index) = 0;
    
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::PointToGrid";
  return false;
  }
  uint32_t c = std::min(
      cols_ - 1, static_cast<uint32_t>((x - bound_x_min_) / resolution_x_));
  uint32_t r = std::min(
      rows_ - 1, static_cast<uint32_t>((y - bound_y_min_) / resolution_y_));
  (*grid_index) = GetIndex(r, c, cols_);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::PointToGrid";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: GroundServiceContent::PointToGrid";
 }

float GroundServiceContent::PointToPlaneDistance(
    const Eigen::Vector3d& world_point) const {
    AINFO<<"(DMCZP) EnteringMethod: GroundServiceContent::PointToPlaneDistance";

  uint32_t grid_index = 0;
  if (!PointToGrid(world_point, &grid_index)) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::PointToPlaneDistance";
  return std::numeric_limits<float>::max();
  }

  float offset_pt[3];
  float params[4];
  float out = std::numeric_limits<float>::max();

  offset_pt[0] = static_cast<float>(world_point(0) - grid_center_(0));
  offset_pt[1] = static_cast<float>(world_point(1) - grid_center_(1));
  offset_pt[2] = static_cast<float>(world_point(2) - grid_center_(2));

  const GroundNode* node = grid_.DataPtr() + grid_index;
  if (node->confidence > 0.f) {
    params[0] = node->params(0);
    params[1] = node->params(1);
    params[2] = node->params(2);
    params[3] = node->params(3);
    out = common::IPlaneToPointSignedDistanceWUnitNorm(params, offset_pt);
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::PointToPlaneDistance";
  return out;
 }

bool GroundServiceContent::Init(double roi_x, double roi_y, uint32_t rows,
                                uint32_t cols) {
    AINFO<<"(DMCZP) EnteringMethod: GroundServiceContent::Init";

  bound_x_min_ = -roi_x;
  bound_y_min_ = -roi_y;
  bound_x_max_ = roi_x;
  bound_y_max_ = roi_y;

  rows_ = rows;
  cols_ = cols;

  resolution_x_ = (bound_x_max_ - bound_x_min_) / cols_;
  resolution_y_ = (bound_y_max_ - bound_y_min_) / rows_;

  grid_.Init(rows_, cols_);
  
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundServiceContent::Init";
  return true;
 }

bool GroundService::Init(const SceneServiceInitOptions& options) {
    AINFO<<"(DMCZP) EnteringMethod: GroundService::Init";

  self_content_.reset(new GroundServiceContent);
  ground_content_ref_ =
      dynamic_cast<GroundServiceContent*>(self_content_.get());
  // initialize ground service content from config manager and proto
  // including resolution, grid size, ...
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config))
      << "Failed to get model config: SceneManager";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";

  std::string config_file;
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "ground_service.conf");

  GroundServiceConfig config_params;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config_params))
      << "Failed to parse GroundServiceConfig config file.";

  double roi_region_rad_x = config_params.roi_rad_x();
  double roi_region_rad_y = config_params.roi_rad_y();
  uint32_t rows = config_params.grid_size();
  uint32_t cols = config_params.grid_size();

  ground_content_ref_->Init(roi_region_rad_x, roi_region_rad_y, rows, cols);

  
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundService::Init";
  return true;
 }

float GroundService::QueryPointToGroundDistance(
    const Eigen::Vector3d& world_point) {
    AINFO<<"(DMCZP) EnteringMethod: GroundService::QueryPointToGroundDistance";

  std::lock_guard<std::mutex> lock(mutex_);
  float distance =
      QueryPointToGroundDistance(world_point, *ground_content_ref_);
  
  
  AINFO<<"(DMCZP) (return) LeaveMethod: GroundService::QueryPointToGroundDistance";
  return distance;

 }

float GroundService::QueryPointToGroundDistance(
    const Eigen::Vector3d& world_point, const GroundServiceContent& content) {
  return content.PointToPlaneDistance(world_point);
}

PERCEPTION_REGISTER_SCENESERVICECONTENT(GroundServiceContent);
PERCEPTION_REGISTER_SCENESERVICE(GroundService);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
