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

#include "modules/perception/tool/benchmark/lidar/util/object.h"
#include <sstream>
#include "modules/perception/tool/benchmark/lidar/util/macros.h"

namespace apollo {
namespace perception {
namespace benchmark {

Object::Object() {
    AINFO<<"(DMCZP) EnteringMethod: Object::Object";

  direction = Eigen::Vector3d(1, 0, 0);
  center = Eigen::Vector3d::Zero();
  velocity = Eigen::Vector3d::Zero();
  cloud.reset(new PointCloud);
  indices.reset(new PointIndices);
  type_probs.resize(MAX_OBJECT_TYPE, 0);
  internal_type_probs.resize(INT_MAX_OBJECT_TYPE, 0);
  confidence = 1.f;

   AINFO<<"(DMCZP) LeaveMethod: Object::Object";
 }

Object::Object(const Object& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: Object::Object";

  id = rhs.id;
  cloud = rhs.cloud;
  indices = rhs.indices;
  confidence = rhs.confidence;
  direction = rhs.direction;
  yaw = rhs.yaw;
  roll = rhs.roll;
  pitch = rhs.pitch;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  truncated = rhs.truncated;
  occluded = rhs.occluded;
  type = rhs.type;
  type_probs = rhs.type_probs;
  internal_type = rhs.internal_type;
  internal_type_probs = rhs.internal_type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  is_in_roi = rhs.is_in_roi;
  is_in_main_lanes = rhs.is_in_main_lanes;
  sensor_type = rhs.sensor_type;
  reserve = rhs.reserve;
  lidar_supplement = rhs.lidar_supplement;
  radar_supplement = rhs.radar_supplement;
  camera_supplement = rhs.camera_supplement;

   AINFO<<"(DMCZP) LeaveMethod: Object::Object";
 }

Object& Object::operator=(const Object& rhs) {
  id = rhs.id;
  cloud = rhs.cloud;
  indices = rhs.indices;
  confidence = rhs.confidence;
  direction = rhs.direction;
  yaw = rhs.yaw;
  roll = rhs.roll;
  pitch = rhs.pitch;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  truncated = rhs.truncated;
  occluded = rhs.occluded;
  type = rhs.type;
  type_probs = rhs.type_probs;
  internal_type = rhs.internal_type;
  internal_type_probs = rhs.internal_type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  is_in_roi = rhs.is_in_roi;
  is_in_main_lanes = rhs.is_in_main_lanes;
  sensor_type = rhs.sensor_type;
  reserve = rhs.reserve;
  lidar_supplement = rhs.lidar_supplement;
  radar_supplement = rhs.radar_supplement;
  camera_supplement = rhs.camera_supplement;
  return (*this);
}

void Object::clone(const Object& rhs) {
    AINFO<<"(DMCZP) EnteringMethod: Object::clone";

  id = rhs.id;
  pcl::copyPointCloud<Point, Point>(*(rhs.cloud), *cloud);
  indices->indices = rhs.indices->indices;
  confidence = rhs.confidence;
  direction = rhs.direction;
  yaw = rhs.yaw;
  roll = rhs.roll;
  pitch = rhs.pitch;
  center = rhs.center;
  length = rhs.length;
  width = rhs.width;
  height = rhs.height;
  truncated = rhs.truncated;
  occluded = rhs.occluded;
  type = rhs.type;
  type_probs = rhs.type_probs;
  internal_type = rhs.internal_type;
  internal_type_probs = rhs.internal_type_probs;
  is_background = rhs.is_background;
  track_id = rhs.track_id;
  velocity = rhs.velocity;
  tracking_time = rhs.tracking_time;
  latest_tracked_time = rhs.latest_tracked_time;
  is_in_roi = rhs.is_in_roi;
  is_in_main_lanes = rhs.is_in_main_lanes;
  sensor_type = rhs.sensor_type;
  reserve = rhs.reserve;
  lidar_supplement = nullptr;
  if (rhs.lidar_supplement != nullptr) {
    lidar_supplement.reset(new LidarSupplement());
    lidar_supplement->clone(*(rhs.lidar_supplement));
  }

  radar_supplement = nullptr;
  if (rhs.radar_supplement != nullptr) {
    radar_supplement.reset(new RadarSupplement());
    radar_supplement->clone(*(rhs.radar_supplement));
  }

  camera_supplement = nullptr;
  if (rhs.camera_supplement != nullptr) {
    camera_supplement.reset(new CameraSupplement());
    camera_supplement->clone(*(rhs.camera_supplement));
  }

   AINFO<<"(DMCZP) LeaveMethod: Object::clone";
 }

std::string Object::to_string() const {
    AINFO<<"(DMCZP) EnteringMethod: Object::to_string";

  std::ostringstream oss;
  oss << "Object[id: " << id << ", track_id: " << track_id
      << ", cloud_size: " << cloud->size()
      << ", indices_size: " << indices->indices.size()
      << ", direction: " << direction.transpose() << ", yaw: " << yaw
      << ", center: " << center.transpose()
      << " velocity: " << velocity.transpose() << ", width: " << width
      << ", length: " << length << ", height: " << height
      << ", confidence: " << confidence << ", type: " << type
      << ", is_background: " << is_background << ", is_in_roi: " << is_in_roi
      << ", is_in_main_lanes: " << is_in_main_lanes
      << ", tracking_time: " << GLOG_TIMESTAMP(tracking_time)
      << ", latest_tracked_time: " << GLOG_TIMESTAMP(latest_tracked_time)
      << "]";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: Object::to_string";
  return oss.str();

   AINFO<<"(DMCZP) LeaveMethod: Object::to_string";
 }

std::string get_object_name(ObjectType obj_type) {
    AINFO<<"(DMCZP) EnteringMethod: get_object_name";

  std::string obj_name;
  switch (obj_type) {
    case UNKNOWN:
      obj_name = "unknown";
      break;
    case UNKNOWN_MOVABLE:
      obj_name = "unknown_movable";
      break;
    case UNKNOWN_UNMOVABLE:
      obj_name = "unknown_unmovable";
      break;
    case PEDESTRIAN:
      obj_name = "pedestrian";
      break;
    case BICYCLE:
      obj_name = "bicycle";
      break;
    case VEHICLE:
      obj_name = "vehicle";
      break;
    default:
      obj_name = "error";
      break;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: get_object_name";
  return obj_name;

   AINFO<<"(DMCZP) LeaveMethod: get_object_name";
 }

std::string get_sensor_name(SensorType sensor_type) {
    AINFO<<"(DMCZP) EnteringMethod: get_sensor_name";

  std::string sensor_name;
  switch (sensor_type) {
    case VELODYNE_64:
      sensor_name = "velodyne_64";
      break;
    case VELODYNE_16:
      sensor_name = "velodyne_16";
      break;
    case RADAR:
      sensor_name = "radar";
      break;
    case CAMERA:
      sensor_name = "camera";
      break;
    case UNKNOWN_SENSOR_TYPE:
      sensor_name = "unknown_sensor_type";
      break;
  }
  
  AINFO<<"(DMCZP) (return) LeaveMethod: get_sensor_name";
  return sensor_name;

   AINFO<<"(DMCZP) LeaveMethod: get_sensor_name";
 }

std::string SensorObjects::to_string() const {
    AINFO<<"(DMCZP) EnteringMethod: SensorObjects::to_string";

  std::ostringstream oss;
  oss << "SensorObjects[sensor_type: " << get_sensor_name(type)
      << ", name: " << name << ", timestamp:" << GLOG_TIMESTAMP(timestamp)
      << ", sensor2world_pose:\n";
  oss << sensor2world_pose << "\n, objects: " << objects.size() << " < ";
  for (auto obj : objects) {
    oss << "\n" << obj->to_string();
  }
  oss << " >]";
  
  AINFO<<"(DMCZP) (return) LeaveMethod: SensorObjects::to_string";
  return oss.str();

   AINFO<<"(DMCZP) LeaveMethod: SensorObjects::to_string";
 }

ObjectType translate_string_to_type(const std::string& str) {
    AINFO<<"(DMCZP) EnteringMethod: translate_string_to_type";

  if (str == "bigMot" || str == "smallMot" || str == "vehicle" ||
      str == "midMot" || str == "5") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_type";
  return VEHICLE;
  } else if (str == "pedestrian" || str == "3") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_type";
  return PEDESTRIAN;
  } else if (str == "nonMot" || str == "cyclist" || str == "motorcyclist" ||
             str == "bicyclist" || str == "4") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_type";
  return BICYCLE;
  } else {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_type";
  return UNKNOWN;
  }

   AINFO<<"(DMCZP) LeaveMethod: translate_string_to_type";
 }

unsigned int translate_type_to_index(const ObjectType& type) {
    AINFO<<"(DMCZP) EnteringMethod: translate_type_to_index";

  if (type <= UNKNOWN) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_index";
  return 0;
  } else if (type < MAX_OBJECT_TYPE) {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_index";
  return static_cast<unsigned int>(type) - 2;
  } else {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_index";
  return 0;
  }

   AINFO<<"(DMCZP) LeaveMethod: translate_type_to_index";
 }

std::string translate_type_index_to_string(unsigned int index) {
    AINFO<<"(DMCZP) EnteringMethod: translate_type_index_to_string";

  switch (index) {
    case 0:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_index_to_string";
  return "others";
    case 1:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_index_to_string";
  return "pedestrian";
    case 2:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_index_to_string";
  return "cyclist";
    case 3:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_index_to_string";
  return "vehicle";
    default:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_index_to_string";
  return "others";
  }

   AINFO<<"(DMCZP) LeaveMethod: translate_type_index_to_string";
 }

SensorType translate_string_to_sensor_type(const std::string& str) {
    AINFO<<"(DMCZP) EnteringMethod: translate_string_to_sensor_type";

  if (str == "velodyne_64") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_sensor_type";
  return VELODYNE_64;
  } else if (str == "velodyne_16") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_sensor_type";
  return VELODYNE_16;
  } else if (str == "radar") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_sensor_type";
  return RADAR;
  } else if (str == "camera") {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_sensor_type";
  return CAMERA;
  } else {
    
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_string_to_sensor_type";
  return UNKNOWN_SENSOR_TYPE;
  }

   AINFO<<"(DMCZP) LeaveMethod: translate_string_to_sensor_type";
 }

std::string translate_type_to_string(ObjectType type) {
    AINFO<<"(DMCZP) EnteringMethod: translate_type_to_string";

  switch (type) {
    case UNKNOWN:
    case UNKNOWN_MOVABLE:
    case UNKNOWN_UNMOVABLE:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_string";
  return "others";
    case PEDESTRIAN:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_string";
  return "pedestrian";
    case BICYCLE:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_string";
  return "cyclist";
    case VEHICLE:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_string";
  return "vehicle";
    case MAX_OBJECT_TYPE:
    default:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_type_to_string";
  return "others";
  }

   AINFO<<"(DMCZP) LeaveMethod: translate_type_to_string";
 }

std::string translate_sensor_type_to_string(const SensorType& type) {
    AINFO<<"(DMCZP) EnteringMethod: translate_sensor_type_to_string";

  switch (type) {
    case VELODYNE_64:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_sensor_type_to_string";
  return "velodyne_64";
    case VELODYNE_16:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_sensor_type_to_string";
  return "velodyne_16";
    case RADAR:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_sensor_type_to_string";
  return "radar";
    case CAMERA:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_sensor_type_to_string";
  return "camera";
    default:
      
  AINFO<<"(DMCZP) (return) LeaveMethod: translate_sensor_type_to_string";
  return "unknown_sensor_type";
  }

   AINFO<<"(DMCZP) LeaveMethod: translate_sensor_type_to_string";
 }

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo
