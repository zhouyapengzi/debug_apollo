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

#include "modules/canbus/vehicle/lexus/protocol/door_rpt_417.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace lexus {

using ::apollo::drivers::canbus::Byte;

Doorrpt417::Doorrpt417() {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::Doorrpt417";
}
const int32_t Doorrpt417::ID = 0x417;

void Doorrpt417::Parse(const std::uint8_t* bytes, int32_t length,
                       ChassisDetail* chassis) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::Parse";

  chassis->mutable_lexus()->mutable_door_rpt_417()->set_fuel_door_open_is_valid(
      fuel_door_open_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_trunk_open_is_valid(
      trunk_open_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_hood_open_is_valid(
      hood_open_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_door_rpt_417()
      ->set_rear_pass_door_open_is_valid(
          rear_pass_door_open_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_door_rpt_417()
      ->set_rear_driver_door_open_is_valid(
          rear_driver_door_open_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_pass_door_open_is_valid(
      pass_door_open_is_valid(bytes, length));
  chassis->mutable_lexus()
      ->mutable_door_rpt_417()
      ->set_driver_door_open_is_valid(driver_door_open_is_valid(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_fuel_door_open(
      fuel_door_open(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_trunk_open(
      trunk_open(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_hood_open(
      hood_open(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_rear_pass_door_open(
      rear_pass_door_open(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_rear_driver_door_open(
      rear_driver_door_open(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_pass_door_open(
      pass_door_open(bytes, length));
  chassis->mutable_lexus()->mutable_door_rpt_417()->set_driver_door_open(
      driver_door_open(bytes, length));
}

// config detail: {'name': 'fuel_door_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 14, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::fuel_door_open_is_valid(const std::uint8_t* bytes,
                                         int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::fuel_door_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'trunk_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 13, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::trunk_open_is_valid(const std::uint8_t* bytes,
                                     int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::trunk_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'hood_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 12, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::hood_open_is_valid(const std::uint8_t* bytes,
                                    int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::hood_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_pass_door_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 11, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::rear_pass_door_open_is_valid(const std::uint8_t* bytes,
                                              int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::rear_pass_door_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_driver_door_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 10, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::rear_driver_door_open_is_valid(const std::uint8_t* bytes,
                                                int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::rear_driver_door_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_door_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 9, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::pass_door_open_is_valid(const std::uint8_t* bytes,
                                         int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::pass_door_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_door_open_is_valid', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 8, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::driver_door_open_is_valid(const std::uint8_t* bytes,
                                           int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::driver_door_open_is_valid";

  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'fuel_door_open', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 6,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::fuel_door_open(const std::uint8_t* bytes,
                                int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::fuel_door_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(6, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'trunk_open', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 5, 'type':
// 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::trunk_open(const std::uint8_t* bytes, int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::trunk_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'hood_open', 'offset': 0.0, 'precision': 1.0, 'len':
// 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 4, 'type':
// 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::hood_open(const std::uint8_t* bytes, int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::hood_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(4, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_pass_door_open', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 3, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::rear_pass_door_open(const std::uint8_t* bytes,
                                     int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::rear_pass_door_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'rear_driver_door_open', 'offset': 0.0,
// 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range':
// '[0|1]', 'bit': 2, 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::rear_driver_door_open(const std::uint8_t* bytes,
                                       int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::rear_driver_door_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(2, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'pass_door_open', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 1,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::pass_door_open(const std::uint8_t* bytes,
                                int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::pass_door_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'driver_door_open', 'offset': 0.0, 'precision': 1.0,
// 'len': 1, 'is_signed_var': False, 'physical_range': '[0|1]', 'bit': 0,
// 'type': 'bool', 'order': 'motorola', 'physical_unit': ''}
bool Doorrpt417::driver_door_open(const std::uint8_t* bytes,
                                  int32_t length) const {
    AINFO<<"(DMCZP) EnteringMethod: Doorrpt417::driver_door_open";

  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}
}  // namespace lexus
}  // namespace canbus
}  // namespace apollo
