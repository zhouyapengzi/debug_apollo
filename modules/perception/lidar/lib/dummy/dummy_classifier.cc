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

#include "modules/perception/lidar/lib/dummy/dummy_classifier.h"

namespace apollo {
namespace perception {
namespace lidar {

bool DummyClassifier::Init(const ClassifierInitOptions& options) {
    AINFO<<"(DMCZP) EnteringMethod: DummyClassifier::Init";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: DummyClassifier::Init";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: DummyClassifier::Init";
 }

bool DummyClassifier::Classify(const ClassifierOptions& options,
                               LidarFrame* frame) {
    AINFO<<"(DMCZP) EnteringMethod: DummyClassifier::Classify";

  
  AINFO<<"(DMCZP) (return) LeaveMethod: DummyClassifier::Classify";
  return true;

   AINFO<<"(DMCZP) LeaveMethod: DummyClassifier::Classify";
 }

PERCEPTION_REGISTER_CLASSIFIER(DummyClassifier);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
