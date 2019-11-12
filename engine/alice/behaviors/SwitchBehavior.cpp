/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SwitchBehavior.hpp"

#include <string>

namespace isaac {
namespace alice {
namespace behaviors {

void SwitchBehavior::start() {
  tickPeriodically(0.01);  // Ideally we would tick if the configuration changes, or if a child
                           // updates its status.
  current_behavior_ = std::nullopt;
  switchTo(try_get_desired_behavior());
}

void SwitchBehavior::tick() {
  switchTo(try_get_desired_behavior());
  // FIXME The switch behavior should succeed/fail based on the status of the selected child.
  //       However this can currently not be implemented as we would have to wait here until the
  //       child is running again before querying its status to avoid getting an outdated status.
  // if (current_behavior_) {
  //   const auto status = getChildStatus(*current_behavior_);
  //   if (status != Status::RUNNING) {
  //     updateStatus(status, "child status changed");
  //   }
  // }
}

void SwitchBehavior::switchTo(const std::optional<std::string>& desired_behavior) {
  // Stop current behavior it there is a behavior running and the desired one is different.
  if (current_behavior_ && (!desired_behavior || *desired_behavior != *current_behavior_)) {
    stopChild(*current_behavior_);
  }
  // Switch to new behavior
  current_behavior_ = desired_behavior;
  // Start new behavior
  if (current_behavior_ && !current_behavior_->empty()) {
    if (Node* child = findChildByName(*current_behavior_)) {
      startChild(*child);
    } else {
      LOG_ERROR("No child with name '%s'", current_behavior_->c_str());
    }
  }
}

}  // namespace behaviors
}  // namespace alice
}  // namespace isaac
