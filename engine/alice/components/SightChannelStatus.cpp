/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "SightChannelStatus.hpp"

#include <set>
#include <string>

namespace isaac {
namespace alice {

bool SightChannelStatus::isChannelActive(const alice::MessageLedger::Endpoint& endpoint) {
  const std::string name = endpoint.nameWithApp();
  {
    std::shared_lock<std::shared_timed_mutex> lock(mutex_);
    const auto it = channel_listeners_.find(name);
    if (it != channel_listeners_.end()) {
      return it->second > 0;
    }
  }
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  // Create the object if it does not exist, but do not change the value if it does.
  channel_listeners_[name];
  // For the first message we let it go through to notify the channel exist
  // TODO implement a better way to notify of a new channel
  return true;
}

void SightChannelStatus::addChannelListener(const std::string& name) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  channel_listeners_[name]++;
}

void SightChannelStatus::removeChannelListener(const std::string& name) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  channel_listeners_[name]--;
}

std::set<std::string> SightChannelStatus::getListChannels() {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  std::set<std::string> tags;
  for (const auto& it : channel_listeners_) {
    tags.insert(it.first);
  }
  return tags;
}

}  // namespace alice
}  // namespace isaac
