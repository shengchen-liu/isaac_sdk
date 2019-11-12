/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "config_backend.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

void ConfigBackend::set(const nlohmann::json& json) {
  for (auto it1 = json.begin(); it1 != json.end(); ++it1) {
    const std::string& key1 = it1.key();
    for (auto it2 = it1->begin(); it2 != it1->end(); ++it2) {
      const std::string& key2 = it2.key();
      if (it2->is_object()) {
        // This is a json object, thus it's config for a component
        for (auto it3 = it2->begin(); it3 != it2->end(); ++it3) {
          const std::string& key3 = it3.key();
          nlohmann::json copy = *it3;
          setJson(key1, key2, key3, std::move(copy));
        }
      } else {
        // This is a base value, thus it's config for the node
        nlohmann::json copy = *it2;
        setJson(key1, key2, "", std::move(copy));
      }
    }
  }
}

const nlohmann::json* ConfigBackend::tryGetJson(const std::string& key1, const std::string& key2,
                                                const std::string& key3) const {
  std::shared_lock<std::shared_timed_mutex> lock(mutex_);
  const auto it = cache_.find(GetCacheKey(key1, key2, key3));
  return (it == cache_.end()) ? nullptr : &it->second;
}

void ConfigBackend::setJson(const std::string& key1, const std::string& key2,
                            const std::string& key3, nlohmann::json&& json) {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  auto it1 = json_root_.find(key1);
  if (it1 == json_root_.end()) {
    json_root_[key1] = {};
    it1 = json_root_.find(key1);
  }
  auto it2 = it1->find(key2);
  if (it2 == it1->end()) {
    (*it1)[key2] = {};
    it2 = it1->find(key2);
  }
  if (!key3.empty()) {
    // set a config for a component
    auto it3 = it2->find(key3);
    if (it3 == it2->end()) {
      (*it2)[key3] = nlohmann::json{};
      it3 = it2->find(key3);
    }
    *it3 = std::move(json);
    // Also add the value to the cache
    cache_[GetCacheKey(key1, key2, key3)] = *it3;
  } else {
    // set a config for the node
    *it2 = std::move(json);
  }
}

nlohmann::json ConfigBackend::getAllForComponent(const std::string& key1,
                                                 const std::string& key2) const {
  std::unique_lock<std::shared_timed_mutex> lock(mutex_);
  auto it1 = json_root_.find(key1);
  if (it1 == json_root_.end()) {
    return {};
  }
  auto it2 = it1->find(key2);
  if (it2 == it1->end()) {
    return {};
  }
  return *it2;
}

std::string ConfigBackend::GetCacheKey(const std::string& node, const std::string& component,
                                       const std::string& key) {
  return node + "/" + component + "/" + key;
}

}  // namespace alice
}  // namespace isaac
