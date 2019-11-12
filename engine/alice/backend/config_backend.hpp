/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <shared_mutex>
#include <string>
#include <unordered_map>

#include "engine/alice/backend/component_backend.hpp"
#include "engine/alice/components/Config.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

// Backend for configuration components. It stores a four-level deep tree of configuration values
// of the form: node -> group -> key -> value. Each component has its own group in a node.
// To increase performance the configuration values are also stored in a cache which is used for
// reading configuration values.
class ConfigBackend : public ComponentBackend<Config> {
 public:
  // Sets configuration using another configuration JSON object
  void set(const nlohmann::json& json);

  // Gets an existing JSON value for the given node/component/key, or returns null if it does
  // not exist.
  const nlohmann::json* tryGetJson(const std::string& node, const std::string& component,
                                   const std::string& key) const;

  // Sets a JSON value for a given node/component/key. The value is overwritten it in case it
  // already exists.
  void setJson(const std::string& node, const std::string& component, const std::string& key,
               nlohmann::json&& json);

  // Gets all configuration for a given node and component in this node
  nlohmann::json getAllForComponent(const std::string& node, const std::string& component) const;

  // Get the whole configuration JSON object
  const nlohmann::json& root() const { return json_root_; }

 private:
  // Gets the key used in the cache for a configuration value
  static std::string GetCacheKey(const std::string& node, const std::string& component,
                                 const std::string& key);

  mutable std::shared_timed_mutex mutex_;
  nlohmann::json json_root_;
  std::unordered_map<std::string, nlohmann::json> cache_;
};

}  // namespace alice
}  // namespace isaac
