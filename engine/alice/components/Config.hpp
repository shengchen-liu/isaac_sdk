/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <utility>

#include "engine/alice/component.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

class ConfigBackend;

// Stores node configuration in form of key-value pairs.
//
// This component is added to every node by default and does not have to be added manually.
//
// The config component is used by other components and the node itself to store structure and
// state. Most notable config can be used directly in codelets to access custom configuration
// values. Support for basic types and some math types is built-in.
// Configuration is stored in a group-key-value format. Each component and the node itself are
// defining separate groups of key-value pairs. Additionally custom groups of configuration can be
// added by the user.
class Config : public Component {
 public:
  virtual ~Config() {}
  void initialize() override;
  void start() override;
  void deinitialize() override;

  // Tries to get the configuration value for the given key in a component in this node
  template<typename T>
  std::optional<T> tryGet(Component* component, const std::string& key) const {
    const nlohmann::json* ptr = tryGetJson(component, key);
    if (ptr == nullptr) {
      return std::nullopt;
    }
    return serialization::TryGet<T>(*ptr);
  }

  // Sets the configuration value for the given key in a component in this node
  template<typename T>
  void set(Component* component, const std::string& key, T value) {
    nlohmann::json json;
    serialization::Set(json, std::move(value));
    setJson(component, key, std::move(json));
  }

  // Gets all configuration for a given component in this node
  nlohmann::json getAll(Component* component) const;

 private:
  friend class ConfigHook;

  // Tries to get a JSON value for the given key in a component in this node
  const nlohmann::json* tryGetJson(Component* component, const std::string& key) const;
  // Sets a JSON value for the given key in a component in this node
  void setJson(Component* component, const std::string& key, nlohmann::json&& json);
  // Gets the key used to identify a component in this node in the configuration database
  std::string getComponentKey(Component* component) const;

  ConfigBackend* backend_;
};

}  // namespace alice
}  // namespace isaac

ISAAC_ALICE_REGISTER_COMPONENT(isaac::alice::Config)
