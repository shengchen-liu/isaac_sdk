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

#include "engine/alice/hooks/hook.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"
#include "engine/gems/serialization/json_formatter.hpp"

namespace isaac {
namespace alice {

class Component;
class Config;

// Common base class for configuration hooks, aka parameters
class ConfigHook : public Hook {
 public:
  // Creates a parameter and associates it with a component
  ConfigHook(Component* component, const char* key);

  // The name of the configuration value
  const std::string& key() const { return key_; }

  // The name of the type of this configuration parameter
  virtual std::string type_name() const = 0;

  // Gets the default value (or null if none)
  virtual nlohmann::json getDefault() const = 0;

 protected:
  friend class Component;  // for calling connect

  // Demangles config typename for readability
  static std::string ConfigTypenameDemangle(const std::string& type_name);

  virtual ~ConfigHook() = default;
  // Connects the parameter to the configuration and initializes default values
  void connect() override;
  // Prints a warning that the configuration does not contain the value (used by derived)
  void printUnsetWarning();

  // Tries to get the value of this configuration parameter (via serialized JSON object) from the
  // configuration backend. Returns nullptr in case the value is not set in the configuration
  // backend.
  const nlohmann::json* tryGetJson() const;

  // Helper function to set the value of this configuration parameter (via serialized JSON object)
  // in the configuration backend. In case the value is already set it will be overwritten.
  void setJson(nlohmann::json&& json);

 private:
  std::string key_;
  Config* config_;
};

// Parameters are used by component to gives access to values stored in configuration. They are
// automatically registered on object creation and thus can also be used to get component
// configuration.
template <typename T>
class Parameter : public ConfigHook {
 public:
  // Disallow copying for parameters
  Parameter(const Parameter&) = delete;
  Parameter& operator=(const Parameter&) = delete;

  // Creates a parameter and associates it with a component
  Parameter(Component* component, const char* key) : ConfigHook(component, key) {}
  Parameter(Component* component, const char* key, T default_value)
      : ConfigHook(component, key), default_value_(std::move(default_value)) {}

  // Gets the type name of the parameter in a human-readable form
  std::string type_name() const override { return ConfigTypenameDemangle(typeid(T).name()); }

  // Gets the default value
  nlohmann::json getDefault() const {
    nlohmann::json result;
    if (default_value_) {
      serialization::Set(result, *default_value_);
    }
    return result;
  }

  // Gets the value of the parameter
  T get() const {
    auto maybe = tryGet();
    ASSERT(maybe, "Parameter '%s/%s' not found or wrong type", component()->full_name().c_str(),
           key().c_str());
    return *maybe;
  }
  // Tries to get the parameter
  std::optional<T> tryGet() const {
    const nlohmann::json* ptr = tryGetJson();
    if (ptr == nullptr) {
      return std::nullopt;
    }
    return serialization::TryGet<T>(*ptr);
  }
  // Sets the value of the parameter
  void set(T value) {
    nlohmann::json json;
    serialization::Set(json, std::move(value));
    setJson(std::move(json));
  }

 protected:
  void connect() override {
    ConfigHook::connect();
    if (!tryGet()) {
      if (default_value_) {
        set(*default_value_);
      } else {
        printUnsetWarning();
      }
    }
  }

  // The default value to use in case the parameter was not set in the configuration
  std::optional<T> default_value_;
};

}  // namespace alice
}  // namespace isaac

// Define macros to declare parameter member variable and get/set functions.
// Uses sliding macro trick for macro overloading.
#define _ISAAC_PARAM_IMPL_1(TYPE, KEY)                                          \
 private:                                                                       \
  isaac::alice::Parameter<TYPE> param_##KEY##_{this, #KEY};                     \
                                                                                \
 public:                                                                        \
  TYPE get_##KEY() const { return param_##KEY##_.get(); }                       \
  std::optional<TYPE> try_get_##KEY() const { return param_##KEY##_.tryGet(); } \
  void set_##KEY(TYPE x) { param_##KEY##_.set(std::move(x)); }
#define _ISAAC_PARAM_IMPL_2(TYPE, KEY, DEFAULT)                      \
 private:                                                            \
  isaac::alice::Parameter<TYPE> param_##KEY##_{this, #KEY, DEFAULT}; \
                                                                     \
 public:                                                             \
  TYPE get_##KEY() const { return param_##KEY##_.get(); }            \
  void set_##KEY(TYPE x) { param_##KEY##_.set(std::move(x)); }
// This additional EVALUATE macro is required to be compatible with MSVC
#define _ISAAC_PARAM_EVALUATE(x) x
#define _ISAAC_PARAM_GET_OVERRIDE(_1, _2, _3, NAME, ...) NAME
#define ISAAC_PARAM(...)                                                            \
  _ISAAC_PARAM_EVALUATE(_ISAAC_PARAM_GET_OVERRIDE(__VA_ARGS__, _ISAAC_PARAM_IMPL_2, \
                                                  _ISAAC_PARAM_IMPL_1)(__VA_ARGS__))
