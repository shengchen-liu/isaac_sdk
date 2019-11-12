/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Config.hpp"

#include <string>
#include <utility>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/backend/config_backend.hpp"
#include "engine/alice/node.hpp"
#include "engine/core/assert.hpp"

namespace isaac {
namespace alice {

void Config::initialize() {
  backend_ = node()->app()->backend()->getBackend<ConfigBackend>();
}

void Config::start() {
  reportSuccess();  // do not participate in status updates TODO solver differently
}

void Config::deinitialize() {
  backend_ = nullptr;
}

nlohmann::json Config::getAll(Component* component) const {
  return backend_->getAllForComponent(node()->name(), getComponentKey(component));
}

const nlohmann::json* Config::tryGetJson(Component* component, const std::string& key) const {
  return backend_->tryGetJson(node()->name(), getComponentKey(component), key);
}

void Config::setJson(Component* component, const std::string& key, nlohmann::json&& json) {
  backend_->setJson(node()->name(), getComponentKey(component), key, std::move(json));
}

std::string Config::getComponentKey(Component* component) const {
  if (component == nullptr) {
    return "_";
  } else {
    ASSERT(component->node() == this->node(), "Component not part of this node");
    return component->name();
  }
}

}  // namespace alice
}  // namespace isaac
