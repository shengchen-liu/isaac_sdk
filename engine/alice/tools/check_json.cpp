/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include <memory>
#include <set>
#include <string>

#include "engine/alice/backend/modules.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {

// Load a json file, load the modules specified, and go through all the components and try to
// build them.
void Main(int argc, char* argv[]) {
  ASSERT(argc > 1, "Please provide a json file as argument");
  Json json = serialization::LoadJsonFromFile(argv[1]);

  // Load modules
  alice::ModuleManager module_manage;
  module_manage.loadStaticallyLinked();
  auto it = json.find("modules");
  if (it != json.end()) {
    module_manage.loadModules(*it);
  }

  // Extract components
  std::set<std::string> components;
  it = json["graph"].find("nodes");
  if (it == json["graph"].end()) {
    LOG_WARNING("No nodes were found");
    return;
  }
  ASSERT(it->is_array(), "Must be an array: %s", it->dump(2).c_str());
  auto nodes = *it;
  for (auto& node : nodes) {
    for (auto& comp : node["components"]) {
      components.insert(std::string(comp["type"]));
    }
  }

  // Check we can build each component.
  for (const auto& comp : components) {
    alice::Component* component = module_manage.createComponent(comp);
    ASSERT(component != nullptr, "Cannot create the component:", comp.c_str());
    delete component;
  }
}

}  // namespace isaac

int main(int argc, char* argv[]) {
  isaac::Main(argc, argv);
  return 0;
}
