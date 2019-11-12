/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <string>
#include <vector>

#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

class Application;
class Component;

// Helper class to load applications from JSON
class ApplicationJsonLoader {
 public:
  ApplicationJsonLoader(const std::string& asset_path = "");

  // Gets absolute filename for relative asset filename. Identity in case of absolute filename.
  std::string getAssetPath(const std::string& path = "") const;

  // Loads an application from a JSON object
  void loadApp(const nlohmann::json& json);

  // Loads more config and graph from a JSON object
  void loadMore(const nlohmann::json& json);

  // Adds more paths where to look for modules
  void appendModulePaths(const std::vector<std::string>& module_paths);

  // Loads configuration from the given JSON object. `node_prefix` can be used to add a string
  // prefix to all node names.
  void loadConfig(const nlohmann::json& json, const std::string& node_prefix = "");
  // Loads JSON from a file and then proceeds like `loadConfig` for JSON objects.
  void loadConfigFromFile(const std::string& filename);
  // Loads JSON from a string and then proceeds like `loadConfig` for JSON objects.
  void loadConfigFromText(const std::string& text);

  // Loads a graph from the given JSON object. `node_prefix` can be used to add a string prefix
  // to all node names.
  void loadGraph(const nlohmann::json& json, const std::string& node_prefix = "");
  // Loads JSON from a file and then proceeds like `loadConfig` for JSON objects.
  void loadGraphFromFile(const std::string& filename);
  // Loads JSON from a string and then proceeds like `loadConfig` for JSON objects.
  void loadGraphFromText(const std::string& text);

  // Gets all nodes and connections as JSON
  static nlohmann::json GetGraphJson(Application& app);

  // Writes given configuration to a file
  static bool WriteConfigToFile(const std::string& filename, const Json& json);

  // Gets graph json for all the given components
  static nlohmann::json GraphToJson(const std::vector<Component*>& components);
  // Gets configuration json for all the given components
  static nlohmann::json ConfigToJson(const std::vector<Component*>& components);

 private:
  friend class Application;

  // Information about a node in JSON format
  struct NodeJson {
    // The accumulated subgraph prefix under which this node was loaded
    std::string prefix;
    // The JSON object describing the nodes, e.g. it's components
    nlohmann::json json;
  };

  // Information about a message connection
  struct EdgeJson {
    std::string source;
    std::string target;
  };

  // Helper function for loadGraph
  void loadGraphRecursive(const nlohmann::json& json, const std::string& node_prefix);

  std::string asset_path_;

  std::string name_;
  std::string config_backup_;
  std::string performance_report_out_;
  std::string minidump_path_;

  nlohmann::json scheduler_json_;

  std::vector<std::string> module_paths_;

  std::vector<std::string> modules_;

  size_t level_;
  std::vector<nlohmann::json> config_by_level_;

  // Loaded nodes
  std::vector<NodeJson> nodes_;

  // Loaded edges
  std::vector<EdgeJson> edges_;
};

}  // namespace alice
}  // namespace isaac
