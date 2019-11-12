/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_application.hpp"

#include "engine/alice/application.hpp"
#include "engine/alice/backend/application_json_loader.hpp"
#include "engine/alice/tools/websight.hpp"
#include "engine/gems/serialization/json.hpp"

namespace isaac {
namespace alice {

PybindApplication::PybindApplication(const std::string& app_filename) {
  const auto json = serialization::TryLoadJsonFromFile(app_filename);
  ASSERT(json, "Could not parse JSON from file '%s'", app_filename.c_str());
  ApplicationJsonLoader loader;
  LoadWebSight(loader);
  loader.loadApp(*json);
  app_.reset(new Application(loader));
  InitializeSightApi(*app_);
}

PybindApplication::~PybindApplication() {}

void PybindApplication::start() {
  app_->start();
}

void PybindApplication::stop() {
  app_->stop();
}

void PybindApplication::start_wait_stop() {
  app_->startWaitStop();
}

pybind11::str PybindApplication::uuid() const {
  std::string uuid;
  {
    pybind11::gil_scoped_release release_gil;
    uuid = app_->uuid().str();
  }
  return pybind11::str(uuid);
}

PybindNode PybindApplication::findNodeByName(const std::string& name) {
  return PybindNode(app_->findNodeByName(name));
}

void InitPybindApplication(pybind11::module& m) {
  pybind11::class_<PybindApplication>(m, "PybindApplication")
      .def(pybind11::init<const std::string&>())
      .def("start", &PybindApplication::start, pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("stop", &PybindApplication::stop, pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("start_wait_stop", &PybindApplication::start_wait_stop,
           pybind11::call_guard<pybind11::gil_scoped_release>())
      .def("uuid", &PybindApplication::uuid)
      .def("findNodeByName", &PybindApplication::findNodeByName);
}

}  // namespace alice
}  // namespace isaac
