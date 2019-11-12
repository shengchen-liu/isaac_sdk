/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "pybind_node.hpp"

#include "engine/alice/node.hpp"

namespace isaac {
namespace alice {

PybindNode::PybindNode() : node_(nullptr) {}

PybindNode::PybindNode(alice::Node* node) : node_(node) {}

PybindNode::~PybindNode() {}

void InitPybindNode(pybind11::module& m) {
  pybind11::class_<PybindNode>(m, "PybindNode")
      .def(pybind11::init<>())
      .def("isValid", &isaac::alice::PybindNode::isValid);
}

}  // namespace alice
}  // namespace isaac
