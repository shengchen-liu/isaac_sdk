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
#include <vector>

#include "engine/core/logger.hpp"
#include "engine/core/math/types.hpp"
#include "engine/gems/sight/sop.hpp"

namespace isaac {
namespace sight {

// Interface for sight.
// Provide a default implementation which does nothing.
class SightInterface {
 public:
  virtual ~SightInterface() {}

  // Plot value
  virtual void plotValue(const std::string& name, int64_t timestamp, float value) {}
  virtual void plotValue(const std::string& name, int64_t timestamp, double value) {}
  virtual void plotValue(const std::string& name, int64_t timestamp, int value) {}
  virtual void plotValue(const std::string& name, int64_t timestamp, int64_t value) {}

  // Plot 3D laser point
  virtual void plotLaser(const std::string& name,
                         const std::vector<Vector<float, 6>>& point_cloud) {}

  // Display Log message
  virtual void log(const char* file, int line, logger::LoggingSeverity severity, const char* log,
                   int64_t timestamp) {}

  // Display images and draw on it.
  virtual void drawCanvas(const std::string& name, sight::Sop canvas) {}
};

}  // namespace sight
}  // namespace isaac
