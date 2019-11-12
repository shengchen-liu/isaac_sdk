/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "color.hpp"

#include <cstdio>
#include <string>

#include "engine/gems/interpolation/utils.hpp"

namespace isaac {

std::string ToHexString(const Pixel3ub& color) {
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "#%02x%02x%02x", color[0], color[1], color[2]);
  return std::string(buffer);
}

std::string ToHexString(const Pixel3ub& color, unsigned char alpha) {
  char buffer[10];
  std::snprintf(buffer, sizeof(buffer), "#%02x%02x%02x%02x", color[0], color[1], color[2], alpha);
  return std::string(buffer);
}

ColorGradient BlackWhiteColorGradient() {
  return ColorGradient({Colors::Black(), Colors::White()});
}

ColorGradient StarryNightColorGradient() {
  return ColorGradient({
      Pixel3ub{22, 37, 52}, Pixel3ub{66, 104, 114}, Pixel3ub{147, 177, 162},
      Pixel3ub{221, 220, 152}, Pixel3ub{244, 207, 94}});
}

ColorGradient BlackGreenColorGradient() {
  return ColorGradient({Colors::Black(), Colors::NvidiaGreen()});
}

ColorGradient RainbowColorGradient() {
  return ColorGradient({
      Colors::Red(),  Colors::Yellow(),
      Colors::Green(), Colors::Cyan(),
      Colors::Blue(), Colors::Magenta()});
}

ColorGradient ErrorColorGradient() {
  return ColorGradient({
      Pixel3ub{255, 255, 255}, Pixel3ub{255, 215, 107}, Pixel3ub{253, 151, 26},
      Pixel3ub{221, 86, 47}, Pixel3ub{117, 41, 118}, Pixel3ub{0, 0, 0}});
}

ColorGradient DistanceColorGradient() {
  return ColorGradient({Pixel3ub{51, 51, 51}, Pixel3ub{127, 51, 51}, Pixel3ub{255, 229, 165},
                        Pixel3ub{255, 255, 255}, Pixel3ub{216, 242, 255}});
}

ColorGradient RedBlueColorGradient() {
  return ColorGradient{{Pixel3f{0.450385, 0.157961, 0.217975},
                        Pixel3f{0.857126, 0.848339,  0.734867},
                        Pixel3f{0.139681, 0.311666, 0.550652}}};
}

void Colorize(const ImageConstView1f& input, const ColorGradient& gradient, float min, float max,
              Image3ub& colored) {
  colored.resize(input.rows(), input.cols());
  for (size_t i = 0; i < input.num_pixels(); i++) {
    colored[i] = gradient(Rescale(input[i], min, max));
  }
}

void Colorize(const ImageConstView1d& input, const ColorGradient& gradient, double min, double max,
              Image3ub& colored) {
  colored.resize(input.rows(), input.cols());
  for (size_t i = 0; i < input.num_pixels(); i++) {
    colored[i] = gradient(static_cast<float>(Rescale(input[i], min, max)));
  }
}

}  // namespace isaac
