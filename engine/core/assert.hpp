/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdlib>

#include "engine/core/logger.hpp"

#define PANIC(...) \
    { \
      ::isaac::logger::LogImpl(__FILE__, __LINE__, \
          ::isaac::logger::LoggingSeverity::LOG_LEVEL_PANIC, __VA_ARGS__); \
      std::abort(); \
    }

#define ASSERT(expr, ...) \
  if (!(expr)) { \
    ::isaac::logger::LogImpl(__FILE__, __LINE__, \
        ::isaac::logger::LoggingSeverity::LOG_LEVEL_PANIC, __VA_ARGS__); \
    std::abort(); \
  }
