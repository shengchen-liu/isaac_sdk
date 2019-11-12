/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <cstdarg>
#include <cstdio>
#include <vector>

#define LOG_DEBUG(...) \
    ::isaac::logger::LogImpl(__FILE__, __LINE__, \
                             ::isaac::logger::LoggingSeverity::LOG_LEVEL_DEBUG, __VA_ARGS__)
#define LOG_INFO(...) \
    ::isaac::logger::LogImpl(__FILE__, __LINE__, \
                             ::isaac::logger::LoggingSeverity::LOG_LEVEL_INFO, __VA_ARGS__)
#define LOG_WARNING(...) \
    ::isaac::logger::LogImpl(__FILE__, __LINE__, \
                             ::isaac::logger::LoggingSeverity::LOG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_ERROR(...) \
    ::isaac::logger::LogImpl(__FILE__, __LINE__, \
                             ::isaac::logger::LoggingSeverity::LOG_LEVEL_ERROR, __VA_ARGS__)

namespace isaac {
namespace logger {

// Enum to map severity to channel
enum class LoggingSeverity {
  LOG_LEVEL_ALL = -1,
  LOG_LEVEL_PANIC = 0,  // Need to start at 0
  LOG_LEVEL_ERROR,
  LOG_LEVEL_WARNING,
  LOG_LEVEL_INFO,
  LOG_LEVEL_DEBUG,
  LOG_LEVEL_NUM_SEVERITY
};

// Returns the severity type in a string format.
const char* LogSeverityToString(LoggingSeverity severity);

// External function used for logging, it can be changed to intercept the logged messages.
extern void (*LoggingFct)(const char* file, int line, LoggingSeverity severity, const char* log);

// Redirects the output for a given severity (use -1 to replace all of them)
void Redirect(std::FILE* file, LoggingSeverity severity = LoggingSeverity::LOG_LEVEL_ALL);

// Converts the message and argument into a string and pass it to LoggingFct.
template<typename... Args>
void LogImpl(const char* file, int line, LoggingSeverity severity, const char* txt, ...) {
  va_list args1;
  va_start(args1, txt);
  va_list args2;
  va_copy(args2, args1);
  std::vector<char> buf(1 + std::vsnprintf(NULL, 0, txt, args1));
  va_end(args1);
  std::vsnprintf(buf.data(), buf.size(), txt, args2);
  va_end(args2);
  LoggingFct(file, line, severity, buf.data());
}

}  // namespace logger
}  // namespace isaac
