/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "logger.hpp"

#include <sys/time.h>

#include <ctime>

#include "engine/core/assert.hpp"

namespace isaac {
namespace logger {

// Documentation for console coloring:
// https://misc.flogisoft.com/bash/tip_colors_and_formatting

namespace {
// Where to write logging messages
std::FILE* files_[] = {
  stderr,  // PANIC
  stderr,  // LOG_ERROR
  stdout,  // LOG_WARNING
  stdout,  // LOG_INFO
  stdout,  // LOG_DEBUG
};
constexpr int kNumSeverity = sizeof(files_) / sizeof(std::FILE*);
static_assert(kNumSeverity == static_cast<int>(LoggingSeverity::LOG_LEVEL_NUM_SEVERITY),
              "Mismatch between the size of files and the number of log level.");
}  // namespace

void Redirect(std::FILE* file, LoggingSeverity severity) {
  if (severity == LoggingSeverity::LOG_LEVEL_ALL) {
    for (int i = 0; i < kNumSeverity; i++) {
      files_[i] = file;
    }
  } else {
    files_[static_cast<int>(severity)] = file;
  }
}

const char* LogSeverityToString(LoggingSeverity severity) {
  switch (severity) {
    case LoggingSeverity::LOG_LEVEL_ALL:     return "ALL    ";
    case LoggingSeverity::LOG_LEVEL_PANIC:   return "PANIC  ";
    case LoggingSeverity::LOG_LEVEL_ERROR:   return "ERROR  ";
    case LoggingSeverity::LOG_LEVEL_WARNING: return "WARNING";
    case LoggingSeverity::LOG_LEVEL_INFO:    return "INFO   ";
    case LoggingSeverity::LOG_LEVEL_DEBUG:   return "DEBUG  ";
    case LoggingSeverity::LOG_LEVEL_NUM_SEVERITY: return "NUM_SEVERITY";
    default: std::abort();
  }
}


void LoggingFctImpl(const char* file, int line, LoggingSeverity severity, const char* log) {
  if (severity == LoggingSeverity::LOG_LEVEL_ALL ||
      severity == LoggingSeverity::LOG_LEVEL_NUM_SEVERITY) {
    std::fprintf(stderr, "Log severity cannot be `%s`.", LogSeverityToString(severity));
    std::abort();
  }
  std::FILE* outstream = files_[static_cast<int>(severity)];
  if (outstream == nullptr) {
    return;
  }
  struct tm* tm_info;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  tm local_tm;
  tm_info = localtime_r(&tv.tv_sec, &local_tm);
  char buff[20];
  strftime(buff, 20, "%Y-%m-%d %H:%M:%S", tm_info);

  if (severity == LoggingSeverity::LOG_LEVEL_PANIC) {
    std::fprintf(outstream, "\033[1;3;31m%s.%03ld %s %s@%d: %s\033[0m\n",
                 buff, tv.tv_usec / 1000, LogSeverityToString(severity), file, line, log);
  } else if (severity == LoggingSeverity::LOG_LEVEL_ERROR) {
    std::fprintf(outstream, "\033[1;31m%s.%03ld %s %s@%d: %s\033[0m\n",
                 buff, tv.tv_usec / 1000, LogSeverityToString(severity), file, line, log);
  } else if (severity == LoggingSeverity::LOG_LEVEL_WARNING) {
    std::fprintf(outstream, "\033[33m%s.%03ld %s %s@%d: %s\033[0m\n",
                 buff, tv.tv_usec / 1000, LogSeverityToString(severity), file, line, log);
  } else if (severity == LoggingSeverity::LOG_LEVEL_INFO) {
    std::fprintf(outstream, "\033[0m%s.%03ld %s %s@%d: %s\033[0m\n",
                 buff, tv.tv_usec / 1000, LogSeverityToString(severity), file, line, log);
  } else {
    std::fprintf(outstream, "\033[90m%s.%03ld %s %s@%d: %s\033[0m\n",
                 buff, tv.tv_usec / 1000, LogSeverityToString(severity), file, line, log);
  }
  std::fflush(outstream);
}

void (*LoggingFct)(const char*, int, LoggingSeverity, const char*) = LoggingFctImpl;

}  // namespace logger
}  // namespace isaac
