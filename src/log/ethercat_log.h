/*
 * @Descripttion:
 * @Author: editorxu
 * @version:
 * @Date: 2024-02-27 10:00:38
 * @LastEditors: editorxu
 * @LastEditTime: 2024-03-05 10:26:02
 */
#pragma once

#include <iostream>
#include <source_location>

#include "fmt/chrono.h"
#include "fmt/core.h"

#include <stdio.h>

#define aimrt_fmt fmt

namespace ether_control {

#define LOG_CLRSTR_NONE "\033[m"
#define LOG_CLRSTR_RED "\033[0;32;31m"
#define LOG_CLRSTR_GREEN "\033[0;32;32m"
#define LOG_CLRSTR_BLUE "\033[0;32;34m"
#define LOG_CLRSTR_DARY_GRAY "\033[1;30m"
#define LOG_CLRSTR_CYAN "\033[0;36m"
#define LOG_CLRSTR_PURPLE "\033[0;35m"
#define LOG_CLRSTR_BROWN "\033[0;33m"
#define LOG_CLRSTR_YELLOW "\033[1;33m"
#define LOG_CLRSTR_WHITE "\033[1;37m"

#define ETHER_HANDLE_LOG_v1(__lgr__, __lvl__, __clr__, __fmt__, ...)        \
  do {                                                                      \
    std::string __log_str__ = std::format(__fmt__, ##__VA_ARGS__);          \
    constexpr auto __location__ = std::source_location::current();          \
    __lgr__ << __clr__ << __lvl__ << " file: " << __location__.file_name()  \
            << " line: " << __location__.line()                             \
            << " column: " << __location__.column()                         \
            << " function: " << __location__.function_name()                \
            << " content: " << __log_str__ << LOG_CLRSTR_NONE << std::endl; \
  } while (0)

#define ETHER_HANDLE_LOG(__lgr__, __lvl__, __clr__, __fmt__, ...)      \
  do {                                                                 \
    std::string __log_str__ = std::format(__fmt__, ##__VA_ARGS__);     \
    constexpr auto __location__ = std::source_location::current();     \
    __lgr__ << __clr__ << __log_str__ << LOG_CLRSTR_NONE << std::endl; \
  } while (0)

#define ETHER_DEFAULT_LOGGER_HANDLE std::cout
#define Name2Str(name) (#name)

#define ETHER_TRACE(__fmt__, ...)                                \
  ETHER_HANDLE_LOG(ETHER_DEFAULT_LOGGER_HANDLE, Name2Str(TRACE), \
                   LOG_CLRSTR_WHITE, __fmt__, ##__VA_ARGS__)
#define ETHER_DEBUG(__fmt__, ...)                                \
  ETHER_HANDLE_LOG(ETHER_DEFAULT_LOGGER_HANDLE, Name2Str(DEBUG), \
                   LOG_CLRSTR_BROWN, __fmt__, ##__VA_ARGS__)
#define ETHER_INFO(__fmt__, ...)                                \
  ETHER_HANDLE_LOG(ETHER_DEFAULT_LOGGER_HANDLE, Name2Str(INFO), \
                   LOG_CLRSTR_GREEN, __fmt__, ##__VA_ARGS__)
#define ETHER_WARN(__fmt__, ...)                                \
  ETHER_HANDLE_LOG(ETHER_DEFAULT_LOGGER_HANDLE, Name2Str(WARN), \
                   LOG_CLRSTR_YELLOW, __fmt__, ##__VA_ARGS__)
#define ETHER_ERROR(__fmt__, ...)                                \
  ETHER_HANDLE_LOG(ETHER_DEFAULT_LOGGER_HANDLE, Name2Str(ERROR), \
                   LOG_CLRSTR_RED, __fmt__, ##__VA_ARGS__)
#define ETHER_FATAL(__fmt__, ...)                                \
  ETHER_HANDLE_LOG(ETHER_DEFAULT_LOGGER_HANDLE, Name2Str(FATAL), \
                   LOG_CLRSTR_RED, __fmt__, ##__VA_ARGS__)

}  // namespace ether_control