include(FetchContent)

message(STATUS "get fmt ...")

set(fmt_DOWNLOAD_URL
    "https://github.com/fmtlib/fmt/archive/9.1.0.tar.gz"
    CACHE STRING "")

FetchContent_Declare(
  fmt
  URL ${fmt_DOWNLOAD_URL}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  OVERRIDE_FIND_PACKAGE)

FetchContent_GetProperties(fmt)
if(NOT fmt_POPULATED)
  set(FMT_MASTER_PROJECT
      OFF
      CACHE BOOL "")
  set(FMT_INSTALL
      ON
      CACHE BOOL "")

  FetchContent_MakeAvailable(fmt)
endif()

# import targets：
# fmt::fmt
