include(FetchContent)

message(STATUS "get ethercat soem ...")

set(soem_DOWNLOAD_URL
    "https://github.com/OpenEtherCATsociety/SOEM/archive/refs/tags/v1.4.0.tar.gz"
    CACHE STRING "")

FetchContent_Declare(
  soem
  URL ${soem_DOWNLOAD_URL}
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

FetchContent_GetProperties(soem)
if(NOT soem_POPULATED)
  if(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-stringop-truncation -Wno-maybe-uninitialized")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-stringop-truncation -Wno-maybe-uninitialized")
  endif()
  FetchContent_MakeAvailable(soem)
  if(WIN32)
    set(OS "win32")
  elseif(UNIX AND NOT APPLE)
    set(OS "linux")
  elseif(APPLE)
    set(OS "macosx")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "rt-kernel")
    set(OS "rtk")
  elseif(${CMAKE_SYSTEM_NAME} MATCHES "rtems")
    set(OS "rtems")
  endif()

  add_library(soem::soem ALIAS soem)
  target_include_directories(soem PUBLIC ${soem_SOURCE_DIR} ${soem_SOURCE_DIR}/soem ${soem_SOURCE_DIR}/oshw ${soem_SOURCE_DIR}/oshw/${OS} ${soem_SOURCE_DIR}/osal
                                         ${soem_SOURCE_DIR}/osal/${OS})
endif()
