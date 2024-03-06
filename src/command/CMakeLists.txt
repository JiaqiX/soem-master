# Get the current folder name
string(REGEX REPLACE ".*/\(.*\)" "\\1" CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# Get namespace
get_namespace(CUR_SUPERIOR_NAMESPACE)
string(REPLACE "::" "_" CUR_SUPERIOR_NAMESPACE_UNDERLINE ${CUR_SUPERIOR_NAMESPACE})

# Set target name
set(CUR_TARGET_NAME ${CUR_SUPERIOR_NAMESPACE_UNDERLINE}_${CUR_DIR})
set(CUR_TARGET_ALIAS_NAME ${CUR_SUPERIOR_NAMESPACE}::${CUR_DIR})

# Set file collection
file(GLOB_RECURSE head_files ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/*.cc)
file(GLOB_RECURSE test_files ${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc)
list(REMOVE_ITEM src ${test_files})

# Add target
add_library(${CUR_TARGET_NAME} STATIC)
add_library(${CUR_TARGET_ALIAS_NAME} ALIAS ${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
         $<INSTALL_INTERFACE:include>)

# Set head files of target
target_sources(${CUR_TARGET_NAME} INTERFACE FILE_SET HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} FILES ${head_files})

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PUBLIC ${CUR_SUPERIOR_NAMESPACE}::master)

# Set installation of target
set_property(TARGET ${CUR_TARGET_NAME} PROPERTY EXPORT_NAME ${CUR_TARGET_ALIAS_NAME})
install(
  TARGETS ${CUR_TARGET_NAME}
  EXPORT ${INSTALL_CONFIG_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
          FILE_SET HEADERS
          DESTINATION include/${CUR_DIR})

# Set test of target
if(AGIBOT_Z2_HARDWARE_MODULE_BUILD_TESTS AND test_files)
  add_gtest_target(TEST_TARGET ${CUR_TARGET_NAME} TEST_SRC ${test_files})
endif()
