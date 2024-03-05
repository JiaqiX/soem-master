# Set namespace
set_root_namespace("soem-master")

add_subdirectory(ethercat_control)
add_subdirectory(frontend)
add_subdirectory(backend)
add_subdirectory(command)

# Set target name
set(CUR_TARGET_NAME soem-master)

# Set file collection
file(GLOB_RECURSE src ${CMAKE_CURRENT_SOURCE_DIR}/main.cc)

# Add target
add_executable(${CUR_TARGET_NAME})

# Set source file of target
target_sources(${CUR_TARGET_NAME} PRIVATE ${src})

# Set include path of target
target_include_directories(
  ${CUR_TARGET_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
                            $<INSTALL_INTERFACE:include>)

# Set link libraries of target
target_link_libraries(
  ${CUR_TARGET_NAME}
  PRIVATE yaml-cpp::yaml-cpp
  PUBLIC soem-master::backend soem-master::frontend soem-master::command)

install(
  TARGETS ${CUR_TARGET_NAME}
  EXPORT ${INSTALL_CONFIG_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
          FILE_SET HEADERS
          DESTINATION include/${CUR_DIR})
