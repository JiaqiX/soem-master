macro(set_root_namespace arg1)
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY AIMRT_CURRENT_NAMESPACE ${arg1})
endmacro()

macro(set_namespace)
  string(REGEX REPLACE ".*/\(.*\)" "\\1" AIMRT_CUR_DIR ${CMAKE_CURRENT_SOURCE_DIR})
  get_directory_property(AIMRT_CUR_DIRECTORY_PARENT PARENT_DIRECTORY)
  get_property(
    AIMRT_SUPERIOR_NAMESPACE
    DIRECTORY ${AIMRT_CUR_DIRECTORY_PARENT}
    PROPERTY AIMRT_CURRENT_NAMESPACE)
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY AIMRT_CURRENT_NAMESPACE ${AIMRT_SUPERIOR_NAMESPACE}::${AIMRT_CUR_DIR})
endmacro()

macro(get_namespace arg1)
  get_directory_property(AIMRT_CUR_DIRECTORY_PARENT PARENT_DIRECTORY)
  get_property(
    ${arg1}
    DIRECTORY ${AIMRT_CUR_DIRECTORY_PARENT}
    PROPERTY AIMRT_CURRENT_NAMESPACE)
  if(${arg1} STREQUAL "")
    message(FATAL_ERROR "Can not get namespace for ${CMAKE_CURRENT_SOURCE_DIR}")
  endif()
endmacro()
