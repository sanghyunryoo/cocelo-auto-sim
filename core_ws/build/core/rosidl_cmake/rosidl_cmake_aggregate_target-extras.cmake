# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target core::core
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${core_TARGETS}.
if(core_TARGETS AND NOT TARGET core::core)
  add_library(core::core INTERFACE IMPORTED)
  set_target_properties(core::core PROPERTIES
    INTERFACE_LINK_LIBRARIES "${core_TARGETS}")
endif()
