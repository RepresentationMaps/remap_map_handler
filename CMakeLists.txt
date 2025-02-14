cmake_minimum_required(VERSION 3.8)
project(remap_map_handler)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}_lib
  src/remap_map_handler/frustum_fov.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
  ${Tbb_LIBRARIES}
  ${OpenVDB_LIBRARIES}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}_lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
