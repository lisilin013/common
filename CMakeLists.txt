cmake_minimum_required(VERSION 3.0.2)
project(common)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS serialization system filesystem regex program_options iostreams)
find_package(absl)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

cs_add_library(${PROJECT_NAME} 
  src/file/csv_reader.cc
  src/file/file_path.cc
  src/file/yaml_reader.cc
  src/pcl_utils/pcl_utils.cc
  src/ros_utils/ros_utils.cc
  src/ros_utils/tf_transform_broadcaster.cc
)

target_link_libraries(${PROJECT_NAME} 
  absl::strings
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

cs_install()
cs_export()
