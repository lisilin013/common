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

# include_directories(
#   ${Boost_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
# )


cs_add_library(${PROJECT_NAME} 
  src/common/csv_reader.cc
  src/common/file_path.cc
  src/common/pcl_utils.cc
  src/common/yaml_reader.cc
)

target_link_libraries(${PROJECT_NAME} 
  absl::strings
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

cs_install()
cs_export()
