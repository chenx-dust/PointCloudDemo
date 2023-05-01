cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(point_cloud_demo)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(mongocxx REQUIRED)
find_package(bsoncxx REQUIRED)
include_directories(${LIBMONGOCXX_INCLUDE_DIR})
include_directories(${LIBBSONCXX_INCLUDE_DIR})

find_package(spdlog REQUIRED)

include_directories("./include")
set(SRC_LIST    "./src/db_reader.cpp"
                "./src/ground_rm.cpp"
                "./src/main.cpp" )

add_executable(point_cloud_demo ${SRC_LIST})
target_link_libraries(point_cloud_demo ${PCL_LIBRARIES})
target_link_libraries(point_cloud_demo mongo::mongocxx_shared)
target_link_libraries(point_cloud_demo mongo::bsoncxx_shared)
target_link_libraries(point_cloud_demo spdlog::spdlog)
