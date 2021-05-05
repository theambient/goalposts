# cmake needs this line
cmake_minimum_required(VERSION 3.4)

# Define project name
project(goalposts)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

file(GLOB_RECURSE SOURCES "source/*")

add_executable(goalposts ${SOURCES})

target_link_libraries(goalposts PRIVATE ${OpenCV_LIBS})
