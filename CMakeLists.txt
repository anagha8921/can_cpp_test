cmake_minimum_required(VERSION 3.8)
project(cpp_can_test)


# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


# Set the path to the can-utils library
set(CAN_UTILS_INCLUDE_DIRS "/home/devuser/AOX/ROS2/src/can-utils")

# Find can-utils headers
#find_path(CAN_UTILS_INCLUDE_DIRS NAMES dbc/dbc_parser.h PATHS ${CAN_UTILS_PATH}/include)

# Check if include directories are found
if(NOT CAN_UTILS_INCLUDE_DIRS)
    message(FATAL_ERROR "can-utils headers not found. Set CAN_UTILS_PATH to the correct location.")
endif()
set(CMAKE_BUILD_TYPE Debug)


# Include can-utils headers
include_directories(${CAN_UTILS_INCLUDE_DIRS})

# Add your executable
add_executable(can_cpp_test main.cpp ${CAN_UTILS_INCLUDE_DIRS}/dbc/dbc_parser.cpp ${CAN_UTILS_INCLUDE_DIRS}/v2c/v2c_transcoder.cpp)


# Set the C++ standard specifically for your target
target_compile_features(can_cpp_test PRIVATE cxx_std_20)
# Install the can_converter
install(TARGETS
can_cpp_test
  DESTINATION lib/${PROJECT_NAME})

