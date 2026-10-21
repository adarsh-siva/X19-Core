cmake_minimum_required(VERSION 3.10)
project(MessagingProject CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(cppzmq REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

file(GLOB_RECURSE PROTO_FILES "proto/*.proto" "src/protocols/cpp/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(protocols_lib ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(protocols_lib PUBLIC "${CMAKE_CURRENT_BINARY_DIR}")
target_link_libraries(protocols_lib PUBLIC protobuf::libprotobuf)

# Header-only ZMQ/protobuf wrappers. (Globbing src/cpp/*.cpp in here pulled
# every node's main() into one library, so no node could link.)
add_library(messaging_lib INTERFACE)
target_include_directories(messaging_lib INTERFACE "src/cpp" "src/cpp/messaging")
target_link_libraries(messaging_lib INTERFACE cppzmq protocols_lib Eigen3::Eigen)

# PID + EMA filter shared by depth control and the autonomy behaviors
add_library(controller_lib src/cpp/depth_control/controller.cpp)
target_include_directories(controller_lib PUBLIC "src/cpp/depth_control")

# ---- nodes ----
add_executable(depth_control src/cpp/depth_control/depth_control.cpp)
target_link_libraries(depth_control PRIVATE messaging_lib controller_lib)

add_executable(thruster_control src/cpp/thruster_control/thruster_control.cpp)
target_include_directories(thruster_control PRIVATE "src/cpp/thruster_control")
target_link_libraries(thruster_control PRIVATE messaging_lib)

add_executable(straight_line src/cpp/autonomy/straight_line.cpp)
target_link_libraries(straight_line PRIVATE messaging_lib controller_lib)

enable_testing()
file(GLOB TEST_SOURCES "testing/cpp/*.cpp")

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(${TEST_NAME} PRIVATE "testing/cpp")
    target_link_libraries(${TEST_NAME} PRIVATE messaging_lib controller_lib)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()

