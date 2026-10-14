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

file(GLOB_RECURSE MESSAGING_SOURCES "src/cpp/*.cpp" "src/cpp/*.hpp")
add_library(messaging_lib ${MESSAGING_SOURCES})
target_include_directories(messaging_lib PUBLIC "src/cpp" "src/cpp/messaging")
target_link_libraries(messaging_lib PUBLIC cppzmq protocols_lib Eigen3::Eigen)

enable_testing()
file(GLOB TEST_SOURCES "testing/cpp/*.cpp")

foreach(TEST_SOURCE ${TEST_SOURCES})
    get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(${TEST_NAME} PRIVATE "testing/cpp")
    target_link_libraries(${TEST_NAME} PRIVATE messaging_lib protocols_lib cppzmq)
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endforeach()

