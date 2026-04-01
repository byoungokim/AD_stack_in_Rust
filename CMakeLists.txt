cmake_minimum_required(VERSION 3.16)
project(limo_drive VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# --- Dependencies ---
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- Protobuf generation ---
file(GLOB PROTO_FILES "${CMAKE_SOURCE_DIR}/proto/*.proto")
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

add_library(limo_proto STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(limo_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(limo_proto PUBLIC protobuf::libprotobuf)

# --- Core library ---
add_library(limo_core STATIC
  core/transport/src/zmq_transport.cpp
)
target_include_directories(limo_core PUBLIC
  core/transport/include
  core/node/include
  ${ZMQ_INCLUDE_DIRS}
)
target_link_libraries(limo_core PUBLIC
  limo_proto
  ${ZMQ_LIBRARIES}
  spdlog::spdlog
  yaml-cpp
)

# --- Control process ---
# (entry points added as modules are implemented)
# add_executable(limo_control control/main.cpp)
# target_link_libraries(limo_control PRIVATE limo_core)

# --- SensPerc process ---
# add_executable(limo_sensperc sensperc/main.cpp)
# target_link_libraries(limo_sensperc PRIVATE limo_core)

# --- Planning process ---
# add_executable(limo_planning planning/main.cpp)
# target_link_libraries(limo_planning PRIVATE limo_core)
