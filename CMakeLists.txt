cmake_minimum_required(VERSION 3.16)
project(RoboticsSensorApp CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Submodule Dependencies ---
# spdlog: For high-performance, configurable logging.
add_subdirectory(libs/spdlog)
# CLI11: For robust command-line argument parsing.
add_subdirectory(libs/CLI11)

# --- Submodule Dependencies: INTERFACE library ---
# httplib.h: A simple, single-header HTTP server/client library.
add_library(httplib INTERFACE)
target_include_directories(httplib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs/httplib.h)

# nlohmann_json: For modern C++ JSON manipulation.
add_library(nlohmann_json INTERFACE)
target_include_directories(nlohmann_json INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs/json/include)

# Eigen: For linear algebra (quaternions, vectors).
add_library(Eigen INTERFACE)
target_include_directories(Eigen INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/libs/eigen)

# Serial: For cross-platform serial port communication.
add_library(serial STATIC libs/serial/src/serial.cc)
target_include_directories(serial PUBLIC libs/serial/include)

# --- COMMON Library ---
add_library(common INTERFACE)
target_include_directories(common INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src/common/include)

# --- Executable Targets ---
# Emulator Application
add_executable(emulator src/emulator/main.cpp)
target_link_libraries(emulator PRIVATE common spdlog CLI11::CLI11 serial)

# Consumer Application
add_executable(consumer src/consumer/main.cpp)
target_link_libraries(consumer PRIVATE common spdlog CLI11::CLI11 serial httplib nlohmann_json Eigen)

# --- Installation ---
# Define the output directory for the compiled binaries.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)