cmake_minimum_required(VERSION 3.18)
project(vpi_apriltag_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# VPI 3.x
# ---------------------------------------------------------------------------

set(VPI_ROOT "/opt/nvidia/vpi3" CACHE PATH "VPI 3.x install prefix")

find_path(VPI_INCLUDE_DIR
    NAMES vpi/algo/AprilTags.h
    HINTS "${VPI_ROOT}/include" "/usr/include")

find_library(VPI_LIBRARY
    NAMES nvvpi
    HINTS "${VPI_ROOT}/lib" "${VPI_ROOT}/lib64" "/usr/lib/aarch64-linux-gnu")

if(NOT VPI_INCLUDE_DIR OR NOT VPI_LIBRARY)
    message(FATAL_ERROR
        "VPI 3.x not found. Install libnvvpi3 and vpi3-dev, or set -DVPI_ROOT=...")
endif()

message(STATUS "VPI include: ${VPI_INCLUDE_DIR}")
message(STATUS "VPI library: ${VPI_LIBRARY}")

# ---------------------------------------------------------------------------
# pybind11
# ---------------------------------------------------------------------------

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -m pybind11 --cmakedir
        OUTPUT_VARIABLE _pb11_dir OUTPUT_STRIP_TRAILING_WHITESPACE)
    find_package(pybind11 CONFIG REQUIRED HINTS "${_pb11_dir}")
endif()

# ---------------------------------------------------------------------------
# Extension module
# ---------------------------------------------------------------------------

pybind11_add_module(_vpi_apriltag_detector MODULE
    src/bindings.cpp
    src/detector.cpp
    src/preprocess.cpp
    src/corner_order.cpp)

target_include_directories(_vpi_apriltag_detector PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${VPI_INCLUDE_DIR}")

target_link_libraries(_vpi_apriltag_detector PRIVATE "${VPI_LIBRARY}")

target_compile_options(_vpi_apriltag_detector PRIVATE -O3 -Wall -Wextra)

install(TARGETS _vpi_apriltag_detector DESTINATION vpi_apriltag_detector)

option(VPI_APRILTAG_BUILD_TESTS "Build native unit tests" OFF)
if(VPI_APRILTAG_BUILD_TESTS)
    enable_testing()
    add_executable(test_corners tests/test_corners.cpp src/corner_order.cpp)
    target_include_directories(test_corners PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
    target_compile_options(test_corners PRIVATE -Wall -Wextra)
    add_test(NAME corner_order COMMAND test_corners)
endif()
