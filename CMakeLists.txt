cmake_minimum_required(VERSION 3.12)
project(tmx_cpp_py)

# find_package(Boost REQUIRED COMPONENTS system thread)

add_subdirectory(libs/tmx-cpp)

# Setup propper linking
set_target_properties(tmx_cpp async_serial PROPERTIES POSITION_INDEPENDENT_CODE TRUE)

add_subdirectory(pybind11)

# set(CMAKE_CXX_STANDARD 11)

include_directories(
  include/ libs/tmx-cpp/include/ libs/tmx-cpp/libs/async_serial/include/
  pybind11/include/
)

pybind11_add_module(tmx_cpp_py src/main.cpp)

set_property(TARGET tmx_cpp_py PROPERTY CXX_STANDARD 20)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_definitions(
    tmx_cpp_py PRIVATE _LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION
                       _LIBCPP_ENABLE_CXX20_REMOVED_TYPE_TRAITS
  )
endif()

target_link_libraries(tmx_cpp_py PRIVATE tmx_cpp)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a define
# (VERSION_INFO) here.
target_compile_definitions(tmx_cpp_py PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
