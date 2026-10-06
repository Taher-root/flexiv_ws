cmake_minimum_required(VERSION 3.16.3)
project(aico2_rt_control_tests CXX)

# These tests need no RDK, no ROS and no robot: they cover the shared-memory
# contract and the sampler's maths, which is where the correctness risk in this
# package actually lives. So they run anywhere, including in CI and on a laptop.
#
#   cmake -S . -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
#
# Warnings are errors here. The sampler is numerical code and the protocol is
# shared across a process boundary; a shadowed variable or a narrowing
# conversion in either is a bug, not a style question.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

enable_testing()
find_package(Threads REQUIRED)

foreach(t test_traj_sampler test_shm_protocol test_rt_executor)
  add_executable(${t} ${t}.cpp)
  target_include_directories(${t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
  target_compile_features(${t} PRIVATE cxx_std_17)
  target_compile_options(${t} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(${t} PRIVATE Threads::Threads)
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# shm_open/shm_unlink live in librt on glibc before 2.34; harmless after.
target_link_libraries(test_shm_protocol PRIVATE rt)
