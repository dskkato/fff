cmake_minimum_required(VERSION 3.14)
project(pointcloud_tool)

set(CMAKE_CXX_STANDARD 17)

# Option to control whether to build shared or static libraries.
# By default we build static libraries so the final executable can be a
# self-contained binary without external `.so` dependencies. Users can enable
# shared libraries via `-DBUILD_SHARED_LIBS=ON`.
option(BUILD_SHARED_LIBS "Build project libraries as shared" OFF)

include(FetchContent)

FetchContent_Declare(
  fastcdr
  URL https://github.com/eProsima/Fast-CDR/archive/refs/tags/v1.0.24.tar.gz
)
FetchContent_MakeAvailable(fastcdr)

FetchContent_Declare(
  spdlog
  URL https://github.com/gabime/spdlog/archive/refs/tags/v1.11.0.tar.gz
)
FetchContent_MakeAvailable(spdlog)

FetchContent_Declare(
  zstd
  URL https://github.com/facebook/zstd/archive/refs/tags/v1.5.5.tar.gz
)
FetchContent_GetProperties(zstd)
if(NOT zstd_POPULATED)
  FetchContent_Populate(zstd)
  set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
  set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  add_subdirectory(${zstd_SOURCE_DIR}/build/cmake ${zstd_BINARY_DIR})
endif()

FetchContent_Declare(
  mcap
  URL https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v2.0.2.tar.gz
)
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
  FetchContent_Populate(mcap)
endif()
add_library(mcap src/mcap.cpp)
target_include_directories(mcap PUBLIC ${mcap_SOURCE_DIR}/cpp/mcap/include)
target_link_libraries(mcap PUBLIC libzstd_static)
target_compile_definitions(mcap PUBLIC MCAP_COMPRESSION_NO_LZ4)

add_library(pointcloud2 src/pointcloud2.cpp)
target_include_directories(pointcloud2 PUBLIC include)
target_link_libraries(pointcloud2 PUBLIC fastcdr)
target_compile_options(pointcloud2 PRIVATE -Wall -Werror)

add_executable(pointcloud_tool src/pointcloud_tool.cpp)
target_link_libraries(pointcloud_tool PRIVATE pointcloud2 spdlog::spdlog mcap)
target_compile_options(pointcloud_tool PRIVATE -Wall -Werror)

FetchContent_Declare(
  Catch2
  URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.4.0.tar.gz
)
FetchContent_MakeAvailable(Catch2)

enable_testing()
add_executable(pointcloud2_test tests/test_pointcloud2.cpp)
target_link_libraries(pointcloud2_test PRIVATE pointcloud2 Catch2::Catch2WithMain mcap)
target_compile_options(pointcloud2_test PRIVATE -Wall -Werror)
add_test(NAME pointcloud2_test COMMAND pointcloud2_test)

add_executable(mcap_test tests/test_mcap.cpp)
target_link_libraries(mcap_test PRIVATE pointcloud2 mcap Catch2::Catch2WithMain)
target_compile_options(mcap_test PRIVATE -Wall -Werror)
add_test(NAME mcap_test COMMAND mcap_test)

