cmake_minimum_required(VERSION 3.14)
project(pointcloud_tool)

include(FetchContent)

FetchContent_Declare(
  fastcdr
  GIT_REPOSITORY https://github.com/eProsima/Fast-CDR.git
  GIT_TAG v1.0.24
)
FetchContent_MakeAvailable(fastcdr)

FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG v1.11.0
)
FetchContent_MakeAvailable(spdlog)

add_library(pointcloud2 src/pointcloud2.cpp)
target_include_directories(pointcloud2 PUBLIC include)
target_link_libraries(pointcloud2 PUBLIC fastcdr)
target_compile_options(pointcloud2 PRIVATE -Wall -Werror)

add_executable(pointcloud_tool src/pointcloud_tool.cpp)
target_link_libraries(pointcloud_tool PRIVATE pointcloud2 spdlog::spdlog)
target_compile_options(pointcloud_tool PRIVATE -Wall -Werror)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

enable_testing()
add_executable(pointcloud2_test tests/test_pointcloud2.cpp)
target_link_libraries(pointcloud2_test PRIVATE pointcloud2 Catch2::Catch2WithMain)
target_compile_options(pointcloud2_test PRIVATE -Wall -Werror)
add_test(NAME pointcloud2_test COMMAND pointcloud2_test)

