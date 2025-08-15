#include <catch2/catch_test_macros.hpp>
#include <cstring>

#include "pointcloud2.hpp"

using namespace pc2;

TEST_CASE("serialize_deserialize_roundtrip") {
  PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.height = 1;
  cloud.width = 2;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 12;  // 3 floats
  cloud.row_step = cloud.point_step * cloud.width;

  PointField fx{"x", 0, FLOAT32, 1};
  PointField fy{"y", 4, FLOAT32, 1};
  PointField fz{"z", 8, FLOAT32, 1};
  cloud.fields = {fx, fy, fz};

  cloud.data.resize(cloud.row_step * cloud.height);
  float points[6] = {0.f, 0.f, 0.f, 1.f, 2.f, 3.f};
  std::memcpy(cloud.data.data(), points, sizeof(points));

  auto buffer = serialize(cloud);
  PointCloud2 restored = deserialize(buffer);

  REQUIRE(restored.header.frame_id == cloud.header.frame_id);
  REQUIRE(restored.width == cloud.width);
  REQUIRE(restored.height == cloud.height);
  REQUIRE(restored.fields.size() == cloud.fields.size());
  REQUIRE(restored.data == cloud.data);
}
