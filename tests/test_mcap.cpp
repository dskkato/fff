#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <cstring>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include "pointcloud2.hpp"

using namespace pc2;

static PointCloud2 makeSampleCloud() {
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
  return cloud;
}

TEST_CASE("mcap_write_read_roundtrip") {
  PointCloud2 cloud = makeSampleCloud();
  auto buffer = serialize(cloud);

  mcap::McapWriter writer;
  mcap::McapWriterOptions options("");
  options.compression = mcap::Compression::Zstd;
  REQUIRE(writer.open("test.mcap", options).ok());
  mcap::Channel channel("pointcloud", "cdr", 0);
  writer.addChannel(channel);
  mcap::Message message;
  message.channelId = channel.id;
  message.sequence = 0;
  message.logTime = 0;
  message.publishTime = 0;
  message.dataSize = buffer.size();
  message.data = reinterpret_cast<const std::byte*>(buffer.data());
  REQUIRE(writer.write(message).ok());
  writer.close();

  mcap::McapReader reader;
  REQUIRE(reader.open("test.mcap").ok());
  auto view = reader.readMessages();
  PointCloud2 restored;
  for (const auto& msgView : view) {
    std::vector<uint8_t> data(msgView.message.dataSize);
    std::memcpy(data.data(), msgView.message.data, msgView.message.dataSize);
    restored = deserialize(data);
    break;
  }
  REQUIRE(restored.data == cloud.data);
  std::remove("test.mcap");
}
