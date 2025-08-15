#include <spdlog/spdlog.h>

#include <cstring>
#include <mcap/reader.hpp>
#include <mcap/writer.hpp>

#include "pointcloud2.hpp"

using namespace pc2;

void print_usage() {
  spdlog::info("Usage: pointcloud_tool [write <file>|read <file>]");
}

int main(int argc, char** argv) {
  if (argc < 3) {
    print_usage();
    return 1;
  }
  std::string mode = argv[1];
  std::string file = argv[2];
  auto make_sample_cloud = []() {
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
  };

  if (mode == "write") {
    PointCloud2 cloud = make_sample_cloud();
    auto buffer = serialize(cloud);
    mcap::McapWriter writer;
    mcap::McapWriterOptions options("");
    options.compression = mcap::Compression::Zstd;
    if (!writer.open(file, options).ok()) {
      spdlog::error("Failed to open MCAP file for writing");
      return 1;
    }
    mcap::Channel channel("pointcloud", "cdr", 0);
    writer.addChannel(channel);
    mcap::Message message;
    message.channelId = channel.id;
    message.sequence = 0;
    message.logTime = 0;
    message.publishTime = 0;
    message.dataSize = buffer.size();
    message.data = reinterpret_cast<const std::byte*>(buffer.data());
    if (!writer.write(message).ok()) {
      spdlog::error("Failed to write message");
      return 1;
    }
    writer.close();
    spdlog::info("Wrote MCAP message size {}", buffer.size());
    return 0;
  } else if (mode == "read") {
    mcap::McapReader reader;
    if (!reader.open(file).ok()) {
      spdlog::error("Failed to open MCAP file");
      return 1;
    }
    auto view = reader.readMessages();
    for (const auto& msgView : view) {
      std::vector<uint8_t> data(msgView.message.dataSize);
      std::memcpy(data.data(), msgView.message.data, msgView.message.dataSize);
      PointCloud2 cloud = deserialize(data);
      spdlog::info("Cloud width: {} height: {} fields: {} points: {}",
                   cloud.width, cloud.height, cloud.fields.size(),
                   cloud.width * cloud.height);
      break;
    }
    return 0;
  } else {
    print_usage();
    return 1;
  }
}
