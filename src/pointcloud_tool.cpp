#include <spdlog/spdlog.h>

#include <cstring>
#include <fstream>

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
  if (mode == "write") {
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
    std::ofstream ofs(file, std::ios::binary);
    ofs.write(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    spdlog::info("Wrote {} bytes", buffer.size());
    return 0;
  } else if (mode == "read") {
    std::ifstream ifs(file, std::ios::binary);
    if (!ifs) {
      spdlog::error("Failed to open file");
      return 1;
    }
    std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(ifs)),
                                std::istreambuf_iterator<char>());
    PointCloud2 cloud = deserialize(buffer);
    spdlog::info("Cloud width: {} height: {} fields: {} points: {}",
                 cloud.width, cloud.height, cloud.fields.size(),
                 cloud.width * cloud.height);
    return 0;
  } else {
    print_usage();
    return 1;
  }
}
