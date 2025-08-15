#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace pc2 {

struct Time {
  int32_t sec{0};
  uint32_t nanosec{0};
};

struct Header {
  Time stamp{};
  std::string frame_id;
};

struct PointField {
  std::string name;
  uint32_t offset{0};
  uint8_t datatype{0};
  uint32_t count{0};
};

// Datatype constants matching sensor_msgs/PointField
enum PointFieldType : uint8_t {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8
};

struct PointCloud2 {
  Header header{};
  uint32_t height{0};
  uint32_t width{0};
  std::vector<PointField> fields;
  bool is_bigendian{false};
  uint32_t point_step{0};
  uint32_t row_step{0};
  std::vector<uint8_t> data;
  bool is_dense{false};
};

// Serialize PointCloud2 to CDR byte vector
std::vector<uint8_t> serialize(const PointCloud2 &msg);

// Deserialize PointCloud2 from CDR byte vector
PointCloud2 deserialize(const std::vector<uint8_t> &buffer);

}  // namespace pc2
