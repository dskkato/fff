#include "pointcloud2.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/exceptions/Exception.h>

#include <stdexcept>

namespace pc2 {

using eprosima::fastcdr::Cdr;
using eprosima::fastcdr::FastBuffer;

std::vector<uint8_t> serialize(const PointCloud2 &msg) {
  // Estimate buffer size: data + some overhead for metadata
  size_t size = msg.data.size() + msg.fields.size() * 100 + 1024;
  std::vector<uint8_t> buffer(size);
  FastBuffer fb(reinterpret_cast<char *>(buffer.data()), buffer.size());
  Cdr ser(fb);

  try {
    ser << msg.header.stamp.sec;
    ser << msg.header.stamp.nanosec;
    ser << msg.header.frame_id;
    ser << msg.height;
    ser << msg.width;
    ser << static_cast<uint32_t>(msg.fields.size());
    for (const auto &f : msg.fields) {
      ser << f.name;
      ser << f.offset;
      ser << f.datatype;
      ser << f.count;
    }
    ser << msg.is_bigendian;
    ser << msg.point_step;
    ser << msg.row_step;
    ser << static_cast<uint32_t>(msg.data.size());
    if (!msg.data.empty()) {
      ser.serializeArray(msg.data.data(), msg.data.size());
    }
    ser << msg.is_dense;
  } catch (const eprosima::fastcdr::exception::Exception &e) {
    throw std::runtime_error(e.what());
  }

  buffer.resize(ser.getSerializedDataLength());
  return buffer;
}

PointCloud2 deserialize(const std::vector<uint8_t> &buffer) {
  FastBuffer fb(reinterpret_cast<char *>(const_cast<uint8_t *>(buffer.data())),
                buffer.size());
  Cdr des(fb);
  PointCloud2 msg;

  try {
    des >> msg.header.stamp.sec;
    des >> msg.header.stamp.nanosec;
    des >> msg.header.frame_id;
    des >> msg.height;
    des >> msg.width;
    uint32_t fields_size = 0;
    des >> fields_size;
    msg.fields.resize(fields_size);
    for (uint32_t i = 0; i < fields_size; ++i) {
      des >> msg.fields[i].name;
      des >> msg.fields[i].offset;
      des >> msg.fields[i].datatype;
      des >> msg.fields[i].count;
    }
    des >> msg.is_bigendian;
    des >> msg.point_step;
    des >> msg.row_step;
    uint32_t data_size = 0;
    des >> data_size;
    msg.data.resize(data_size);
    if (data_size > 0) {
      des.deserializeArray(msg.data.data(), data_size);
    }
    des >> msg.is_dense;
  } catch (const eprosima::fastcdr::exception::Exception &e) {
    throw std::runtime_error(e.what());
  }

  return msg;
}

}  // namespace pc2
