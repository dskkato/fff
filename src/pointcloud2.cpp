#include "pointcloud2.hpp"

#include <fastcdr/Cdr.h>
#include <fastcdr/FastBuffer.h>
#include <fastcdr/exceptions/BadParamException.h>

#include <stdexcept>

namespace pc2 {

using eprosima::fastcdr::Cdr;
using eprosima::fastcdr::FastBuffer;

std::vector<uint8_t> serialize(const PointCloud2 &msg) {
  // Estimate buffer size: data + some overhead for metadata
  size_t size = msg.data.size() + msg.fields.size() * 100 + 1024;
  std::vector<uint8_t> buffer(size);
  FastBuffer fb(reinterpret_cast<char *>(buffer.data()), buffer.size());
  // Use DDS CDR serialization with encapsulation to match ROS 2 messages
  Cdr ser(fb, Cdr::DEFAULT_ENDIAN, Cdr::DDS_CDR);

  try {
    // Write the standard CDR encapsulation header so that the resulting
    // buffer can be consumed by ROS 2 tooling which expects it.
    ser.serialize_encapsulation();
    ser.serialize(msg.header.stamp.sec);
    ser.serialize(msg.header.stamp.nanosec);
    ser.serialize(msg.header.frame_id);
    ser.serialize(msg.height);
    ser.serialize(msg.width);
    ser.serialize(static_cast<uint32_t>(msg.fields.size()));
    for (const auto &f : msg.fields) {
      ser.serialize(f.name);
      ser.serialize(f.offset);
      ser.serialize(f.datatype);
      ser.serialize(f.count);
    }
    ser.serialize(msg.is_bigendian);
    ser.serialize(msg.point_step);
    ser.serialize(msg.row_step);
    ser.serialize(static_cast<uint32_t>(msg.data.size()));
    if (!msg.data.empty()) {
      ser.serializeArray(msg.data.data(), msg.data.size());
    }
    ser.serialize(msg.is_dense);
  } catch (const eprosima::fastcdr::exception::Exception &e) {
    throw std::runtime_error(e.what());
  }

  buffer.resize(ser.getSerializedDataLength());
  return buffer;
}

PointCloud2 deserialize(const std::vector<uint8_t> &buffer) {
  FastBuffer fb(reinterpret_cast<char *>(const_cast<uint8_t *>(buffer.data())),
                buffer.size());
  // Use DDS CDR deserialization and consume encapsulation to properly
  // interpret buffers produced by ROS 2.
  Cdr des(fb, Cdr::DEFAULT_ENDIAN, Cdr::DDS_CDR);
  PointCloud2 msg{};

  try {
    // Attempt to read the encapsulation header. If the buffer was produced
    // without encapsulation this will throw an exception, in which case we fall
    // back to assuming no encapsulation is present.
    des.read_encapsulation();

    des.deserialize(msg.header.stamp.sec);
    des.deserialize(msg.header.stamp.nanosec);
    des.deserialize(msg.header.frame_id);
    des.deserialize(msg.height);
    des.deserialize(msg.width);
    uint32_t fields_size = 0;
    des.deserialize(fields_size);
    msg.fields.resize(fields_size);
    for (uint32_t i = 0; i < fields_size; ++i) {
      des.deserialize(msg.fields[i].name);
      des.deserialize(msg.fields[i].offset);
      des.deserialize(msg.fields[i].datatype);
      des.deserialize(msg.fields[i].count);
    }
    des.deserialize(msg.is_bigendian);
    des.deserialize(msg.point_step);
    des.deserialize(msg.row_step);
    uint32_t data_size = 0;
    des.deserialize(data_size);
    msg.data.resize(data_size);
    if (data_size > 0) {
      des.deserializeArray(msg.data.data(), data_size);
    }
    des.deserialize(msg.is_dense);
  } catch (const eprosima::fastcdr::exception::Exception &e) {
    throw std::runtime_error(e.what());
  }

  return msg;
}

}  // namespace pc2
