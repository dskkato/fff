# PointCloud2 CDR Tool

This project provides a small C++ utility to serialize and deserialize
ROS 2 `PointCloud2` messages using CDR encoding without relying on the
full ROS 2 stack or large middleware implementations.

## Building

```bash
mkdir build && cd build
cmake ..
make
```

Run unit tests:

```bash
ctest --test-dir build
```

## Usage

Write a sample point cloud to an MCAP file:

```bash
./pointcloud_tool write cloud.mcap
```

Read a cloud back from an MCAP file:

```bash
./pointcloud_tool read cloud.mcap
```

