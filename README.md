# ROS 2 MCAP Decoder Sample

This repository demonstrates decoding ROS 2 MCAP files without
depending on the full ROS 2 stack. It provides a minimal C++ CMake
project that uses [mcap](https://github.com/foxglove/mcap) and
[Fast-CDR](https://github.com/eProsima/Fast-CDR) to work with
`PointCloud2` messages.

## Building

```bash
mkdir build && cd build
cmake ..
make
```

The default build links all project components statically so `pointcloud_tool`
does not depend on accompanying `.so` files. If shared libraries are desired,
enable them explicitly when configuring CMake:

```bash
cmake -DBUILD_SHARED_LIBS=ON ..
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

Filter messages by topic name when reading:

```bash
./pointcloud_tool read cloud.mcap --topic pointcloud
```

