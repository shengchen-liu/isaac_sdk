/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "engine/core/buffers/aligned_buffer.hpp"

#include <cstdint>

#include "gtest/gtest.h"

namespace isaac {

TEST(AlignedBuffer, MallocConstructEmpty) {
  CpuAlignedBuffer buffer;
  EXPECT_EQ(buffer.rows(), 0);
  EXPECT_EQ(buffer.stride(), 0);
  EXPECT_EQ(buffer.pointer(), nullptr);
}

TEST(AlignedBuffer, MallocConstruct) {
  CpuAlignedBuffer buffer(10, 20);
  EXPECT_EQ(buffer.rows(), 10);
  EXPECT_EQ(buffer.stride(), 20);
  EXPECT_NE(buffer.pointer(), nullptr);
}

struct Malloc1024AlignedAllocator {
  static byte* Allocate(size_t rows, size_t& stride) {
    if (stride % 1024 != 0) {
      stride = 1024 * (stride / 1024 + 1);
    }
    return MallocAlignedAllocator::Allocate(rows, stride);
  }

  static void Deallocate(byte* pointer) {
    return MallocAlignedAllocator::Deallocate(pointer);
  }
};

TEST(AlignedBuffer, Malloc1024Construct1) {
  AlignedBuffer<HostPointer<byte>, Malloc1024AlignedAllocator> buffer(10, 20);
  EXPECT_EQ(buffer.rows(), 10);
  EXPECT_EQ(buffer.stride(), 1024);
  EXPECT_NE(buffer.pointer(), nullptr);
}

TEST(AlignedBuffer, Malloc1024Construct2) {
  AlignedBuffer<HostPointer<byte>, Malloc1024AlignedAllocator> buffer(10, 1025);
  EXPECT_EQ(buffer.rows(), 10);
  EXPECT_EQ(buffer.stride(), 2048);
  EXPECT_NE(buffer.pointer(), nullptr);
}

TEST(AlignedBuffer, Move) {
  CpuAlignedBuffer buffer(10, 45);
  EXPECT_EQ(buffer.rows(), 10);
  EXPECT_EQ(buffer.stride(), 45);
  EXPECT_NE(buffer.pointer(), nullptr);

  CpuAlignedBuffer buffer2 = std::move(buffer);
  EXPECT_EQ(buffer2.rows(), 10);
  EXPECT_EQ(buffer2.stride(), 45);
  EXPECT_NE(buffer2.pointer(), nullptr);
}

}  // namespace isaac
