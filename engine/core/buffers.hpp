/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/buffers/cpu_array.hpp"
#include "engine/core/buffers/cpu_array_view.hpp"
#include "engine/core/buffers/cuda_array.hpp"
#include "engine/core/buffers/cuda_array_view.hpp"
#include "engine/core/byte.hpp"

namespace isaac {

// A buffer which allocates CPU memory with malloc and handles its ownership
using CpuMallocByteBuffer = CpuArray<byte>;

// An alias for a generic byte buffer which owns its memory. This will soon be replaced by a new
// buffer type which can handle both CPU and GPU memory.
using ByteBuffer = CpuMallocByteBuffer;

// A mutable buffer of bytes which does not own its memory
using ByteBufferView = CpuArrayView<byte>;
// A non-mutable buffer of bytes which does not own its memory
using ByteBufferConstView = ConstCpuArrayView<byte>;

}  // namespace isaac
