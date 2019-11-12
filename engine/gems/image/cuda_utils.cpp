/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "cuda_utils.hpp"

#include <algorithm>

#include "engine/core/math/utils.hpp"
#include "engine/gems/image/color.hpp"

#include "nppi_arithmetic_and_logical_operations.h"  // NOLINT(build/include)
#include "nppi_data_exchange_and_initialization.h"   // NOLINT(build/include)
#include "nppi_filtering_functions.h"                // NOLINT(build/include)
#include "nppi_geometry_transforms.h"                // NOLINT(build/include)
#include "npps_initialization.h"                     // NOLINT(build/include)

namespace isaac {

namespace {

// Invokes matching Npp clear function for different data types
template <typename K>
void NppClear(const int length, K* device_memory);

template <>
void NppClear(const int length, uint8_t* device_memory) {
  nppsZero_8u(static_cast<Npp8u*>(device_memory), length);
}
template <>
void NppClear(const int length, int16_t* device_memory) {
  nppsZero_16s(reinterpret_cast<Npp16s*>(device_memory), length);
}
template <>
void NppClear(const int length, int* device_memory) {
  nppsZero_32s(static_cast<Npp32s*>(device_memory), length);
}
template <>
void NppClear(const int length, float* device_memory) {
  nppsZero_32f(static_cast<Npp32f*>(device_memory), length);
}
template <>
void NppClear(const int length, double* device_memory) {
  nppsZero_64f(static_cast<Npp64f*>(device_memory), length);
}

// Converts RegionOfInterest into Npp counterpart NppiRect. Npp has (col, row) coordinate instead of
// (row, col) in isaac.
NppiRect RoiToRect(const RegionOfInterest& roi) {
  return NppiRect{roi.offset.y(), roi.offset.x(), roi.size.y(), roi.size.x()};
}

}  // namespace

template <typename K, int N>
void FillElementsWithZero(CudaImageView<K, N> dst_img) {
  const int length = dst_img.data().size();
  NppClear<K>(length, reinterpret_cast<K*>(dst_img.data().begin()));
}

#define ISAAC_DECLARE_ZERO_IMPL(N)                                       \
  template void FillElementsWithZero(CudaImageView<uint8_t, N> dst_img); \
  template void FillElementsWithZero(CudaImageView<int16_t, N> dst_img); \
  template void FillElementsWithZero(CudaImageView<int, N> dst_img);     \
  template void FillElementsWithZero(CudaImageView<float, N> dst_img);   \
  template void FillElementsWithZero(CudaImageView<double, N> dst_img);

ISAAC_DECLARE_ZERO_IMPL(1)
ISAAC_DECLARE_ZERO_IMPL(2)
ISAAC_DECLARE_ZERO_IMPL(3)
ISAAC_DECLARE_ZERO_IMPL(4)

void DenormalizeImage(CudaImageView3f src_image, ImageNormalizationMode mode,
                      CudaImageView3ub dst_image) {
  ASSERT(src_image.cols() == dst_image.cols() && src_image.channels() == dst_image.channels(),
         "Mismatched input-output dimensions.");

  NppiSize roi = ImageNppSize(src_image);
  const int src_step = static_cast<int>(src_image.getStride());
  const int dst_step = static_cast<int>(dst_image.getStride());
  Npp32f bias[3] = {1.0f, 1.0f, 1.0f};
  Npp32f factor[3] = {.5f * 255.0f, .5f * 255.0f, .5f * 255.0f};
  Npp32f zerounit_factor[3] = {255.0f, 255.0f, 255.0f};

  NppStatus ok = NPP_SUCCESS;

  switch (mode) {
    case ImageNormalizationMode::kPositiveNegativeUnit:
      ok = nppiAddC_32f_C3R(static_cast<Npp32f*>(src_image.element_wise_begin()), src_step, bias,
                            static_cast<Npp32f*>(src_image.element_wise_begin()), src_step, roi);
      ASSERT(ok == NPP_SUCCESS, "nppiConvert_32f8u_C3R Failed: %d", ok);
      ok = nppiMulC_32f_C3R(static_cast<Npp32f*>(src_image.element_wise_begin()), src_step, factor,
                            static_cast<Npp32f*>(src_image.element_wise_begin()), src_step, roi);
      ASSERT(ok == NPP_SUCCESS, "nppiMulC_32f_C3R Failed for kPositiveNegative: %d", ok);
      break;
    case ImageNormalizationMode::kZeroUnit:
      ok = nppiMulC_32f_C3R(static_cast<Npp32f*>(src_image.element_wise_begin()), src_step,
                            zerounit_factor, static_cast<Npp32f*>(src_image.element_wise_begin()),
                            src_step, roi);
      ASSERT(ok == NPP_SUCCESS, "nppiMulC_32f_C3R Failed for kZeroUnit: %d", ok);
      break;
    case ImageNormalizationMode::kCast:
      break;
    default:
      PANIC("Unsupported Mode.");
  }
  ok = nppiConvert_32f8u_C3R(static_cast<Npp32f*>(src_image.element_wise_begin()), src_step,
                             static_cast<Npp8u*>(dst_image.element_wise_begin()), dst_step, roi,
                             NPP_RND_NEAR);
  ASSERT(ok == NPP_SUCCESS, "nppiConvert_32f8u_C3R Failed: %d", ok);
}

template <>
void CropImageToRoi(const CudaImageConstView1ub src_image, const RegionOfInterest& src_roi,
                    const RegionOfInterest& dst_roi, CudaImageView1ub dst_image) {
  const int src_step = static_cast<int>(src_image.getStride());
  const int dst_step = static_cast<int>(dst_image.getStride());
  NppStatus ok = NPP_SUCCESS;
  const NppiRect src_npp_rect = RoiToRect(src_roi);
  const NppiRect dst_npp_rect = RoiToRect(dst_roi);
  ok = nppiResize_8u_C1R(static_cast<const Npp8u*>(src_image.data().begin()), src_step,
                         ImageNppSize(src_image), src_npp_rect,
                         static_cast<Npp8u*>(dst_image.data().begin()), dst_step,
                         ImageNppSize(dst_image), dst_npp_rect, NPPI_INTER_NN);
  ASSERT(ok == NPP_SUCCESS, "nppiResize_8u_C1R Failed: %d", ok);
}

template <>
void CropImageToRoi(const CudaImageConstView3ub src_image, const RegionOfInterest& src_roi,
                    const RegionOfInterest& dst_roi, CudaImageView3ub dst_image) {
  const int src_step = static_cast<int>(src_image.getStride());
  const int dst_step = static_cast<int>(dst_image.getStride());
  NppStatus ok = NPP_SUCCESS;
  const NppiRect src_npp_rect = RoiToRect(src_roi);
  const NppiRect dst_npp_rect = RoiToRect(dst_roi);
  ok = nppiResize_8u_C3R(static_cast<const Npp8u*>(src_image.data().begin()), src_step,
                         ImageNppSize(src_image), src_npp_rect,
                         static_cast<Npp8u*>(dst_image.data().begin()), dst_step,
                         ImageNppSize(dst_image), dst_npp_rect, NPPI_INTER_NN);
  ASSERT(ok == NPP_SUCCESS, "nppiResize_8u_C3R Failed: %d", ok);
}

template void CropImageToRoi(const CudaImageConstView1ub src_image, const RegionOfInterest& src_roi,
                             const RegionOfInterest& dst_roi, CudaImageView1ub dst_image);
template void CropImageToRoi(const CudaImageConstView3ub src_image, const RegionOfInterest& src_roi,
                             const RegionOfInterest& dst_roi, CudaImageView3ub dst_image);

template <typename K, int N>
void ResizeWithAspectRatio(const CudaImageConstView<K, N> src_image,
                           CudaImageView<K, N> dst_image) {
  // Compute the aspect ratio
  size_t resized_rows, resized_cols;

  if (dst_image.rows() * src_image.cols() <= dst_image.cols() * src_image.rows()) {
    resized_rows = dst_image.rows();
    resized_cols = src_image.cols() * dst_image.rows() / src_image.rows();
  } else {
    resized_rows = src_image.rows() * dst_image.cols() / src_image.cols();
    resized_cols = dst_image.cols();
  }

  const size_t row_padding = (dst_image.rows() - resized_rows) / 2;
  const size_t col_padding = (dst_image.cols() - resized_cols) / 2;

  // Fill the image with zeroes
  FillElementsWithZero(dst_image);

  // Resize with aspect ratio
  RegionOfInterest dst_roi{{row_padding, col_padding}, {resized_rows, resized_cols}};
  ResizeImageToRoi(src_image, dst_roi, dst_image);
}

template void ResizeWithAspectRatio(const CudaImageConstView1ub src_image,
                                    CudaImageView1ub dst_image);
template void ResizeWithAspectRatio(const CudaImageConstView3ub src_image,
                                    CudaImageView3ub dst_image);

template <>
void ConvImageFilter(const CudaContinuousImageConstView1f filter_image, const Vector2i& anchor,
                     const CudaImageConstView1f src_image, CudaImageView1f dst_image) {
  const int dst_rows = static_cast<int>(dst_image.rows());
  const int dst_cols = static_cast<int>(dst_image.cols());
  const int filter_rows = static_cast<int>(filter_image.rows());
  const int filter_cols = static_cast<int>(filter_image.cols());
  ASSERT(anchor.x() < filter_rows && anchor.y() < filter_cols, "Invalid Anchor value.");

  NppStatus ok =
      nppiFilter_32f_C1R(static_cast<const Npp32f*>(src_image.element_wise_begin()),     // pSrc
                         src_image.getStride(),                                          // nSrcStep
                         static_cast<Npp32f*>(dst_image.element_wise_begin()),           // pDst
                         dst_image.getStride(),                                          // nDstStep
                         NppiSize{dst_cols, dst_rows},                                   // oSizeROI
                         static_cast<const Npp32f*>(filter_image.element_wise_begin()),  // pKernel
                         NppiSize{filter_cols, filter_rows},  // oKernelSize
                         NppiPoint{anchor.y(), anchor.x()});  // oAnchor
  ASSERT(ok == NPP_SUCCESS, "nppiFilter32f_32s_C1R Failed %d", static_cast<int>(ok));
}

template void ConvImageFilter(const CudaContinuousImageConstView1f filter_image,
                              const Vector2i& anchor, const CudaImageConstView1f src_image,
                              CudaImageView1f dst_image);

void NormalizeImage(const CudaImageConstView3ub& src_image, const ImageNormalizationMode mode,
                    CudaImageView3f dst_image) {
  ASSERT(src_image.rows() == dst_image.rows() && src_image.cols() == dst_image.cols() &&
             src_image.channels() == dst_image.channels(),
         "Mismatched input-output dimensions.");

  Npp32f bias[3] = {1.0f, 1.0f, 1.0f};
  Npp32f factor[3] = {.5f * 255.0f, .5f * 255.0f, .5f * 255.0f};
  Npp32f zerounit_factor[3] = {255.0f, 255.0f, 255.0f};

  NppiSize roi = ImageNppSize(src_image);
  const int src_step = static_cast<int>(src_image.getStride());
  const int dst_step = static_cast<int>(dst_image.getStride());
  NppStatus ok =
      nppiConvert_8u32f_C3R(static_cast<const Npp8u*>(src_image.element_wise_begin()),  // pSrc
                            src_step,                                                   // nSrcStep
                            static_cast<Npp32f*>(dst_image.element_wise_begin()),       // pDst
                            dst_step,                                                   // nDstStep
                            roi);                                                       // oSizeROI
  ASSERT(ok == NPP_SUCCESS, "nppiConvert_8u32f_C3R Failed %d", static_cast<int>(ok));

  switch (mode) {
    case ImageNormalizationMode::kPositiveNegativeUnit:
      ok = nppiDivC_32f_C3R(static_cast<Npp32f*>(dst_image.element_wise_begin()), dst_step, factor,
                            static_cast<Npp32f*>(dst_image.element_wise_begin()), dst_step, roi);
      ASSERT(ok == NPP_SUCCESS, "nppiDivC_32f_C3R for positive_negative Failed: %d", ok);
      ok = nppiSubC_32f_C3R(static_cast<Npp32f*>(dst_image.element_wise_begin()), dst_step, bias,
                            static_cast<Npp32f*>(dst_image.element_wise_begin()), dst_step, roi);
      ASSERT(ok == NPP_SUCCESS, "nppiSubC_32f_C3R Failed: %d", ok);
      break;
    case ImageNormalizationMode::kZeroUnit:
      ok = nppiDivC_32f_C3R(static_cast<Npp32f*>(dst_image.element_wise_begin()), dst_step,
                            zerounit_factor, static_cast<Npp32f*>(dst_image.element_wise_begin()),
                            dst_step, roi);
      ASSERT(ok == NPP_SUCCESS, "nppiDivC_32f_C3R for zero_unit Failed: %d", ok);
      break;
    case ImageNormalizationMode::kCast:
      // falls through as nothing to do
      break;
    default:
      PANIC("Unknown mode: %d", mode);
  }
}
}  // namespace isaac
