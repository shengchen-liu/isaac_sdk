/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include "engine/core/image/image.hpp"
#include "engine/core/math/types.hpp"
#include "npp.h"  // NOLINT

namespace isaac {

// Gets the size of an image for NPP
template <typename K, int N, typename Container>
NppiSize ImageNppSize(const ImageBase<K, N, Container>& image) {
  return NppiSize{static_cast<int>(image.cols()), static_cast<int>(image.rows())};
}

// Gets the full ROI of an image for NPP
template <typename K, int N, typename Container>
NppiRect ImageNppiRect(const ImageBase<K, N, Container>& image) {
  return NppiRect{0, 0, static_cast<int>(image.cols()), static_cast<int>(image.rows())};
}

// Specifies the region of interest in image operation.
struct RegionOfInterest {
  // (row, col) top-left corner coordinate in pixels of the region (included).
  Vector2i offset;
  // (rows, cols) specifying the size of the region in pixels.
  Vector2i size;
};

// Writes 0 to all pixels.
template <typename K, int N>
void FillElementsWithZero(CudaImageView<K, N> dst_img);

// Supported modes of converting between float image and uint8_t image.
enum class ImageNormalizationMode {
  kCast = 1,                 // Casted to float value
  kZeroUnit = 2,             // Normalized intensity ranges between [0, 1]
  kPositiveNegativeUnit = 3  // Normalized intensity ranges between [-1, 1]
};

// Converts float 3-channel image to [0,255] uint8_t 3-channel image.
void DenormalizeImage(CudaImageView3f src_image, ImageNormalizationMode mode,
                      CudaImageView3ub dst_image);

// Grabs specified piece of input cuda image and resizes it to specified roi in output cuda image
template <typename K, int N>
void CropImageToRoi(const CudaImageConstView<K, N> src_image, const RegionOfInterest& src_roi,
                    const RegionOfInterest& dst_roi, CudaImageView<K, N> dst_image);

// Resizes input cuda image to output cuda image with specified offset and output size.
template <typename K, int N>
void ResizeImageToRoi(const CudaImageConstView<K, N> src_image, const RegionOfInterest& dst_roi,
                      CudaImageView<K, N> dst_image) {
  const int rows = static_cast<int>(src_image.rows());
  const int cols = static_cast<int>(src_image.cols());
  RegionOfInterest src_roi{Vector2i::Zero(), {rows, cols}};
  CropImageToRoi(src_image, src_roi, dst_roi, dst_image);
}

// Resizes input cuda image to output cuda image with output size while keeping aspect ratio.
template <typename K, int N>
void ResizeWithAspectRatio(const CudaImageConstView<K, N> src_image, CudaImageView<K, N> dst_image);

// Convolutes (dst_pixel = \Sigma_{neighbour_pixels}{filter_pixel * src_pixel}) source image with
// small filter image (rotated 180-degree) and writes result to another image. The filter
// image has to be stored in CudaContinuousImage which has trivial row stride. Pixels that makes
// filter go out of source image domain shall be ignored as they may not have consistent behavior.
template <typename K>
void ConvImageFilter(const CudaContinuousImageConstView<K, 1> filter_image, const Vector2i& anchor,
                     const CudaImageConstView<K, 1> src_image, CudaImageView<K, 1> dst_image);

// Converts normal rgb image to float image of [0,255], [0,1] or [-1,1] range according to
// normalization mode.
void NormalizeImage(const CudaImageConstView3ub& rgb_image, const ImageNormalizationMode mode,
                    CudaImageView3f rgb_cudaf_image);

}  // namespace isaac
