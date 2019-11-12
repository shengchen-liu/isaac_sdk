/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <algorithm>
#include <array>
#include <random>

namespace isaac {
namespace math {

// Helper class to track performance counters. FastRunningMedian samples observations
// with increased spacing to calculate approximate median and a high (90th) percentile
// This function also keeps track of max, min summary statistics
// Usage:
//  .add(new_observation)
//  .add(new_observation)
//  ...
//  .median()
template <typename K, size_t kSampleSize = 16>
class FastRunningMedian {
 public:
  FastRunningMedian()
  : observed_max_(K(0)), observed_min_(K(0)),
    counter_(0), next_sample_index_(0), index_(0) {}

  // Adds a new measurement, updates the observed statistics
  void add(K value) {
    // fast path starts
    if (value > observed_max_) observed_max_ = value;
    if (value < observed_min_) observed_min_ = value;
    if (++counter_ < next_sample_index_) return;  // fast path ends

    // NOTE: Code beyond this point is executed (assuming kSampleSize = 16):
    // ~16 times first 16 samples
    // ~40 times first 100 samples
    // ~70 times first 1,000 samples
    // ~200 times first 1,000,000,000 samples

    // estimate the next sample index to update sample_data_ array
    const size_t deterministic_spacing = counter_ / sample_data_.size();
    const size_t sample_spacing = deterministic_spacing
                                + uniformRandom(0, deterministic_spacing);
    next_sample_index_ = counter_ + sample_spacing;

    // only need to execute once
    if (counter_ == 1) {
      observed_max_ = value;
      observed_min_ = value;
    }

    // update the sample_data_ array sequentially with approximate spacing
    sample_data_[index_] = value;
    index_ = (index_ + 1) % sample_data_.size();
  }

  // Returns the median value approximation
  K median() const {
    return percentile(0.5);
  }

  // Returns the observed max value
  K max() const {return observed_max_;}

  // Returns the observed min value
  K min() const {return observed_min_;}

  // Returns the approximate percentile. For example `percentile = 1` would return the max. The
  // percentile is computed based on the collected samples.
  K percentile(double percentile) const {
    // no observations were recorded
    if (counter_ == 0) return K(0);

    // Sort the recorded samples and look up the element in the array which corresponds to the
    // desired percentile. Note that we are creating a copy of samples to not mess with the order
    // of samples.
    auto samples_clone = sample_data_;
    const int sample_size = std::min(samples_clone.size(), counter_);
    const int index = std::floor(percentile * static_cast<double>(sample_size - 1));
    const auto it = samples_clone.begin() + std::min(std::max(0, index), sample_size - 1);
    std::nth_element(samples_clone.begin(), it, samples_clone.begin() + sample_size);
    return *it;
  }

 private:
  // generate random number uniformally from range [min, max]
  int uniformRandom(int min, int max) {
    if (min == max) return min;
    std::uniform_int_distribution<int> distribution(min, max);
    return distribution(random_number_generator);
  }

  // Keep track of current min, max
  K observed_max_;
  K observed_min_;
  // Keep track of the number of samples so far
  size_t counter_;
  // The index of the next sample to store
  size_t next_sample_index_;
  // Index of sample_data_ to replace
  size_t index_;

  // Array of samples of execution time to estimate median
  std::array<K, kSampleSize> sample_data_;
  std::default_random_engine random_number_generator;
};

}  // namespace math
}  // namespace isaac
