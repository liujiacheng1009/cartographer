// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/foundation/runtime_stats.h"

#include <algorithm>
#include <numeric>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

void Histogram::Add(const float value) { values_.push_back(value); }

std::string Histogram::ToString(const int buckets) const {
  CHECK_GE(buckets, 1);
  if (values_.empty()) return "Count: 0";
  const float min = *std::min_element(values_.begin(), values_.end());
  const float max = *std::max_element(values_.begin(), values_.end());
  const float mean =
      std::accumulate(values_.begin(), values_.end(), 0.f) / values_.size();
  std::string result = absl::StrCat("Count: ", values_.size(), "  Min: ", min,
                                    "  Max: ", max, "  Mean: ", mean);
  if (min == max) return result;
  float lower_bound = min;
  int total_count = 0;
  for (int i = 0; i != buckets; ++i) {
    const float upper_bound =
        i + 1 == buckets
            ? max
            : (max * (i + 1) / buckets + min * (buckets - i - 1) / buckets);
    int count = 0;
    for (const float value : values_) {
      if (lower_bound <= value &&
          (i + 1 == buckets ? value <= upper_bound : value < upper_bound)) {
        ++count;
      }
    }
    total_count += count;
    absl::StrAppendFormat(&result, "\n[%f, %f%c", lower_bound, upper_bound,
                          i + 1 == buckets ? ']' : ')');
    constexpr int kMaxBarChars = 20;
    const int bar =
        (count * kMaxBarChars + values_.size() / 2) / values_.size();
    result += "\t";
    for (int j = 0; j != kMaxBarChars; ++j) {
      result += j < kMaxBarChars - bar ? " " : "#";
    }
    absl::StrAppend(&result, "\tCount: ", count, " (",
                    count * 1e2f / values_.size(), "%)", "\tTotal: ",
                    total_count, " (", total_count * 1e2f / values_.size(),
                    "%)");
    lower_bound = upper_bound;
  }
  return result;
}

}  // namespace common
namespace metrics {
namespace {

class NullCounter : public Counter {
 public:
  void Increment() override {}
  void Increment(double) override {}
};

class NullGauge : public Gauge {
 public:
  void Increment() override {}
  void Increment(double) override {}
  void Decrement() override {}
  void Decrement(double) override {}
  void Set(double) override {}
};

class NullHistogram : public Histogram {
 public:
  void Observe(double) override {}
};

}  // namespace

Counter* Counter::Null() {
  static NullCounter null_counter;
  return &null_counter;
}

Gauge* Gauge::Null() {
  static NullGauge null_gauge;
  return &null_gauge;
}

Histogram* Histogram::Null() {
  static NullHistogram null_histogram;
  return &null_histogram;
}

Histogram::BucketBoundaries Histogram::FixedWidth(double width,
                                                  int num_finite_buckets) {
  BucketBoundaries result;
  double boundary = 0.;
  for (int i = 0; i < num_finite_buckets; ++i) {
    boundary += width;
    result.push_back(boundary);
  }
  return result;
}

Histogram::BucketBoundaries Histogram::ScaledPowersOf(
    double base, double scale_factor, double max_value) {
  CHECK_GT(base, 1.);
  CHECK_GT(scale_factor, 0.);
  BucketBoundaries result;
  for (double boundary = scale_factor; boundary < max_value;
       boundary *= base) {
    result.push_back(boundary);
  }
  return result;
}

}  // namespace metrics
}  // namespace cartographer
