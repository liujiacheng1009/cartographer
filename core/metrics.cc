// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/core/metrics.h"

#include "glog/logging.h"

namespace cartographer {
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
