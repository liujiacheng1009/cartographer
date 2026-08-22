// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#ifndef CARTOGRAPHER_CORE_METRICS_H_
#define CARTOGRAPHER_CORE_METRICS_H_

#include <map>
#include <string>
#include <vector>

namespace cartographer {
namespace common {

// Compact diagnostic histogram used for human-readable timing summaries.
class Histogram {
 public:
  void Add(float value);
  std::string ToString(int buckets) const;

 private:
  std::vector<float> values_;
};

}  // namespace common
namespace metrics {

class Counter {
 public:
  static Counter* Null();
  virtual ~Counter() = default;
  virtual void Increment() = 0;
  virtual void Increment(double by_value) = 0;
};

class Gauge {
 public:
  static Gauge* Null();
  virtual ~Gauge() = default;
  virtual void Increment() = 0;
  virtual void Increment(double by_value) = 0;
  virtual void Decrement() = 0;
  virtual void Decrement(double by_value) = 0;
  virtual void Set(double value) = 0;
};

class Histogram {
 public:
  using BucketBoundaries = std::vector<double>;
  static Histogram* Null();
  static BucketBoundaries FixedWidth(double width, int num_finite_buckets);
  static BucketBoundaries ScaledPowersOf(double base, double scale_factor,
                                         double max_value);
  virtual ~Histogram() = default;
  virtual void Observe(double value) = 0;
};

template <typename MetricType>
class NullFamily;

template <typename MetricType>
class Family {
 public:
  static Family<MetricType>* Null() {
    static NullFamily<MetricType> null_family;
    return &null_family;
  }
  virtual ~Family() = default;
  virtual MetricType* Add(const std::map<std::string, std::string>& labels) = 0;
};

template <typename MetricType>
class NullFamily : public Family<MetricType> {
 public:
  MetricType* Add(const std::map<std::string, std::string>&) override {
    return MetricType::Null();
  }
};

class FamilyFactory {
 public:
  virtual ~FamilyFactory() = default;
  virtual Family<Counter>* NewCounterFamily(const std::string& name,
                                            const std::string& description) = 0;
  virtual Family<Gauge>* NewGaugeFamily(const std::string& name,
                                        const std::string& description) = 0;
  virtual Family<Histogram>* NewHistogramFamily(
      const std::string& name, const std::string& description,
      const Histogram::BucketBoundaries& boundaries) = 0;
};

}  // namespace metrics
}  // namespace cartographer

#endif  // CARTOGRAPHER_CORE_METRICS_H_
