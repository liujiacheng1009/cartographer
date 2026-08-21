// Copyright 2026 The SweepNav Authors
// Licensed under the Apache License, Version 2.0 (the "License").

#include "cartographer/core/parameter_dictionary.h"

#include <cmath>
#include <cstdlib>
#include <exception>
#include <utility>

#include "glog/logging.h"

namespace cartographer {
namespace common {
namespace {

template <typename T>
T Convert(const YAML::Node& node, const std::string& path) {
  try {
    return node.as<T>();
  } catch (const std::exception& error) {
    LOG(FATAL) << "Invalid YAML parameter '" << path << "': " << error.what();
    std::abort();
  }
}

}  // namespace

std::unique_ptr<ParameterDictionary> ParameterDictionary::LoadFile(
    const std::string& filename) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(filename);
  } catch (const std::exception& error) {
    LOG(FATAL) << "Failed to load YAML configuration '" << filename
               << "': " << error.what();
    std::abort();
  }
  CHECK(root.IsMap()) << "YAML configuration root must be a mapping: "
                      << filename;
  return std::unique_ptr<ParameterDictionary>(new ParameterDictionary(
      std::move(root), filename, CheckUsage::YES));
}

ParameterDictionary::ParameterDictionary(YAML::Node node, std::string path,
                                         CheckUsage check_usage)
    : node_(std::move(node)),
      path_(std::move(path)),
      check_usage_(check_usage) {
  CHECK(node_.IsMap() || node_.IsSequence())
      << "YAML parameter must be a mapping or sequence: " << path_;
}

ParameterDictionary::~ParameterDictionary() {
  if (check_usage_ == CheckUsage::YES) {
    CheckAllKeysWereUsedExactlyOnce();
  }
}

std::vector<std::string> ParameterDictionary::GetKeys() const {
  CHECK(node_.IsMap()) << "Expected mapping at " << path_;
  std::vector<std::string> keys;
  for (const auto& entry : node_) {
    keys.push_back(Convert<std::string>(entry.first, path_ + ".<key>"));
  }
  return keys;
}

bool ParameterDictionary::HasKey(const std::string& key) const {
  CHECK(node_.IsMap()) << "Expected mapping at " << path_;
  return node_[key].IsDefined();
}

YAML::Node ParameterDictionary::GetNode(const std::string& key) {
  CHECK(node_.IsMap()) << "Expected mapping at " << path_;
  const YAML::Node value = node_[key];
  CHECK(value.IsDefined()) << "Missing YAML parameter '" << path_ << "."
                           << key << "'.";
  ++reference_counts_[key];
  CHECK_EQ(reference_counts_[key], 1)
      << "YAML parameter read more than once: " << path_ << "." << key;
  return value;
}

std::string ParameterDictionary::GetString(const std::string& key) {
  return Convert<std::string>(GetNode(key), path_ + "." + key);
}

double ParameterDictionary::GetDouble(const std::string& key) {
  const double value = Convert<double>(GetNode(key), path_ + "." + key);
  CHECK(std::isfinite(value)) << "Non-finite YAML parameter: " << path_ << "."
                              << key;
  return value;
}

int ParameterDictionary::GetInt(const std::string& key) {
  return Convert<int>(GetNode(key), path_ + "." + key);
}

bool ParameterDictionary::GetBool(const std::string& key) {
  return Convert<bool>(GetNode(key), path_ + "." + key);
}

int ParameterDictionary::GetNonNegativeInt(const std::string& key) {
  const int value = GetInt(key);
  CHECK_GE(value, 0) << "Negative YAML parameter: " << path_ << "." << key;
  return value;
}

std::unique_ptr<ParameterDictionary> ParameterDictionary::GetDictionary(
    const std::string& key) {
  YAML::Node child = GetNode(key);
  CHECK(child.IsMap()) << "Expected YAML mapping at " << path_ << "." << key;
  return std::unique_ptr<ParameterDictionary>(new ParameterDictionary(
      std::move(child), path_ + "." + key, CheckUsage::YES));
}

std::vector<double> ParameterDictionary::GetArrayValuesAsDoubles() {
  CHECK(node_.IsSequence()) << "Expected YAML sequence at " << path_;
  std::vector<double> values;
  for (std::size_t i = 0; i < node_.size(); ++i) {
    const double value = Convert<double>(node_[i], path_ + "[" +
                                                      std::to_string(i) + "]");
    CHECK(std::isfinite(value)) << "Non-finite YAML value at " << path_;
    values.push_back(value);
  }
  return values;
}

std::vector<std::string> ParameterDictionary::GetArrayValuesAsStrings() {
  CHECK(node_.IsSequence()) << "Expected YAML sequence at " << path_;
  std::vector<std::string> values;
  for (std::size_t i = 0; i < node_.size(); ++i) {
    values.push_back(Convert<std::string>(
        node_[i], path_ + "[" + std::to_string(i) + "]"));
  }
  return values;
}

std::vector<std::unique_ptr<ParameterDictionary>>
ParameterDictionary::GetArrayValuesAsDictionaries() {
  CHECK(node_.IsSequence()) << "Expected YAML sequence at " << path_;
  std::vector<std::unique_ptr<ParameterDictionary>> values;
  for (std::size_t i = 0; i < node_.size(); ++i) {
    CHECK(node_[i].IsMap()) << "Expected YAML mapping at " << path_ << "[" << i
                            << "]";
    values.emplace_back(new ParameterDictionary(
        node_[i], path_ + "[" + std::to_string(i) + "]", CheckUsage::YES));
  }
  return values;
}

std::string ParameterDictionary::ToString() const { return YAML::Dump(node_); }

void ParameterDictionary::CheckAllKeysWereUsedExactlyOnce() const {
  if (!node_.IsMap()) return;
  for (const auto& key : GetKeys()) {
    const auto found = reference_counts_.find(key);
    CHECK(found != reference_counts_.end())
        << "Unused or unknown YAML parameter '" << path_ << "." << key << "'.";
    CHECK_EQ(found->second, 1)
        << "YAML parameter used the wrong number of times: " << path_ << "."
        << key;
  }
}

}  // namespace common
}  // namespace cartographer
