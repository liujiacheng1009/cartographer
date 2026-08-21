// Copyright 2026 The SweepNav Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef CARTOGRAPHER_COMMON_PARAMETER_DICTIONARY_H_
#define CARTOGRAPHER_COMMON_PARAMETER_DICTIONARY_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "glog/logging.h"

namespace cartographer {
namespace common {

// Strict, typed view over a YAML mapping. Every mapping key must be consumed
// exactly once. This keeps misspelled and obsolete parameters from silently
// changing SLAM behavior.
class ParameterDictionary {
 public:
  static std::unique_ptr<ParameterDictionary> LoadFile(
      const std::string& filename);

  ParameterDictionary(const ParameterDictionary&) = delete;
  ParameterDictionary& operator=(const ParameterDictionary&) = delete;
  ~ParameterDictionary();

  std::vector<std::string> GetKeys() const;
  bool HasKey(const std::string& key) const;
  std::string GetString(const std::string& key);
  double GetDouble(const std::string& key);
  int GetInt(const std::string& key);
  bool GetBool(const std::string& key);
  int GetNonNegativeInt(const std::string& key);
  std::unique_ptr<ParameterDictionary> GetDictionary(const std::string& key);

  std::vector<double> GetArrayValuesAsDoubles();
  std::vector<std::string> GetArrayValuesAsStrings();
  std::vector<std::unique_ptr<ParameterDictionary>>
  GetArrayValuesAsDictionaries();

  std::string ToString() const;

 private:
  enum class CheckUsage { YES, NO };
  ParameterDictionary(YAML::Node node, std::string path,
                      CheckUsage check_usage);

  YAML::Node GetNode(const std::string& key);
  void CheckAllKeysWereUsedExactlyOnce() const;

  YAML::Node node_;
  const std::string path_;
  const CheckUsage check_usage_;
  std::map<std::string, int> reference_counts_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_PARAMETER_DICTIONARY_H_
