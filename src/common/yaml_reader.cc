// Copyright (c) 2020 NRSL HITsz. All rights reserved.
// Author: lisilin013@163.com(Silin Li) on 20-9-3.

#include "common/yaml_reader.h"

namespace common {

YamlReader::YamlReader(const std::string &config_file) {
  CHECK(boost::filesystem::exists(config_file));
  node_ = YAML::LoadFile(config_file);
}

YamlReader::YamlReader(const YAML::Node &node) : node_(std::move(node)) {}

} // namespace common