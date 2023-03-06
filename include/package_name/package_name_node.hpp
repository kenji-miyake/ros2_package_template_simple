// SPDX-License-Identifier: Apache-2.0
// Copyright 2021 Kenji Miyake
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PACKAGE_NAME__PACKAGE_NAME_NODE_HPP__
#define PACKAGE_NAME__PACKAGE_NAME_NODE_HPP__

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "example_interfaces/msg/int32.hpp"
#include "package_name_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace package_name
{
class PackageNameNode : public rclcpp::Node
{
public:
  explicit PackageNameNode(const rclcpp::NodeOptions & node_options);

private:
  void onTimer();

  rclcpp::TimerBase::SharedPtr timer_{};
  std::shared_ptr<package_name::ParamListener> param_listener_;
};

}  // namespace package_name

#endif  // PACKAGE_NAME__PACKAGE_NAME_NODE_HPP__
