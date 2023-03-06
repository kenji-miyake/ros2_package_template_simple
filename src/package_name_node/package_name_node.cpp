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

#include "package_name/package_name_node.hpp"

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using namespace std::placeholders;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;

namespace package_name
{
PackageNameNode::PackageNameNode(const rclcpp::NodeOptions & node_options)
: Node("package_name", node_options)
{
  param_listener_ = std::make_shared<package_name::ParamListener>(get_node_parameters_interface());
  const auto params = param_listener_->get_params();
  RCLCPP_INFO(get_logger(), "update_rate_hz: %f", params.update_rate_hz);

  // Timer
  const auto update_period_ns = rclcpp::Rate(params.update_rate_hz).period();
  timer_ = create_wall_timer(update_period_ns, std::bind(&PackageNameNode::onTimer, this));
}

void PackageNameNode::onTimer()
{
  const auto params = param_listener_->get_params();

  const auto color = params.background;
  RCLCPP_INFO(get_logger(), "Background color (r,g,b): %ld, %ld, %ld", color.r, color.g, color.b);
}

}  // namespace package_name

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(package_name::PackageNameNode)
