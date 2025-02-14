// Copyright 2025 PAL Robotics, S.L.
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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include  "remap_map_handler/frustum_fov.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("frustum_debug");

  auto tf_buffer =
    std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener =
    std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  double h_fov = 0.2;
  double v_fov = 0.2;
  double d_min = 0.5;
  double d_max = 5.0;

  tf2::Vector3 axis(0.0, 0.0, 1.0);

  remap::map_handler::fov::FrustumFOV frustum(tf_buffer,
    h_fov, v_fov, d_min, d_max,
    axis);
}
