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

#ifndef REMAP_MAP_HANDLER__FOV_HPP_
#define REMAP_MAP_HANDLER__FOV_HPP_

#include <openvdb/openvdb.h>

#include <tf2_ros/buffer.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>

namespace remap
{
namespace map_handler
{
namespace fov
{
class FOV
{
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::string ref_frame_;

public:
  explicit FOV(std::shared_ptr<tf2_ros::Buffer> tf_buffer)
  : tf_buffer_(tf_buffer) {}

  virtual bool inFovPoint(const geometry_msgs::msg::PointStamped & point) = 0;

  void setReferenceFrame(const std::string & ref_frame)
  {
    ref_frame_ = ref_frame;
  }
};
}  // namespace fov
}  // namespace map_handler
}  // namespace remap
#endif  // REMAP_MAP_HANDLER__FOV_HPP_
