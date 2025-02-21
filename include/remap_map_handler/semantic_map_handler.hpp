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

#ifndef REMAP_MAP_HANDLER__SEMANTIC_MAP_HANDLER_HPP_
#define REMAP_MAP_HANDLER__SEMANTIC_MAP_HANDLER_HPP_

#include <openvdb/openvdb.h>
#include <openvdb/tree/ValueAccessor.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/Statistics.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/blocked_range2d.h>
#include <tbb/blocked_range3d.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <iostream>
#include <cmath>
#include <type_traits>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <limits>
#include <vector>
#include <map>
#include <string>
#include <memory>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "remap_regions_register/regions_register.hpp"

namespace remap
{
namespace map_handler
{
class SemanticMapHandler
{
private:
  std::shared_ptr<openvdb::Int32Grid> grid_;

  bool threaded_;

  float voxel_size_;
  bool vertex_centered_;
  openvdb::math::Vec3d offset_;
  openvdb::math::Transform::Ptr initial_transformation_;

  std::string fixed_frame_;
  std::string map_frame_;  // If these two are the same, then no rotation applied to the map

  using GridAccessorType = typename openvdb::Int32Grid::Accessor;

  static float computeDistance(const openvdb::Vec3d & a, const openvdb::Vec3d & b)
  {
    return std::sqrt(
      std::pow(
        a[0] - b[0],
        2) + std::pow(a[1] - b[1], 2) + std::pow(a[2] - b[2], 2));
  }

  int getAreaId(
    const GridAccessorType & accessor,
    const openvdb::Coord & ijk,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void setVoxelId(
    GridAccessorType & accessor,
    const int & idx_i,
    const int & idx_j,
    const int & idx_k,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void boxSemanticCore(
    openvdb::Int32Tree & tree,
    const int & idx_i,
    const openvdb::Coord & ijk_min,
    const openvdb::Coord & ijk_max,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void sphereSemanticCore(
    openvdb::Int32Tree & tree,
    const int & idx_i,
    const openvdb::Coord & ijk_min,
    const openvdb::Coord & ijk_max,
    const float & radius,
    const openvdb::Vec3d & centre,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void coneSemanticCore(
    openvdb::Int32Tree & tree,
    const int & i,
    int & j,
    int & k,
    const double & theta,
    const openvdb::Vec3d & idx_space_origin,
    const openvdb::math::Quatd & rotation_quat,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void pyramidSemanticCore(
    openvdb::Int32Tree & tree,
    const int & i,
    int & j,
    int & k,
    const double & theta_h,
    const double & theta_v,
    const openvdb::Vec3d & idx_space_origin,
    const openvdb::math::Quatd & rotation_quat,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void performIteration(
    tbb::enumerable_thread_specific<openvdb::Int32Tree> & tbb_thread_pool,
    const tbb::blocked_range<int> & tbb_iteration_range,
    std::function<void(const tbb::blocked_range<int> &)> kernel);

public:
  SemanticMapHandler(
    const bool & threaded,
    const float & voxel_size,
    const bool & vertex_centered = true);

  SemanticMapHandler(
    std::shared_ptr<openvdb::Int32Grid> grid,
    const bool & threaded,
    const float & voxel_size,
    const bool & vertex_centered = true);

  virtual ~SemanticMapHandler();

  void insertSemanticSphere(
    const float & radius,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & centre = openvdb::Vec3d(0.0, 0.0, 0.0));

  void insertSemanticPoints(
    std::vector<pcl::PointXYZ> points,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void insertSemanticBox(
    const float & amplitude_h,
    const float & amplitude_v,
    const float & length,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & centre = openvdb::Vec3d(0.0, 0.0, 0.0));

  void insertSemanticCone(
    const float & amplitude,
    const float & length,
    const openvdb::Vec3d & direction,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & origin = openvdb::Vec3d(0.0, 0.0, 0.0));

  void insertSemanticPyramid(
    const float & amplitude_h,
    const float & amplitude_v,
    const float & length,
    const openvdb::Vec3d & direction,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & origin = openvdb::Vec3d(0.0, 0.0, 0.0));

  bool removeRegion(
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);

  void clear();

  void setFixedFrame(const std::string fixed_frame);  // TODO(lorenzoferrini) implement
  void setMapFrame(const std::string map_frame);  // TODO(lorenzoferrini) implement
  void rotateMap();  // TODO(lorenzoferrini) implement
  void flushFoV();  // TODO(lorenzoferrini) implement

  std::shared_ptr<openvdb::Int32Grid> getGridPtr();
};
}  // namespace map_handler
}  // namespace remap
#endif  // REMAP_MAP_HANDLER__SEMANTIC_MAP_HANDLER_HPP_
