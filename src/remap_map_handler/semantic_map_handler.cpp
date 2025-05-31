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

#include "remap_map_handler/semantic_map_handler.hpp"

namespace remap
{
namespace map_handler
{
SemanticMapHandler::SemanticMapHandler(
  const bool & threaded,
  const float & voxel_size,
  const bool & vertex_centered,
  const std::string & fixed_frame)
: threaded_(threaded),
  voxel_size_(voxel_size),
  vertex_centered_(vertex_centered),
  fixed_frame_(fixed_frame)
{
  grid_ = openvdb::Int32Grid::create();

  if (vertex_centered) {
    offset_ = openvdb::math::Vec3d(0.0, 0.0, 0.0);
  } else {
    offset_ = openvdb::math::Vec3d(voxel_size_ / 2.0, voxel_size_ / 2.0, voxel_size_ / 2.0);
  }

  initial_transformation_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
  initial_transformation_->postTranslate(offset_);

  grid_->setTransform(initial_transformation_);
}

SemanticMapHandler::SemanticMapHandler(
  std::shared_ptr<openvdb::Int32Grid> grid,
  const bool & threaded,
  const float & voxel_size,
  const bool & vertex_centered,
  const std::string & fixed_frame)
: grid_(grid),
  threaded_(threaded),
  voxel_size_(voxel_size),
  vertex_centered_(vertex_centered),
  fixed_frame_(fixed_frame)
{
  if (vertex_centered) {
    offset_ = openvdb::math::Vec3d(0.0, 0.0, 0.0);
  } else {
    offset_ = openvdb::math::Vec3d(voxel_size_ / 2.0, voxel_size_ / 2.0, voxel_size_ / 2.0);
  }
  initial_transformation_ = openvdb::math::Transform::createLinearTransform(voxel_size_);
  initial_transformation_->postTranslate(offset_);

  grid_->setTransform(initial_transformation_);
}

SemanticMapHandler::~SemanticMapHandler()
{
  grid_.reset();
}

void SemanticMapHandler::updateAreaBBox(
  const int & regId,
  const openvdb::Coord & ijk)
{
  if (areas_bbox_.find(regId) == areas_bbox_.end()) {
    areas_bbox_[regId] = openvdb::CoordBBox(ijk, ijk);
  } else {
    areas_bbox_[regId].expand(ijk);
  }
}

void SemanticMapHandler::updateEntityBBox(
  const std::string & entity_id,
  const openvdb::Coord & ijk)
{
  if (entities_bbox_.find(entity_id) == entities_bbox_.end()) {
    entities_bbox_[entity_id] = openvdb::CoordBBox(ijk, ijk);
  } else {
    entities_bbox_[entity_id].expand(ijk);
  }
}

void SemanticMapHandler::deleteRegionBBox(const int & regId)
{
  if (areas_bbox_.find(regId) != areas_bbox_.end()) {
    areas_bbox_.erase(regId);
  }
}

int SemanticMapHandler::getAreaId(
  const GridAccessorType & accessor,
  const openvdb::Coord & ijk,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  int id;
  if (!accessor.isValueOn(ijk)) {
    std::vector<std::string> reg_vec({reg});
    id = reg_register.findRegions(reg_vec);
    if (id == -1) {
      id = reg_register.addArea(reg_vec);
    }
  } else {
    id = accessor.getValue(ijk);
    auto regs = reg_register.findRegionsById(id);
    if (std::find(regs.begin(), regs.end(), reg) == regs.end()) {
      regs.push_back(reg);
      id = reg_register.findRegions(regs);
      if (id == -1) {
        id = reg_register.addArea(regs);
      }
    } else {
      // No need for intervention, as this specific region
      // is already mapped to this voxel
      return -2;
    }
  }
  return id;
}

void SemanticMapHandler::setVoxelId(
  GridAccessorType & accessor,
  const int & idx_i,
  const int & idx_j,
  const int & idx_k,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  openvdb::Coord ijk(idx_i, idx_j, idx_k);
  int id = getAreaId(
    accessor,
    ijk,
    reg,
    reg_register);
  if (id == -2) {
    return;
  }
  accessor.setValue(ijk, id);
  updateAreaBBox(id, ijk);
  updateEntityBBox(reg, ijk);
}

void SemanticMapHandler::boxSemanticCore(
  openvdb::Int32Tree & tree,
  const int & idx_i,
  const openvdb::Coord & ijk_min,
  const openvdb::Coord & ijk_max,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  GridAccessorType accessor(tree);

  for (int idx_j = ijk_min[1]; idx_j <= ijk_max[1]; ++idx_j) {
    for (int idx_k = ijk_min[2]; idx_k <= ijk_max[2]; ++idx_k) {
      setVoxelId(
        accessor,
        idx_i,
        idx_j,
        idx_k,
        reg,
        reg_register);
    }
  }
}

void SemanticMapHandler::sphereSemanticCore(
  openvdb::Int32Tree & tree,
  const int & idx_i,
  const openvdb::Coord & ijk_min,
  const openvdb::Coord & ijk_max,
  const float & radius,
  const openvdb::Vec3d & centre,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  GridAccessorType accessor(tree);

  for (int idx_j = ijk_min[1]; idx_j <= ijk_max[1]; ++idx_j) {
    for (int idx_k = ijk_min[2]; idx_k <= ijk_max[2]; ++idx_k) {
      openvdb::Coord ijk(idx_i, idx_j, idx_k);
      openvdb::Vec3d current_point = initial_transformation_->indexToWorld(ijk);
      float distance = computeDistance(centre, current_point);
      if (distance <= radius) {
        setVoxelId(
          accessor,
          idx_i,
          idx_j,
          idx_k,
          reg,
          reg_register);
      }
    }
  }
}

void SemanticMapHandler::coneSemanticCore(
  openvdb::Int32Tree & tree,
  const int & i,
  int & j,
  int & k,
  const double & theta,
  const openvdb::Vec3d & idx_space_origin,
  const openvdb::math::Quatd & rotation_quat,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  GridAccessorType accessor(tree);

  openvdb::Vec3d x = grid_->indexToWorld(openvdb::Coord(i, 0, 0));
  int j_start = -(static_cast<int>(std::tan(theta) * x.length() / voxel_size_));
  int k_start = -(static_cast<int>(std::tan(theta) * x.length() / voxel_size_));

  for (j = j_start; j <= -j_start; ++j) {
    for (k = k_start; k <= -k_start; ++k) {
      openvdb::Vec3d yz = grid_->indexToWorld(openvdb::Coord(0.0, j, k));
      float distance = computeDistance(openvdb::Vec3d(0.0, 0.0, 0.0), yz);
      if (distance <= (std::tan(theta) * x.length())) {
        openvdb::Vec3d current_point(i, j, k);
        current_point = rotation_quat.rotateVector(current_point);
        current_point = current_point + idx_space_origin;

        int idx_i = static_cast<int>(std::round(current_point[0]));
        int idx_j = static_cast<int>(std::round(current_point[1]));
        int idx_k = static_cast<int>(std::round(current_point[2]));

        setVoxelId(
          accessor,
          idx_i,
          idx_j,
          idx_k,
          reg,
          reg_register);
      }
    }
  }
}

void SemanticMapHandler::pyramidSemanticCore(
  openvdb::Int32Tree & tree,
  const int & i,
  int & j,
  int & k,
  const double & theta_h,
  const double & theta_v,
  const openvdb::Vec3d & idx_space_origin,
  const openvdb::math::Quatd & rotation_quat,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  GridAccessorType accessor(tree);

  openvdb::Vec3d x = grid_->indexToWorld(openvdb::Coord(i, 0, 0));
  int j_start = -(static_cast<int>(std::tan(theta_h) * x.length() / voxel_size_));
  int k_start = -(static_cast<int>(std::tan(theta_v) * x.length() / voxel_size_));

  for (j = j_start; j <= -j_start; ++j) {
    for (k = k_start; k <= -k_start; ++k) {
      openvdb::Vec3d y = grid_->indexToWorld(openvdb::Coord(0.0, j, 0.0));
      float y_distance = computeDistance(openvdb::Vec3d(0.0, 0.0, 0.0), y);

      openvdb::Vec3d z = grid_->indexToWorld(openvdb::Coord(0.0, 0.0, k));
      float z_distance = computeDistance(openvdb::Vec3d(0.0, 0.0, 0.0), z);

      if (y_distance <= (std::tan(theta_h) * x.length()) &&
        z_distance <= (std::tan(theta_v) * x.length()))
      {
        openvdb::Vec3d current_point(i, j, k);
        current_point = rotation_quat.rotateVector(current_point);
        current_point = current_point + idx_space_origin;

        int idx_i = static_cast<int>(std::round(current_point[0]));
        int idx_j = static_cast<int>(std::round(current_point[1]));
        int idx_k = static_cast<int>(std::round(current_point[2]));

        setVoxelId(
          accessor,
          idx_i,
          idx_j,
          idx_k,
          reg,
          reg_register);
      }
    }
  }
}

void SemanticMapHandler::performIteration(
  tbb::enumerable_thread_specific<openvdb::Int32Tree> & tbb_thread_pool,
  const tbb::blocked_range<int> & tbb_iteration_range,
  std::function<void(const tbb::blocked_range<int> &)> kernel)
{
  if (threaded_) {
    tbb::parallel_for(tbb_iteration_range, kernel);
    using RangeT =
      tbb::blocked_range<typename tbb::enumerable_thread_specific<openvdb::Int32Tree>::iterator>;
    struct Op
    {
      const bool mDelete;
      openvdb::Int32Tree * mTree;
      explicit Op(openvdb::Int32Tree & tree)
      : mDelete(false), mTree(&tree) {}
      Op(const Op & other, tbb::split)
      : mDelete(true), mTree(new openvdb::Int32Tree(other.mTree->background())) {}
      ~Op() {if (mDelete) {delete mTree;}}
      void operator()(const RangeT & r)
      {
        for (auto i = r.begin(); i != r.end(); ++i) {
          this->merge(*i);
        }
      }
      void join(Op & other) {this->merge(*(other.mTree));}
      void merge(openvdb::Int32Tree & tree) {mTree->merge(tree, openvdb::MERGE_ACTIVE_STATES);}
    } op(grid_->tree());
    tbb::parallel_reduce(RangeT(tbb_thread_pool.begin(), tbb_thread_pool.end()), op);
  } else {
    kernel(tbb_iteration_range);
  }
}

void SemanticMapHandler::insertSemanticSphere(
  const float & radius,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register,
  const openvdb::Vec3d & centre)
{
  if (radius <= 0.0) {
    std::cerr << "[AddSphere]: The radius cannot be less than or equal to zero! Aborting!" <<
      std::endl;
  } else {
    openvdb::Vec3d xyz_min_value_ws = centre - radius;
    openvdb::Vec3d xyz_min_value_is = grid_->worldToIndex(xyz_min_value_ws);
    openvdb::Coord ijk_min(static_cast<int>(xyz_min_value_is[0]),
      static_cast<int>(xyz_min_value_is[1]), static_cast<int>(xyz_min_value_is[2]));

    openvdb::Vec3d xyz_max_value = centre + radius;
    openvdb::Vec3d xyz_max_value_is = grid_->worldToIndex(xyz_max_value);
    openvdb::Coord ijk_max(static_cast<int>(xyz_max_value_is[0]),
      static_cast<int>(xyz_max_value_is[1]), static_cast<int>(xyz_max_value_is[2]));

    tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
    tbb::blocked_range<int> tbb_iteration_range(ijk_min[0], ijk_max[0], sizeof(ijk_min[0]));

    auto kernel = [&](const tbb::blocked_range<int> & iteration_range)
      {
        int idx_i = 0;
        openvdb::Int32Tree & tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

        for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i) {
          sphereSemanticCore(
            tree,
            idx_i,
            ijk_min,
            ijk_max,
            radius,
            centre,
            reg,
            reg_register);
        }
      };

    performIteration(tbb_thread_pool, tbb_iteration_range, kernel);
  }
}

void SemanticMapHandler::insertSemanticPoints(
  std::vector<pcl::PointXYZ> points,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  if (points.size() == 0) {
    return;
  } else {
    openvdb::Int32Tree & tree = grid_->tree();
    GridAccessorType accessor(tree);

    for (const auto & point : points) {
      openvdb::Vec3d point_tt(point.x, point.y, point.z);
      openvdb::Vec3d grid_coords_tc = grid_->worldToIndex(point_tt);
      openvdb::Coord grid_coords(
        static_cast<int>(grid_coords_tc[0]),
        static_cast<int>(grid_coords_tc[1]),
        static_cast<int>(grid_coords_tc[2]));

      setVoxelId(
        accessor,
        grid_coords[0],
        grid_coords[1],
        grid_coords[2],
        reg,
        reg_register);
    }
  }
}

void SemanticMapHandler::insertSemanticBox(
  const float & amplitude_h,
  const float & amplitude_v,
  const float & length,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register,
  const openvdb::Vec3d & centre)
{
  if (amplitude_h < 0 || amplitude_v < 0 || length < 0) {
    std::cerr <<
      "[AddBox]: The Box length/amplitude cannot be less than zero! Aborting! Aborting!" <<
      std::endl;
  } else {
    // Compute the maximum and minimum value of i,j, and k
    openvdb::Vec3d half_sizes = 0.5 * openvdb::Vec3d(length, amplitude_h, amplitude_v);

    openvdb::Vec3d xyz_min_value_ws = centre - half_sizes;
    openvdb::Vec3d xyz_min_value_is = grid_->worldToIndex(xyz_min_value_ws);
    openvdb::Coord ijk_min(
      static_cast<int>(xyz_min_value_is[0]),
      static_cast<int>(xyz_min_value_is[1]),
      static_cast<int>(xyz_min_value_is[2]));

    openvdb::Vec3d xyz_max_value = centre + half_sizes;
    openvdb::Vec3d xyz_max_value_is = grid_->worldToIndex(xyz_max_value);
    openvdb::Coord ijk_max(
      static_cast<int>(xyz_max_value_is[0]),
      static_cast<int>(xyz_max_value_is[1]),
      static_cast<int>(xyz_max_value_is[2]));

    tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
    tbb::blocked_range<int> tbb_iteration_range(ijk_min[0], ijk_max[0], sizeof(ijk_max[0]));

    auto kernel = [&](const tbb::blocked_range<int> & iteration_range)
      {
        int idx_i = 0;
        openvdb::Int32Tree & tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

        for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i) {
          boxSemanticCore(
            tree,
            idx_i,
            ijk_min,
            ijk_max,
            reg,
            reg_register);
        }
      };

    performIteration(tbb_thread_pool, tbb_iteration_range, kernel);
  }
}

void SemanticMapHandler::insertSemanticCone(
  const float & amplitude,
  const float & length,
  const openvdb::Vec3d & direction,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register,
  const openvdb::Vec3d & origin)
{
  if (length <= 0.0) {
    std::cerr << "[AddCone]: The cone length cannot be less than or equal to zero! Aborting!" <<
      std::endl;
  } else {
    float theta = amplitude / 2.0;
    int i_end = std::ceil(length / voxel_size_);

    openvdb::Vec3d idx_space_origin = grid_->worldToIndex(origin);

    openvdb::Vec3d unit_vector_x(1.0, 0.0, 0.0);
    openvdb::Vec3d rotation_axis = unit_vector_x.cross(direction);
    double rotation_angle = std::atan2(rotation_axis.length(), unit_vector_x.dot(direction));
    rotation_axis.normalize();
    openvdb::math::Quatd rotation_quat(rotation_axis, rotation_angle);

    tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
    tbb::blocked_range<int> tbb_iteration_range(0, i_end, sizeof(i_end));

    auto kernel = [&](const tbb::blocked_range<int> & iteration_range)
      {
        int idx_i = 0;
        int idx_j = 0;
        int idx_k = 0;
        openvdb::Int32Tree & tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

        for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i) {
          coneSemanticCore(
            tree,
            idx_i,
            idx_j,
            idx_k,
            theta,
            idx_space_origin,
            rotation_quat,
            reg,
            reg_register);
        }
      };

    performIteration(tbb_thread_pool, tbb_iteration_range, kernel);
  }
}

void SemanticMapHandler::insertSemanticPyramid(
  const float & amplitude_h,
  const float & amplitude_v,
  const float & length,
  const openvdb::Vec3d & direction,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register,
  const openvdb::Vec3d & origin,
  const float & starting_point)
{
  if (length <= 0.0) {
    std::cerr <<
      "The pyramid length cannot be less than or equal to zero! Aborting!" <<
      std::endl;
  } else {
    float theta_h = amplitude_h / 2.0;
    float theta_v = amplitude_v / 2.0;
    int i_begin = std::ceil(length / voxel_size_ * starting_point);
    int i_end = std::ceil(length / voxel_size_);

    openvdb::Vec3d idx_space_origin = grid_->worldToIndex(origin);

    openvdb::Vec3d unit_vector_x(1.0, 0.0, 0.0);

    // compute the pitch and yaw angles to take unit_vector_x to direction
    auto axis = direction;
    axis.normalize();

    double yaw = -std::atan2(axis[1], axis[0]);
    double pitch = std::asin(axis[2]);

    auto c_pitch = std::cos(pitch);
    auto s_pitch = std::sin(pitch);
    auto c_yaw = std::cos(yaw);
    auto s_yaw = std::sin(yaw);
    openvdb::math::Mat3d rot_matrix(c_pitch * c_yaw, -c_pitch * s_yaw, s_pitch,
      s_yaw, c_yaw, 0.0,
      -s_pitch * c_yaw, s_pitch * s_yaw, c_pitch);
    openvdb::math::Quatd rotation_quat(rot_matrix);
    rotation_quat.normalize();

    tbb::enumerable_thread_specific<openvdb::Int32Tree> tbb_thread_pool(grid_->tree());
    tbb::blocked_range<int> tbb_iteration_range(i_begin, i_end, sizeof(i_end));

    auto kernel = [&](const tbb::blocked_range<int> & iteration_range)
      {
        int idx_i = 0;
        int idx_j = 0;
        int idx_k = 0;
        openvdb::Int32Tree & tree = (threaded_) ? tbb_thread_pool.local() : grid_->tree();

        for (idx_i = iteration_range.begin(); idx_i != iteration_range.end(); ++idx_i) {
          pyramidSemanticCore(
            tree,
            idx_i,
            idx_j,
            idx_k,
            theta_h,
            theta_v,
            idx_space_origin,
            rotation_quat,
            reg,
            reg_register);
        }
      };

    performIteration(tbb_thread_pool, tbb_iteration_range, kernel);
  }
}

void SemanticMapHandler::insertVoxel(
  const float & x,
  const float & y,
  const float & z,
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  openvdb::Vec3d point(x, y, z);
  openvdb::Vec3d grid_coords_tc = grid_->worldToIndex(point);
  openvdb::Coord grid_coords(
    static_cast<int>(grid_coords_tc[0]),
    static_cast<int>(grid_coords_tc[1]),
    static_cast<int>(grid_coords_tc[2]));

  openvdb::Int32Tree & tree = grid_->tree();
  GridAccessorType accessor(tree);

  setVoxelId(
    accessor,
    grid_coords[0],
    grid_coords[1],
    grid_coords[2],
    reg,
    reg_register);
}

bool SemanticMapHandler::removeRegion(
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register)
{
  using ValueIter = typename openvdb::Int32Grid::ValueOnIter;

  // First we find the ID associated with the region-only area
  std::vector<std::string> reg_area = {reg};
  auto reg_id = reg_register.findRegions(reg_area);
  auto ids_to_udpate = reg_register.removeRegion(reg);
  if ((reg_id == -1) && (ids_to_udpate.size() == 0)) {
    return false;
  }

  struct IdUpdater
  {
    int reg_id_;
    std::map<int, int> ids_to_update_;
    std::map<int, openvdb::CoordBBox> & bbox_;
    SemanticMapHandler & map_handler_;
    IdUpdater(
      const int & reg_id,
      const std::map<int, int> & ids_to_udpate,
      std::map<int, openvdb::CoordBBox> & bbox,
      SemanticMapHandler & map_handler)
    : reg_id_(reg_id),
      ids_to_update_(ids_to_udpate),
      bbox_(bbox),
      map_handler_(map_handler) {}
    inline void operator()(const ValueIter & iter) const
    {
      if (!iter.isVoxelValue()) {
        return;
      }
      if (*iter == reg_id_) {
        iter.setValueOff();
        bbox_.erase(*iter);
        return;
      }
      auto ids_it = ids_to_update_.find(*iter);
      if (ids_it == ids_to_update_.end()) {
        return;
      }
      bbox_.erase(*iter);
      iter.setValue(ids_it->second);
      map_handler_.updateAreaBBox(ids_it->second, iter.getCoord());
    }
  };
  // As we are directly modifying the topology
  // of the tree associated with the grid,
  // it is not possible to have a multithreaded
  // foreach
  openvdb::tools::foreach(
    grid_->beginValueOn(),
    IdUpdater(reg_id, ids_to_udpate, areas_bbox_, *this), false);

  if (entities_bbox_.find(reg) != entities_bbox_.end()) {
    entities_bbox_.erase(reg);
  }
  return true;
}

bool SemanticMapHandler::verticallyAligned(
  const openvdb::CoordBBox & bbox1,
  const openvdb::CoordBBox & bbox2,
  const float & min_iou)
{
  // We check whether the XY projection of the bboxes
  // intersects. The general condition to understand if
  // two bounding boxes intersect, is that they both
  // intersect along the x and y axis.
  auto x_a = std::min(bbox1.min()[0], bbox2.min()[0]);
  auto x_b = (x_a == bbox1.min()[0]) ? bbox1.max()[0] : bbox2.max()[0];
  auto x_c = std::max(bbox1.min()[0], bbox2.min()[0]);
  bool x_intersection = x_b > x_c;

  auto y_a = std::min(bbox1.min()[1], bbox2.min()[1]);
  auto y_b = (y_a == bbox1.min()[1]) ? bbox1.max()[1] : bbox2.max()[1];
  auto y_c = std::max(bbox1.min()[1], bbox2.min()[1]);
  bool y_intersection = y_b > y_c;

  auto bbox1_area = (bbox1.max()[0] - bbox1.min()[0]) * (bbox1.max()[1] - bbox1.min()[1]);
  auto bbox2_area = (bbox2.max()[0] - bbox2.min()[0]) * (bbox2.max()[1] - bbox2.min()[1]);

  float intersection_area = (x_b - x_c) * (y_b - y_c);
  float union_area = bbox1_area + bbox2_area - intersection_area;

  float iou = intersection_area / union_area;

  return x_intersection && y_intersection && (iou > min_iou);
}

bool SemanticMapHandler::intersect(
  openvdb::CoordBBox bbox1,
  const openvdb::CoordBBox & bbox2)
{
  bbox1.intersect(bbox2);
  return bbox1.hasVolume();
}

bool SemanticMapHandler::isInside(
  const openvdb::CoordBBox & bbox1,
  const openvdb::CoordBBox & bbox2)
{
  return bbox1.isInside(bbox2);
}

bool SemanticMapHandler::higherThan(
  const openvdb::CoordBBox & bbox1,
  const openvdb::CoordBBox & bbox2,
  const float & above_thresh)
{
  // We are computing whether bbox1 is above bbox2
  if ((bbox1.max()[2] > bbox2.max()[2]) && (bbox1.min()[2] > bbox2.max()[2])) {
    return true;
  }
  if ((bbox1.max()[2] < bbox2.max()[2]) || (bbox1.min()[2] < bbox2.min()[2])) {
    return false;
  }
  float intersection_length = bbox2.max()[2] - bbox1.min()[2] + 1;
  float bbox1_z_length = bbox1.max()[2] - bbox1.min()[2] + 1;
  return (intersection_length / bbox1_z_length) > above_thresh;
}

bool SemanticMapHandler::aboveTouching(
  const openvdb::CoordBBox & bbox1,
  const openvdb::CoordBBox & bbox2,
  const float & min_iou,
  const float & above_thresh)
{
  return higherThan(
    bbox1, bbox2,
    above_thresh) &&
         verticallyAligned(
    bbox1, bbox2,
    min_iou) && (intersect(bbox1, bbox2) || (bbox1.min()[2] == bbox2.max()[2]));
}

std::string SemanticMapHandler::computeSymbolicRelationship(
  const openvdb::CoordBBox & bbox1,
  const openvdb::CoordBBox & bbox2,
  const float & min_iou,
  const float & above_thresh)
{
  if (aboveTouching(bbox1, bbox2, min_iou, above_thresh)) {
    return "aboveTouching";
  } else if (higherThan(bbox1, bbox2, above_thresh) && verticallyAligned(bbox1, bbox2, min_iou)) {
    return "above";
  } else if (isInside(bbox2, bbox1)) {
    return "inside";
  } else if (intersect(bbox1, bbox2)) {
    return "intersect";
  } else {
    return "disjoint";
  }
}

void SemanticMapHandler::processRelationships(
  const std::shared_ptr<remap::regions_register::RegionsRegister> reg_register,
  const bool & entities_relationship,
  const bool & regions_relationship,
  const bool & textual_debugging)
{
  if (areas_bbox_.size() == 0) {
    return;
  }
  if (entities_relationship) {
    for (auto bboxes_it = entities_bbox_.begin(); bboxes_it != entities_bbox_.end();
      bboxes_it++)
    {
      std::map<std::string, std::string> entities_relationship_matrix_row;
      for (auto n_it = entities_bbox_.begin(); n_it != entities_bbox_.end(); n_it++) {
        if (bboxes_it == n_it) {
          continue;
        }
        std::string relationship = computeSymbolicRelationship(bboxes_it->second, n_it->second);
        entities_relationship_matrix_row[n_it->first] = relationship;
        if (textual_debugging) {
          std::cout << bboxes_it->first << " " << relationship << " " << n_it->first << std::endl;
        }
      }
      entities_relationship_matrix_[bboxes_it->first] = entities_relationship_matrix_row;
    }
  }
  if (regions_relationship) {
    for (auto bboxes_it = areas_bbox_.begin(); bboxes_it != std::prev(areas_bbox_.end());
      bboxes_it++)
    {
      std::map<int, std::string> regions_relationship_matrix_row;
      for (auto n_it = std::next(bboxes_it); n_it != areas_bbox_.end(); n_it++) {
        std::string relationship = computeSymbolicRelationship(bboxes_it->second, n_it->second);
        regions_relationship_matrix_row[n_it->first] = relationship;
        if (textual_debugging) {
          std::string region_1_entities = "[";
          std::string region_2_entities = "[";
          for (auto reg : reg_register->findRegionsById(bboxes_it->first)) {
            region_1_entities += reg + " ";
          }
          for (auto reg : reg_register->findRegionsById(n_it->first)) {
            region_2_entities += reg + " ";
          }
          region_1_entities += "]";
          region_2_entities += "]";
          std::cout << "Region " << region_1_entities << " " << relationship << " Region " <<
            region_2_entities << std::endl;
          std::cout << "Region 1: " << bboxes_it->second << std::endl;
          std::cout << "Region 2: " << n_it->second << std::endl;
        }
      }
      regions_relationship_matrix_[bboxes_it->first] = regions_relationship_matrix_row;
    }
  }
}

std::map<int, std::map<int, std::string>> SemanticMapHandler::getRegionsRelationshipMatrix() const
{
  return regions_relationship_matrix_;
}

std::map<std::string,
  std::map<std::string, std::string>> SemanticMapHandler::getEntitiesRelationshipMatrix() const
{
  return entities_relationship_matrix_;
}

void SemanticMapHandler::clear()
{
  grid_.reset();
  grid_ = openvdb::Int32Grid::create();
  grid_->setTransform(initial_transformation_);
}

std::shared_ptr<openvdb::Int32Grid> SemanticMapHandler::getGridPtr()
{
  return grid_;
}

std::string SemanticMapHandler::getFixedFrame() const
{
  return fixed_frame_;
}

float SemanticMapHandler::getVoxelSize() const
{
  return voxel_size_;
}


void SemanticMapHandler::setFixedFrame(const std::string & fixed_frame)
{
  fixed_frame_ = fixed_frame;
}

void SemanticMapHandler::getEntity(
  const std::string & reg,
  remap::regions_register::RegionsRegister & reg_register,
  pcl::PointCloud<pcl::PointXYZI> & points)
{
  using ValueIter = typename openvdb::Int32Grid::ValueOnIter;

  // First we find the IDs associated with the regions containing
  // the semantic entity reg
  auto reg_ids = reg_register.getEntityIds(reg);

  if (reg_ids.size() == 0) {
    return;
  }

  struct EntityGetter
  {
    std::vector<int> reg_ids_;
    pcl::PointCloud<pcl::PointXYZI> & points_;
    SemanticMapHandler & map_handler_;
    EntityGetter(
      const std::vector<int> & reg_ids,
      pcl::PointCloud<pcl::PointXYZI> & points,
      SemanticMapHandler & map_handler)
    : reg_ids_(reg_ids),
      points_(points),
      map_handler_(map_handler) {}
    inline void operator()(const ValueIter & iter) const
    {
      if (!iter.isVoxelValue()) {
        return;
      }
      if (std::find(reg_ids_.begin(), reg_ids_.end(), *iter) != reg_ids_.end()) {
        openvdb::Vec3d point = map_handler_.grid_->indexToWorld(iter.getCoord());
        pcl::PointXYZI pcl_point;
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        pcl_point.intensity = *iter;
        points_.push_back(pcl_point);
      }
    }
  };

  openvdb::tools::foreach(grid_->beginValueOn(), EntityGetter(reg_ids, points, *this), false);
}

}  // namespace map_handler
}  // namespace remap
