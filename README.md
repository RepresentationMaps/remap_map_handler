remap_map_handler
=================

The `remap_map_handler` package provides a comprehensive solution for managing semantic maps by combining a volumetric VDB grid with a regions register. This package enables efficient linking of symbolic and spatial information about semantic entities.

Features
--------

- **Volumetric Map Representation**: Uses OpenVDB for efficient storage and manipulation of 3D spatial data.

- **Semantic Region Management**: Utilizes `remap_regions_register` to manage symbolic areas within the map.

- **Flexible Querying and Updates**: Allows dynamic modification and retrieval of spatial information tied to semantic entities.

Usage
-----

### Creating a map handler

Two different constructor are available for the `SemanticMapHandler` class:

```cpp
SemanticMapHandler(
    const bool & threaded,
    const float & voxel_size,
    const bool & vertex_centered = true,
    const std::string & fixed_frame = "map");
```

and

```cpp
SemanticMapHandler(
    std::shared_ptr<openvdb::Int32Grid> grid,
    const bool & threaded,
    const float & voxel_size,
    const bool & vertex_centered = true,
    const std::string & fixed_frame = "map");
```

The difference between these two constructors is that the first one creates a new grid, while the second one uses an existing grid.

The other parameters are:
- `threaded`: a boolean flag that indicates whether the grid-related operations should be threaded (⚠️ *Not supported yet* ⚠️);
- `voxel_size`: the size of the voxels in the grid;
- `fixed_frame`: the name of the fixed frame of the semantic map;
- `vertex_centered`: a boolean flag that indicates whether the grid should be vertex-centered or not 
(see [this](https://www.openvdb.org/documentation/doxygen/transformsAndMaps.html#sCellVsVertex) for more info).

At any time, it possible to retrieve the pointer to the grid by calling the function `getGridPtr`:

```cpp
std::shared_ptr<openvdb::Int32Grid> getGridPtr();
```

It is also possible to retrieve and update the fixed frame by calling the functions `getFixedFrame` and `setFixedFrame`:

```cpp
void setFixedFrame(const std::string & fixed_frame);
std::string getFixedFrame() const;
```

### Adding spatial information about a semantic entity

The `SemanticMapHandler` class comes with a variety of functions to add spatial information
associated to a semantic entity in the form of a certain shape or a group of points.

#### Adding a spherical volume

The function `insertSemanticSphere` allows the insertion of a spherical shape with a given radius and center.

```cpp
void SemanticMapHandler::insertSemanticSphere(
    const float & radius,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & centre = openvdb::Vec3d(0.0, 0.0, 0.0));
```

Where:
- `radius` is the radius of the sphere;
- `reg` is the name of the semantic entity associated with the spheric volume;
- `reg_register` is the regions register object;
- `centre` is the center of the sphere; this is expected to be expressed wrt the semantic map fixed frame.

For instance, adding a spherical volume associated to the semantic entity *sphere* centered in *(1.0, 1.0, 1.0)*
with radius equal to *1.0m* would be done as follows:

```cpp
map_handler.insertSemanticSphere(1.0, "sphere", reg_register, openvdb::Vec3d(1.0, 1.0, 1.0));
```

#### Adding a vector of XYZ points

The function `insertSemanticPoints` allows the insertion of a group of points in the form of a vector of `pcl::PointXYZ`.

```cpp
void insertSemanticPoints(
    std::vector<pcl::PointXYZ> points,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);
```

Where:
- `points` is a vector of `pcl::PointXYZ` points, expressed in the semantic map fixed frame;
- `reg` is the name of the semantic entity associated with the points;
- `reg_register` is the regions register object;

For instance, adding a group of points associated to the semantic entity *mug_123* would be done as follows:

```cpp
map_handler.insertSemanticPoints(mug_points, "mug_123", reg_register);
```

#### Adding a box volume

The function `insertSemanticBox` allows the insertion of a box-like shape with a given `height`, `width` and `depth`
and centred in `centre`.

```cpp
void insertSemanticBox(
    const float & width,
    const float & height,
    const float & depth,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & centre = openvdb::Vec3d(0.0, 0.0, 0.0));
```

Where:
- `width`, `height` and `depth` are the dimensions of the box;
- `reg` is the name of the semantic entity associated with the box;
- `reg_register` is the regions register object;
- `centre` is the center of the box; this is expected to be expressed wrt the semantic map fixed frame.

For instance, adding a box representing the semantic entity *table* with dimensions *1.0m x 1.0m x 1.0m*
and centered in *(2.0, 2.0, 0.5)* would be done as follows:

```cpp
map_handler.insertSemanticBox(1.0, 1.0, 1.0, "table", reg_register, openvdb::Vec3d(2.0, 2.0, 0.5));
```

#### Adding a cone volume

The function `insertSemanticCone` allows the insertion of a conic shape with a given
`amplitude`, `length` and `direction` and with origin in `origin`.

```cpp
void insertSemanticCone(
    const float & amplitude,
    const float & length,
    const openvdb::Vec3d & direction,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & origin = openvdb::Vec3d(0.0, 0.0, 0.0));
```

Where:
- `amplitude` is the opening angle of the cone;
- `length` is the length of the cone (radiants);
- `direction` is the direction of the cone;
- `reg` is the name of the semantic entity associated with the cone;

For instance, adding a cone representing a person's field of view `fov_456`
with an opening angle of *0.7rad*, length of *1.0m*, direction *(1.0, 0.0, 0.0)*
and originating in the point *(1.0, 1.0, 1.0)* (that is, where the person's head is located)
would be done as follows:

```cpp
map_handler.insertSemanticCone(0.7, 1.0, openvdb::Vec3d(1.0, 0.0, 0.0), "fov_456", reg_register, openvdb::Vec3d(1.0, 1.0, 1.0));
```

All the vectors and coordinates should be expressed in the semantic map fixed frame.

#### Adding a pyramid volume

The function `insertSemanticPyramid` allows the insertion of a pyramid-like shape with a given
horizontal amplitude, vertical amplitude, length, origin and direction.

```cpp
void insertSemanticPyramid(
    const float & amplitude_h,
    const float & amplitude_v,
    const float & length,
    const openvdb::Vec3d & direction,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register,
    const openvdb::Vec3d & origin = openvdb::Vec3d(0.0, 0.0, 0.0));
```

Where:
- `amplitude_h` is the horizontal opening angle of the pyramid;
- `amplitude_v` is the vertical opening angle of the pyramid;
- `length` is the length of the pyramid;
- `direction` is the direction of the pyramid;
- `reg` is the name of the semantic entity associated with the pyramid;
- `reg_register` is the regions register object;
- `origin` is the origin of the pyramid.

For instance, adding a cone representing the robot's field of view `robot_fov`
with an orizontal amplitude of *0.7rad*, vertical amplitude of *0.5rad*, length of *3.0m*,
direction *(1.0, 0.0, 0.0)* and originating in the point *(1.0, 1.0, 1.0)*


```cpp
map_handler.insertSemanticPyramid(0.7, 0.5, 3.0, openvdb::Vec3d(1.0, 0.0, 0.0), "robot_fov", reg_register, openvdb::Vec3d(1.0, 1.0, 1.0));
```

All the vectors and coordinates should be expressed in the semantic map fixed frame.

#### Adding a single voxel

The function `insertSemanticVoxel` allows the insertion of a single voxel at a given point, associated with a given semantic entity.

```cpp
void insertVoxel(
    const float & x,
    const float & y,
    const float & z,
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);
```

Where:
- `x`, `y` and `z` are the coordinates of the voxel, expressed in the semantic map fixed frame;
- `reg` is the name of the semantic entity associated with the voxel;
- `reg_register` is the regions register object.

For instance, adding a voxel at the point *(1.0, 1.0, 1.0)* associated with the semantic entity *hotpoint_789* would be done as follows:

```cpp
map_handler.insertSemanticVoxel(1.0, 1.0, 1.0, "hotpoint_789", reg_register);
```

#### Future developments and missing shapes

This package is designed to be easily extensible to include more shapes in the future.
If you need a specific shape that is not currently supported, please open an issue or a pull request.

### Removing spatial information about a semantic entity

The `SemanticMapHandler` class comes with a function to remove spatial information.

```cpp
bool removeRegion(
    const std::string & reg,
    remap::regions_register::RegionsRegister & reg_register);
```

Where:
- `reg` is the name of the semantic entity associated with the spatial information to be removed;
- `reg_register` is the regions register object.

The function updates the content of the voxels associated with the `reg` entity in order to 
de-associate them from the semantic entity. For instance, removing the semantic entity *mug_123* would be done as follows:

```cpp
map_handler.removeRegion("mug_123", reg_register);
```

### Computing spatial relationships between spatial regions

A spatial region is a region of space containing the same entities, that is, a group of voxels containing the same value.
The `SemanticMapHandler` class provides a function to compute the spatial relationships between all the available regions.
Currently, the available spatial relationships are: *aboveTouching*, *above*, *intersect*, *inside* and *disjoint*.

In order to compute them, it is necessary to call the function `processRelationships`:

```cpp
void processRelationships(
    remap::regions_register::RegionsRegister & reg_register);
```

Where `reg_register` is the regions register object.

```cpp
map_handler.processRelationships(reg_register);
```

It is then possible to retrieve these relationships (in the form of a spatial relationships matrix) by calling the function `getRelationshipsMatrix`:

```cpp
std::map<int, std::map<int, std::string>> getRelationshipsMatrix() const;
```

The relationships matrix is a map where the key is the region ID and the value is another map where the key is the region ID of the related region and the value is the relationship between the two regions.

## License

This package is licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.

## Authors

Developed by **PAL Robotics, S.L.**

