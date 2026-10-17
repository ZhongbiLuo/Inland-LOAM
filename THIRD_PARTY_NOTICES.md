# Third-Party Notices

Inland-LOAM contains or derives from components with their own attribution:

- The LiDAR odometry architecture and portions of the source derive from
  LeGO-LOAM and related ROS 2 ports. The original BSD copyright notices are
  retained in the corresponding source files.
- `nanoflann.hpp` and `nanoflann_pcl.h` retain their BSD notices in-file.
- `mapconversion` and `mapconversion_msgs` derive from
  `LTU-RAI/Map-Conversion-3D-Voxel-Map-to-2D-Occupancy-Map`. They retain the
  upstream MIT license and 2024 Scott Fredriksson copyright notice in each
  package's `LICENSE` file.

External dependencies such as ROS 2, PCL, Eigen, GTSAM, Boost, and OctoMap are
not redistributed by this repository and remain subject to their own licenses.

The corresponding package/source-file license takes precedence when it differs
from the repository-level BSD 3-Clause license.
