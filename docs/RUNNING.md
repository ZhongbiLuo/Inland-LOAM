# Running Inland-LOAM

Build instructions and the Area A demo are in the [README](../README.md).
Use a fresh terminal when changing workspaces, and source ROS 2 before the
workspace. A successful build installs the programs; it does not start them.

## Run the demo or your own ROS 2 bag

Use two terminals. The standard launch starts odometry, mapping, structural
map conversion, TF and RViz, and leaves playback under your control.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/inland_loam_ws/install/local_setup.bash
ros2 launch inland_loam mapping.launch.py
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/inland_loam_ws/install/local_setup.bash
ros2 bag play /path/to/extracted_demo --rate 1.0
```

Replace `/path/to/extracted_demo` with the extracted bag directory containing
`metadata.yaml`.
Download and extract the demo archive linked in the README. Play the
extracted directory containing `metadata.yaml`, not the archive. The Zstd
plugin decompresses the supplied data during playback and needs writable
space beside the bag. Restart the launch before replaying from the beginning.

For your own bag, replace the path in terminal 2. The input is
`sensor_msgs/msg/PointCloud2` on `/velodyne_points`. The supplied YAML is for
RS-32 Helios geometry. For another sensor, prepare a parameter YAML using the
[configuration reference](CONFIGURATION.md), then pass
`config_file:=/absolute/path/to/config.yaml` to `mapping.launch.py`.
The point-cloud frame must be connected to the configured TF tree.
ROS 1 `.bag` files need conversion before they can be played with ROS 2.

## Outputs and visualization

| Topic | Output |
|---|---|
| `/velodyne_points` | Raw LiDAR input |
| `/aft_mapped_to_init` | Mapping odometry |
| `/laser_cloud_surround` | 3D map cloud |
| `/key_pose_origin` | Keyframe trajectory |
| `/octomap_out` | Colored 3D OctoMap |
| `/geometric_map_2` | Structural semantic grid |
| `/structural_semantic_map` | Colored SSM visualization |
| `/bridge_gap_texts` | Optional labels of mean clearance for grouped bridge regions |

