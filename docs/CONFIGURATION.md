# Configuration Reference

The default configuration is `Inland-LOAM/config/loam_config.yaml`. Copy it to
adapt the sensor or output settings, then pass the copy through
`config_file:=/absolute/path/to/config.yaml` when launching Inland-LOAM.

## LiDAR Projection

| Parameter | Default | Meaning |
|---|---:|---|
| `laser.num_vertical_scans` | 32 | Number of RS-32 vertical channels |
| `laser.num_horizontal_scans` | 1800 | Columns in the range image |
| `laser.vertical_angle_bottom` | -55 deg | Lowest elevation angle |
| `laser.vertical_angle_top` | 15 deg | Highest elevation angle |
| `laser.ground_scan_index` | 14 | Upper index of low-beam ground/water labeling |
| `image_projection.segment_valid_point_num` | 5 | Minimum points for the small-segment exception |
| `image_projection.segment_valid_line_num` | 3 | Minimum rings spanned by that exception |
| `image_projection.segment_theta` | 60 deg | Range-image connectivity threshold |
| `image_projection.input_queue_size` | 32 | Incoming scans buffered during short processing stalls |

The input topic is remapped from `/lidar_points` to `/velodyne_points` in
`run.launch.py`. Change that remapping for another driver rather than renaming
recorded topics.

## Feature Association

| Parameter | Default | Meaning |
|---|---:|---|
| `mapping.mapping_frequency_divider` | 5 | Send every fifth associated scan to mapping |
| `featureAssociation.edge_threshold` | 0.1 | Edge-feature curvature threshold |
| `featureAssociation.surf_threshold` | 0.1 | Surface-feature curvature threshold |
| `featureAssociation.nearest_feature_search_distance` | 5.0 m | Maximum correspondence distance |
| `laser.scan_period` | 0.1 s | LiDAR rotation period |


## Mapping

| Parameter | Default | Meaning |
|---|---:|---|
| `mapping.enable_loop_closure` | false | Enable ICP loop closure |
| `mapping.surrounding_keyframe_search_radius` | 50 m | Local-map radius when loop closure is off |
| `mapping.surrounding_keyframe_search_num` | 50 | Local keyframe count when loop closure is on |
| `mapping.history_keyframe_search_radius` | 7 m | Loop candidate radius |
| `mapping.history_keyframe_search_num` | 25 | Keyframes on each side of a loop candidate |
| `mapping.history_keyframe_fitness_score` | 0.3 | Maximum accepted ICP fitness score |
| `mapping.water_plane_huber.enabled` | true | Robustify the water-plane constraint |
| `mapping.water_plane_huber.delta` | 1.345 | Huber transition parameter |

Loop closure remains disabled in this version. Its historical keyframe pose
update needs correction before the option can be used reliably.

## Output And Export

| Parameter | Default | Meaning |
|---|---|---|
| `mapping.save_pcd_path` | `/tmp/inland_loam` | Final point-cloud output directory |
| `mapping.save_final_map` | false | Save the final PCD when the node exits |
| `mapping.offline_export.enable` | false | Export keyframes and source indices |
| `mapping.offline_export.path` | `/tmp/inland_loam_export` | Export root |
| `mapping.offline_export.max_keyframes` | 0 | Maximum exported keyframes; 0 means all |


## Structural Decision Rules

For each observed 2D cell, let `N` be the number of reliable FLAT, ROUGH, or
UNCERTAIN voxels, `R` the FLAT fraction among them, and `m` the fitted local
slope magnitude. Rules are evaluated in this exact order:

| Rule | Class | Condition |
|---|---|---|
| A | VP | `N > 9`, `R >= 0.50`, `m > 0.45` |
| B | VP | `N > 10`, `R >= 0.65`, `m > 0.20` |
| C | HP | `N > 10`, `R >= 0.80`, `m < 0.15` |
| D | HP | `N > 7`, `R >= 0.75`, `m <= 0.10` |
| E | CV | A--D failed and `m > 0.15` |
| F | CH | A--E failed |

This is an ordered decision chain.
Rules A/B and C/D provide alternative combinations of point support,
planarity, and slope. Water and under-bridge rules are applied separately.

## Map Conversion Launch Parameters

`mapconversion/launch/map_conversion.launch.py` currently uses:

| Parameter | Default |
|---|---:|
| `minimum_z` | 1.0 |
| `minimum_occupancy` | 10 |
| `map_frame` | `ssm` |
| `ssm_alpha` | 0.6 |
| `max_slope_ugv` | 0.35 |
| `slope_estimation_size` | 2 |

