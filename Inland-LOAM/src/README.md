# Inland-LOAM Source Modules

- `imageProjection`: range-image projection, segmentation, and water/ground
  plane extraction.
- `featureAssociation`: edge/surface feature selection and scan odometry.
- `mapOptmization`: keyframe optimization, water-plane factor, colored OctoMap,
  and optional offline export.
- `transformFusion`: fusion of scan odometry and optimized mapping odometry.

Runtime configuration belongs in `../config/loam_config.yaml`. Build and run
instructions are maintained in the repository root `README.md`.
