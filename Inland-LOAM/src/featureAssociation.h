#ifndef FEATUREASSOCIATION_H
#define FEATUREASSOCIATION_H

#include "cloud_msgs/msg/plane.hpp"
#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/QR>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

class FeatureAssociation : public rclcpp::Node {

 public:
  FeatureAssociation(const std::string &name, Channel<ProjectionOut>& input_channel,
                     Channel<AssociationOut>& output_channel);

  ~FeatureAssociation();

  void runFeatureAssociation();

 private:

  int _vertical_scans;
  int _horizontal_scans;
  float _scan_period;
  float _edge_threshold;
  float _surf_threshold;
  float _nearest_feature_dist_sqr;
  int _mapping_frequency_div;

  std::thread _run_thread;

  Channel<ProjectionOut>& _input_channel;
  Channel<AssociationOut>& _output_channel;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCornerPointsLessSharp;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsFlat;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPointsLessFlat;

  pcl::PointCloud<PointType>::Ptr segmentedCloud;
  pcl::PointCloud<PointType>::Ptr outlierCloud;
  cloud_msgs::msg::Plane planeInfo;
  std::vector<uint32_t> segmentedCloudRawIndices;
  std::vector<uint32_t> outlierCloudRawIndices;
  uint32_t currentFrameIndex = 0;
  double currentFrameStamp = 0.0;

  pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
  pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
  pcl::PointCloud<PointType>::Ptr surfPointsFlat;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;
  std::vector<uint32_t> cornerPointsSharpRawIndices;
  std::vector<uint32_t> cornerPointsLessSharpRawIndices;
  std::vector<uint32_t> surfPointsFlatRawIndices;
  std::vector<uint32_t> surfPointsLessFlatRawIndices;

  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
  pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;
  std::vector<uint32_t> surfPointsLessFlatScanRawIndices;

  pcl::PointCloud<PointType>::Ptr groundPoints;
  pcl::PointCloud<PointType>::Ptr groundPointsScan;
  pcl::PointCloud<PointType>::Ptr groundPointsScanDS;
  std::vector<uint32_t> groundPointsRawIndices;
  std::vector<uint32_t> groundPointsScanRawIndices;


  pcl::VoxelGrid<PointType> downSizeFilter;

  cloud_msgs::msg::CloudInfo segInfo;
  std_msgs::msg::Header cloudHeader;

  std::vector<smoothness_t> cloudSmoothness;
  std::vector<float> cloudCurvature;
  std::vector<int> cloudNeighborPicked;
  std::vector<int> cloudLabel;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_corner_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_surf_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud_ground_last;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloudLast;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  _pub_new_cloud_corner_last;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  _pub_new_cloud_surf_last;

  int skipFrameNum;
  bool systemInitedLM;

  int laserCloudCornerLastNum;
  int laserCloudSurfLastNum;

  std::vector<float> pointSearchCornerInd1;
  std::vector<float> pointSearchCornerInd2;

  std::vector<float> pointSearchSurfInd1;
  std::vector<float> pointSearchSurfInd2;
  std::vector<float> pointSearchSurfInd3;

  float transformCur[6];
  float transformSum[6];

  pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
  pcl::PointCloud<PointType>::Ptr laserCloudGroundLast;
  pcl::PointCloud<PointType>::Ptr newLaserCloudCornerLast;  
  pcl::PointCloud<PointType>::Ptr newLaserCloudSurfLast;  
  std::vector<uint32_t> laserCloudCornerLastRawIndices;
  std::vector<uint32_t> laserCloudSurfLastRawIndices;
  std::vector<uint32_t> laserCloudGroundLastRawIndices;
  std::vector<uint32_t> newLaserCloudCornerLastRawIndices;
  std::vector<uint32_t> newLaserCloudSurfLastRawIndices;
  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  nanoflann::KdTreeFLANN<PointType> kdtreeCornerLast;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfLast;

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  nav_msgs::msg::Odometry laserOdometry;

  geometry_msgs::msg::TransformStamped laserOdometryTrans;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
  
  bool isDegenerate;

  int frameCount;
  size_t _cycle_count;

 private:
  void initializationValue();
  void adjustDistortion();
  void calculateSmoothness();
  void markOccludedPoints();
  void extractFeatures();

  void TransformToStart(PointType const *const pi, PointType *const po);
  void TransformToEnd(PointType const *const pi, PointType *const po);

  void AccumulateRotation(float cx, float cy, float cz, float lx, float ly,
                          float lz, float &ox, float &oy, float &oz);

  void findCorrespondingCornerFeatures(int iterCount);
  void findCorrespondingSurfFeatures(int iterCount);

  bool calculateTransformationSurf(int iterCount);
  bool calculateTransformationCorner(int iterCount);
  bool calculateTransformation(int iterCount);

  void checkSystemInitialization();
  void updateTransformation();

  void integrateTransformation();
  void publishCloud();
  void publishOdometry();

  void adjustOutlierCloud();
  void publishCloudsLast();

};

#endif // FEATUREASSOCIATION_H
