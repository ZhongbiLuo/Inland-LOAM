#ifndef IMAGEPROJECTION_H
#define IMAGEPROJECTION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/QR>

#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>

class ImageProjection : public rclcpp::Node {
 public:

  ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel);

  ~ImageProjection() = default;

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg);

    struct PlaneParams {
    Eigen::Vector3f normal;   
    float d;                  
    
    PlaneParams() : d(0.0f) {
      normal.setZero();
    }
  };

 private:
  void findStartEndAngle();
  void resetParameters();
  void projectPointCloud();
  void groundRemoval();
  void cloudSegmentation();
  void labelComponents(int row, int col);
  void publishClouds();
  float _plane_offset_distance = 0.4f; 

  pcl::PointCloud<PointType>::Ptr _laser_cloud_in;

  pcl::PointCloud<PointType>::Ptr _full_cloud;
  pcl::PointCloud<PointType>::Ptr _full_info_cloud;

  pcl::PointCloud<PointType>::Ptr _ground_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud;
  pcl::PointCloud<PointType>::Ptr _segmented_cloud_pure;
  pcl::PointCloud<PointType>::Ptr _outlier_cloud;

  std::vector<uint32_t> _laser_cloud_in_raw_indices;
  std::vector<int32_t> _full_cloud_raw_indices;
  std::vector<uint32_t> _segmented_raw_indices;
  std::vector<uint32_t> _outlier_raw_indices;
  uint32_t _frame_index = 0;

  int _vertical_scans;
  int _horizontal_scans;
  float _ang_bottom;
  float _ang_resolution_X;
  float _ang_resolution_Y;
  float _segment_theta;
  int _segment_valid_point_num;
  int _segment_valid_line_num;
  int _ground_scan_index;
  float _sensor_mount_angle;

  std::vector<float> _laser_angles;  
  
  Channel<ProjectionOut>& _output_channel;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub_laser_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_full_info_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_ground_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_segmented_cloud_pure;
  rclcpp::Publisher<cloud_msgs::msg::CloudInfo>::SharedPtr _pub_segmented_cloud_info;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_outlier_cloud;
  rclcpp::Publisher<cloud_msgs::msg::Plane>::SharedPtr _pub_plane;

  cloud_msgs::msg::CloudInfo _seg_msg;
  cloud_msgs::msg::Plane _plane_msgs;

  int _label_count;

  Eigen::MatrixXf _range_mat;   
  Eigen::MatrixXi _label_mat;   
  Eigen::Matrix<int8_t,Eigen::Dynamic,Eigen::Dynamic> _ground_mat;  


};



#endif  // IMAGEPROJECTION_H
