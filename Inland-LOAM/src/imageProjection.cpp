// Copyright 2025, Inland-LOAM 
// All rights reserved.
//
// This file is part of the Inland-LOAM project, which is a modified version of
// the LeGO-LOAM implementation. The original license and copyright details
// are retained below.
//
// ----------------------------------------------------------------------------
//
// The following is the original copyright notice and license statement.
//
// ----------------------------------------------------------------------------


// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <boost/circular_buffer.hpp>
#include <algorithm>
#include "cloud_msgs/msg/plane.hpp"
#include "imageProjection.h"

typedef pcl::PointXYZI PointType;
const std::string PARAM_VERTICAL_SCANS = "laser.num_vertical_scans";
const std::string PARAM_HORIZONTAL_SCANS = "laser.num_horizontal_scans";
const std::string PARAM_ANGLE_BOTTOM = "laser.vertical_angle_bottom";
const std::string PARAM_ANGLE_TOP = "laser.vertical_angle_top";
const std::string PARAM_GROUND_INDEX = "laser.ground_scan_index";
const std::string PARAM_SENSOR_ANGLE = "laser.sensor_mount_angle";
const std::string PARAM_SEGMENT_THETA = "image_projection.segment_theta";
const std::string PARAM_SEGMENT_POINT = "image_projection.segment_valid_point_num";
const std::string PARAM_SEGMENT_LINE = "image_projection.segment_valid_line_num";

ImageProjection::ImageProjection(const std::string &name, Channel<ProjectionOut>& output_channel)
    : Node(name),  _output_channel(output_channel)
{
  // Buffer short processing/rendering stalls during normal-speed playback.
  // A depth of one discarded scans before projection could process them.
  const auto input_queue_size = std::max<int64_t>(1, this->declare_parameter<int64_t>(
      "image_projection.input_queue_size", 32));
  _sub_laser_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", rclcpp::QoS(input_queue_size),
      std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1));

  _pub_full_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_projected", 1);
  _pub_full_info_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/full_cloud_info", 1);
  _pub_ground_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_cloud", 1);
  _pub_segmented_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud", 1);
  _pub_segmented_cloud_pure = this->create_publisher<sensor_msgs::msg::PointCloud2>("/segmented_cloud_pure", 1);
  _pub_segmented_cloud_info = this->create_publisher<cloud_msgs::msg::CloudInfo>("/segmented_cloud_info", 1);
  _pub_outlier_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/outlier_cloud", 1);
  _pub_plane = this->create_publisher<cloud_msgs::msg::Plane>("/plane_info", 1);

  float vertical_angle_top;
  // Declare parameters
  this->declare_parameter(PARAM_VERTICAL_SCANS,_vertical_scans);
  this->declare_parameter(PARAM_HORIZONTAL_SCANS,_horizontal_scans);
  this->declare_parameter(PARAM_ANGLE_BOTTOM,_ang_bottom);
  this->declare_parameter(PARAM_ANGLE_TOP,vertical_angle_top);
  this->declare_parameter(PARAM_GROUND_INDEX,_ground_scan_index);
  this->declare_parameter(PARAM_SENSOR_ANGLE,_sensor_mount_angle);
  this->declare_parameter(PARAM_SEGMENT_THETA,_segment_theta);
  this->declare_parameter(PARAM_SEGMENT_POINT,_segment_valid_point_num);
  this->declare_parameter(PARAM_SEGMENT_LINE,_segment_valid_line_num);


  // Read parameters
  if (!this->get_parameter(PARAM_VERTICAL_SCANS, _vertical_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_VERTICAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_HORIZONTAL_SCANS, _horizontal_scans)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_HORIZONTAL_SCANS.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_BOTTOM, _ang_bottom)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_BOTTOM.c_str());
  }
  if (!this->get_parameter(PARAM_ANGLE_TOP, vertical_angle_top)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_ANGLE_TOP.c_str());
  }
  if (!this->get_parameter(PARAM_GROUND_INDEX, _ground_scan_index)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s found", PARAM_GROUND_INDEX.c_str());
  }
  if (!this->get_parameter(PARAM_SENSOR_ANGLE, _sensor_mount_angle)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SENSOR_ANGLE.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_THETA, _segment_theta)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_THETA.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_POINT, _segment_valid_point_num)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_POINT.c_str());
  }
  if (!this->get_parameter(PARAM_SEGMENT_LINE, _segment_valid_line_num)) {
    RCLCPP_WARN(this->get_logger(), "Parameter %s not found", PARAM_SEGMENT_LINE.c_str());
  }

  _ang_resolution_X = (M_PI*2) / (_horizontal_scans);
  _ang_resolution_Y = DEG_TO_RAD*(vertical_angle_top - _ang_bottom) / float(_vertical_scans-1);
  _ang_bottom = -( _ang_bottom - 0.1) * DEG_TO_RAD;
  _segment_theta *= DEG_TO_RAD;
  _sensor_mount_angle *= DEG_TO_RAD;

  const size_t cloud_size = _vertical_scans * _horizontal_scans;

  _laser_cloud_in.reset(new pcl::PointCloud<PointType>());
  _full_cloud.reset(new pcl::PointCloud<PointType>());
  _full_info_cloud.reset(new pcl::PointCloud<PointType>());

  _ground_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud.reset(new pcl::PointCloud<PointType>());
  _segmented_cloud_pure.reset(new pcl::PointCloud<PointType>());
  _outlier_cloud.reset(new pcl::PointCloud<PointType>());

  _full_cloud->points.resize(cloud_size);
  _full_info_cloud->points.resize(cloud_size);
  _full_cloud_raw_indices.assign(cloud_size, -1);

  // Initialize laser angles
_laser_angles = {
      -55.0f, -52.0f, -49.0f, -46.0f, -43.0f, -40.0f, -37.0f, -34.0f, -31.0f, -28.0f,
      -25.0f, -22.0f, -19.0f, -16.0f, -13.0f, -10.0f, -8.0f, -6.67f, -5.33f, -4.0f,
      -2.67f, -1.33f, 0.0f, 1.33f, 2.67f, 4.0f, 5.5f, 7.0f, 9.0f, 11.0f, 13.0f, 15.0f
  };
_vertical_scans = _laser_angles.size();  // Should be 32

}

void ImageProjection::resetParameters() {
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  PointType nanPoint;
  nanPoint.x = std::numeric_limits<float>::quiet_NaN();
  nanPoint.y = std::numeric_limits<float>::quiet_NaN();
  nanPoint.z = std::numeric_limits<float>::quiet_NaN();

  _laser_cloud_in->clear();
  _laser_cloud_in_raw_indices.clear();
  _ground_cloud->clear();
  _segmented_cloud->clear();
  _segmented_cloud_pure->clear();
  _outlier_cloud->clear();
  _segmented_raw_indices.clear();
  _outlier_raw_indices.clear();

  _range_mat.resize(_vertical_scans, _horizontal_scans);
  _ground_mat.resize(_vertical_scans, _horizontal_scans);
  _label_mat.resize(_vertical_scans, _horizontal_scans);

  _range_mat.fill(FLT_MAX);
  _ground_mat.setZero();
  _label_mat.setZero();

  _label_count = 1;

  std::fill(_full_cloud->points.begin(), _full_cloud->points.end(), nanPoint);
  std::fill(_full_info_cloud->points.begin(), _full_info_cloud->points.end(),
            nanPoint);
  std::fill(_full_cloud_raw_indices.begin(), _full_cloud_raw_indices.end(), -1);

  _seg_msg.start_ring_index.assign(_vertical_scans, 0);
  _seg_msg.end_ring_index.assign(_vertical_scans, 0);

  _seg_msg.segmented_cloud_ground_flag.assign(cloud_size, false);
  _seg_msg.segmented_cloud_col_ind.assign(cloud_size, 0);
  _seg_msg.segmented_cloud_range.assign(cloud_size, 0);
}

void ImageProjection::cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg) {
  // Reset parameters
  resetParameters();

  pcl::PointCloud<PointType>::Ptr rawCloud(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*laserCloudMsg, *rawCloud);
  _seg_msg.header = laserCloudMsg->header;

  // Limit the XYZ range of the point cloud
  const float min_x = -70.0f;  
  const float max_x = 70.0f;  
  const float min_y = -70.0f;  
  const float max_y = 70.0f;  

  // Traverse the raw frame once so source indices keep the original bag order.
  uint32_t semantic_source_index = 0;
  for (size_t i = 0; i < rawCloud->points.size(); ++i) {
    const auto& point = rawCloud->points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const uint32_t source_index = semantic_source_index++;
    if (point.x >= min_x && point.x <= max_x &&
        point.y >= min_y && point.y <= max_y ) {
      _laser_cloud_in->points.push_back(point);
      _laser_cloud_in_raw_indices.push_back(source_index);
    }
  }

  if (_laser_cloud_in->points.empty()) {
    ++_frame_index;
    return;
  }

  findStartEndAngle();

  projectPointCloud();

  groundRemoval();

  cloudSegmentation();

  publishClouds();
}


void ImageProjection::projectPointCloud() {
  const size_t cloudSize = _laser_cloud_in->points.size();

  for (size_t i = 0; i < cloudSize; ++i) {
    PointType thisPoint = _laser_cloud_in->points[i];


    float range = sqrt(thisPoint.x * thisPoint.x +
                       thisPoint.y * thisPoint.y +
                       thisPoint.z * thisPoint.z);

    if (range < 0.1) {
      continue; 
    }


    float verticalAngle = std::asin(thisPoint.z / range);


    int rowIdn = -1;
    float minAngleDiff = std::numeric_limits<float>::max();
    for (int j = 0; j < _vertical_scans; ++j) {
      float angleDiff = std::abs(verticalAngle - DEG_TO_RAD * _laser_angles[j]);
      if (angleDiff < minAngleDiff) {
        minAngleDiff = angleDiff;
        rowIdn = j;
      }
    }


    if (rowIdn < 0 || rowIdn >= _vertical_scans || minAngleDiff > DEG_TO_RAD * 1.5) {
      continue;
    }


    float horizonAngle = std::atan2(thisPoint.y, thisPoint.x);
    int columnIdn = -round((horizonAngle - M_PI_2) / _ang_resolution_X) + _horizontal_scans / 2;


    if (columnIdn >= _horizontal_scans) {
      columnIdn -= _horizontal_scans;
    }
    if (columnIdn < 0 || columnIdn >= _horizontal_scans) {
      continue;
    }


    if (_range_mat(rowIdn, columnIdn) == FLT_MAX || range < _range_mat(rowIdn, columnIdn)) {
      _range_mat(rowIdn, columnIdn) = range;

      thisPoint.intensity = static_cast<float>(rowIdn) + static_cast<float>(columnIdn) / 10000.0;

      size_t index = columnIdn + rowIdn * _horizontal_scans;
      _full_cloud->points[index] = thisPoint;
      _full_info_cloud->points[index] = thisPoint;
      _full_info_cloud->points[index].intensity = range;
      _full_cloud_raw_indices[index] = static_cast<int32_t>(rawIndexAt(_laser_cloud_in_raw_indices, i));
    }

  }


}


void ImageProjection::findStartEndAngle() {
  // start and end orientation of this cloud
  auto point = _laser_cloud_in->points.front();
  _seg_msg.start_orientation = -std::atan2(point.y, point.x);

  point = _laser_cloud_in->points.back();
  _seg_msg.end_orientation = -std::atan2(point.y, point.x) + 2 * M_PI;

  if (_seg_msg.end_orientation - _seg_msg.start_orientation > 3 * M_PI) {
    _seg_msg.end_orientation -= 2 * M_PI;
  } else if (_seg_msg.end_orientation - _seg_msg.start_orientation < M_PI) {
    _seg_msg.end_orientation += 2 * M_PI;
  }
  _seg_msg.orientation_diff =
      _seg_msg.end_orientation - _seg_msg.start_orientation;
}


void ImageProjection::groundRemoval() {
    const float zThreshold = -1.4; 
    const float planeOffset = -0.1; 
    const float planeOffset_lower = 0.2;


    std::vector<PointType, Eigen::aligned_allocator<PointType>> potential_ground_points;
    std::vector<std::pair<int, int>> ground_indices; 

    for (int j = 0; j < _horizontal_scans; ++j) {
        for (int i = 0; i < _vertical_scans; ++i) {
            int lowerInd = j + i * _horizontal_scans;


            if (_full_cloud->points[lowerInd].intensity == -1 )

            {
                _ground_mat(i, j) = -1;
                continue;
            }


            if (_full_cloud->points[lowerInd].z < zThreshold )
            // && _full_cloud->points[upperInd].z < zThreshold）
             {

                potential_ground_points.push_back(_full_cloud->points[lowerInd]);
                ground_indices.emplace_back(i, j);

            }
        }
    }

    // 2. 使用RANSAC拟合平面
    if (potential_ground_points.size() < 50) { 
        RCLCPP_WARN(this->get_logger(), "Not enough ground points for RANSAC plane fitting.");
        return;
    }

    pcl::PointCloud<PointType>::Ptr ground_cloud_pcl(new pcl::PointCloud<PointType>());
    ground_cloud_pcl->points = potential_ground_points;

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    pcl::SACSegmentation<PointType> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.1); 

    seg.setInputCloud(ground_cloud_pcl);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        RCLCPP_WARN(this->get_logger(), "RANSAC could not find a planar model.");
        return;
    }


    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    Eigen::Vector3f normal(a, b, c);
    normal.normalize(); 

    float d_offset = d + planeOffset; 
    float d_lower_offset = d + planeOffset_lower;

        // Fill-in planar message
    _plane_msgs.header.stamp = _seg_msg.header.stamp; 
    _plane_msgs.header.frame_id = "base_link";
    _plane_msgs.nx = normal.x();
    _plane_msgs.ny = normal.y();
    _plane_msgs.nz = normal.z();
    _plane_msgs.d = d;


    _ground_mat.setZero(); 
    _ground_cloud->clear(); 

    for (int i = 0; i < _ground_scan_index; ++i) {
        for (int j = 0; j < _horizontal_scans; ++j) {
            int cloudIdx = j + i * _horizontal_scans;
            if (_full_cloud->points[cloudIdx].intensity == -1) continue;


            float distance = a * _full_cloud->points[cloudIdx].x +
                              b * _full_cloud->points[cloudIdx].y +
                              c * _full_cloud->points[cloudIdx].z + d_offset;

            float distanceLower = a * _full_cloud->points[cloudIdx].x +
                                  b * _full_cloud->points[cloudIdx].y +
                                  c * _full_cloud->points[cloudIdx].z + d_lower_offset;


            if (distance < 0 && distanceLower >= 0) {
                _ground_mat(i, j) = 1;
            }
        }
    }

    for (int i = 0; i < _vertical_scans; ++i) {
      for (int j = 0; j < _horizontal_scans; ++j) {
          if (_ground_mat(i, j) == 1 || _range_mat(i, j) == FLT_MAX) {
              _label_mat(i, j) = -1;
          }
        }
    }

    for (int i = 0; i < _ground_scan_index; ++i) {
      for (int j = 0; j < _horizontal_scans; ++j) {
            if (_ground_mat(i, j) == 1) {
                PointType groundPoint = _full_cloud->points[j + i * _horizontal_scans];
                groundPoint.intensity = 0; 
                _ground_cloud->push_back(groundPoint);  
            }
        }
    }

    _pub_plane->publish(_plane_msgs);
}


void ImageProjection::cloudSegmentation() {
  // segmentation process
  for (int i = 0; i < _vertical_scans; ++i)
    for (int j = 0; j < _horizontal_scans; ++j)
      if (_label_mat(i, j) == 0) labelComponents(i, j);
  

  int sizeOfSegCloud = 0;
  // extract segmented cloud for lidar odometry
  for (int i = 0; i < _vertical_scans; ++i) {
    _seg_msg.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 || _ground_mat(i, j) == 1) {
        // outliers that will not be used for optimization (always continue)
        if (_label_mat(i, j) == 999999) {
          if (i > _ground_scan_index && j % 5 == 0) {
            const int cloud_index = j + i * _horizontal_scans;
            _outlier_cloud->push_back(_full_cloud->points[cloud_index]);
            const int32_t raw_index = _full_cloud_raw_indices[cloud_index];
            _outlier_raw_indices.push_back(raw_index >= 0 ? static_cast<uint32_t>(raw_index) : INVALID_RAW_POINT_INDEX);
            continue;
          } else {
            continue;
          }
        }
        // majority of ground points are skipped
        if (_ground_mat(i, j) == 1) {
          if (j % 5 != 0 && j > 5 && j < _horizontal_scans - 5) continue;
        }
        _seg_msg.segmented_cloud_ground_flag[sizeOfSegCloud] =
            (_ground_mat(i, j) == 1);

        _seg_msg.segmented_cloud_col_ind[sizeOfSegCloud] = j;
        // save range info
        _seg_msg.segmented_cloud_range[sizeOfSegCloud] =
            _range_mat(i, j);

        const int cloud_index = j + i * _horizontal_scans;
        _segmented_cloud->push_back(_full_cloud->points[cloud_index]);
        const int32_t raw_index = _full_cloud_raw_indices[cloud_index];
        _segmented_raw_indices.push_back(raw_index >= 0 ? static_cast<uint32_t>(raw_index) : INVALID_RAW_POINT_INDEX);

        ++sizeOfSegCloud;
      }
    }

    _seg_msg.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
  }

  for (int i = 0; i < _vertical_scans; ++i) {
    for (int j = 0; j < _horizontal_scans; ++j) {
      if (_label_mat(i, j) > 0 && _label_mat(i, j) != 999999) {
        _segmented_cloud_pure->push_back(
            _full_cloud->points[j + i * _horizontal_scans]);
        _segmented_cloud_pure->points.back().intensity =
            _label_mat(i, j);
      }
    }
  }
}

void ImageProjection::labelComponents(int row, int col) {

  const float segmentThetaThreshold = tan(_segment_theta);

  std::vector<bool> lineCountFlag(_vertical_scans, false);
  const size_t cloud_size = _vertical_scans * _horizontal_scans;
  using Coord2D = Eigen::Vector2i;
  boost::circular_buffer<Coord2D> queue(cloud_size);
  boost::circular_buffer<Coord2D> all_pushed(cloud_size);

  queue.push_back({ row,col } );
  all_pushed.push_back({ row,col } );

  const Coord2D neighborIterator[4] = {
      {0, -1}, {-1, 0}, {1, 0}, {0, 1}};

  while (queue.size() > 0) {

    Coord2D fromInd = queue.front();
    queue.pop_front();


    _label_mat(fromInd.x(), fromInd.y()) = _label_count;

    for (const auto& iter : neighborIterator) {
      int thisIndX = fromInd.x() + iter.x();
      int thisIndY = fromInd.y() + iter.y();

      if (thisIndX < 0 || thisIndX >= _vertical_scans){
        continue;
      }
      if (thisIndY < 0){
        thisIndY = _horizontal_scans - 1;
      }
      if (thisIndY >= _horizontal_scans){
        thisIndY = 0;
      }
      if (_label_mat(thisIndX, thisIndY) != 0){
        continue;
      }

      float d1 = std::max(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));
      float d2 = std::min(_range_mat(fromInd.x(), fromInd.y()),
                    _range_mat(thisIndX, thisIndY));

      float alpha;
      if (iter.x() == 0) {

          alpha = _ang_resolution_X;
      } else {

          float angleFrom = DEG_TO_RAD * _laser_angles[fromInd.x()];
          float angleTo = DEG_TO_RAD * _laser_angles[thisIndX];
          alpha = std::abs(angleFrom - angleTo)+0.5; // Calculate the actual vertical angular resolution
      }

      float tang = (d2 * sin(alpha) / (d1 - d2 * cos(alpha)));

      if (tang > segmentThetaThreshold) {
        queue.push_back( {thisIndX, thisIndY } );

        _label_mat(thisIndX, thisIndY) = _label_count;
        lineCountFlag[thisIndX] = true;

        all_pushed.push_back(  {thisIndX, thisIndY } );
      }
    }
  }

  bool feasibleSegment = false;
  if (all_pushed.size() >= 30){
    feasibleSegment = true;
  }
  else if (static_cast<int>(all_pushed.size()) >= _segment_valid_point_num) {
    int lineCount = 0;
    for (int i = 0; i < _vertical_scans; ++i) {
      if (lineCountFlag[i] == true) ++lineCount;
    }
    if (lineCount >= _segment_valid_line_num) feasibleSegment = true;
  }

  if (feasibleSegment == true) {
    ++_label_count;
  } else {  
    for (size_t i = 0; i < all_pushed.size(); ++i) {
      _label_mat(all_pushed[i].x(), all_pushed[i].y()) = 999999;
    }
  }


}

void ImageProjection::publishClouds() {

  sensor_msgs::msg::PointCloud2 temp;
  temp.header.stamp = _seg_msg.header.stamp;
  temp.header.frame_id = "base_link";

  auto PublishCloud = [](rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, sensor_msgs::msg::PointCloud2& temp,
                          const pcl::PointCloud<PointType>::Ptr& cloud) {
    if (pub->get_subscription_count() != 0) {
       sensor_msgs::msg::PointCloud2 _temp;
      pcl::toROSMsg(*cloud, _temp);
      _temp.header.stamp = temp.header.stamp;
      _temp.header.frame_id = temp.header.frame_id;
      pub->publish(_temp);
    }
  };

  PublishCloud(_pub_outlier_cloud, temp, _outlier_cloud);
  PublishCloud(_pub_segmented_cloud, temp, _segmented_cloud);
  PublishCloud(_pub_full_cloud, temp, _full_cloud);
  PublishCloud(_pub_ground_cloud, temp, _ground_cloud);
  PublishCloud(_pub_segmented_cloud_pure, temp, _segmented_cloud_pure);
  PublishCloud(_pub_full_info_cloud, temp, _full_info_cloud);

  if (_pub_segmented_cloud_info->get_subscription_count() != 0) {
    _pub_segmented_cloud_info->publish(_seg_msg);
  }

  // 3. 发布 plane_msgs
  if (_pub_plane->get_subscription_count() != 0) {
    _plane_msgs.header.stamp = _seg_msg.header.stamp;
    _plane_msgs.header.frame_id = "base_link";
  }

  //--------------------
  ProjectionOut out;
  out.outlier_cloud.reset(new pcl::PointCloud<PointType>());
  out.segmented_cloud.reset(new pcl::PointCloud<PointType>());

  std::swap( out.seg_msg, _seg_msg);
  std::swap(out.outlier_cloud, _outlier_cloud);
  std::swap(out.segmented_cloud, _segmented_cloud);
  std::swap(out.outlier_raw_indices, _outlier_raw_indices);
  std::swap(out.segmented_raw_indices, _segmented_raw_indices);

  out.plane_msg = _plane_msgs;
  out.frame_index = _frame_index;
  out.stamp = rclcpp::Time(temp.header.stamp).seconds();
  // RCLCPP_INFO(this->get_logger(), "ImageProjection PlaneMsg: stamp=%u, frame_id=%s, nx=%f, ny=%f, nz=%f, d=%f",
  //           _plane_msgs.header.stamp.sec,
  //           _plane_msgs.header.frame_id.c_str(),
  //           _plane_msgs.nx,
  //           _plane_msgs.ny,
  //           _plane_msgs.nz,
  //           _plane_msgs.d);

  _output_channel.send( std::move(out) );
  ++_frame_index;

}
