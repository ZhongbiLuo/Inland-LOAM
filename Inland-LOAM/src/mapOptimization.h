#ifndef MAPOPTIMIZATION_H
#define MAPOPTIMIZATION_H

#include "lego_loam/utility.h"
#include "lego_loam/channel.h"
#include "lego_loam/nanoflann_pcl.h"
#include <pcl/io/pcd_io.h>



#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/RangeFactor.h>         
#include <gtsam/base/numericalDerivative.h> 

#include <gtsam/geometry/Point3.h>
#include <gtsam/linear/NoiseModel.h>
#include <boost/optional.hpp> 
#include <boost/bind/bind.hpp> 

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <unordered_map>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

inline gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint) {
  return gtsam::Pose3(
      gtsam::Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll),
                          double(thisPoint.pitch)),
      gtsam::Point3(double(thisPoint.z), double(thisPoint.x),
                    double(thisPoint.y)));
}

inline Eigen::Affine3f pclPointToAffine3fCameraToLidar(
    PointTypePose thisPoint) {
  return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y,
                                thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
}

class WaterPlaneFactor : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::OrientedPlane3> {
 private:
  gtsam::Vector4 measured_plane_;
  using Base = gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::OrientedPlane3>;

 public:
  WaterPlaneFactor(const gtsam::Vector4& measured_plane,
                   const gtsam::SharedNoiseModel& noise_model,
                   gtsam::Key pose_key,
                   gtsam::Key plane_key)
      : Base(noise_model, pose_key, plane_key), measured_plane_(measured_plane) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      const gtsam::OrientedPlane3& plane,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    const auto unit_noise = gtsam::noiseModel::Unit::Create(3);
    gtsam::OrientedPlane3Factor gaussian_factor(measured_plane_, unit_noise, 0, 1);
    return gaussian_factor.evaluateError(pose, plane, H1, H2);
  }
};

class MapOptimization : public rclcpp::Node {

 public:
  MapOptimization(const std::string &name, Channel<AssociationOut> &input_channel);

  ~MapOptimization();

  void run();

 private:
  
  
    // 记录角点 / 面点 / 外点出现次数
    struct CategoryCount {
        int corner   = 0;
        int surf     = 0;
        int outlier  = 0;
    };

    // 自定义 octomap::OcTreeKey 的哈希
    struct OcTreeKeyHash {
        std::size_t operator()(const octomap::OcTreeKey& key) const {
            // 简易示例哈希, 也可用 boost::hash_combine
            std::size_t h1 = std::hash<unsigned int>()(key.k[0]);
            std::size_t h2 = std::hash<unsigned int>()(key.k[1]);
            std::size_t h3 = std::hash<unsigned int>()(key.k[2]);

            // 组合
            std::size_t seed = 0;
            seed ^= h1 + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            seed ^= h2 + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            seed ^= h3 + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    // 相等判断
    struct OcTreeKeyEq {
        bool operator()(const octomap::OcTreeKey& a, const octomap::OcTreeKey& b) const {
            return (a.k[0] == b.k[0]) && (a.k[1] == b.k[1]) && (a.k[2] == b.k[2]);
        }
    };
  
  
  struct PlaneParams {
    double nx, ny, nz, d; // 法向量 (nx, ny, nz) 和距离 d
  };
  
  static const gtsam::Key earth_key;
  static constexpr double R_e = 6371000.0;        // 地球半径 (m)

  std::shared_ptr<octomap::ColorOcTree> m_octree_;  // 1) 增加一个 ColorOcTree 指针或智能指针

  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_octomap_;   // 2) 增加一个发布器，用来发布 Octomap
  gtsam::NonlinearFactorGraph gtSAMgraph;
  gtsam::Values initialEstimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isamCurrentEstimate;

  bool _loop_closure_enabled;
  bool _water_plane_huber_enabled = true;
  double _water_plane_huber_delta = 1.345;

  float _surrounding_keyframe_search_radius;
  int   _surrounding_keyframe_search_num;
  float _history_keyframe_search_radius;
  int   _history_keyframe_search_num;
  float _history_keyframe_fitness_score;
  float _global_map_visualization_search_radius;

  Channel<AssociationOut>& _input_channel;
  std::thread _run_thread;

  Channel<bool> _publish_global_signal;
  std::thread _publish_global_thread;
  void publishGlobalMapThread();

  Channel<bool> _loop_closure_signal;
  std::thread _loop_closure_thread;
  void loopClosureThread();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
  //  Rivz Additional
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalMapNoOutliers;
  void publishGlobalMapNoOutliers();

  cloud_msgs::msg::Plane planeParameters;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::TransformStamped aftMappedTrans;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;

  std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> groundCloudKeyFrames;
  std::vector<pcl::PointCloud<PointType>::Ptr> newSurfKeyFrame;
  std::vector<pcl::PointCloud<PointType>::Ptr> newCornerKeyFrame;
  std::vector<std::vector<uint32_t>> groundCloudKeyFrameRawIndices;
  std::vector<std::vector<uint32_t>> newSurfKeyFrameRawIndices;
  std::vector<std::vector<uint32_t>> newCornerKeyFrameRawIndices;

  std::deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
  int latestFrameID;

  std::vector<int> surroundingExistingKeyPosesID;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
  std::deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

  PointType previousRobotPosPoint;
  PointType currentRobotPosPoint;

  pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

  pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
  pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      newlaserCloudCornerLast;  // new corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      newlaserCloudSurfLast;  // new surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudGroundLast;  // ground feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudCornerLastDS;  // downsampled corner featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfLastDS;  // downsampled surf featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudGroundLastDS;  // downsampled ground featuer set from
    // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      newlaserCloudCornerLastDS;  // new downsampled corner featuer set from
      // odoOptimization
  pcl::PointCloud<PointType>::Ptr
      newlaserCloudSurfLastDS;  // new downsampled surf featuer set from


  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLast;  // corner feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudOutlierLastDS;  // corner feature set from odoOptimization
  std::vector<uint32_t> laserCloudCornerLastRawIndices;
  std::vector<uint32_t> laserCloudSurfLastRawIndices;
  std::vector<uint32_t> laserCloudGroundLastRawIndices;
  std::vector<uint32_t> laserCloudOutlierLastRawIndices;
  std::vector<uint32_t> newlaserCloudCornerLastRawIndices;
  std::vector<uint32_t> newlaserCloudSurfLastRawIndices;
  std::vector<uint32_t> laserCloudCornerLastDSRawIndices;
  std::vector<uint32_t> laserCloudSurfLastDSRawIndices;
  std::vector<uint32_t> laserCloudGroundLastDSRawIndices;
  std::vector<uint32_t> laserCloudOutlierLastDSRawIndices;
  std::vector<uint32_t> newlaserCloudCornerLastDSRawIndices;
  std::vector<uint32_t> newlaserCloudSurfLastDSRawIndices;
  uint32_t currentFrameIndex = 0;
  double currentFrameStamp = 0.0;

  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLast;  // surf feature set from odoOptimization
  pcl::PointCloud<PointType>::Ptr
      laserCloudSurfTotalLastDS;  // downsampled corner featuer set from
      // odoOptimization

  pcl::PointCloud<PointType>::Ptr laserCloudOri;
  pcl::PointCloud<PointType>::Ptr coeffSel;

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

  std::vector<Eigen::Vector4f> optimizedPlanes;


  nanoflann::KdTreeFLANN<PointType> kdtreeCornerFromMap;
  nanoflann::KdTreeFLANN<PointType> kdtreeSurfFromMap;

  nanoflann::KdTreeFLANN<PointType> kdtreeSurroundingKeyPoses;
  nanoflann::KdTreeFLANN<PointType> kdtreeHistoryKeyPoses;

  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr nearHistorySurfKeyFrameCloudDS;

  pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloud;
  pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

  nanoflann::KdTreeFLANN<PointType> kdtreeGlobalMap;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
  pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
  pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;
  

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::VoxelGrid<PointType> downSizeFilterCorner;
  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterOutlier;
  pcl::VoxelGrid<PointType>
      downSizeFilterHistoryKeyFrames;  // for histor key frames of loop closure
  pcl::VoxelGrid<PointType>
      downSizeFilterSurroundingKeyPoses;  // for surrounding key poses of
      // scan-to-map optimization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyPoses;  // for global map visualization
  pcl::VoxelGrid<PointType>
      downSizeFilterGlobalMapKeyFrames;  // for global map visualization

  rclcpp::Time timeLaserOdometry;

  float transformLast[6];
  float transformSum[6];
  float transformIncre[6];
  float transformTobeMapped[6];
  float transformBefMapped[6];
  float transformAftMapped[6];

  std::mutex mtx;

  PointType pointOri, pointSel, pointProj, coeff;

  Eigen::Matrix<float, 5, 3> matA0;
  Eigen::Matrix<float, 5, 1> matB0;
  Eigen::Vector3f matX0;

  Eigen::Matrix3f matA1;
  Eigen::Matrix<float, 1, 3> matD1;
  Eigen::Matrix3f matV1;

  Eigen::Matrix<float, 6, 6> matP;


  bool isDegenerate;

  int laserCloudCornerFromMapDSNum;
  int laserCloudSurfFromMapDSNum;
  int laserCloudCornerLastDSNum;
  int newlaserCloudSurfLastDSNum;
  int newlaserCloudCornerLastDSNum;
  int laserCloudGroundLastDSNum;
  int laserCloudSurfLastDSNum;
  int laserCloudOutlierLastDSNum;
  int laserCloudSurfTotalLastDSNum;

  bool potentialLoopFlag;
  int closestHistoryFrameID;
  int latestFrameIDLoopCloure;

  bool aLoopIsClosed;

  float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
  float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

  // 平面坐标系 -> 世界坐标系 octotree 转换
  Eigen::Affine3f planeRotation;
  Eigen::Affine3f planeTranslation;
  Eigen::Affine3f planeTransform;
  bool hasPlaneTransform = false;
  gtsam::OrientedPlane3 lastOptimizedPlane;  // 上一次优化后的平面参数
  bool isPlaneStable = false; // 记录当前帧平面是否稳定
  int stableCount = 0;  // 记录连续稳定的次数
  int stableThreshold = 2; // 设定平面误差稳定的最小次数

     // 添加成员变量
  std::string map_save_path_;
  bool save_final_map_;
  bool offline_export_enabled_ = false;
  std::string offline_export_path_;
  int offline_export_max_keyframes_ = 0;

    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesNoOutliers;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesNoOutliersDS;


 private:


    
  // 添加成员函数====================
    template<class PointT>
    void saveCloudToPCD(const typename pcl::PointCloud<PointT>::Ptr& cloud, const std::string& filename);

    // 添加类型别名
    using PointCloudRGBPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;
    using PointCloudIPtr = pcl::PointCloud<PointType>::Ptr;
  void saveVoxelMapToPCD(const std::string& filename);
  void saveCurrentGlobalMap();
  void exportOfflineKeyFrame(int keyframe_id,
                             const PointTypePose& pose,
                             const pcl::PointCloud<PointType>::Ptr& new_corner,
                             const std::vector<uint32_t>& new_corner_indices,
                             const pcl::PointCloud<PointType>::Ptr& new_surf,
                             const std::vector<uint32_t>& new_surf_indices,
                             const pcl::PointCloud<PointType>::Ptr& ground,
                             const std::vector<uint32_t>& ground_indices);
//============================


  void allocateMemory();
  void transformAssociateToMap();
  void transformUpdate();
  void updatePointAssociateToMapSinCos();
  void pointAssociateToMap(PointType const *const pi, PointType *const po);
  void updateTransformPointCloudSinCos(PointTypePose *tIn) ;

  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn);

  pcl::PointCloud<PointType>::Ptr transformPointCloud(
      pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn);
  
  Eigen::Vector4f transformPlaneToMap(const Eigen::Vector4f& localPlane, const Eigen::Matrix4f& transform);

  void publishTF();
  void publishKeyPosesAndFrames();
  void publishGlobalMap();

  bool detectLoopClosure();
  void performLoopClosure();

  void extractSurroundingKeyFrames();
  void downsampleCurrentScan();
  void cornerOptimization();
  void surfOptimization();

  bool LMOptimization(int iterCount);
  void scan2MapOptimization();

  void saveKeyFramesAndFactor();
  void correctPoses();
  void insertAllKeyFramesIntoOctoMap(); 
  void calculatePlaneTransform(const PlaneParams& plane);

  void clearCloud();
};

#endif // MAPOPTIMIZATION_H
