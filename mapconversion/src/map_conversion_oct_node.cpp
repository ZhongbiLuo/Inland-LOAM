#include "MapConverter.hh"
#include "octomap/octomap_types.h"
#include "octomap_msgs/msg/octomap.hpp"
#include <cstddef>
#include <cstdio>
#include <mapconversion_msgs/msg/height_map.hpp>
#include <mapconversion_msgs/msg/slope_map.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.h>
#include <octomap/ColorOcTree.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/color_rgba.hpp>  
#include <queue>
#include <limits> 
#include <chrono> 
#include <fstream>   
#include "std_msgs/msg/float64.hpp" 
#include <cmath> 

using namespace std;

class MapToMap : public rclcpp::Node {
private:

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr subOctMap;

  // pub
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pubMapUGV,
      pubMapUAV, pubMapFloor, pubMapCeiling, pubMapSlopeVis, pubMapGeometric, pubMapGeometric_2;
  rclcpp::Publisher<mapconversion_msgs::msg::HeightMap>::SharedPtr pubHeightMap;
  rclcpp::Publisher<mapconversion_msgs::msg::SlopeMap>::SharedPtr pubMapSlope;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubBridgeGapMarkers;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pubStructuralMapMarker;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_time_ms_; //****

  // msg
  nav_msgs::msg::OccupancyGrid mapMsg;
  mapconversion_msgs::msg::HeightMap heightMsg;
  mapconversion_msgs::msg::SlopeMap slopeMsg;


  MapConverter *MC;
  octomap::ColorOcTree *OcMap;

  double resolution;
  double slopeMax;
  int slopeEstimationSize;
  double minimumZ;
  int minimumOccupancy;
  string mapFrame;
  double mapZpos;
  double ssmAlpha;
  int update_count_ = 0; 

  

  visualization_msgs::msg::Marker createTextMarker(
    int id, double x, double y, double z, const std::string& text,
    const std_msgs::msg::ColorRGBA& color, double scale = 3)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = mapFrame;
    marker.header.stamp = this->now();  
    marker.ns = "bridge_gap_text";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;

    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    marker.text = text;
    marker.scale.z = scale;
    marker.color = color;
    marker.lifetime = rclcpp::Duration::from_seconds(1.5);  

    return marker;
  }

  double publishCallbackDuration(
    const std::chrono::steady_clock::time_point& t_start,
    const char* status)
  {
    const auto t_end = std::chrono::steady_clock::now();
    const double elapsed_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();

    RCLCPP_INFO(
      this->get_logger(),
      "mapCallback total (%s): %.3f ms",
      status,
      elapsed_ms);

    if (pub_time_ms_) {
      std_msgs::msg::Float64 tmsg;
      tmsg.data = elapsed_ms;
      pub_time_ms_->publish(tmsg);
    }

    return elapsed_ms;
  }

public:
  MapToMap() : Node("map_conversion") {
    slopeMax = this->declare_parameter("max_slope_ugv", INFINITY);
    slopeEstimationSize = this->declare_parameter("slope_estimation_size", 1);
    slopeEstimationSize = max(slopeEstimationSize, 1);
    minimumZ = this->declare_parameter("minimum_z", 1.0);
    minimumOccupancy = this->declare_parameter("minimum_occupancy", 10);
    mapFrame = this->declare_parameter("map_frame", string("ssm"));
    mapZpos = this->declare_parameter("map_position_z", 0.0);
    ssmAlpha = min(1.0, max(0.0, this->declare_parameter("ssm_alpha", 0.6)));

    pub_time_ms_ = this->create_publisher<std_msgs::msg::Float64>(
                    "/map_callback_dur_ms", 10);


    subOctMap = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_out", 1,
        std::bind(&MapToMap::mapCallback, this, std::placeholders::_1));

    pubMapUGV =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapUGV", 5);
    pubMapUAV =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/mapUAV", 5);
    pubMapFloor = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_floor_map", 5);
    pubMapCeiling = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_ceiling_map", 5);
    pubHeightMap = this->create_publisher<mapconversion_msgs::msg::HeightMap>(
        "/heightMap", 5);
    pubMapSlopeVis = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/visualization_slope_map", 5);
    pubMapSlope = this->create_publisher<mapconversion_msgs::msg::SlopeMap>(
        "/slopeMap", 5);
    pubMapGeometric = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/geometric_map", 5);
    const auto latched_visualization_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    pubMapGeometric_2 = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/geometric_map_2", latched_visualization_qos);
    pubStructuralMapMarker =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/structural_semantic_map", latched_visualization_qos);
    pubBridgeGapMarkers = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/bridge_gap_texts", 5);
    OcMap = NULL;
    MC = NULL;
  }

  ~MapToMap() {}


  void mapCallback(octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
    const auto t_start = std::chrono::steady_clock::now();

    // Projection operates directly on incoming voxel coordinates, with no TF
    // conversion. Never relabel a differently framed OctoMap as this grid.
    if (msg->header.frame_id != mapFrame) {
      RCLCPP_ERROR(this->get_logger(),
                   "OctoMap frame '%s' differs from map_frame '%s'; skipping map.",
                   msg->header.frame_id.c_str(), mapFrame.c_str());
      return;
    }

    // Set resolution
    if (MC == NULL) {
      resolution = msg->resolution;
      MC = new MapConverter(resolution, slopeEstimationSize, minimumZ,
                            minimumOccupancy,this->get_logger());
    }
    // Convert ROS message to Octomap
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*msg));
    octomap::ColorOcTree *newOcMap = dynamic_cast<octomap::ColorOcTree *>(tree.get());
    if (!(newOcMap)) {
      publishCallbackDuration(t_start, "invalid_octomap_type");
      return;
    }
    tree.release();

   double min_x_new, min_y_new, min_z_new; 
    double max_x_new, max_y_new, max_z_new;
    newOcMap->getMetricMin(min_x_new, min_y_new, min_z_new);
    newOcMap->getMetricMax(max_x_new, max_y_new, max_z_new);

    RCLCPP_INFO(this->get_logger(),
    "newOcMap Bounds: X [%.2f, %.2f], Y [%.2f, %.2f], Z [%.2f, %.2f]",
    min_x_new, max_x_new, min_y_new, max_y_new, min_z_new, max_z_new);

    vector<double> minMax(6);
    if (OcMap == NULL) {
      RCLCPP_INFO(this->get_logger(), "OcMap is NULL - Initializing OcMap with the first incoming map.");
      minMax[0] = min_x_new;
      minMax[1] = max_x_new;
      minMax[2] = min_y_new;
      minMax[3] = max_y_new;
      MC->map.setOffset(min_x_new, min_y_new);
 
      OcMap = newOcMap;
    } else {
      computeBoundingBox(minMax, newOcMap, OcMap); 
      delete OcMap;
      OcMap = newOcMap;
    }
    minMax[4] = min_z_new;
    minMax[5] = max_z_new;

    update2Dmap(minMax); 
    pub();
    publishCallbackDuration(t_start, "complete");

  }

  void computeBoundingBox(vector<double> &minMax, octomap::ColorOcTree *tree1,
                            octomap::ColorOcTree *tree2) {

      double min_x = INFINITY;
      double min_y = INFINITY;
      double min_z = INFINITY;
      double max_x = -INFINITY;
      double max_y = -INFINITY;
      double max_z = -INFINITY;


      for (octomap::ColorOcTree::leaf_iterator it = tree1->begin_leafs(),
                                          end = tree1->end_leafs();
          it != end; ++it) {
        octomap::OcTreeKey key = it.getKey();
        octomap::OcTreeNode *node2 = tree2->search(key);
        if (node2 != nullptr) {
          bool occupied1 = tree1->isNodeOccupied(*it);
          bool occupied2 = tree2->isNodeOccupied(node2);
          if (occupied1 != occupied2) {

            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();
            updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
          }
        } else {

          double x = it.getX();
          double y = it.getY();
          double z = it.getZ();
          updateBoundingBox(x, y, z, min_x, min_y, min_z, max_x, max_y, max_z);
        }
      }

      minMax[0] = min_x;
      minMax[1] = max_x;
      minMax[2] = min_y;
      minMax[3] = max_y;


    }


  void updateBoundingBox(double x, double y, double z, double &min_x,
                         double &min_y, double &min_z, double &max_x,
                         double &max_y, double &max_z) {
    if (x < min_x)
      min_x = x;
    if (y < min_y)
      min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }

  // Update 2D map in the region of the aabb
  void update2Dmap(vector<double> minMax) {
    update_count_++;
    const double HEIGHT_THRESHOLD = 2.5; 
    static const int MAX_UPDATES_FOR_THRESHOLDING = 250; 

    if (minMax[4] >= minMax[5]) {
      RCLCPP_ERROR(
        this->get_logger(),
        "无效的Z轴范围: [%.2f, %.2f]",
        minMax[4], minMax[5]
      );
      return;
    }

    for (double mM : minMax)
      if (isinf(mM))
        return;

    vector<voxel> voxelList;
    octomap::point3d minPoint(minMax[0], minMax[2], minMax[4]);
    octomap::point3d maxPoint(minMax[1], minMax[3], minMax[5]);
    for (auto it = OcMap->begin_leafs_bbx(minPoint, maxPoint),
              it_end = OcMap->end_leafs_bbx();
         it != it_end; ++it) {

    const double z_pos = it.getZ();
    
      voxel v;
      v.position.x = it.getX();
      v.position.y = it.getY();
      v.position.z = it.getZ();
      
      v.halfSize = it.getSize() / 2;
      v.occupied = OcMap->isNodeOccupied(*it);

      octomap::ColorOcTreeNode::Color color = it->getColor();
      v.r = color.r;
      v.g = color.g;
      v.b = color.b;


      if (v.b > 200 && v.r < 50 && v.g < 50) {
          v.semantic_label = GROUND; 
      } else if (v.r > 200 && v.g < 50 && v.b < 50) {
          v.semantic_label = FLAT;  
      } else if (v.g > 200 && v.r < 50 && v.b < 50) {
          v.semantic_label = ROUGH; 
      }  
      else if (v.r > 120 && v.r < 136 && v.g > 120 && v.g < 136 && v.b > 120 && v.b < 136) {
          v.semantic_label = UNCERTAIN; 
      }  else {
          v.semantic_label = UNKNOWN;
      }
      voxelList.push_back(v);
    }
    MC->updateMap(voxelList, minMax);
    if (voxelList.empty()) {
  RCLCPP_WARN(this->get_logger(), "voxelList is empty!");
} else {
  RCLCPP_INFO(this->get_logger(), "voxelList has %zu voxels", voxelList.size());
}
  }

  void pub() {
    mapMsg.header.stamp = this->now();
    mapMsg.header.frame_id = mapFrame;
    heightMsg.header = mapMsg.header;
    mapMsg.info.width = MC->map.sizeX();
    mapMsg.info.height = MC->map.sizeY();
    mapMsg.info.resolution = MC->map.getResulution();
    mapMsg.info.origin.position.x = MC->map.offsetX();
    mapMsg.info.origin.position.y = MC->map.offsetY();
    mapMsg.info.origin.position.z = mapZpos;
    mapMsg.info.origin.orientation.x = 0;
    mapMsg.info.origin.orientation.y = 0;
    mapMsg.info.origin.orientation.z = 0;
    mapMsg.info.origin.orientation.w = 1;
    heightMsg.info = mapMsg.info;
    slopeMsg.info = mapMsg.info;
    mapMsg.data.resize(mapMsg.info.width * mapMsg.info.height);
    heightMsg.top.resize(mapMsg.info.width * mapMsg.info.height);
    heightMsg.bottom.resize(mapMsg.info.width * mapMsg.info.height);
    slopeMsg.slope.resize(mapMsg.info.width * mapMsg.info.height);
    
    for (int y = 0; y < MC->map.sizeY(); y++) {
      for (int x = 0; x < MC->map.sizeX(); x++) {
        int index = x + y * MC->map.sizeX();
        mapMsg.data[index] = MC->map.get(x, y); 

        if (x < 3 && y < 3) { 
          RCLCPP_INFO(this->get_logger(),
            "[pub] mapMsg.data[%d] = %d (cell=%d,%d)", 
            index, mapMsg.data[index], x, y
          );
        }
      }
    }

    // pub map for UGV
    if (pubMapUGV->get_subscription_count() != 0) {
      if (isinf(slopeMax)) {
        RCLCPP_WARN(
            this->get_logger(),
            "The max slope UGV is not set; obstacles will not be included in "
            "the UGV map. \nAdd \'_max_slope_ugv:=x\' to rosrun command, or "
            "\'<param name=\"max_slope_ugv\" value=\"x\"/>\' in launch file.");
      }
      mapMsg.header.stamp = this->now();
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          mapMsg.data[index] = MC->map.get(x, y, slopeMax);
        }
      }
      pubMapUGV->publish(mapMsg);
      RCLCPP_INFO(this->get_logger(), 
    "[OctoMap] Done. Published octomap with total UGV keyframes");
    }




  if (pubMapGeometric->get_subscription_count() != 0) {

      int vMax = 2, vMin = 0;  
      for (int y = 0; y < MC->map.sizeY(); y++) {
          for (int x = 0; x < MC->map.sizeX(); x++) {
              int geomValue = MC->map.getGeometricMap(x, y);
              vMax = max(vMax, geomValue);
          }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
          for (int x = 0; x < MC->map.sizeX(); x++) {
              int index = x + y * MC->map.sizeX();
              int geomValue = MC->map.getGeometricMap(x, y);

              if (geomValue >= 0) {
                  double value = (geomValue - vMin) / (double)(vMax - vMin) * 100;
                  mapMsg.data[index] = static_cast<int>(value);
              } else {
                  mapMsg.data[index] = -1; 
              }
          }
      }

      pubMapGeometric->publish(mapMsg);
      RCLCPP_INFO(this->get_logger(), "Published geometric map.");
  }


  visualization_msgs::msg::Marker structuralMapMarker;
  structuralMapMarker.header = mapMsg.header;
  structuralMapMarker.ns = "structural_semantic_map";
  structuralMapMarker.id = 0;
  structuralMapMarker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  structuralMapMarker.action = visualization_msgs::msg::Marker::ADD;
  structuralMapMarker.pose.orientation.w = 1.0;
  structuralMapMarker.scale.x = 1.0;
  structuralMapMarker.scale.y = 1.0;
  structuralMapMarker.scale.z = 1.0;
  // Triangle markers multiply global and vertex alpha. Apply opacity once.
  structuralMapMarker.color.a = 1.0;

  auto semanticColor = [this](int geomValue) {
    std_msgs::msg::ColorRGBA color;
    color.a = static_cast<float>(ssmAlpha);
    switch (geomValue) {
      case 0:
        color.g = 1.0f;  // CH
        break;
      case 1:
        color.b = 1.0f;  // reserved legacy class
        break;
      case 2:
        color.r = 1.0f;
        color.g = 105.0f / 255.0f;
        color.b = 180.0f / 255.0f;  // VP
        break;
      case 3:
        color.r = 1.0f;  // HP
        break;
      case 4:
        color.r = 1.0f;
        color.g = 1.0f;  // CV
        break;
      case 5:
        color.g = 191.0f / 255.0f;
        color.b = 1.0f;  // OW
        break;
      case 6:
        // UB: black
        break;
      default:
        color.r = color.g = color.b = 240.0f / 255.0f;  // Reference image background
        break;
    }
    return color;
  };

  // Keep OccupancyGrid for downstream consumers, and publish a colored Marker
  // for RViz. The Marker supports the semantic palette and avoids the indexed
  // map shader that fails on some Mesa/virtualized OpenGL drivers.
    for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
            int index = x + y * MC->map.sizeX();
            int geomValue = MC->map.getGeometricMap(x, y);


            if (geomValue == 0) {
                mapMsg.data[index] = 105;
            } else if (geomValue == 1) {
                mapMsg.data[index] = 50;  
            } else if (geomValue == 2) {
                mapMsg.data[index] = 100; 
            } else if (geomValue == 3) {
                mapMsg.data[index] = 98;  
            } else if (geomValue == 4) {
                mapMsg.data[index] = -10;
            } else if (geomValue == 5) {
                mapMsg.data[index] = 25;  
            } else if (geomValue == 6) {
                mapMsg.data[index] = 0; 
            } else {
                mapMsg.data[index] = -1;  
            }

        }
    }

    // Draw the complete SSM image, including its off-white unknown cells.
    // Merge equal adjacent cells into horizontal rectangles to avoid drawing
    // hundreds of thousands of separate background cubes. Each cell is
    // covered once, so the white background does not change class opacity.
    auto addVertex = [&](double x, double y, const std_msgs::msg::ColorRGBA& color) {
      geometry_msgs::msg::Point point;
      point.x = x;
      point.y = y;
      point.z = mapMsg.info.origin.position.z + 0.02;
      structuralMapMarker.points.push_back(point);
      structuralMapMarker.colors.push_back(color);
    };
    const int width = MC->map.sizeX();
    const double cellSize = mapMsg.info.resolution;
    for (int y = 0; y < MC->map.sizeY(); ++y) {
      for (int x = 0; x < width;) {
        const int begin = x;
        const int value = mapMsg.data[y * width + x];
        while (x < width && mapMsg.data[y * width + x] == value) ++x;
        const auto color = semanticColor(MC->map.getGeometricMap(begin, y));
        const double x0 = mapMsg.info.origin.position.x + begin * cellSize;
        const double x1 = mapMsg.info.origin.position.x + x * cellSize;
        const double y0 = mapMsg.info.origin.position.y + y * cellSize;
        const double y1 = y0 + cellSize;
        addVertex(x0, y0, color);
        addVertex(x1, y0, color);
        addVertex(x1, y1, color);
        addVertex(x0, y0, color);
        addVertex(x1, y1, color);
        addVertex(x0, y1, color);
      }
    }

      pubMapGeometric_2->publish(mapMsg);
      pubStructuralMapMarker->publish(structuralMapMarker);
      RCLCPP_INFO(this->get_logger(), "Published geometric map 2.");

  if (pubBridgeGapMarkers->get_subscription_count() > 0) {
    visualization_msgs::msg::MarkerArray markerArray;
    int idCounter = 0;

    const int patchSize = 4;
    const int minClusterPoints = 200; 
    const double distanceThreshold = 5; 
    const double valueThreshold = 0.8;  
    int mapSizeX = MC->map.sizeX();
    int mapSizeY = MC->map.sizeY();
    int numXPatches = (mapSizeX + patchSize - 1) / patchSize;
    int numYPatches = (mapSizeY + patchSize - 1) / patchSize;


    std::vector<std::vector<PatchData>> patchGrid(numYPatches, std::vector<PatchData>(numXPatches));

    for (int yIdx = 0; yIdx < numYPatches; ++yIdx) {
        for (int xIdx = 0; xIdx < numXPatches; ++xIdx) {
            int x0 = xIdx * patchSize;
            int y0 = yIdx * patchSize;
            PatchData pd;
            pd.x0 = x0;
            pd.y0 = y0;
            pd.valid = false;

            double sum = 0.0;
            double maxGap = -INFINITY;
            double minGap = INFINITY;
            int validCount = 0;

            for (int dy = 0; dy < patchSize; ++dy) {
                for (int dx = 0; dx < patchSize; ++dx) {
                    int x = x0 + dx;
                    int y = y0 + dy;
                    if (x >= mapSizeX || y >= mapSizeY) continue;

                    double gap = MC->map.getBridgeGap(x, y);
                    if (!std::isnan(gap)) {
                        sum += gap;
                        validCount++;
                        maxGap = std::max(maxGap, gap);
                        minGap = std::min(minGap, gap);
                    }
                }
            }

            if (validCount > 0) {
                pd.valid = true;
                pd.sum = sum;
                pd.count = validCount;
                pd.maxGap = maxGap;
                pd.minGap = minGap;
                pd.diff = maxGap - minGap;
                pd.isSplit = (pd.diff >= 0.5);
            }
            patchGrid[yIdx][xIdx] = pd;
        }
    }


    std::vector<std::vector<PatchData>> clusters;
    std::vector<std::vector<bool>> visited(numYPatches, std::vector<bool>(numXPatches, false));

    const std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};

    for (int yIdx = 0; yIdx < numYPatches; ++yIdx) {
        for (int xIdx = 0; xIdx < numXPatches; ++xIdx) {
            if (!visited[yIdx][xIdx] && patchGrid[yIdx][xIdx].valid) {
                std::queue<std::pair<int, int>> q;
                std::vector<PatchData> cluster;
                bool currentType = patchGrid[yIdx][xIdx].isSplit;

                q.push({xIdx, yIdx});
                visited[yIdx][xIdx] = true;

                while (!q.empty()) {
                    auto current = q.front();
                    q.pop();
                    int cx = current.first;
                    int cy = current.second;
                    cluster.push_back(patchGrid[cy][cx]);

                    for (const auto& dir : directions) {
                        int nx = cx + dir.first;
                        int ny = cy + dir.second;
                        if (nx >= 0 && nx < numXPatches && ny >= 0 && ny < numYPatches) {
                            if (!visited[ny][nx] && patchGrid[ny][nx].valid && 
                                patchGrid[ny][nx].isSplit == currentType) {
                                visited[ny][nx] = true;
                                q.push({nx, ny});
                            }
                        }
                    }
                }
                clusters.push_back(cluster);
            }
        }
    }

    std::vector<MergedCluster> mergedClusters;

    for (const auto& cluster : clusters) {
        if (cluster.empty()) continue;

        MergedCluster mc;
        for (const auto& pd : cluster) {
            mc.totalSum += pd.sum;
            mc.totalCount += pd.count;
            mc.minX = std::min(mc.minX, pd.x0);
            mc.maxX = std::max(mc.maxX, pd.x0 + patchSize);
            mc.minY = std::min(mc.minY, pd.y0);
            mc.maxY = std::max(mc.maxY, pd.y0 + patchSize);
        }
        if (mc.totalCount >= minClusterPoints) {
            mergedClusters.push_back(mc);
        }
    }


    bool merged = true;

    while (merged) {
        merged = false;
        for (size_t i = 0; i < mergedClusters.size(); ++i) {
            if (mergedClusters[i].merged) continue;

            for (size_t j = i + 1; j < mergedClusters.size(); ++j) {
                if (mergedClusters[j].merged) continue;

                double ci_x = (mergedClusters[i].minX + mergedClusters[i].maxX) / 2.0 * resolution + MC->map.offsetX();
                double ci_y = (mergedClusters[i].minY + mergedClusters[i].maxY) / 2.0 * resolution + MC->map.offsetY();
                double cj_x = (mergedClusters[j].minX + mergedClusters[j].maxX) / 2.0 * resolution + MC->map.offsetX();
                double cj_y = (mergedClusters[j].minY + mergedClusters[j].maxY) / 2.0 * resolution + MC->map.offsetY();

                double dx = ci_x - cj_x;
                double dy = ci_y - cj_y;
                double distance = std::hypot(dx, dy);

                double avg_i = mergedClusters[i].totalSum / mergedClusters[i].totalCount;
                double avg_j = mergedClusters[j].totalSum / mergedClusters[j].totalCount;
                double valueDiff = std::abs(avg_i - avg_j);

                if (distance < distanceThreshold && valueDiff < valueThreshold) {
                    mergedClusters[i].totalSum += mergedClusters[j].totalSum;
                    mergedClusters[i].totalCount += mergedClusters[j].totalCount;
                    mergedClusters[i].minX = std::min(mergedClusters[i].minX, mergedClusters[j].minX);
                    mergedClusters[i].maxX = std::max(mergedClusters[i].maxX, mergedClusters[j].maxX);
                    mergedClusters[i].minY = std::min(mergedClusters[i].minY, mergedClusters[j].minY);
                    mergedClusters[i].maxY = std::max(mergedClusters[i].maxY, mergedClusters[j].maxY);
                    mergedClusters[j].merged = true;
                    merged = true;
                }
            }
        }
    }

    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.g = 1.0;
    color.b = 0.0;
    color.a = 1.0;

    for (const auto& mc : mergedClusters) {
        if (mc.merged || mc.totalCount < minClusterPoints) continue;

        double centerX = (mc.minX + mc.maxX) / 2.0 * resolution + MC->map.offsetX();
        double centerY = (mc.minY + mc.maxY) / 2.0 * resolution + MC->map.offsetY();
        double clusterAvg = mc.totalSum / mc.totalCount;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << clusterAvg << "m";

        markerArray.markers.push_back(
            createTextMarker(idCounter++, centerX, centerY, 4, ss.str(), color));
    }

    pubBridgeGapMarkers->publish(markerArray);
    RCLCPP_INFO(this->get_logger(), "Published %ld clustered bridge gap markers (merged)", markerArray.markers.size());
  }


    // pub map for UAV
    if (pubMapUAV->get_subscription_count() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          mapMsg.data[index] = MC->map.get(x, y);
        }
      }
      pubMapUAV->publish(mapMsg);
    }

    // pub rviz visualization for floor heigth map
    if (pubMapFloor->get_subscription_count() != 0) {
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)
        return;
      double vMax = NAN, vMin = NAN;
      // find max min for normalization
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeight(x, y);
          if (isnan(v))
            continue;
          if (isnan(vMax))
            vMax = v;
          else
            vMax = max(vMax, v);
          if (isnan(vMin))
            vMin = v;
          else
            vMin = min(vMin, v);
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();
          double value = MC->map.getHeight(x, y);
          if (isnan(value)) {
            mapMsg.data[index] = -1;
          } else {
            if (vMax > vMin) {
              value = (value - vMin) / (vMax - vMin) * 199;
              if (value > 99)
                value -= 199; // To get the full color range in the rviz cost map
            } else {
              value = 50;
            }
            mapMsg.data[index] = static_cast<int>(value);
          }
        }
      }

      pubMapFloor->publish(mapMsg);
    }
    // pub rviz visualization for cieling heigth map
    if (pubMapCeiling->get_subscription_count() != 0) {
      if (MC->map.sizeY() == 0 || MC->map.sizeX() == 0)
        return;
      double vMax = NAN, vMin = NAN;
      // find max min for normalization
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          double v = MC->map.getHeightTop(x, y);
          if (isnan(v))
            continue;
          if (isnan(vMax))
            vMax = v;
          else
            vMax = max(vMax, v);
          if (isnan(vMin))
            vMin = v;
          else
            vMin = min(vMin, v);
        }
      }

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();
          double value = MC->map.getHeightTop(x, y);
          if (isnan(value)) {
            mapMsg.data[index] = -1;
          } else {
            if (vMax > vMin) {
              value = (value - vMin) / (vMax - vMin) * 199;
              if (value > 99)
                value -= 199; // To get the full color range in the rviz cost map
            } else {
              value = 50;
            }
            mapMsg.data[index] = static_cast<int>(value);
          }
        }
      }

      pubMapCeiling->publish(mapMsg);
    }


    // pub height map
    if (pubHeightMap->get_subscription_count() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          heightMsg.bottom[index] = MC->map.getHeight(x, y);
          heightMsg.top[index] = MC->map.getHeightTop(x, y);
        }
      }
      pubHeightMap->publish(heightMsg);
    }

    // pub rviz visualization fro slope map
    if (pubMapSlopeVis->get_subscription_count() != 0) {
      double vMax = slopeMax * 2, vMin = 0;

      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();
          double value = MC->map.getSlope(x, y);
          if (isnan(value)) {
            mapMsg.data[index] = -1;
          } else {
            value = min(value, slopeMax * 2);
            value = (value - vMin) / (vMax - vMin) * 198 + 1;
            if (value > 99)
              value -= 199; // soooo basically, rviz costmap are weird...
            mapMsg.data[index] = static_cast<int>(value);
          }
        }
      }
      pubMapSlopeVis->publish(mapMsg);
    }

    // pub slope map
    if (pubMapSlope->get_subscription_count() != 0) {
      for (int y = 0; y < MC->map.sizeY(); y++) {
        for (int x = 0; x < MC->map.sizeX(); x++) {
          int index = x + y * MC->map.sizeX();

          slopeMsg.slope[index] = MC->map.getSlope(x, y);
        }
      }
      pubMapSlope->publish(slopeMsg);
    }

            RCLCPP_INFO(this->get_logger(), "Publishing mapUGV. Frame: %s, Size: %dx%d", 
            mapMsg.header.frame_id.c_str(), 
            mapMsg.info.width, 
            mapMsg.info.height);

  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto Node = make_shared<MapToMap>();
  rclcpp::spin(Node);
  rclcpp::shutdown();

  return 0;
}
