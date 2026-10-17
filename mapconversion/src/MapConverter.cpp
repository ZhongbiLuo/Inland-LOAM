
#include "MapConverter.hh"
#include "HeightRangeMap.hh"


MapConverter::MapConverter(double resolution, int slopeEstimationSize,
                           double minimumZ, int minimumOccupancy,rclcpp::Logger logger) : logger_(logger){
  MapConverter::resolution = resolution;
  MapConverter::slopeEstimationSize = slopeEstimationSize;
  MapConverter::minOcc = minimumOccupancy;
  MapConverter::minZ = minimumZ;
  Map2D m(resolution);
  MapConverter::map = m;
}

MapConverter::~MapConverter() {}

void MapConverter::updateMap(vector<voxel> vMap, vector<double> minMax) {
  auto t0 = std::chrono::steady_clock::now();
  if (vMap.size() == 0)
    return;
  for (double mM : minMax)
    if (isinf(mM))
      return; 

  if (minMax[1] <= minMax[0] || minMax[3] <= minMax[2]) {
    RCLCPP_ERROR(rclcpp::get_logger("map_conversion"), 
                "无效的BoundingBox: x[%f→%f] y[%f→%f]", 
                minMax[0], minMax[1], minMax[2], minMax[3]);
    return;
  }

  if (vMap.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("map_conversion"), "Input voxel is empty!");
    return;
  }
  int xSize = (minMax[1] - minMax[0]) / MapConverter::map.getResulution();
  int ySize = (minMax[3] - minMax[2]) / MapConverter::map.getResulution();
  RCLCPP_INFO(rclcpp::get_logger("map_conversion"), 
           "map size calculation: xSize=%d (min_x=%.2f, max_x=%.2f), ySize=%d (min_y=%.2f, max_y=%.2f)",
           xSize, minMax[0], minMax[1], ySize, minMax[2], minMax[3]);

  HeightRangeMap hMap(xSize, ySize);
  for (voxel v : vMap) {

    int sizeIndex = v.halfSize * 2 / MapConverter::map.getResulution();
    for (int x = 0; x < sizeIndex; x++) {
      for (int y = 0; y < sizeIndex; y++) {
        int posX = (v.position.x - v.halfSize + resolution * x - minMax[0]) /
                   resolution;
        int posY = (v.position.y - v.halfSize + resolution * y - minMax[2]) /
                   resolution;
        if (posX < 0 || posX >= xSize)
          continue; 
        if (posY < 0 || posY >= ySize)
          continue;

        heightRange hr = {v.position.z + v.halfSize + resolution * 0.001,
                          v.position.z - v.halfSize - resolution * 0.001};

        hMap.addRange(posX, posY, hr, v.occupied,
                      v.position.x - v.halfSize + resolution * x,
                      v.position.y - v.halfSize + resolution * y, minMax[5], v.position.z, v.semantic_label);

        // auto t1 = std::chrono::steady_clock::now();
        // auto cost = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
        // RCLCPP_INFO(logger_, "updateMap 耗时: %ld ms", cost);
      }
    }
  }

  // remove free space that is smaller than robots is safety margin
  for (int x = 1; x < xSize - 1; x++) {
    for (int y = 1; y < ySize - 1; y++) {
      hMap.removeFreeRanges(x, y, minZ);
      int mapValue = 0;
      if (hMap.free[x][y].size() == 0) {
        hMap.posMap[x][y].x = x * resolution + minMax[0];
        hMap.posMap[x][y].y = y * resolution + minMax[2];
        mapValue = -1;
      }
      if (mapValue == -1 &&
          map.get(hMap.posMap[x][y].x, hMap.posMap[x][y].y) == -1)
        continue;


      map.set(hMap.posMap[x][y].x, hMap.posMap[x][y].y, mapValue);
      if (mapValue == -1)
        continue;
      map.setHeight(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                    hMap.free[x][y].back().bottom);
      map.setHeightTop(hMap.posMap[x][y].x, hMap.posMap[x][y].y,
                       hMap.free[x][y][0].top);
    }
  }

  for (int x = 2; x < xSize - 2; x++) {
    for (int y = 2; y < ySize - 2; y++) {

      if (hMap.free[x][y].size() == 0)
        continue;

      for (auto d : DIRECTIONS) {
        if (hMap.free[x + d.x][y + d.y].size() != 0)
          continue;
        map.set(hMap.posMap[x + d.x][y + d.y].x, hMap.posMap[x][y].y,
                hMap.getOccupation(x + d.x, y + d.y, -1, minOcc));
      }
    }
  }

  map.updateSlope_new(slopeEstimationSize, minMax[0], minMax[1], minMax[2],
                  minMax[3], hMap.voxelData);

                  
}

vector<double> MapConverter::minMaxVoxel(vector<voxel> list) {
  vector<double> minMax(6);
  minMax[0] = INFINITY;
  minMax[1] = -INFINITY;
  minMax[2] = INFINITY;
  minMax[3] = -INFINITY;
  minMax[4] = INFINITY;
  minMax[5] = -INFINITY;
  for (voxel v : list) {

    if (v.halfSize > resolution * .9)
      continue;
    minMax[0] = min(minMax[0], v.position.x);
    minMax[1] = max(minMax[1], v.position.x);
    minMax[2] = min(minMax[2], v.position.y);
    minMax[3] = max(minMax[3], v.position.y);
    minMax[4] = min(minMax[4], v.position.z);
    minMax[5] = max(minMax[5], v.position.z);
  }
  return minMax;
}
