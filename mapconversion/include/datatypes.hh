#pragma once
#include <Eigen/Dense>
#include <vector>
#include <cfloat>


using namespace std;
using namespace Eigen;



struct point_int {
  int x, y;
};

struct point2D {
  double x, y;
};

struct point3D {
  double x, y, z;
};

struct voxel {
  point3D position;
  float halfSize;
  bool occupied;
  uint8_t r, g, b;  // Add RGB color information
  int semantic_label; 
};

struct heightRange {
  double top, bottom;
};


enum SemanticLabel {
    UNKNOWN = 0,
    GROUND = 1, 
    FLAT = 2,  
    ROUGH = 3,   
    UNCERTAIN = 4 
};

struct voxelInfo {
    double z;    
    int semantic;  
};

struct PatchData {
    int x0 = 0;
    int y0 = 0;
    bool valid = false;
    bool isSplit = false;
    double sum = 0.0;
    int count = 0;
    double maxGap = -INFINITY;
    double minGap = INFINITY;
    double diff = 0.0;
};


struct MergedCluster {
    int minX = INT_MAX;
    int maxX = -1;
    int minY = INT_MAX;
    int maxY = -1;
    double totalSum = 0.0;
    int totalCount = 0;
    bool merged = false;
};