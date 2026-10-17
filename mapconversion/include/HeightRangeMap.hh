#pragma once
#include "datatypes.hh"

point_int DIRECTIONS[] = {{1, 0},  {1, 1},   {0, 1},  {-1, 1},
                          {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

class HeightRangeMap {
public:

  void swapFreeOccupied() {
    free.swap(occupied); 
  }

  vector<vector<vector<heightRange>>> free;
  vector<vector<vector<heightRange>>> occupied;
  vector<vector<point2D>> posMap;
  vector<vector<vector<voxelInfo>>> voxelData;

  HeightRangeMap(int width = 0, int height = 0) { setSize(width, height); }

  // update size and init height map
  void setSize(int width, int height) {
    free.resize(width);
    occupied.resize(width);
    posMap.resize(width);
    voxelData.resize(width); 
    for (int i = 0; i < width; i++) {
      free[i].resize(height);
      occupied[i].resize(height);
      posMap[i].resize(height);
      voxelData[i].resize(height); 
    }
  }

  // add occupied and free ranges, ranges are merged with existing ranges if
  // they overlap
  void addRange(int x, int y, heightRange hr, bool isOccupied, double posX,
                double posY, double maxZ, double posZ, int semantic) {
    point2D p = {posX, posY};
    posMap[x][y] = p;

    voxelInfo voxel = {posZ, semantic};
    voxelData[x][y].push_back(voxel); 

     if (isOccupied) {
      if (occupied[x][y].empty()) {

        heightRange freeSpace = {maxZ, hr.top};

        inserthghtRange(freeSpace, &free[x][y]);
      } else {

        double existingBottom = occupied[x][y].back().bottom;
        if (hr.top < existingBottom) {
          heightRange freeSpace = {existingBottom, hr.top};
          inserthghtRange(freeSpace, &free[x][y]);
        }
      }

      inserthghtRange(hr, &occupied[x][y]);
      
      mergeOccupiedWithFree(x, y);
    } else {
      inserthghtRange(hr, &free[x][y]);
    }
  }

  void mergeOccupiedWithFree(int x, int y) {
    for (auto occ : occupied[x][y]) {
      for (int i = 0; i < free[x][y].size(); i++) {
        heightRange freeHr = free[x][y][i];

        if (occ.bottom <= freeHr.bottom && occ.top >= freeHr.top) {
          free[x][y].erase(free[x][y].begin() + i);
          i--;
        }

        else if (occ.bottom <= freeHr.top && occ.top > freeHr.bottom) {
          if (occ.bottom > freeHr.bottom) {

            free[x][y][i].top = occ.bottom;
          }
          if (occ.top < freeHr.top) {

            heightRange newFree = {occ.top, freeHr.top};
            free[x][y].insert(free[x][y].begin() + i + 1, newFree);
          }
        }
      }
    }
  }

  // remove all ranges that are shorter than minSize at x y position
  void removeFreeRanges(int x, int y, double minSize) {
    for (int i = 0; i < free[x][y].size(); i++) {
      if (free[x][y][i].top - free[x][y][i].bottom > minSize)
        continue;
      free[x][y].erase(free[x][y].begin() + i);
      i--;
    }
  }

  // returns the occupation level of a cell compared to a free heightRange in
  // range 0-100
  int getOccupation(int x, int y, int index, int minOcc) {
    int occupation = -1;
    for (auto d : DIRECTIONS) {
      if (x + d.x < 0 || x + d.x >= free.size())
        continue;
      if (y + d.y < 0 || y + d.y >= free.back().size())
        continue;
      occupation =
          max(occupation, getOccupation(x, y, index, x + d.x, y + d.y, minOcc));
    }
    return occupation;
  }

  // returns the occupation level of a cell compared to a free heightRange in
  // range 0-100
  int getOccupation(int x1, int y1, int index, int x2, int y2, int minOcc) {
    if (index < 0)
      index = free[x2][y2].size() + index;
    if (index < 0 || index >= free[x2][y2].size())
      return -1;
    heightRange freeHr = free[x2][y2][index];
    vector<heightRange> occupiedHr = occupied[x1][y1];

    return getOccupation(freeHr, occupiedHr, minOcc);
  }


  int getOccupation(heightRange freeHr, vector<heightRange> occupiedHr,
                    int minOcc) {
    double overlapDistans = 0;
    for (auto oHr : occupiedHr) {
      heightRange compare = {min(freeHr.top, oHr.top),
                             max(freeHr.bottom, oHr.bottom)};
      if (compare.top - compare.bottom < 0)
        continue;
      overlapDistans += compare.top - compare.bottom;
    }
    int occupation =
        int(overlapDistans / (freeHr.top - freeHr.bottom) * 100) * 2;
    if (occupation < minOcc)
      return -1;
    return min(occupation, 100);
  }

private:

  void inserthghtRange(heightRange hr, vector<heightRange> *list) {
    int overlapIndex = -1;
    for (int i = 0; i < list->size(); i++) {
      if (hr.top < list->at(i).bottom)
        continue;
      if (hr.bottom > list->at(i).top)
        continue;
      overlapIndex = i;
      break;
    }
    if (overlapIndex != -1) {
      heightRange hrNew;
      heightRange hrOld = list->at(overlapIndex);
      list->erase(list->begin() + overlapIndex);
      hrNew.top = max(hrOld.top, hr.top);
      hrNew.bottom = min(hrOld.bottom, hr.bottom);
      inserthghtRange(hrNew, list);
      return;
    }

    list->push_back(hr);
  }
};
