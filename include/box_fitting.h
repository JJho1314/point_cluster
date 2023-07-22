
#ifndef MY_PCL_TUTORIAL_BOX_FITTING_H
#define MY_PCL_TUTORIAL_BOX_FITTING_H

#include <array>
#include <pcl/io/pcd_io.h>
#include <vector>
#include "render/box.h"

// #include "component_clustering.h"

using namespace std;
using namespace pcl;

extern float picScale; // picScale * roiM = 30 * 30
// const float picScale = 30;
extern int ramPoints;
extern int lSlopeDist;
extern int lnumPoints;

extern float tHeightMin;
extern float tHeightMax;
extern float tWidthMin;
extern float tWidthMax;
extern float tLenMin;
extern float tLenMax;
extern float tAreaMax;
extern float tRatioMin;
extern float tRatioMax;
extern float minLenRatio;
extern float tPtPerM3;

//  将最小面积矩形（MAR）[128]应用于每个聚类对象，从而生成一个2D框，当与保留在聚类过程中的高度信息结合后，它便成为3D边界框
void getBoundingBox(vector<PointCloud<PointXYZ>> clusteredPoints, vector<PointCloud<PointXYZ>> &bbPoints);

Eigen::Quaternionf calculateOrientation(const std::vector<Eigen::Vector3f> &vertices);

// Function to calculate the bounding box center, dimensions, and orientation quaternion
void calculateBoundingBoxes(const std::vector<pcl::PointCloud<pcl::PointXYZ>> &boundingBoxes, std::vector<BoxQ> &BBoxes);

#endif // MY_PCL_TUTORIAL_BOX_FITTING_H