#pragma once

#include <iostream>
#include <vector>
#include <string.h>
#include <Eigen/Dense>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

extern Eigen::Matrix<double, 3, 4> P2;
extern Eigen::Matrix<double, 4, 4> R_rect;
extern Eigen::Matrix<double, 4, 4> Tr_velo_to_cam;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointICloud;
typedef PointICloud::Ptr PointICloudPtr;
typedef PointICloud::ConstPtr PointICloudConstPtr;

struct PointXYZILID
{
    PCL_ADD_POINT4D; // quad-word XYZ
    float intensity; ///< laser intensity reading
    uint16_t label;  ///< point label
    uint16_t id;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZILID,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, label, label)(uint16_t, id, id))

void load_Calibration(std::string file_name);

void fov_segmentation(PointICloudPtr &cloudXYZI, PointICloudPtr &cloud_fov);
