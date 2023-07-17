#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <unistd.h>

#include "common.hpp"

Eigen::Matrix<double, 3, 4> P2;
Eigen::Matrix<double, 4, 4> R_rect;
Eigen::Matrix<double, 4, 4> Tr_velo_to_cam;
void load_Calibration(std::string file_name)
{
    FILE *fp = fopen(file_name.c_str(), "r");
    if (!fp)
    {
        printf("open Calib error!!!\n");
        return;
    }
    char str[255];
    double temp[12];

    // P0 && p1
    for (int i = 0; i < 2; i++)
    {
        fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
               str, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6],
               &temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);
    }
    // p2
    fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
           str, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6],
           &temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);

    P2 << temp[0], temp[1], temp[2], temp[3],
        temp[4], temp[5], temp[6], temp[7],
        temp[8], temp[9], temp[10], temp[11];
    // p3
    fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
           str, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6],
           &temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);

    /// R0_rect
    fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf",
           str, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6],
           &temp[7], &temp[8]);

    R_rect << temp[0], temp[1], temp[2], 0,
        temp[3], temp[4], temp[5], 0,
        temp[6], temp[7], temp[8], 0,
        0, 0, 0, 1;

    fscanf(fp, "%s %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
           str, &temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5], &temp[6],
           &temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);

    Tr_velo_to_cam << temp[0], temp[1], temp[2], temp[3],
        temp[4], temp[5], temp[6], temp[7],
        temp[8], temp[9], temp[10], temp[11],
        0, 0, 0, 1;

    fclose(fp);
}

const int UMax = 1242;
const int VMax = 375;
void fov_segmentation(PointICloudPtr &cloudXYZI, PointICloudPtr &cloud_fov)
{
    /****************************************************/
    //=============     只在KITTI检测范围内的点云    =============//

    for (int i = 0; i < cloudXYZI->size(); ++i)
    {
        // lidar_to_rect
        if ((*cloudXYZI)[i].x < 0 || (*cloudXYZI)[i].x > 70 || std::abs((*cloudXYZI)[i].y) > 40)
            continue;
        Eigen::Matrix<double, 4, 1> center;
        Eigen::Matrix<double, 4, 1> center_3D;
        center_3D << (*cloudXYZI)[i].x, (*cloudXYZI)[i].y, (*cloudXYZI)[i].z, 1;
        center = R_rect * Tr_velo_to_cam * center_3D;
        // rect_to_img
        Eigen::Matrix<double, 3, 1> pts_2d_hom = P2 * center;
        pts_2d_hom(0, 0) /= pts_2d_hom(2, 0);
        pts_2d_hom(1, 0) /= pts_2d_hom(2, 0);
        double pts_rect_depth = pts_2d_hom(2, 0) - P2(2, 3);

        // get_fov_flag
        if (pts_2d_hom(0, 0) >= 0 && pts_2d_hom(0, 0) < UMax &&
            pts_2d_hom(1, 0) >= 0 && pts_2d_hom(1, 0) < VMax &&
            pts_rect_depth >= 0)
        {
            PointI point = (*cloudXYZI)[i];
            cloud_fov->push_back(point);
        }
    }
    cloudXYZI = cloud_fov;
}