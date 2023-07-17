#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include <stdlib.h>
#include <time.h>
#include <algorithm>
#include <deque>
#include <unordered_set>
#include <math.h>
// PCL
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include "patchwork.hpp"
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp> //自定义点云类型时要加
#include <pcl/filters/conditional_removal.h>

#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "common.hpp"
#include "CVC_cluster.h"
#include "patchwork.hpp"
#include "GuassianProcess.h"

using namespace std;
using namespace autosense;

// 1.释放内存 *data 2.保存point_cloud为PCD，注意设定长和高
void load_bin_cloud(std::string kitti_filename, pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud)
{
    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float *)malloc(num * sizeof(float)); // void *malloc(size_t size) 分配所需的内存空间，并返回一个指向它的指针。

    // pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    FILE *stream;
    stream = fopen(kitti_filename.c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;
    point_cloud->width = num;      // 设定长
    point_cloud->height = 1;       // 设定高
    point_cloud->is_dense = false; // 如果没有无效点（例如，具有NaN或Inf值），则为True
    for (int32_t i = 0; i < num; i++)
    {
        // vector<int32_t> point_cloud;
        pcl::PointXYZI point;
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.intensity = *pr;
        point_cloud->points.push_back(point);
        px += 4;
        py += 4;
        pz += 4;
        pr += 4;
    }
    fclose(stream);
    free(data); // 释放内存
}

using PointType = PointXYZILID;
boost::shared_ptr<PatchWork<PointType>> PatchworkGroundSeg;

int main()
{
    std::string kitti_cloud_filename = "/home/jjho/workspace/point_cluster/data/velodyne/000010.bin";
    std::string kitti_image_filename = "/home/jjho/workspace/point_cluster/data/image/000010.png";
    std::string kitti_label_filename = "/home/jjho/workspace/point_cluster/data/label/000010.txt";
    std::string kitti_calib_filename = "/home/jjho/workspace/point_cluster/data/calib/000010.txt";

    // load point cloud
    PointICloudPtr point_cloud(new PointICloud);
    PointICloudPtr cloud_fov(new PointICloud);

    load_bin_cloud(kitti_cloud_filename, point_cloud);
    load_Calibration(kitti_calib_filename);

    fov_segmentation(point_cloud, cloud_fov);

    /*******************GP INSAC**************************/
    // std::cout << "GP_INSAC" << std::endl;
    // PointICloudPtr cloud_ground(new PointICloud);
    // PointICloudPtr cloud_nonground(new PointICloud);
    // segmenter::GP_INSAC(*cloud_fov, *cloud_nonground, *cloud_ground);

    /****************    CVC    **************************/
    PatchworkGroundSeg.reset(new PatchWork<PointXYZILID>());
    pcl::PointCloud<PointType>::Ptr pc_curr(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_ground(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_non_ground(new pcl::PointCloud<PointType>);
    double time_taken;

    for (int i = 0; i < cloud_fov->points.size(); ++i)
    {
        PointType p;
        p.x = cloud_fov->points[i].x;
        p.y = cloud_fov->points[i].y;
        p.z = cloud_fov->points[i].z;
        p.intensity = cloud_fov->points[i].intensity;
        pc_curr->points.push_back(p);
    }

    PatchworkGroundSeg->estimate_ground(*pc_curr, *pc_ground, *pc_non_ground, time_taken);

    cout << time_taken << endl;

    vector<float> param(3, 0);
    param[0] = 2;
    param[1] = 0.4;
    param[2] = 1.5;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_point(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < pc_non_ground->points.size(); ++i)
    {
        pcl::PointXYZ p;
        p.x = pc_non_ground->points[i].x;
        p.y = pc_non_ground->points[i].y;
        p.z = pc_non_ground->points[i].z;
        cluster_point->points.push_back(p);
    }

    cout << "cluster_point " << cluster_point->points.size() << endl;
    CVC Cluster(param);
    std::vector<PointAPR> capr;
    Cluster.calculateAPR(*cluster_point, capr);

    cout << "capr " << capr.size() << endl;
    std::unordered_map<int, Voxel> hash_table;
    Cluster.build_hash_table(capr, hash_table);
    vector<int> cluster_indices;
    cluster_indices = Cluster.cluster(hash_table, capr);
    vector<int> cluster_id;
    Cluster.most_frequent_value(cluster_indices, cluster_id);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer("pcd")); // PCLVisualizer 可视化类
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1);

    cv::RNG rng(12345);

    int r = rng.uniform(0, 200);
    int g = rng.uniform(0, 200);
    int b = rng.uniform(0, 200);
    pcl::visualization::PointCloudColorHandlerCustom<PointType> colorob(pc_ground, (r), (g), (b));
    string csob = "cloudfinalob";
    // viewer->addPointCloud(pc_ground, colorob, csob);

    // pcl::visualization::PointCloudColorHandlerCustom <PointType> colorob2(pc_non_ground, (0), (255),(0));
    // string csob2 = "cloudfinalob1";
    // viewer->addPointCloud(pc_non_ground,colorob2, csob2);

    for (int j = 0; j < cluster_id.size(); ++j)
    {
        int r = rng.uniform(20, 255);
        int g = rng.uniform(20, 255);
        int b = rng.uniform(20, 255);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudcluster(new pcl::PointCloud<pcl::PointXYZ>); // 初始化
        for (int i = 0; i < cluster_indices.size(); ++i)
        {
            if (cluster_indices[i] == cluster_id[j])
            {
                cloudcluster->points.push_back(cluster_point->points[i]);
            }
        }

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloudcluster, (r), (g), (b));
        string text = "cloud" + toString(j);
        viewer->addPointCloud(cloudcluster, color, text);
    }

    while (!viewer->wasStopped())
    {
        viewer->spin();
    }

    ;

    // // 写成PCD格式文件
    // pcl::io::savePCDFileASCII("000000.pcd", *point_cloud);
    // std::cerr << "Saved " << point_cloud->size() << " data points to 000000.pcd." << std::endl;

    return 0;
}