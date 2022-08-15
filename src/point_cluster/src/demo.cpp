#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include "CVC.h"
#include "patchworkpp/patchworkpp.hpp"
#include "clip.h"
#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"
#include <ros/ros.h>
#include <signal.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using PointType = pcl::PointXYZ;
using namespace std;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;
ros::Publisher pub_cluster;
ros::Publisher pub_box;

template <typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map")
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

void BigBox2SmallBox(autosense::ObjectPtr &object, vector<PtC::Ptr> &small_clusters)
{
    float delta_box_length = 5.0;
    Eigen::Vector4f min_object, max_object;
    pcl::getMinMax3D(*object->cloud, min_object, max_object);
    int box_num = int((max_object[0] - min_object[0]) / delta_box_length) + 1;
    // small_clusters.resize(box_num);
    for (int i = 0; i < box_num; ++i)
    {
        PtC::Ptr small_cluster(new PtC);
        for (int j = 0; j < object->cloud->points.size(); ++j)
        {
            int num = int((object->cloud->points[j].x - min_object[0]) / delta_box_length);
            if (num == i)
            {
                small_cluster->push_back(object->cloud->points[j]);
            }
        }
        assert(small_cluster->size() != 0);
        small_clusters.push_back(small_cluster);
    }
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;
    float theta = -M_PI / 35; // 旋转弧度

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::PointCloud<PointType> clipped_cloud;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;
    Clip clip(nh, private_nh);
    CVC cvc_cluster(nh, private_nh);

    pcl::fromROSMsg(*cloud_msg, pc_curr);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    pcl::transformPointCloud(pc_curr, transformed_cloud, transform);

    clock_t start = clock();

    clip.Process(transformed_cloud, clipped_cloud); //删除多余的点

    PatchworkppGroundSeg->estimate_ground(clipped_cloud, pc_ground, pc_non_ground, time_taken);

    cout << "\033[1;32m"
         << "Result: Input PointCloud: " << clipped_cloud.size() << " -> Ground: " << clipped_cloud.size()
         << " (running_time: " << time_taken << " sec)"
         << "\033[0m" << endl;

    vector<int> cluster_indices;
    double t0 = ros::Time::now().toSec();
    cvc_cluster.Process(pc_non_ground, cluster_indices); // CVC聚类
    vector<int> cluster_index;                           //数量超过10的点云簇序号
    cvc_cluster.SelectMajorCluster(cluster_indices, cluster_index);

    std::vector<PtC::Ptr> cloud_clusters; //点云簇列表
    //遍历索引表，寻找点数超过10的点云簇，并传入cloud_clusters
    for (int i = 0; i < cluster_index.size(); ++i)
    {
        pcl::PointCloud<PointType> cluster;
        PtC::Ptr cluster_ptr(new PtC);

        for (int j = 0; j < cluster_indices.size(); ++j)
        {
            if (cluster_indices[j] == cluster_index[i])
            {
                cluster.points.push_back(pc_non_ground.points[j]);
            }
        }
        pcl::copyPointCloud(cluster, *cluster_ptr);
        cloud_clusters.push_back(cluster_ptr);
    }

    PtC::Ptr clusters(new PtC);
    for (int i = 0; i < cloud_clusters.size(); ++i)
    {
        *clusters += *cloud_clusters[i];
    }
    double t1 = ros::Time::now().toSec();
    ROS_INFO("Time of clustering: %f ms", (t1 - t0) * 1000);

    // clock_t end = clock();
    // ROS_INFO("Total time: %lf s", (double)(end - start) / CLOCKS_PER_SEC);
    // ROS_INFO("FPS: %lf  Hz", (double)CLOCKS_PER_SEC / (end - start));

    pub_cloud.publish(cloud2msg(transformed_cloud));
    pub_ground.publish(cloud2msg(pc_ground));
    pub_non_ground.publish(cloud2msg(pc_non_ground));
    //发布聚类点云
    pub_cluster.publish(cloud2msg(*clusters));

    // Apollo bounding box
    // define object builder
    boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;

    // create object builder by manager
    object_builder_ = autosense::object_builder::createObjectBuilder();

    // build 3D orientation bounding box for clustering point cloud

    std::vector<autosense::ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);

    //将大型包围盒划分成几个小的包围盒
    visualization_msgs::MarkerArray boxes;
    int count = 0;
    for (int i = 0; i < objects.size(); ++i)
    {
        if (objects[i]->length > 10)
        {
            vector<PtC::Ptr> small_clusters;

            BigBox2SmallBox(objects[i], small_clusters);

            for (int j = 0; j < small_clusters.size(); ++j)
            {
                Eigen::Vector4f min, max;
                pcl::getMinMax3D(*small_clusters[j], min, max);
                std::vector<cv::Point2f> points;
                for (int k = 0; k < small_clusters[j]->size(); ++k)
                {
                    cv::Point2f pt;
                    pt.x = small_clusters[j]->points[k].x;
                    pt.y = small_clusters[j]->points[k].y;
                    points.push_back(pt);
                }

                std::vector<cv::Point2f> hull;
                cv::convexHull(points, hull);
                geometry_msgs::PolygonStamped polygon;
                for (size_t k = 0; k < hull.size() + 1; k++)
                {
                    geometry_msgs::Point32 point;
                    point.x = hull[k % hull.size()].x;
                    point.y = hull[k % hull.size()].y;
                    point.z = min[2];

                    polygon.polygon.points.push_back(point);
                }

                for (size_t k = 0; k < hull.size() + 1; k++)
                {
                    geometry_msgs::Point32 point;
                    point.x = hull[k % hull.size()].x;
                    point.y = hull[k % hull.size()].y;
                    point.z = max[2];

                    polygon.polygon.points.push_back(point);
                }
                visualization_msgs::Marker box;
                box.lifetime = ros::Duration(0.1);
                box.header = cloud_msg->header;
                box.header.frame_id = "map";
                box.type = visualization_msgs::Marker::LINE_STRIP;
                box.action = visualization_msgs::Marker::ADD;
                box.id = count;
                box.ns = "small_box";
                box.scale.x = 0.1;
                box.pose.orientation.x = 0.0;
                box.pose.orientation.y = 0.0;
                box.pose.orientation.z = 0.0;
                box.pose.orientation.w = 1.0;
                box.color.a = 0.5;
                box.color.g = 1.0;
                box.color.b = 0.0;
                box.color.r = 0.0;
                for (auto const &point : polygon.polygon.points)
                {
                    geometry_msgs::Point tmp_point;
                    tmp_point.x = point.x;
                    tmp_point.y = point.y;
                    tmp_point.z = point.z;
                    box.points.push_back(tmp_point);
                }
                boxes.markers.push_back(box);
                count++;
            }
        }
        else
        {
            //计算凸包
            Eigen::Vector4f min, max;
            pcl::getMinMax3D(*cloud_clusters[i], min, max);
            geometry_msgs::PolygonStamped polygon;
            for (size_t j = 0; j < objects[i]->polygon.points.size() + 1; j++)
            {
                geometry_msgs::Point32 point;
                point.x = objects[i]->polygon.points[j % objects[i]->polygon.points.size()].x;
                point.y = objects[i]->polygon.points[j % objects[i]->polygon.points.size()].y;
                point.z = min[2];

                polygon.polygon.points.push_back(point);
            }

            for (size_t j = 0; j < objects[i]->polygon.points.size() + 1; j++)
            {
                geometry_msgs::Point32 point;
                point.x = objects[i]->polygon.points[j % (objects[i]->polygon.points.size())].x;
                point.y = objects[i]->polygon.points[j % (objects[i]->polygon.points.size())].y;
                point.z = max[2];

                polygon.polygon.points.push_back(point);
            }
            visualization_msgs::Marker box;
            box.lifetime = ros::Duration(0.1);
            box.header = cloud_msg->header;
            box.header.frame_id = "map";
            box.type = visualization_msgs::Marker::LINE_STRIP;
            box.action = visualization_msgs::Marker::ADD;
            box.id = count;
            box.ns = "polygongal_box";
            box.scale.x = 0.1;
            box.pose.orientation.x = 0.0;
            box.pose.orientation.y = 0.0;
            box.pose.orientation.z = 0.0;
            box.pose.orientation.w = 1.0;
            box.color.a = 0.5;
            box.color.g = 1.0;
            box.color.b = 0.0;
            box.color.r = 0.0;
            for (auto const &point : polygon.polygon.points)
            {
                geometry_msgs::Point tmp_point;
                tmp_point.x = point.x;
                tmp_point.y = point.y;
                tmp_point.z = point.z;
                box.points.push_back(tmp_point);
            }
            boxes.markers.push_back(box);
            count++;
        }
    }
    pub_box.publish(boxes);
    clock_t end = clock();
    ROS_INFO("Total time: %lf  s", (double)(end - start) / CLOCKS_PER_SEC);

    ROS_INFO("FPS: %lf  Hz", (double)CLOCKS_PER_SEC / (end - start));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<string>("/cloud_topic", cloud_topic, "/pointcloud");

    cout << "Operating patchwork++..." << endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&nh));

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/demo/cloud", 100, true);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/demo/ground", 100, true);
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("/demo/nonground", 100, true);
    pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/demo/cluster", 100, true);
    pub_box = nh.advertise<visualization_msgs::MarkerArray>("/box", 100, true);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
