#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include "CVC.h"
#include "clip.h"
#include "detect_object.h"
#include "patchworkpp/patchworkpp.hpp"

#include "object_builders/base_object_builder.hpp"
#include "object_builders/object_builder_manager.hpp"

#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/Centroids.h"
#include "autoware_msgs/CloudCluster.h"
#include "autoware_msgs/CloudClusterArray.h"
#include "autoware_msgs/DetectedObject.h"
#include "autoware_msgs/DetectedObjectArray.h"

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
using namespace cv;

std::vector<cv::Scalar> _colors;

static double _cluster_merge_threshold;

std::string _output_frame;

std_msgs::Header _velodyne_header;

boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

tf::StampedTransform *_transform;
tf::TransformListener *_transform_listener;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;
ros::Publisher pub_cluster;
ros::Publisher pub_box;
ros::Publisher pub_detected_objects;
ros::Publisher pub_centroid;
ros::Publisher pub_clusters_message;

template <typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "livox_frame")
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

static void downsamplePoints(const Mat &src, Mat &dst, size_t count)
{
    CV_Assert(count >= 2);
    CV_Assert(src.cols == 1 || src.rows == 1);
    CV_Assert(src.total() >= count);
    CV_Assert(src.type() == CV_8UC3);

    dst.create(1, (int)count, CV_8UC3);
    // TODO: optimize by exploiting symmetry in the distance matrix
    Mat dists((int)src.total(), (int)src.total(), CV_32FC1, Scalar(0));
    if (dists.empty())
        std::cerr << "Such big matrix cann't be created." << std::endl;

    for (int i = 0; i < dists.rows; i++)
    {
        for (int j = i; j < dists.cols; j++)
        {
            float dist = (float)norm(src.at<Point3_<uchar>>(i) - src.at<Point3_<uchar>>(j));
            dists.at<float>(j, i) = dists.at<float>(i, j) = dist;
        }
    }

    double maxVal;
    Point maxLoc;
    minMaxLoc(dists, 0, &maxVal, 0, &maxLoc);

    dst.at<Point3_<uchar>>(0) = src.at<Point3_<uchar>>(maxLoc.x);
    dst.at<Point3_<uchar>>(1) = src.at<Point3_<uchar>>(maxLoc.y);

    Mat activedDists(0, dists.cols, dists.type());
    Mat candidatePointsMask(1, dists.cols, CV_8UC1, Scalar(255));
    activedDists.push_back(dists.row(maxLoc.y));
    candidatePointsMask.at<uchar>(0, maxLoc.y) = 0;

    for (size_t i = 2; i < count; i++)
    {
        activedDists.push_back(dists.row(maxLoc.x));
        candidatePointsMask.at<uchar>(0, maxLoc.x) = 0;

        Mat minDists;
        reduce(activedDists, minDists, 0, cv::REDUCE_MIN);
        minMaxLoc(minDists, 0, &maxVal, 0, &maxLoc, candidatePointsMask);
        dst.at<Point3_<uchar>>((int)i) = src.at<Point3_<uchar>>(maxLoc.x);
    }
}

void generateColors(std::vector<cv::Scalar> &colors, size_t count, size_t factor = 100)
{
    if (count < 1)
        return;

    colors.resize(count);

    if (count == 1)
    {
        colors[0] = cv::Scalar(0, 0, 255); // red
        return;
    }
    if (count == 2)
    {
        colors[0] = cv::Scalar(0, 0, 255); // red
        colors[1] = cv::Scalar(0, 255, 0); // green
        return;
    }

    // Generate a set of colors in RGB space. A size of the set is severel times (=factor) larger then
    // the needed count of colors.
    cv::Mat bgr(1, (int)(count * factor), CV_8UC3);
    cv::randu(bgr, 0, 256);

    // Convert the colors set to Lab space.
    // Distances between colors in this space correspond a human perception.
    cv::Mat lab;
    cv::cvtColor(bgr, lab, cv::COLOR_BGR2Lab);

    // Subsample colors from the generated set so that
    // to maximize the minimum distances between each other.
    // Douglas-Peucker algorithm is used for this.
    cv::Mat lab_subset;
    downsamplePoints(lab, lab_subset, count);

    // Convert subsampled colors back to RGB
    cv::Mat bgr_subset;
    cv::cvtColor(lab_subset, bgr_subset, cv::COLOR_BGR2Lab);

    CV_Assert(bgr_subset.total() == count);
    for (size_t i = 0; i < count; i++)
    {
        cv::Point3_<uchar> c = bgr_subset.at<cv::Point3_<uchar>>((int)i);
        colors[i] = cv::Scalar(c.x, c.y, c.z);
    }
}

geometry_msgs::Point transformPoint(const geometry_msgs::Point &point, const tf::Transform &tf)
{
    tf::Point tf_point;
    tf::pointMsgToTF(point, tf_point);

    tf_point = tf * tf_point;

    geometry_msgs::Point ros_point;
    tf::pointTFToMsg(tf_point, ros_point);

    return ros_point;
}

void transformBoundingBox(const jsk_recognition_msgs::BoundingBox &in_boundingbox,
                          jsk_recognition_msgs::BoundingBox &out_boundingbox, const std::string &in_target_frame,
                          const std_msgs::Header &in_header)
{
    geometry_msgs::PoseStamped pose_in, pose_out;
    pose_in.header = in_header;
    pose_in.pose = in_boundingbox.pose;
    try
    {
        _transform_listener->transformPose(in_target_frame, ros::Time(), pose_in, in_header.frame_id, pose_out);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("transformBoundingBox: %s", ex.what());
    }
    out_boundingbox.pose = pose_out.pose;
    out_boundingbox.header = in_header;
    out_boundingbox.header.frame_id = in_target_frame;
    out_boundingbox.dimensions = in_boundingbox.dimensions;
    out_boundingbox.value = in_boundingbox.value;
    out_boundingbox.label = in_boundingbox.label;
}

void publishDetectedObjects(const autoware_msgs::CloudClusterArray &in_clusters)
{
    autoware_msgs::DetectedObjectArray detected_objects;
    detected_objects.header = in_clusters.header;

    for (size_t i = 0; i < in_clusters.clusters.size(); i++)
    {
        autoware_msgs::DetectedObject detected_object;
        detected_object.header = in_clusters.header;
        detected_object.label = "unknown";
        detected_object.score = 1.;
        detected_object.space_frame = in_clusters.header.frame_id;
        detected_object.pose = in_clusters.clusters[i].bounding_box.pose;
        detected_object.dimensions = in_clusters.clusters[i].dimensions;
        detected_object.pointcloud = in_clusters.clusters[i].cloud;
        detected_object.convex_hull = in_clusters.clusters[i].convex_hull;
        detected_object.valid = true;

        detected_objects.objects.push_back(detected_object);
    }
    pub_detected_objects.publish(detected_objects);
}

void publishCloudClusters(const ros::Publisher *in_publisher, const autoware_msgs::CloudClusterArray &in_clusters,
                          const std::string &in_target_frame, const std_msgs::Header &in_header)
{
    double _initial_quat_w = 1.0;

    if (in_target_frame != in_header.frame_id)
    {
        autoware_msgs::CloudClusterArray clusters_transformed;
        clusters_transformed.header = in_header;
        clusters_transformed.header.frame_id = in_target_frame;
        for (auto i = in_clusters.clusters.begin(); i != in_clusters.clusters.end(); i++)
        {
            autoware_msgs::CloudCluster cluster_transformed;
            cluster_transformed.header = in_header;
            try
            {
                _transform_listener->lookupTransform(in_target_frame, _velodyne_header.frame_id, ros::Time(),
                                                     *_transform);
                pcl_ros::transformPointCloud(in_target_frame, *_transform, i->cloud, cluster_transformed.cloud);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->min_point, in_header.frame_id,
                                                    cluster_transformed.min_point);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->max_point, in_header.frame_id,
                                                    cluster_transformed.max_point);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->avg_point, in_header.frame_id,
                                                    cluster_transformed.avg_point);
                _transform_listener->transformPoint(in_target_frame, ros::Time(), i->centroid_point, in_header.frame_id,
                                                    cluster_transformed.centroid_point);

                cluster_transformed.dimensions = i->dimensions;
                cluster_transformed.eigen_values = i->eigen_values;
                cluster_transformed.eigen_vectors = i->eigen_vectors;

                cluster_transformed.convex_hull = i->convex_hull;
                cluster_transformed.bounding_box.pose.position = i->bounding_box.pose.position;
                if (true)
                {
                    cluster_transformed.bounding_box.pose.orientation = i->bounding_box.pose.orientation;
                }
                else
                {
                    cluster_transformed.bounding_box.pose.orientation.w = _initial_quat_w;
                }
                clusters_transformed.clusters.push_back(cluster_transformed);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("publishCloudClusters: %s", ex.what());
            }
        }
        in_publisher->publish(clusters_transformed);
        publishDetectedObjects(clusters_transformed);
    }
    else
    {
        in_publisher->publish(in_clusters);
        publishDetectedObjects(in_clusters);
    }
}

void publishCentroids(const ros::Publisher *in_publisher, const autoware_msgs::Centroids &in_centroids,
                      const std::string &in_target_frame, const std_msgs::Header &in_header)
{
    if (in_target_frame != in_header.frame_id)
    {
        autoware_msgs::Centroids centroids_transformed;
        centroids_transformed.header = in_header;
        centroids_transformed.header.frame_id = in_target_frame;
        for (auto i = centroids_transformed.points.begin(); i != centroids_transformed.points.end(); i++)
        {
            geometry_msgs::PointStamped centroid_in, centroid_out;
            centroid_in.header = in_header;
            centroid_in.point = *i;
            try
            {
                _transform_listener->transformPoint(in_target_frame, ros::Time(), centroid_in, in_header.frame_id,
                                                    centroid_out);

                centroids_transformed.points.push_back(centroid_out.point);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("publishCentroids: %s", ex.what());
            }
        }
        in_publisher->publish(centroids_transformed);
    }
    else
    {
        in_publisher->publish(in_centroids);
    }
}

void publishCloud(const ros::Publisher *in_publisher, const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;
    in_publisher->publish(cloud_msg);
}

void publishColorCloud(const ros::Publisher *in_publisher,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_to_publish_ptr)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = _velodyne_header;
    in_publisher->publish(cloud_msg);
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;
    float theta = -M_PI / 38; // 旋转弧度s
    // float theta = 0; // 旋转弧度s

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
    _velodyne_header = cloud_msg->header;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX())); //同理，UnitX(),绕X轴；UnitY(),绕Y轴
    pcl::transformPointCloud(pc_curr, transformed_cloud, transform);

    double start = ros::Time::now().toSec();

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

    /////////////////////////////////////////// autoware bbox ////////////////////////////////////////////////////

    std::vector<ClusterPtr> all_clusters;

    for (int i = 0; i < cluster_index.size(); ++i)
    {
        pcl::PointCloud<PointType> in_origin_cloud_cluster;

        for (int j = 0; j < cluster_indices.size(); ++j)
        {
            if (cluster_indices[j] == cluster_index[i])
            {
                in_origin_cloud_cluster.points.push_back(pc_non_ground.points[j]);
            }
        }
        ClusterPtr cluster(new Cluster());
        cluster->SetCloud_cluster(in_origin_cloud_cluster, _velodyne_header, i, (int)_colors[i].val[0],
                                  (int)_colors[i].val[1],
                                  (int)_colors[i].val[2], "", true);
        all_clusters.push_back(cluster);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    autoware_msgs::Centroids centroids;
    autoware_msgs::CloudClusterArray cloud_clusters;

    detect_object(all_clusters, colored_clustered_cloud_ptr, centroids, cloud_clusters);

    publishColorCloud(&pub_cluster, colored_clustered_cloud_ptr);

    centroids.header = _velodyne_header;

    publishCentroids(&pub_centroid, centroids, _output_frame, _velodyne_header);

    cloud_clusters.header = _velodyne_header;

    publishCloudClusters(&pub_clusters_message, cloud_clusters, _output_frame, _velodyne_header);

    /*------------------------------------------------------------------------------------------------------------------------ */

    // std::vector<PtC::Ptr> cloud_clusters; //点云簇列表
    // //遍历索引表，寻找点数超过10的点云簇，并传入cloud_clusters

    // for (int i = 0; i < cluster_index.size(); ++i)
    // {
    //     pcl::PointCloud<PointType> cluster;
    //     PtC::Ptr cluster_ptr(new PtC);

    //     for (int j = 0; j < cluster_indices.size(); ++j)
    //     {
    //         if (cluster_indices[j] == cluster_index[i])
    //         {
    //             cluster.points.push_back(pc_non_ground.points[j]);
    //         }
    //     }

    //     pcl::copyPointCloud(cluster, *cluster_ptr);
    //     cloud_clusters.push_back(cluster_ptr);
    // }

    // PtC::Ptr clusters(new PtC);
    // for (int i = 0; i < cloud_clusters.size(); ++i)
    // {
    //     *clusters += *cloud_clusters[i];
    // }
    // double t1 = ros::Time::now().toSec();
    // ROS_INFO("Time of clustering: %f ms", (t1 - t0) * 1000);

    // //发布聚类点云
    // pub_cluster.publish(cloud2msg(*clusters));

    // // Apollo bounding box
    // // define object builder
    // boost::shared_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_;

    // // create object builder by manager
    // object_builder_ = autosense::object_builder::createObjectBuilder();

    // // build 3D orientation bounding box for clustering point cloud

    // std::vector<autosense::ObjectPtr> objects;
    // object_builder_->build(cloud_clusters, &objects);

    // //将大型包围盒划分成几个小的包围盒
    // visualization_msgs::MarkerArray boxes;
    // int count = 0;
    // for (int i = 0; i < objects.size(); ++i)
    // {
    //     if (objects[i]->length > 10)
    //     {
    //         vector<PtC::Ptr> small_clusters;

    //         BigBox2SmallBox(objects[i], small_clusters);

    //         for (int j = 0; j < small_clusters.size(); ++j)
    //         {
    //             Eigen::Vector4f min, max;
    //             pcl::getMinMax3D(*small_clusters[j], min, max);
    //             std::vector<cv::Point2f> points;
    //             for (int k = 0; k < small_clusters[j]->size(); ++k)
    //             {
    //                 cv::Point2f pt;
    //                 pt.x = small_clusters[j]->points[k].x;
    //                 pt.y = small_clusters[j]->points[k].y;
    //                 points.push_back(pt);
    //             }

    //             std::vector<cv::Point2f> hull;
    //             cv::convexHull(points, hull);
    //             geometry_msgs::PolygonStamped polygon;
    //             for (size_t k = 0; k < hull.size() + 1; k++)
    //             {
    //                 geometry_msgs::Point32 point;
    //                 point.x = hull[k % hull.size()].x;
    //                 point.y = hull[k % hull.size()].y;
    //                 point.z = min[2];

    //                 polygon.polygon.points.push_back(point);
    //             }

    //             for (size_t k = 0; k < hull.size() + 1; k++)
    //             {
    //                 geometry_msgs::Point32 point;
    //                 point.x = hull[k % hull.size()].x;
    //                 point.y = hull[k % hull.size()].y;
    //                 point.z = max[2];

    //                 polygon.polygon.points.push_back(point);
    //             }
    //             visualization_msgs::Marker box;
    //             box.lifetime = ros::Duration(0.1);
    //             box.header = cloud_msg->header;
    //             box.header.frame_id = "map";
    //             box.type = visualization_msgs::Marker::LINE_STRIP;
    //             box.action = visualization_msgs::Marker::ADD;
    //             box.id = count;
    //             box.ns = "small_box";
    //             box.scale.x = 0.1;
    //             box.pose.orientation.x = 0.0;
    //             box.pose.orientation.y = 0.0;
    //             box.pose.orientation.z = 0.0;
    //             box.pose.orientation.w = 1.0;
    //             box.color.a = 0.8;
    //             box.color.g = 1.0;
    //             box.color.b = 0.0;
    //             box.color.r = 0.0;
    //             for (auto const &point : polygon.polygon.points)
    //             {
    //                 geometry_msgs::Point tmp_point;
    //                 tmp_point.x = point.x;
    //                 tmp_point.y = point.y;
    //                 tmp_point.z = point.z;
    //                 box.points.push_back(tmp_point);
    //             }
    //             boxes.markers.push_back(box);
    //             count++;
    //         }
    //     }
    //     else
    //     {
    //         //计算凸包
    //         Eigen::Vector4f min, max;
    //         pcl::getMinMax3D(*cloud_clusters[i], min, max);
    //         geometry_msgs::PolygonStamped polygon;
    //         for (size_t j = 0; j < objects[i]->polygon.points.size() + 1; j++)
    //         {
    //             geometry_msgs::Point32 point;
    //             point.x = objects[i]->polygon.points[j % objects[i]->polygon.points.size()].x;
    //             point.y = objects[i]->polygon.points[j % objects[i]->polygon.points.size()].y;
    //             point.z = min[2];

    //             polygon.polygon.points.push_back(point);
    //         }

    //         for (size_t j = 0; j < objects[i]->polygon.points.size() + 1; j++)
    //         {
    //             geometry_msgs::Point32 point;
    //             point.x = objects[i]->polygon.points[j % (objects[i]->polygon.points.size())].x;
    //             point.y = objects[i]->polygon.points[j % (objects[i]->polygon.points.size())].y;
    //             point.z = max[2];

    //             polygon.polygon.points.push_back(point);
    //         }
    //         visualization_msgs::Marker box;
    //         box.lifetime = ros::Duration(0.1);
    //         box.header = cloud_msg->header;
    //         box.header.frame_id = "map";
    //         box.type = visualization_msgs::Marker::LINE_STRIP;
    //         box.action = visualization_msgs::Marker::ADD;
    //         box.id = count;
    //         box.ns = "polygongal_box";
    //         box.scale.x = 0.1;
    //         box.pose.orientation.x = 0.0;
    //         box.pose.orientation.y = 0.0;
    //         box.pose.orientation.z = 0.0;
    //         box.pose.orientation.w = 1.0;
    //         box.color.a = 0.5;
    //         box.color.g = 1.0;
    //         box.color.b = 0.0;
    //         box.color.r = 0.0;
    //         for (auto const &point : polygon.polygon.points)
    //         {
    //             geometry_msgs::Point tmp_point;
    //             tmp_point.x = point.x;
    //             tmp_point.y = point.y;
    //             tmp_point.z = point.z;
    //             box.points.push_back(tmp_point);
    //         }
    //         boxes.markers.push_back(box);
    //         count++;
    //     }
    // }
    // pub_box.publish(boxes);

    /*--------------------------------------------------------------------------------------------------------------*/

    pub_cloud.publish(cloud2msg(transformed_cloud));
    pub_ground.publish(cloud2msg(pc_ground));
    pub_non_ground.publish(cloud2msg(pc_non_ground));
    double end = ros::Time::now().toSec();
    ROS_INFO("Total time: %lf  s", (end - start));

    ROS_INFO("FPS: %lf  Hz", (double)1 / (end - start));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;

    tf::StampedTransform transform;
    tf::TransformListener listener;

    std::string cloud_topic;
    nh.param<string>("/cloud_topic", cloud_topic, "/pointcloud");
    _output_frame = "livox_frame";

    std::cout << "Operating patchwork++..." << std::endl;
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&nh));
    generateColors(_colors, 255, 100);

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/demo/cloud", 100, true);
    pub_ground = nh.advertise<sensor_msgs::PointCloud2>("/demo/ground", 100, true);
    pub_non_ground = nh.advertise<sensor_msgs::PointCloud2>("/demo/nonground", 100, true);
    pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/demo/cluster", 100, true);
    // pub_box = nh.advertise<visualization_msgs::MarkerArray>("/box", 100, true);
    pub_detected_objects = nh.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1);
    pub_clusters_message = nh.advertise<autoware_msgs::CloudClusterArray>("/detection/lidar_detector/cloud_clusters", 1);
    pub_centroid = nh.advertise<autoware_msgs::Centroids>("/cluster_centroids", 1);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}
