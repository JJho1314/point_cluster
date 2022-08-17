#include "detect_object.h"

static double _cluster_merge_threshold = 1.5;

Cluster::Cluster()
{
    valid_cluster_ = true;
}

geometry_msgs::PolygonStamped Cluster::GetPolygon()
{
    return polygon_;
}

jsk_recognition_msgs::BoundingBox Cluster::GetBoundingBox()
{
    return bounding_box_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cluster::GetCloud()
{
    return pointcloud_;
}

pcl::PointXYZ Cluster::GetMinPoint()
{
    return min_point_;
}

pcl::PointXYZ Cluster::GetMaxPoint()
{
    return max_point_;
}

pcl::PointXYZ Cluster::GetCentroid()
{
    return centroid_;
}

pcl::PointXYZ Cluster::GetAveragePoint()
{
    return average_point_;
}

double Cluster::GetOrientationAngle()
{
    return orientation_angle_;
}

Eigen::Matrix3f Cluster::GetEigenVectors()
{
    return eigen_vectors_;
}

Eigen::Vector3f Cluster::GetEigenValues()
{
    return eigen_values_;
}

void Cluster::ToROSMessage(std_msgs::Header in_ros_header, autoware_msgs::CloudCluster &out_cluster_message)
{
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(*(this->GetCloud()), cloud_msg);
    cloud_msg.header = in_ros_header;

    out_cluster_message.header = in_ros_header;

    out_cluster_message.cloud = cloud_msg;
    out_cluster_message.min_point.header = in_ros_header;
    out_cluster_message.min_point.point.x = this->GetMinPoint().x;
    out_cluster_message.min_point.point.y = this->GetMinPoint().y;
    out_cluster_message.min_point.point.z = this->GetMinPoint().z;

    out_cluster_message.max_point.header = in_ros_header;
    out_cluster_message.max_point.point.x = this->GetMaxPoint().x;
    out_cluster_message.max_point.point.y = this->GetMaxPoint().y;
    out_cluster_message.max_point.point.z = this->GetMaxPoint().z;

    out_cluster_message.avg_point.header = in_ros_header;
    out_cluster_message.avg_point.point.x = this->GetAveragePoint().x;
    out_cluster_message.avg_point.point.y = this->GetAveragePoint().y;
    out_cluster_message.avg_point.point.z = this->GetAveragePoint().z;

    out_cluster_message.centroid_point.header = in_ros_header;
    out_cluster_message.centroid_point.point.x = this->GetCentroid().x;
    out_cluster_message.centroid_point.point.y = this->GetCentroid().y;
    out_cluster_message.centroid_point.point.z = this->GetCentroid().z;

    out_cluster_message.estimated_angle = this->GetOrientationAngle();

    out_cluster_message.dimensions = this->GetBoundingBox().dimensions;

    out_cluster_message.bounding_box = this->GetBoundingBox();

    out_cluster_message.convex_hull = this->GetPolygon();

    Eigen::Vector3f eigen_values = this->GetEigenValues();
    out_cluster_message.eigen_values.x = eigen_values.x();
    out_cluster_message.eigen_values.y = eigen_values.y();
    out_cluster_message.eigen_values.z = eigen_values.z();

    Eigen::Matrix3f eigen_vectors = this->GetEigenVectors();
    for (unsigned int i = 0; i < 3; i++)
    {
        geometry_msgs::Vector3 eigen_vector;
        eigen_vector.x = eigen_vectors(i, 0);
        eigen_vector.y = eigen_vectors(i, 1);
        eigen_vector.z = eigen_vectors(i, 2);
        out_cluster_message.eigen_vectors.push_back(eigen_vector);
    }

    /*std::vector<float> fpfh_descriptor = GetFpfhDescriptor(8, 0.3, 0.3);
    out_cluster_message.fpfh_descriptor.data = fpfh_descriptor;*/
}

void Cluster::SetCloud_cluster(const pcl::PointCloud<pcl::PointXYZ> in_origin_cloud_cluster,
                               std_msgs::Header in_ros_header, int in_id, int in_r,
                               int in_g, int in_b, std::string in_label, bool in_estimate_pose)
{
    label_ = in_label;
    id_ = in_id;
    r_ = in_r;
    g_ = in_g;
    b_ = in_b;
    // extract pointcloud using the indices
    // calculate min and max points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0;

    for (int i = 0; i < in_origin_cloud_cluster.size(); i++)
    {
        // fill new colored cluster point by point
        pcl::PointXYZRGB p;
        p.x = in_origin_cloud_cluster[i].x;
        p.y = in_origin_cloud_cluster[i].y;
        p.z = in_origin_cloud_cluster[i].z;
        p.r = in_r;
        p.g = in_g;
        p.b = in_b;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        centroid_.x += p.x;
        centroid_.y += p.y;
        centroid_.z += p.z;
        current_cluster->points.push_back(p);

        if (p.x < min_x)
            min_x = p.x;
        if (p.y < min_y)
            min_y = p.y;
        if (p.z < min_z)
            min_z = p.z;
        if (p.x > max_x)
            max_x = p.x;
        if (p.y > max_y)
            max_y = p.y;
        if (p.z > max_z)
            max_z = p.z;
    }
    // min, max points
    min_point_.x = min_x;
    min_point_.y = min_y;
    min_point_.z = min_z;
    max_point_.x = max_x;
    max_point_.y = max_y;
    max_point_.z = max_z;

    // calculate centroid, average
    if (in_origin_cloud_cluster.size() > 0)
    {
        centroid_.x /= in_origin_cloud_cluster.size();
        centroid_.y /= in_origin_cloud_cluster.size();
        centroid_.z /= in_origin_cloud_cluster.size();

        average_x /= in_origin_cloud_cluster.size();
        average_y /= in_origin_cloud_cluster.size();
        average_z /= in_origin_cloud_cluster.size();
    }

    average_point_.x = average_x;
    average_point_.y = average_y;
    average_point_.z = average_z;

    // calculate bounding box
    length_ = max_point_.x - min_point_.x;
    width_ = max_point_.y - min_point_.y;
    height_ = max_point_.z - min_point_.z;

    bounding_box_.header = in_ros_header;

    bounding_box_.pose.position.x = min_point_.x + length_ / 2;
    bounding_box_.pose.position.y = min_point_.y + width_ / 2;
    bounding_box_.pose.position.z = min_point_.z + height_ / 2;

    bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
    bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
    bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

    // pose estimation
    double rz = 0;

    {
        std::vector<cv::Point2f> points;
        for (unsigned int i = 0; i < current_cluster->points.size(); i++)
        {
            cv::Point2f pt;
            pt.x = current_cluster->points[i].x;
            pt.y = current_cluster->points[i].y;
            points.push_back(pt);
        }

        std::vector<cv::Point2f> hull;
        cv::convexHull(points, hull);

        polygon_.header = in_ros_header;
        for (size_t i = 0; i < hull.size() + 1; i++)
        {
            geometry_msgs::Point32 point;
            point.x = hull[i % hull.size()].x;
            point.y = hull[i % hull.size()].y;
            point.z = min_point_.z;
            polygon_.polygon.points.push_back(point);
        }

        if (in_estimate_pose)
        {
            cv::RotatedRect box = minAreaRect(hull);
            rz = box.angle * 3.14 / 180;
            bounding_box_.pose.position.x = box.center.x;
            bounding_box_.pose.position.y = box.center.y;
            bounding_box_.dimensions.x = box.size.width;
            bounding_box_.dimensions.y = box.size.height;
        }
    }

    // set bounding box direction
    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
    tf::quaternionTFToMsg(quat, bounding_box_.pose.orientation);

    current_cluster->width = current_cluster->points.size();
    current_cluster->height = 1;
    current_cluster->is_dense = true;

    // Get EigenValues, eigenvectors
    if (current_cluster->points.size() > 3)
    {
        pcl::PCA<pcl::PointXYZ> current_cluster_pca;
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster_mono(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(*current_cluster, *current_cluster_mono);

        current_cluster_pca.setInputCloud(current_cluster_mono);
        eigen_vectors_ = current_cluster_pca.getEigenVectors();
        eigen_values_ = current_cluster_pca.getEigenValues();
    }

    valid_cluster_ = true;
    pointcloud_ = current_cluster;
}

void Cluster::SetCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_origin_cloud_ptr,
                       const std::vector<int> &in_cluster_indices, std_msgs::Header in_ros_header, int in_id, int in_r,
                       int in_g, int in_b, std::string in_label, bool in_estimate_pose)
{
    label_ = in_label;
    id_ = in_id;
    r_ = in_r;
    g_ = in_g;
    b_ = in_b;
    // extract pointcloud using the indices
    // calculate min and max points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    float average_x = 0, average_y = 0, average_z = 0;

    for (auto pit = in_cluster_indices.begin(); pit != in_cluster_indices.end(); ++pit)
    {
        // fill new colored cluster point by point
        pcl::PointXYZRGB p;
        p.x = in_origin_cloud_ptr->points[*pit].x;
        p.y = in_origin_cloud_ptr->points[*pit].y;
        p.z = in_origin_cloud_ptr->points[*pit].z;
        p.r = in_r;
        p.g = in_g;
        p.b = in_b;

        average_x += p.x;
        average_y += p.y;
        average_z += p.z;
        centroid_.x += p.x;
        centroid_.y += p.y;
        centroid_.z += p.z;
        current_cluster->points.push_back(p);

        if (p.x < min_x)
            min_x = p.x;
        if (p.y < min_y)
            min_y = p.y;
        if (p.z < min_z)
            min_z = p.z;
        if (p.x > max_x)
            max_x = p.x;
        if (p.y > max_y)
            max_y = p.y;
        if (p.z > max_z)
            max_z = p.z;
    }
    // min, max points
    min_point_.x = min_x;
    min_point_.y = min_y;
    min_point_.z = min_z;
    max_point_.x = max_x;
    max_point_.y = max_y;
    max_point_.z = max_z;

    // calculate centroid, average
    if (in_cluster_indices.size() > 0)
    {
        centroid_.x /= in_cluster_indices.size();
        centroid_.y /= in_cluster_indices.size();
        centroid_.z /= in_cluster_indices.size();

        average_x /= in_cluster_indices.size();
        average_y /= in_cluster_indices.size();
        average_z /= in_cluster_indices.size();
    }

    average_point_.x = average_x;
    average_point_.y = average_y;
    average_point_.z = average_z;

    // calculate bounding box
    length_ = max_point_.x - min_point_.x;
    width_ = max_point_.y - min_point_.y;
    height_ = max_point_.z - min_point_.z;

    bounding_box_.header = in_ros_header;

    bounding_box_.pose.position.x = min_point_.x + length_ / 2;
    bounding_box_.pose.position.y = min_point_.y + width_ / 2;
    bounding_box_.pose.position.z = min_point_.z + height_ / 2;

    bounding_box_.dimensions.x = ((length_ < 0) ? -1 * length_ : length_);
    bounding_box_.dimensions.y = ((width_ < 0) ? -1 * width_ : width_);
    bounding_box_.dimensions.z = ((height_ < 0) ? -1 * height_ : height_);

    // pose estimation
    double rz = 0;

    {
        std::vector<cv::Point2f> points;
        for (unsigned int i = 0; i < current_cluster->points.size(); i++)
        {
            cv::Point2f pt;
            pt.x = current_cluster->points[i].x;
            pt.y = current_cluster->points[i].y;
            points.push_back(pt);
        }

        std::vector<cv::Point2f> hull;
        cv::convexHull(points, hull);

        polygon_.header = in_ros_header;
        for (size_t i = 0; i < hull.size() + 1; i++)
        {
            geometry_msgs::Point32 point;
            point.x = hull[i % hull.size()].x;
            point.y = hull[i % hull.size()].y;
            point.z = min_point_.z;
            polygon_.polygon.points.push_back(point);
        }

        if (in_estimate_pose)
        {
            cv::RotatedRect box = minAreaRect(hull);
            rz = box.angle * 3.14 / 180;
            bounding_box_.pose.position.x = box.center.x;
            bounding_box_.pose.position.y = box.center.y;
            bounding_box_.dimensions.x = box.size.width;
            bounding_box_.dimensions.y = box.size.height;
        }
    }

    // set bounding box direction
    tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, 0.0, rz);
    tf::quaternionTFToMsg(quat, bounding_box_.pose.orientation);

    current_cluster->width = current_cluster->points.size();
    current_cluster->height = 1;
    current_cluster->is_dense = true;

    // Get EigenValues, eigenvectors
    if (current_cluster->points.size() > 3)
    {
        pcl::PCA<pcl::PointXYZ> current_cluster_pca;
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cluster_mono(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::copyPointCloud<pcl::PointXYZRGB, pcl::PointXYZ>(*current_cluster, *current_cluster_mono);

        current_cluster_pca.setInputCloud(current_cluster_mono);
        eigen_vectors_ = current_cluster_pca.getEigenVectors();
        eigen_values_ = current_cluster_pca.getEigenValues();
    }

    valid_cluster_ = true;
    pointcloud_ = current_cluster;
}

void checkClusterMerge(size_t in_cluster_id, std::vector<ClusterPtr> &in_clusters,
                       std::vector<bool> &in_out_visited_clusters, std::vector<size_t> &out_merge_indices,
                       double in_merge_threshold)
{
    // std::cout << "checkClusterMerge" << std::endl;
    pcl::PointXYZ point_a = in_clusters[in_cluster_id]->GetCentroid();
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
        if (i != in_cluster_id && !in_out_visited_clusters[i])
        {
            pcl::PointXYZ point_b = in_clusters[i]->GetCentroid();
            double distance = sqrt(pow(point_b.x - point_a.x, 2) + pow(point_b.y - point_a.y, 2));
            if (distance <= in_merge_threshold)
            {
                in_out_visited_clusters[i] = true;
                out_merge_indices.push_back(i);
                // std::cout << "Merging " << in_cluster_id << " with " << i << " dist:" << distance << std::endl;
                checkClusterMerge(i, in_clusters, in_out_visited_clusters, out_merge_indices, in_merge_threshold);
            }
        }
    }
}

void mergeClusters(const std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                   std::vector<size_t> in_merge_indices, const size_t &current_index,
                   std::vector<bool> &in_out_merged_clusters)
{
    // std::cout << "mergeClusters:" << in_merge_indices.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB> sum_cloud;
    pcl::PointCloud<pcl::PointXYZ> mono_cloud;
    ClusterPtr merged_cluster(new Cluster());
    for (size_t i = 0; i < in_merge_indices.size(); i++)
    {
        sum_cloud += *(in_clusters[in_merge_indices[i]]->GetCloud());
        in_out_merged_clusters[in_merge_indices[i]] = true;
    }
    std::vector<int> indices(sum_cloud.points.size(), 0);
    for (size_t i = 0; i < sum_cloud.points.size(); i++)
    {
        indices[i] = i;
    }

    if (sum_cloud.points.size() > 0)
    {
        pcl::copyPointCloud(sum_cloud, mono_cloud);
        merged_cluster->SetCloud(mono_cloud.makeShared(), indices, _velodyne_header, current_index,
                                 (int)_colors[current_index].val[0], (int)_colors[current_index].val[1],
                                 (int)_colors[current_index].val[2], "", true);
        out_clusters.push_back(merged_cluster);
    }
}

void checkAllForMerge(std::vector<ClusterPtr> &in_clusters, std::vector<ClusterPtr> &out_clusters,
                      float in_merge_threshold)
{
    // std::cout << "checkAllForMerge" << std::endl;
    std::vector<bool> visited_clusters(in_clusters.size(), false);
    std::vector<bool> merged_clusters(in_clusters.size(), false);
    size_t current_index = 0;
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
        if (!visited_clusters[i])
        {
            visited_clusters[i] = true;
            std::vector<size_t> merge_indices;
            checkClusterMerge(i, in_clusters, visited_clusters, merge_indices, in_merge_threshold);
            mergeClusters(in_clusters, out_clusters, merge_indices, current_index++, merged_clusters);
        }
    }
    for (size_t i = 0; i < in_clusters.size(); i++)
    {
        // check for clusters not merged, add them to the output
        if (!merged_clusters[i])
        {
            out_clusters.push_back(in_clusters[i]);
        }
    }

    // ClusterPtr cluster(new Cluster());
}

void detect_object(std::vector<ClusterPtr> all_clusters,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud_ptr,
                   autoware_msgs::Centroids &in_out_centroids, autoware_msgs::CloudClusterArray &in_out_clusters)
{
    // Clusters can be merged or checked in here
    //....
    // check for mergable clusters
    std::vector<ClusterPtr> mid_clusters;
    std::vector<ClusterPtr> final_clusters;

    if (all_clusters.size() > 0)
        checkAllForMerge(all_clusters, mid_clusters, _cluster_merge_threshold);
    else
        mid_clusters = all_clusters;

    if (mid_clusters.size() > 0)
        checkAllForMerge(mid_clusters, final_clusters, _cluster_merge_threshold);
    else
        final_clusters = mid_clusters;

    // Get final PointCloud to be published
    for (unsigned int i = 0; i < final_clusters.size(); i++)
    {
        *out_cloud_ptr = *out_cloud_ptr + *(final_clusters[i]->GetCloud());

        jsk_recognition_msgs::BoundingBox bounding_box = final_clusters[i]->GetBoundingBox();
        geometry_msgs::PolygonStamped polygon = final_clusters[i]->GetPolygon();
        jsk_rviz_plugins::Pictogram pictogram_cluster;
        pictogram_cluster.header = _velodyne_header;

        // PICTO
        pictogram_cluster.mode = pictogram_cluster.STRING_MODE;
        pictogram_cluster.pose.position.x = final_clusters[i]->GetMaxPoint().x;
        pictogram_cluster.pose.position.y = final_clusters[i]->GetMaxPoint().y;
        pictogram_cluster.pose.position.z = final_clusters[i]->GetMaxPoint().z;
        tf::Quaternion quat(0.0, -0.7, 0.0, 0.7);
        tf::quaternionTFToMsg(quat, pictogram_cluster.pose.orientation);
        pictogram_cluster.size = 4;
        std_msgs::ColorRGBA color;
        color.a = 1;
        color.r = 1;
        color.g = 1;
        color.b = 1;
        pictogram_cluster.color = color;
        pictogram_cluster.character = std::to_string(i);
        // PICTO

        // pcl::PointXYZ min_point = final_clusters[i]->GetMinPoint();
        // pcl::PointXYZ max_point = final_clusters[i]->GetMaxPoint();
        pcl::PointXYZ center_point = final_clusters[i]->GetCentroid();
        geometry_msgs::Point centroid;
        centroid.x = center_point.x;
        centroid.y = center_point.y;
        centroid.z = center_point.z;
        bounding_box.header = _velodyne_header;
        polygon.header = _velodyne_header;

        if (final_clusters[i]->IsValid())
        {
            in_out_centroids.points.push_back(centroid);
            autoware_msgs::CloudCluster cloud_cluster;
            final_clusters[i]->ToROSMessage(_velodyne_header, cloud_cluster);
            in_out_clusters.clusters.push_back(cloud_cluster);
        }
    }
}

std::vector<float> Cluster::GetFpfhDescriptor(const unsigned int &in_ompnum_threads,
                                              const double &in_normal_search_radius,
                                              const double &in_fpfh_search_radius)
{
    std::vector<float> cluster_fpfh_histogram(33, 0.0);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr norm_tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    if (pointcloud_->points.size() > 0)
        norm_tree->setInputCloud(pointcloud_);

    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
    normal_estimation.setNumberOfThreads(in_ompnum_threads);
    normal_estimation.setInputCloud(pointcloud_);
    normal_estimation.setSearchMethod(norm_tree);
    normal_estimation.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                                   std::numeric_limits<float>::max());
    normal_estimation.setRadiusSearch(in_normal_search_radius);
    normal_estimation.compute(*normals);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_histograms(new pcl::PointCloud<pcl::FPFHSignature33>());

    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setNumberOfThreads(in_ompnum_threads);
    fpfh.setInputCloud(pointcloud_);
    fpfh.setInputNormals(normals);
    fpfh.setSearchMethod(norm_tree);
    fpfh.setRadiusSearch(in_fpfh_search_radius);
    fpfh.compute(*fpfh_histograms);

    float fpfh_max = std::numeric_limits<float>::min();
    float fpfh_min = std::numeric_limits<float>::max();

    for (unsigned int i = 0; i < fpfh_histograms->size(); i++) // for each point fpfh
    {
        for (unsigned int j = 0; j < cluster_fpfh_histogram.size();
             j++) // sum each histogram's bin for all points, get min/max
        {
            cluster_fpfh_histogram[j] = cluster_fpfh_histogram[j] + fpfh_histograms->points[i].histogram[j];
            if (cluster_fpfh_histogram[j] < fpfh_min)
                fpfh_min = cluster_fpfh_histogram[j];
            if (cluster_fpfh_histogram[j] > fpfh_max)
                fpfh_max = cluster_fpfh_histogram[j];
        }

        float fpfh_dif = fpfh_max - fpfh_min;
        for (unsigned int j = 0; fpfh_dif > 0 && j < cluster_fpfh_histogram.size();
             j++) // substract the min from each and normalize
        {
            cluster_fpfh_histogram[j] = (cluster_fpfh_histogram[j] - fpfh_min) / fpfh_dif;
        }
    }

    return cluster_fpfh_histogram;
}

bool Cluster::IsValid()
{
    return valid_cluster_;
}

void Cluster::SetValidity(bool in_valid)
{
    valid_cluster_ = in_valid;
}

int Cluster::GetId()
{
    return id_;
}

Cluster::~Cluster()
{
    // TODO Auto-generated destructor stub
}