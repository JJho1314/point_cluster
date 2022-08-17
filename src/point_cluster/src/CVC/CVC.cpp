#include "CVC.h"

CVC::CVC(ros::NodeHandle &nh, ros::NodeHandle &private_nh)
{
    private_nh.param<float>("min_phi", min_phi, -15.0);
    private_nh.param<float>("max_phi", max_phi, 15.0);
    private_nh.param<float>("delta_theta", delta_theta, 1.0);
    private_nh.param<float>("delta_rho", delta_rho, 0.3);
    private_nh.param<float>("delta_phi", delta_phi, 1.2);
    // private_nh.param<float>("max_dist_",max_dist_, 0.5);
    private_nh.param<int>("min_cluster_num_", min_cluster_num_, 10);
}

void CVC::Process(const pcl::PointCloud<pcl::PointXYZ> &in_cloud, vector<int> &cluster_indices)
{
    vector<APR> APR_cloud;
    //                   APR_cloud.resize(in_cloud->size());

    CalculateAPR(in_cloud, APR_cloud);

    unordered_map<int, Voxel> map;
    BuildHashTable(APR_cloud, map);

    cluster_indices = Cluster(in_cloud, map, APR_cloud);
}

CVC::~CVC()
{
}

//转换坐标
void CVC::CalculateAPR(const pcl::PointCloud<pcl::PointXYZ> &in_cloud, vector<APR> &APR_cloud)
{
    for (size_t i = 0; i < in_cloud.size(); ++i)
    {
        APR apr; //存储theta，rho，phi
        apr.theta = CalculateAngle(in_cloud.points[i].x, in_cloud.points[i].y);

        apr.rho = sqrt(pow(in_cloud.points[i].x, 2) + pow(in_cloud.points[i].y, 2));
        apr.phi = atan2(in_cloud.points[i].z, apr.rho) * 180 / PI;

        if (apr.rho < min_rho)
        {
            min_rho = apr.rho;
        }
        if (apr.rho > max_rho)
        {
            max_rho = apr.rho;
        }
        APR_cloud.push_back(apr);
    }
    theta_num = round(360 / delta_theta);
    rho_num = int((max_rho - min_rho) / delta_rho) + 1;
    phi_num = int((max_phi - min_phi) / delta_phi) + 1;
}

//创建哈希表
void CVC::BuildHashTable(const vector<APR> &APR_cloud, std::unordered_map<int, Voxel> &map)
{
    for (size_t i = 0; i < APR_cloud.size(); ++i)
    {
        int theta_index = int(APR_cloud[i].theta / delta_theta);
        int rho_index = int((APR_cloud[i].rho - min_rho) / delta_rho);
        int phi_index = int((APR_cloud[i].phi - min_phi) / delta_phi);
        //计算索引
        int voxel_index = phi_index * theta_num * rho_num + theta_index * rho_num + rho_index;

        std::unordered_map<int, Voxel>::iterator iter;
        iter = map.find(voxel_index);
        if (iter != map.end())
        {
            iter->second.index.push_back(i);
        }
        else
        {
            Voxel vox; //如果Hash表中不存在该点所在体素，则新建体素
            vox.theta_index = theta_index;
            vox.rho_index = rho_index;
            vox.phi_index = phi_index;
            vox.index.push_back(i);
            vox.index.swap(vox.index);
            //在end（）插入pair
            map.insert(std::make_pair(voxel_index, vox));
        }
    }
}
//聚类
vector<int> CVC::Cluster(const pcl::PointCloud<pcl::PointXYZ> &in_cloud, std::unordered_map<int, Voxel> &map, const vector<APR> &APR_cloud)
{
    int current_cluster_id = 0; //当前点云所属聚类序号，初始设为0
    vector<int> cluster_indices = vector<int>(APR_cloud.size(), -1);
    for (size_t i = 0; i < APR_cloud.size(); ++i)
    {
        if (cluster_indices[i] != -1) //-1表示该体素还未聚类
        {
            continue;
        }
        int theta_index = int(APR_cloud[i].theta / delta_theta);
        int rho_index = int((APR_cloud[i].rho - min_rho) / delta_rho);
        int phi_index = int((APR_cloud[i].phi - min_phi) / delta_phi);

        vector<int> neighbors_index;        // map中体素点云邻居索引
        vector<int> origin_neighbors_index; //原始点云邻居索引

        FindNeighbors(in_cloud, map, theta_index, rho_index, phi_index, neighbors_index); //寻找邻近点索引
        std::unordered_map<int, Voxel>::iterator iter;                                    //体素迭代器
        for (size_t j = 0; j < neighbors_index.size(); ++j)
        {
            iter = map.find(neighbors_index[j]);
            if (iter != map.end())
            {

                for (int k = 0; k < iter->second.index.size(); ++k)
                {
                    origin_neighbors_index.push_back(iter->second.index[k]);
                }
            }
        }

        // origin_neighbors_index.swap(origin_neighbors_index);
        if (origin_neighbors_index.size() > 0)
        {
            for (size_t j = 0; j < origin_neighbors_index.size(); ++j)
            {
                if (cluster_indices[i] != -1 && cluster_indices[origin_neighbors_index[j]] != -1)
                {
                    if (cluster_indices[i] != cluster_indices[origin_neighbors_index[j]])
                    {
                        Merge(cluster_indices, cluster_indices[i], cluster_indices[origin_neighbors_index[j]]);
                    }
                }
                else
                {
                    if (cluster_indices[i] != -1)
                    {
                        cluster_indices[origin_neighbors_index[j]] = cluster_indices[i];
                    }
                    else
                    {
                        if (cluster_indices[origin_neighbors_index[j]] != -1)
                        {
                            cluster_indices[i] = cluster_indices[origin_neighbors_index[j]];
                        }
                    }
                }
            }
        }
        if (cluster_indices[i] == -1)
        {
            current_cluster_id++;
            cluster_indices[i] = current_cluster_id;
            for (int k = 0; k < origin_neighbors_index.size(); ++k)
            {
                cluster_indices[origin_neighbors_index[k]] = current_cluster_id;
            }
        }
    }
    return cluster_indices;
}

//寻找邻近体素的索引
void CVC::FindNeighbors(const pcl::PointCloud<pcl::PointXYZ> &in_cloud, std::unordered_map<int, Voxel> &map, const int &theta_index, const int &rho_index,
                        const int &phi_index, vector<int> &neighbors_index)
{
    // int self_index=phi_index*theta_num*rho_num+theta_index*rho_num+rho_index;
    // Pt self_center=CalculateCenter(in_cloud,self_index,map);
    // vector<int> temp_neighbors_index;
    for (size_t i = phi_index - 1; i <= phi_index + 1; ++i)
    {
        if (i < 0 || i > phi_num - 1)
        {
            continue;
        }
        for (size_t j = rho_index - 1; j <= rho_index + 1; ++j)
        {
            if (j < 0 || j > rho_num - 1)
            {
                continue;
            }
            for (size_t k = theta_index - 1; k <= theta_index + 1; ++k)
            {
                int pk = k;
                if (k < 0)
                {
                    pk = theta_num - 1;
                }
                if (k > theta_num - 1)
                {
                    pk = 0;
                }
                //顺序，顺序，顺序
                neighbors_index.push_back(i * theta_num * rho_num + k * rho_num + j);
            }
        }
    }

    // for(int j=0;j<temp_neighbors_index.size();++j)
    // {
    //     Pt center;
    //     center=CalculateCenter(in_cloud,temp_neighbors_index[j],map);
    //     if(CalculateDist(self_center,center))
    //     {
    //         neighbors_index.push_back(temp_neighbors_index[j]);
    //     }
    // }
}

Pt CVC::CalculateCenter(const pcl::PointCloud<pcl::PointXYZ> &in_cloud, int self_index, std::unordered_map<int, Voxel> &map)
{
    Pt center;
    for (int i = 0; i < map[self_index].index.size(); ++i)
    {
        center.x += in_cloud.points[map[self_index].index[i]].x;
        center.y += in_cloud.points[map[self_index].index[i]].y;
        center.z += in_cloud.points[map[self_index].index[i]].z;
    }
    center.x = center.x / map[self_index].index.size();
    center.y = center.y / map[self_index].index.size();
    center.z = center.z / map[self_index].index.size();
    return center;
}
// bool CVC::CalculateDist(const Pt pt1,const Pt pt2)
// {
//     float dist_x=pt1.x-pt2.x;
//     float dist_y=pt1.y-pt2.y;
//     float dist_z=pt1.z-pt2.z;
//     float dist=sqrt(pow(dist_x,2)+pow(dist_y,2)+pow(dist_z,2));

//     return (dist<max_dist_);
// }

void CVC::Merge(vector<int> &cluster_indices, int a, int b)
{
    for (size_t i = 0; i < cluster_indices.size(); ++i)
    {
        if (cluster_indices[i] == a)
        {
            cluster_indices[i] = b;
        }
    }
}

void CVC::SelectMajorCluster(vector<int> &cluster_indices, vector<int> &cluster_index)
{
    //创建哈希表，first存储点云索引，second存储该点云簇点的数量
    std::unordered_map<int, int> index_amount_map;

    for (int i = 0; i < cluster_indices.size(); ++i)
    {
        // std::unordered_map<int,int>::iterator it=index_amount_map.find(cluster_indices[i]);
        if (index_amount_map.find(cluster_indices[i]) != index_amount_map.end())
        {
            index_amount_map[cluster_indices[i]] += 1;
        }
        else
        {
            index_amount_map[cluster_indices[i]] = 1;
        }
    }
    std::vector<std::pair<int, int>> tr(index_amount_map.begin(), index_amount_map.end());
    sort(tr.begin(), tr.end(), compare_cluster);
    for (int i = 0; i < tr.size(); ++i)
    {
        if (tr[i].second > min_cluster_num_)
        {
            cluster_index.push_back(tr[i].first);
        }
    }
}
