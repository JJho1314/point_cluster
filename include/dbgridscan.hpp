/*
 * dbscan.h
 *
 *  Created on: 2013-3-18
 *      Author: Administrator
 */
#pragma once
#ifndef DBSCAN_H_
#define DBSCAN_H_
#include "basics.h"
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "common/types/type.h" // PointICloudPtr

using namespace std;

namespace autosense
{
    namespace segmenter
    {

        class CDBscan
        {
        public:
            // ��������
            DataStruct *load_data(char filepath[]);
            // ŷʽ����
            double euclideanDistance(DataStruct *point1, DataStruct *point2);
            // �ɸ�����kֵ��������ʵİ뾶Eps
            int init_eps(DataStruct *head);
            double get_eps(DataStruct *head, int k, int notenum);
            // ���ݵĲ������
            Eps *insert_data(Eps *head, double data);
            // ��Ǻ��ĵ㣬��ɾ���߽��
            DataStruct *core_point(DataStruct *head, int k, double eps);
            // ��������к��ĵ�
            void cluster_point(DataStruct *head, double eps);
            // �ϲ�ĳЩ��صĴ�
            void combine_cluster(DataStruct *head, double eps);
            // ���������
            void SaveResult(DataStruct *head);
            // ����ڵ�
            void print_note(DataStruct *head);
            // �ͷ��ڴ�
            void freeNode(DataStruct *head);
            // ��ȡ������
            vector<vector<MyPoint>> getPointsVector(DataStruct *head);
            // ��������
            DataStruct *myload_data(vector<MyPoint> &points, int k, double eps);

            // 2019.01.10
            void myload_data2(pcl::PointCloud<pcl::PointXYZI> &points, vector<MySTRUCT> &head, double eps, int grid_num_x, int grid_num_y, vector<vector<int>> &grid);
            void core_point2(vector<MySTRUCT> &head, int k, double eps, int grid_num_x, int grid_num_y, vector<vector<int>> &grid);
            void cluster_point2(vector<MySTRUCT> &head, double eps, int grid_num_x, int grid_num_y, vector<vector<int>> &grid);
            void combine_cluster2(vector<MySTRUCT> &head, double eps, int grid_num_x, int grid_num_y, vector<vector<int>> &grid);
            vector<vector<MyPoint>> getPointsVector2(vector<MySTRUCT> &head);
            vector<PointICloudPtr> getClusters(vector<MySTRUCT> &head);
            void FindNeighbor(vector<MySTRUCT> &head, int grid_num_x, int grid_num_y, int now, vector<vector<int>> &grid, vector<int> &Neighbor);
        };

    } // namespace segmenter
} // namespace autosense

#endif /* DBSCAN_H_ */
