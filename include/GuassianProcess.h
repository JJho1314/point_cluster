#ifndef _GUASSIANPROCESS_
#define _GUASSIANPROCESS_

#include <iostream>
#include "basics.h"
#include <vector>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <malloc.h>
#include "iterator"
#include <omp.h>
using namespace std;
using namespace Eigen;

#define mypi 3.1415926
#define GP_M 60 // 180//72
#define GP_N 81 // 160
#define divider 1
// #define binlen 0.3125//22(50/GP_N)
#define binlen 1.0 //(50/GP_N)

namespace autosense
{
    namespace segmenter
    {

        struct VEC
        {
            vector<MyPoint> Point;
        };

        void GP_INSAC_Gridlize3D2(vector<MyPoint> &AfterGridSize, vector<MyPoint> inputPoints);
        void GP_INSAC_Gridlize3D(vector<MyPoint> &AfterGridSize, vector<MyPoint> inputPoints);
        void GP_INSAC(const pcl::PointCloud<pcl::PointXYZI> &After, pcl::PointCloud<pcl::PointXYZI> &Ob, pcl::PointCloud<pcl::PointXYZI> &Gr);

    }
}
#endif