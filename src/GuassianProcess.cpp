
#include <pcl/filters/extract_indices.h> /* pcl::ExtractIndices */
#include <pcl/io/io.h>                   /* pcl::copyPointCloud */
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/model_types.h>  /* pcl::SACMODEL_PLANE */
#include <pcl/sample_consensus/method_types.h> /* pcl::SAC_RANSAC */
#include <pcl/ModelCoefficients.h>             /* pcl::ModelCoefficients */
#include <pcl/segmentation/sac_segmentation.h> /* pcl::SACSegmentation */
#include "GuassianProcess.h"

#include <time.h>

#include <array>

using namespace std;
namespace autosense
{
    namespace segmenter
    {

#define GET_ARRAY_LEN(array, len)                 \
    {                                             \
        len = (sizeof(array) / sizeof(array[0])); \
    }

        float LiDARHeight = 1.75, Mapx = 100.0, Mapz = 4.0, GridSize = 0.2;
        // float T_data = 0.2, T_model = 0.3, T_g = 0.1, GP_Bs = 0.3;
        // float T_data = 1, T_model = 0.5, T_g = 0.3, GP_Bs = 0.3;

        // by hongxz ,2020.9.
        // float delta_data = 0 , delta_model = 0 , delta_g = 0 , delta_bs = 0 ;
        float delta_data = 1 - 0.2, delta_model = 0.5 - 0.3, delta_g = 0.3 - 0.1, delta_bs = 0.5 - 0.2;

        float DB_MinPts = 20.0, DB_R = 0.8;
        float Sigmaf = 1.3298, Sigman = 0.1, delta = 1.0, L = 6; // L = 0.3;
        int Border[10] = {0, 20, 40, 60, 80, 100, 120, 140, 159, 140};
        VEC Pb[GP_M], PG2[GP_M];

        // add by ck 2020.8.29
        // old
        // float T_data = 1, T_model = 0.5, T_g = 0.3, GP_Bs = 0.3;
        float T_data = 1, T_model = 0.5, T_g = 0.3, GP_Bs = 0.3;

        // Pb，分sector的点； PG 每个sector每个bin的最小值，360*160； PG2 去除空bin的PG
        vector<MyPoint> s_p, s_new, test, GroundFinal, ObstacleFinal;
        // parallel 变量
        array<vector<MyPoint>, GP_M> p_s_p, p_s_new, p_test, p_GroundFinal, p_ObstacleFinal;
        array<Mat, GP_M> p_My_Model, p_Standard_Op;
        // vector<float> my_model;
        Mat My_Model, Standard_Op;

        void PolarGridMap(vector<MyPoint> &AfterGridSize);
        bool LessSort(MyPoint a, MyPoint b);
        bool LessSort2(MyPoint a, MyPoint b);
        void seed(vector<MyPoint> PGi, float BS);
        void p_seed(vector<MyPoint> PGi, float BS, int n);
        void p_seed_enlarge(vector<MyPoint> PGi, float BS, int n);
        void p_seed_enlarge_2(vector<MyPoint> PGi, float BS, int n);
        void p_seed1(vector<MyPoint> PGi, float BS, int n);
        void Union();
        void p_Union(int n);
        void regression();
        void p_regression(int n);
        void setdiff(); // vector<MyPoint> PG2i
        void p_setdiff(int n);
        void my_eval();
        void p_my_eval(int n);
        void Segment(vector<MyPoint> Pbi);
        void p_Segment(vector<MyPoint> Pbi, int n);
        void GetFinalPoint();
        void p_GetFinalPoint();

        // Initial() 初始化一些变量；设置标准高度的r的位置；
        void Initial()
        {
            int i, j;
            GroundFinal.clear();
            ObstacleFinal.clear();
            s_new.clear();
            s_p.clear();
            test.clear();
            Standard_Op.setZero(GP_N, 1);
            My_Model.setZero(0, 0);

            // Standard_Op 标准高度的r值。因为我们每个扇形区域分了很多bin，这里提前取每个bin的中心位置
            for (i = 0; i < GP_N; i++)
            {
                Standard_Op(i) = binlen * (i + 0.5); // 0.1562 + binlen*i
            }

            for (int i = 0; i < GP_M; i++)
            {
                p_Standard_Op[i] = Standard_Op;
                p_ObstacleFinal[i].clear();
                p_GroundFinal[i].clear();
                p_test[i].clear();
            }

            for (i = 0; i < GP_M; i++)
            {
                Pb[i].Point.clear();
                PG2[i].Point.clear();
            }
        }

        void GP_INSAC_Gridlize3D(vector<MyPoint> &AfterGridSize, vector<MyPoint> inputPoints)
        {

            const int NumOfPackage = 170; // 把常量参数都写在一个头文件中，然后include,extern进来行不行？
            int i = 0, j = 0;
            long int Len = 0, Number;
            int xIndex, yIndex, zIndex, longIndex;
            float GridNum = Mapx / GridSize, HighNum = Mapz / GridSize;
            float *Tx, *Ty, *Tz;
            long int *TIndex;     // 存栅格坐标
            int *TIsit, *Tnumber; // TIsit 表示某个栅格内是否有点 Tnumber 表示某个栅格内有多少个点
            // vector<MyPoint> AfterGridSize;
            MyPoint point;

            Number = GridNum * GridNum * HighNum;
            Tx = (float *)malloc(Number * sizeof(float));
            Ty = (float *)malloc(Number * sizeof(float));
            Tz = (float *)malloc(Number * sizeof(float));
            TIndex = (long int *)malloc(Number * sizeof(long int));
            TIsit = (int *)malloc(Number * sizeof(int));
            Tnumber = (int *)malloc(Number * sizeof(int));
            // memset(TIsit, 0, Number*sizeof(int));
            for (i = 0; i < Number; i++)
            {
                TIsit[i] = 0;
                Tnumber[i] = 0;
                Tx[i] = 0;
                Ty[i] = 0;
                Tz[i] = 0;
            }
            // AfterGridSize.reserve(GridNum*GridNum*HighNum);
            // memset(&point, 0, TempData.size()*sizeof(MyPoint));
            // for (int i = 0; i < (int)GridNum*GridNum*HighNum;i++)
            //{
            //	MyPoint point;
            //	AfterGridSize.push_back(point);
            // }

            for (i = 0; i < inputPoints.size(); i++)
            {
                inputPoints[i].x += 0.5 * Mapx;
                inputPoints[i].y += 0.5 * Mapx;
                inputPoints[i].z += LiDARHeight; // x、y、z设置为正值，加上雷达高度、50米
                if (inputPoints[i].z > Mapz || inputPoints[i].z < 0 || inputPoints[i].x > Mapx || inputPoints[i].x < 0 || inputPoints[i].y > Mapx || inputPoints[i].y < 0)
                    continue;
                else
                {
                    xIndex = (int)(inputPoints[i].x / GridSize) + 1; // 这是当前点在栅格中的位置
                    yIndex = (int)(inputPoints[i].y / GridSize) + 1;
                    zIndex = (int)(inputPoints[i].z / GridSize) + 1;
                    longIndex = GridNum * GridNum * (zIndex - 1) + GridNum * (yIndex - 1) + xIndex - 1; // 因为栅格不是按照二维数组存取的，而是展开为一维存。所以要计算一个一维的位置

                    Tx[longIndex] += inputPoints[i].x - 0.5 * Mapx; // 减去50，回到原始位置
                    Ty[longIndex] += inputPoints[i].y - 0.5 * Mapx;
                    Tz[longIndex] += inputPoints[i].z;
                    TIsit[longIndex] = 1;
                    Tnumber[longIndex]++;
                    TIndex[longIndex] = longIndex; // 留下顺序
                    //}
                }
            } // 限制范围，去除多余点，循环之后j就是NewData的长度

            for (i = 0; i < Number; i++)
            {
                if (TIsit[i] == 1)
                {
                    if (Tnumber[i] <= 3) // 点数太少
                    {
                        TIsit[i] = 0;
                    }
                    else
                    {
                        j++;
                    }
                }
            }

            AfterGridSize.resize(j);
            j = 0;
            for (i = 0; i < Number; i++) // TempData.size()
            {
                if (TIsit[i]) // TempData[i].Isit
                {
                    // AfterGridSize[j] = TempData[i];
                    AfterGridSize[j].x = Tx[i] / Tnumber[i]; // 最后的输出取栅格内点的质心
                    AfterGridSize[j].y = Ty[i] / Tnumber[i];
                    AfterGridSize[j].z = Tz[i] / Tnumber[i];
                    j++;
                }
            }

            free(Tx);
            free(Ty);
            free(Tz);
            free(TIndex);
            free(TIsit);
            free(Tnumber);
        }

        void GP_INSAC_Gridlize3D2(vector<MyPoint> &AfterGridSize, vector<MyPoint> inputPoints)
        {

            const int NumOfPackage = 170; // 把常量参数都写在一个头文件中，然后include,extern进来行不行？
            int i = 0, j = 0;
            long int Len = 0, Number;
            int xIndex, yIndex, zIndex, longIndex;
            float GridNum = Mapx / GridSize, HighNum = Mapz / GridSize;
            float *Tx, *Ty, *Tz;
            long int *TIndex;
            int *TIsit, *Tnumber;
            // vector<MyPoint> AfterGridSize;
            MyPoint point;
            // vector <MyPoint> TempData;
            // TempData.resize(GridNum*GridNum*HighNum);

            Number = GridNum * GridNum * HighNum;
            Tx = (float *)malloc(Number * sizeof(float));
            Ty = (float *)malloc(Number * sizeof(float));
            Tz = (float *)malloc(Number * sizeof(float));
            TIndex = (long int *)malloc(Number * sizeof(long int));
            TIsit = (int *)malloc(Number * sizeof(int));
            Tnumber = (int *)malloc(Number * sizeof(int));
            // memset(TIsit, 0, Number*sizeof(int));
            for (i = 0; i < Number; i++)
            {
                TIsit[i] = 0;
                Tnumber[i] = 0;
                Tx[i] = 0;
                Ty[i] = 0;
                Tz[i] = 0;
            }
            // AfterGridSize.reserve(GridNum*GridNum*HighNum);
            // memset(&point, 0, TempData.size()*sizeof(MyPoint));
            // for (int i = 0; i < (int)GridNum*GridNum*HighNum;i++)
            //{
            //	MyPoint point;
            //	AfterGridSize.push_back(point);
            // }

            for (i = 0; i < inputPoints.size(); i++)
            {
                inputPoints[i].x += 0.5 * Mapx;
                inputPoints[i].y += 0.5 * Mapx;
                inputPoints[i].z += LiDARHeight;
                if (inputPoints[i].z > Mapz || inputPoints[i].z < 0 || inputPoints[i].x > Mapx || inputPoints[i].x < 0 || inputPoints[i].y > Mapx || inputPoints[i].y < 0)
                    continue;
                else
                {
                    xIndex = (int)(inputPoints[i].x / GridSize) + 1;
                    yIndex = (int)(inputPoints[i].y / GridSize) + 1;
                    zIndex = (int)(inputPoints[i].z / GridSize) + 1;
                    longIndex = GridNum * GridNum * (zIndex - 1) + GridNum * (yIndex - 1) + xIndex - 1;
                    if (!TIsit[longIndex]) // 没有记录就是Isit0
                    {
                        Tx[longIndex] = (xIndex - 1) * GridSize - 0.5 * Mapx; // 刚开始xy加了50，这里需要剪掉 //comment by YS 2018/10/31
                        Ty[longIndex] = (yIndex - 1) * GridSize - 0.5 * Mapx;
                        Tz[longIndex] = (zIndex - 1) * GridSize;
                        // Tx[longIndex] = inputPoints[i].x - 0.5*Mapx;
                        // Ty[longIndex] = inputPoints[i].y - 0.5*Mapx;
                        // Tz[longIndex] = inputPoints[i].z;
                        TIsit[longIndex] = 1;
                        // Tnumber[longIndex]++;
                        TIndex[longIndex] = longIndex; // 留下顺序
                    }
                }
            } // 限制范围，去除多余点，循环之后j就是NewData的长度

            // AfterGridSize.resize(j);
            // j = 0;
            for (i = 0; i < Number; i++) // TempData.size()
            {
                if (TIsit[i]) // TempData[i].Isit
                {
                    // AfterGridSize[j] = TempData[i];
                    MyPoint temp;
                    temp.x = Tx[i];
                    temp.y = Ty[i];
                    temp.z = Tz[i];
                    temp.Index = TIndex[i];
                    temp.Isit = TIsit[i];
                    AfterGridSize.push_back(temp);
                    // AfterGridSize[j].x = Tx[i];
                    // AfterGridSize[j].y = Ty[i];
                    // AfterGridSize[j].z = Tz[i];
                    // AfterGridSize[j].Index = TIndex[i];
                    // AfterGridSize[j].Isit = TIsit[i];
                    // j++;
                }
            }

            free(Tx);
            free(Ty);
            free(Tz);
            free(TIndex);
            free(TIsit);
            free(Tnumber);
        }

        void GP_INSAC(const pcl::PointCloud<pcl::PointXYZI> &After, pcl::PointCloud<pcl::PointXYZI> &Ob, pcl::PointCloud<pcl::PointXYZI> &Gr)
        {

            // 地面分割程序
            int i, j, k = 0, Num = 0, temp;
            vector<MyPoint> AfterGridSize;
            MyPoint PG[GP_M][GP_N + 1]; // release模式下，不初始化可能会出问题
            // AfterGridSize.assign(After.begin(), After.end());//栅格化后的数据

            Initial(); // 初始化
            AfterGridSize.clear();
            AfterGridSize.reserve(After.size());
            for (int i = 0; i < After.size(); i++)
            {
                MyPoint temp;
                temp.x = After[i].x;
                temp.y = After[i].y;
                temp.z = After[i].z + LiDARHeight;
                temp.intensity = After[i].intensity;
                temp.Index = i;
                AfterGridSize.push_back(temp);
            }
            PolarGridMap(AfterGridSize); // 返回指针, x->r y->theta size没了？？点分扇形

            for (i = 0; i < GP_M; i++) // 360
            {
                // cout << " Pb[i].Point.size() = " << Pb[i].Point.size() << endl;
                for (j = 0; j < Pb[i].Point.size(); j++)
                {
                    temp = (int)(Pb[i].Point[j].x / binlen) + 1; // 属于的bin 下面将pb分为160bins，用pg存
                    // cout << "temp = " << temp <<"   xxxxxxxxxxxxxxx   " << PG[i][temp - 1].Isit<< endl;
                    if ((PG[i][temp - 1].Isit == 1) && (Pb[i].Point[j].z < PG[i][temp - 1].z)) // 为1 说明已经放过值了,然后再比较
                    {
                        PG[i][temp - 1].x = Pb[i].Point[j].x;
                        PG[i][temp - 1].y = Pb[i].Point[j].y;
                        PG[i][temp - 1].z = Pb[i].Point[j].z;
                        PG[i][temp - 1].Index = Pb[i].Point[j].Index;
                    }
                    else if (PG[i][temp - 1].Isit != 1)
                    {
                        PG[i][temp - 1].x = Pb[i].Point[j].x;
                        PG[i][temp - 1].y = Pb[i].Point[j].y;
                        PG[i][temp - 1].z = Pb[i].Point[j].z;
                        PG[i][temp - 1].Index = Pb[i].Point[j].Index;
                        PG[i][temp - 1].Isit = 1;
                        k++; // 计数
                    }
                }

                PG[i][GP_N].x = k; // 第161元素用来计数
                Num += k;
                k = 0;
                // 开始PG转PG2
                for (j = 0; j < GP_N; j++)
                {
                    if (PG[i][j].Isit == 1)
                    {
                        PG2[i].Point.push_back(PG[i][j]);
                        k++;
                    }
                    if (k == PG[i][GP_N].x)
                    {
                        k = 0;
                        break;
                    }
                }
                k = 0;

            } // for PG:每个bin里面的最小值，但有的bin里面没有点

            // #pragma omp parallel for
            // GetSystemTime(&t1);
            // for (i = 0; i < GP_M; i++)
            //{
            //	test = PG2[i].Point;
            //	//cout << "Pb[i].size() = " << Pb[i].Point.size() << endl;
            //	if (PG[i][GP_N].x)
            //	{
            //		seed(PG2[i].Point, GP_Bs);//return s_new 取种子点 需要改进！
            //		while (s_new.size())
            //		{
            //			if (s_p.size())
            //				Union();//合并两个点集
            //			else
            //				s_p.assign(s_new.begin(), s_new.end());
            //			//cout << s_new.size() << " ";
            //			//cout << s_p.size() << endl;
            //			regression();//计算模型
            //			setdiff();//计算差集
            //			my_eval();
            //			//cout << s_new.size() << ", ";
            //			//cout << s_p.size() << endl;
            //		}
            //		//cout << "Pb[i].size() = " << Pb[i].Point.size() << endl;
            //		Segment(Pb[i].Point);//return ground
            //	}
            //	s_new.clear();
            //	s_p.clear();
            //	test.clear();
            //	My_Model.setZero(0, 0);
            // }

            // GetFinalPoint();
            array<int, GP_M> values;
            for (int i = 0; i < GP_M; i++)
            {
                values[i] = i;
            }

            // use openmp for speed up

#pragma omp parallel
            {
#pragma omp for
                for (int i = 0; i < GP_M; i++)
                {
                    // printf("i = %d, I am Thread %d \n", i , omp_get_thread_num());
                    p_test[i] = PG2[i].Point;
                    // cout << "Pb[i].size() = " << Pb[i].Point.size() << endl;

                    if (PG[i][GP_N].x)
                    {
                        // p_seed(PG2[i].Point, GP_Bs, i);//return s_new 取种子点 需要改进！
                        p_seed_enlarge_2(PG2[i].Point, GP_Bs, i); // return s_new 取种子点 需要改进！
                        while (p_s_new[i].size())
                        {
                            if (p_s_p[i].size())
                                p_Union(i); // 合并两个点集
                            else
                                p_s_p[i].assign(p_s_new[i].begin(), p_s_new[i].end());

                            p_regression(i); // 计算模型
                            p_setdiff(i);    // 计算差集
                            p_my_eval(i);
                        }

                        //	cout << "Pb[i].size() = " << Pb[i].Point.size() << endl;
                        p_Segment(Pb[i].Point, i); // return ground
                    }
                    p_s_new[i].clear();
                    p_s_p[i].clear();
                    p_test[i].clear();
                    p_My_Model[i].setZero(0, 0);
                } // parallel
            }

            p_GetFinalPoint();

            Ob.width = ObstacleFinal.size();
            Ob.height = 1;
            Ob.is_dense = false;
            Ob.points.resize(Ob.width * Ob.height);
            for (int i = 0; i < ObstacleFinal.size(); i++)
            {
                Ob[i].x = ObstacleFinal[i].x;
                Ob[i].y = ObstacleFinal[i].y;
                Ob[i].z = ObstacleFinal[i].z - LiDARHeight;
                Ob[i].intensity = ObstacleFinal[i].intensity;
            }
            Gr.width = GroundFinal.size();
            Gr.height = 1;
            Gr.is_dense = false;
            Gr.points.resize(Gr.width * Gr.height);
            for (int i = 0; i < GroundFinal.size(); i++)
            {
                Gr[i].x = GroundFinal[i].x;
                Gr[i].y = GroundFinal[i].y;
                Gr[i].z = GroundFinal[i].z - LiDARHeight;
                Gr[i].intensity = GroundFinal[i].intensity;
            }
            // Ob.assign(ObstacleFinal.begin(), ObstacleFinal.end());
            // Gr.assign(GroundFinal.begin(), GroundFinal.end());
        }

        void PolarGridMap(vector<MyPoint> &AfterGridSize)
        {

            int R_limit = 80, i, j;
            float r = 0.0, theta = 0.0;
            vector<MyPoint>::iterator it;
            // int Start[GP_M * GP_N], Ends[GP_M * GP_N], S_Start[GP_M * GP_N], S_Ends[GP_M * GP_N];
            for (i = 0; i < AfterGridSize.size(); i++)
            {
                r = sqrt(AfterGridSize[i].x * AfterGridSize[i].x + AfterGridSize[i].y * AfterGridSize[i].y); /// r^2 计算距离
                if (r >= R_limit)
                    AfterGridSize[i].Isit = 0; // 超出阈值范围，标记Isit为0，即将删除
                else
                {
                    if (abs(AfterGridSize[i].x) <= 1e-3)
                    {
                        if (AfterGridSize[i].y < 0)
                            theta = 270;
                        else
                            theta = 90;
                    }
                    else if (AfterGridSize[i].y > 0)
                        theta = acos(AfterGridSize[i].x / r) * 180 / mypi; // acos 0~pi
                    else if (AfterGridSize[i].y < 0)
                        theta = 360 - acos(AfterGridSize[i].x / r) * 180 / mypi;
                    AfterGridSize[i].x = r;
                    AfterGridSize[i].y = theta > 359 ? 359 : theta; // z保留 到这里就转化为极坐标，用的原来的变量
                    AfterGridSize[i].Isit = 1;
                }
            }

            for (i = 0; i < AfterGridSize.size(); i++)
            {
                if (AfterGridSize[i].Isit == 0)
                {
                    continue;
                }
                j = (int)(AfterGridSize[i].y / (360 / GP_M)); /// divider
                Pb[j].Point.push_back(AfterGridSize[i]);      /// 分配到那个sector去
            }
        }

        bool LessSort(MyPoint a, MyPoint b) { return (a.y < b.y); } ////按照theta升序排序

        bool LessSort2(MyPoint a, MyPoint b)
        {
            if (a.x < b.x)
            {
                return true;
            }
            else if ((a.x == b.x) && (a.y < b.y))
                return true;
            else if ((a.x == b.x) && (a.y == b.y) && (a.z < b.z))
                return true;
            else
                return false;
        }

        void seed(vector<MyPoint> PGi, float BS)
        {
            int i, num = PGi.size();
            if (num)
            {
                for (i = 0; i < num; i++)
                {
                    if (PGi[i].z <= BS)
                    {
                        s_new.push_back(PGi[i]);
                    }
                }
            }
            else
            {
                return;
            }
        }

        void p_seed1(vector<MyPoint> PGi, float BS, int n)
        {
            int i, num = PGi.size();
            ////////pcl  ransac  line/////////////
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointXYZRGB temp;
            for (i = 0; i < num; i++)
            {
                temp.x = PGi[i].x;
                temp.y = PGi[i].y;
                temp.z = 0;
                temp.r = PGi[i].Index;
                temp.g = PGi[i].Isit;
                temp.b = PGi[i].z;
                cloud->push_back(temp);
            }

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.15);

            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            MyPoint tempPoint;
            if (inliers->indices.size() > 0)
            {
                pcl::ExtractIndices<pcl::PointXYZRGB> indiceExtractor;
                indiceExtractor.setInputCloud(cloud);
                indiceExtractor.setIndices(inliers);

                // extract ground points
                // indiceExtractor.setNegative(false);
                indiceExtractor.filter(*cloud1);
                int j = cloud1->points.size();
                for (i = 0; i < j; i++)
                {
                    tempPoint.x = cloud->points[j].x;
                    tempPoint.y = cloud->points[j].y;
                    tempPoint.z = cloud->points[j].b;
                    tempPoint.Index = cloud->points[j].r;
                    tempPoint.Isit = cloud->points[j].g;
                    p_s_new[n].push_back(tempPoint);
                }
            }
            // if (num)
            //{
            //	for (i = 0; i < num; i++)
            //	{
            //		if (PGi[i].z <= BS)
            //		{
            //			p_s_new[n].push_back(PGi[i]);
            //		}
            //	}
            // }
            else
            {
                return;
            }
        }

        void p_seed(vector<MyPoint> PGi, float BS, int n)
        {
            int i, num = PGi.size();
            ////////pcl  ransac  line/////////////

            if (num)
            {
                for (i = 0; i < num; i++)
                {
                    if (abs(PGi[i].z) <= BS) // if (abs(PGi[i].z) <= (BS + delta_bs*(PGi[i].x / 50)))
                    {
                        p_s_new[n].push_back(PGi[i]);
                    }
                }
            }
            else
            {
                return;
            }
        }
        // 2020.6.7 add
        void p_seed_enlarge(vector<MyPoint> PGi, float BS, int n)
        {
            int i, num = PGi.size();

            if (num)
            {
                for (i = 0; i < num; i++)
                {
                    if (PGi[i].x <= 5 || p_s_new[n].size() == 0)
                    {
                        if (abs(PGi[i].z) <= BS) // if (abs(PGi[i].z) <= (BS + delta_bs*(PGi[i].x / 50)))
                        {
                            p_s_new[n].push_back(PGi[i]);
                        }
                        continue;
                    }
                    int en_h = 0, en_num = 0;
                    vector<MyPoint>::iterator it;
                    // it = p_s_new[n].end() - 1;
                    // en_h += (*it).z;

                    // for(it = p_s_new[n].begin(); it != p_s_new[n].end() - 1; it++)
                    // {
                    // 	if (abs((*it).x - PGi[i].x) <= 5)
                    // 	{
                    // 		en_h += (*it).z;
                    // 		en_num += 1;
                    // 	}
                    // }
                    for (it = p_s_new[n].end() - 1; it != p_s_new[n].begin() - 1; it--)
                    {
                        if (en_num < 2)
                        {
                            en_h += (*it).z;
                            en_num += 1;
                        }
                        else
                        {
                            break;
                        }
                    }

                    en_h = en_h / en_num;

                    if (abs(PGi[i].z - en_h) <= BS || abs(PGi[i].z) <= BS) // if (abs(PGi[i].z) <= (BS + delta_bs*(PGi[i].x / 50)))
                    {
                        p_s_new[n].push_back(PGi[i]);
                    }
                }
            }
            else
            {
                return;
            }
        }

        // 2020.11.14 add
        void p_seed_enlarge_2(vector<MyPoint> PGi, float BS, int n)
        {
            int i, num = PGi.size();

            if (num)
            {
                for (i = 0; i < num; i++)
                {
                    if (PGi[i].x <= 10 || p_s_new[n].size() == 0)
                    {
                        if (PGi[i].z >= -0.4 && PGi[i].z <= 0.25) // if (abs(PGi[i].z) <= (BS + delta_bs*(PGi[i].x / 50)))
                        {
                            p_s_new[n].push_back(PGi[i]);
                        }
                        continue;
                    }
                    int en_h = 0, en_num = 0;
                    vector<MyPoint>::iterator it;

                    for (it = p_s_new[n].end() - 1; it != p_s_new[n].begin() - 1; it--)
                    {
                        if (en_num < 4)
                        {
                            en_h += (*it).z;
                            en_num += 1;
                        }
                        else
                        {
                            break;
                        }
                    }

                    en_h = en_h / en_num;

                    if (((PGi[i].z - en_h) >= -0.2 && (PGi[i].z - en_h) <= 0.2) ||
                        (PGi[i].z >= -0.4 && PGi[i].z <= 0.25))
                    {
                        p_s_new[n].push_back(PGi[i]);
                    }
                }
            }
            else
            {
                return;
            }
        }

        void Union()
        {
            int i, j;
            vector<MyPoint> temp;
            // sort(s_p.begin(), s_p.end());
            temp.assign(s_p.begin(), s_p.end());
            for (i = 0; i < s_new.size(); i++)
            {
                for (j = 0; j < s_p.size(); j++)
                {
                    if (s_new[i].Index == s_p[i].Index)
                        break;
                }
                if (j == s_p.size())
                    temp.push_back(s_new[i]); // s_new的这一点在sp中找不到，则应当纳入合并序列
            }
            /*sort(s_p.begin(), s_p.end(), LessSort2);
            sort(s_new.begin(), s_new.end(), LessSort2);
            set_union(s_p.begin(), s_p.end(), s_new.begin(), s_new.end(), back_inserter(temp));*/
            s_p.clear();
            s_p.assign(temp.begin(), temp.end());
        }

        void p_Union(int n)
        {
            int i, j;
            vector<MyPoint> temp;
            // sort(s_p.begin(), s_p.end());
            temp.assign(p_s_p[n].begin(), p_s_p[n].end());
            for (i = 0; i < p_s_new[n].size(); i++)
            {
                for (j = 0; j < p_s_p[n].size(); j++)
                {
                    if (p_s_new[n][i].Index == p_s_p[n][j].Index)
                    {
                        break; // 只跳出当前循环
                    }
                }
                if (j == p_s_p[n].size())
                    temp.push_back(p_s_new[n][i]); // s_new的这一点在sp中找不到，则应当纳入合并序列
            }
            /*sort(s_p.begin(), s_p.end(), LessSort2);
            sort(s_new.begin(), s_new.end(), LessSort2);
            set_union(s_p.begin(), s_p.end(), s_new.begin(), s_new.end(), back_inserter(temp));*/
            p_s_p[n].clear();
            p_s_p[n].assign(temp.begin(), temp.end());
        }

        void regression()
        {
            int i, j, k;
            float temp;
            long int Len = s_p.size();
            Mat sp, Temp;
            My_Model.setZero(Len, Len); // 初始化
            sp.setZero(Len, 1);
            for (i = 0; i < Len; i++)
            {
                sp(i) = s_p[i].x;
            }
            for (i = 0; i < Len; i++)
            {
                Temp = sp(i) - sp.array();
                Temp = ((-1 / (2 * L)) * (Temp.array().square()).array()).array().exp();
                Temp = Sigman * delta + (Sigmaf * Temp.array()).array(); // Len*1
                My_Model.row(i) = Temp.adjoint();                        // 1*Len
            }
            /*cout << "Model" << endl;
            cout << My_Model << endl;*/

            // 重写,以下是之前写的
            // vector<float>  KXX;
            // C.reserve(Len*Len);
            // my_model.reserve(Len*Len);
            // for (i = 0; i < Len*Len; i++)
            //{
            //	j = (int)((i + 1) / Len);//i从0开始，故先要加1
            //	k = i - j*Len - 1;//从0开始， k要减1，j本来要加1，这里再减1则不用加了
            //	temp = s_p[j].x - s_p[k].x;
            //	temp = exp((double)-temp*temp / (2 * L*L));
            //	temp = Sigmaf*temp + Sigman*delta;
            //	my_model.push_back(temp);
            // }
        }
        void p_regression(int n)
        {
            int i, j, k;
            float temp;
            long int Len = p_s_p[n].size();
            Mat sp, Temp;
            p_My_Model[n].setZero(Len, Len); // 初始化
            sp.setZero(Len, 1);
            for (i = 0; i < Len; i++)
            {
                sp(i) = p_s_p[n][i].x;
            }
            for (i = 0; i < Len; i++)
            {
                Temp = sp(i) - sp.array();
                // if (n == 260)
                //{
                //	cout << endl << Temp << endl;
                //	//cout << endl << (Temp.array().square()) << endl;
                //	cout << endl << ((-1 / (2 * L * L))*(Temp.array().square()).array()) << endl;
                // }
                // Temp = sp(i) - sp.array();
                Temp = ((-1 / (2 * L * L)) * (Temp.array().square()).array()).array().exp();
                // if (n == 260)
                //{
                //	cout << endl << Temp << endl;
                // }
                Temp = Sigman * delta + (Sigmaf * Temp.array()).array(); // Len*1
                // if (n == 260)
                //{
                //	cout << endl << Temp << endl;
                // }
                p_My_Model[n].row(i) = Temp.adjoint(); // 1*Len
            }
        }

        void setdiff()
        {
            int i, j;
            vector<MyPoint> temp;
            // sort(s_p.begin(), s_p.end(), LessSort2);
            // sort(PG2i.begin(), PG2i.end(), LessSort2);
            ////出错：没有排序
            // set_difference(PG2i.begin(), PG2i.end(), s_p.begin(), s_p.end(), back_inserter(temp));// back_inserter(temp)
            // test.assign(temp.begin(), temp.end());
            // 自己实现
            temp.clear();
            for (i = 0; i < test.size(); i++)
            {
                for (j = 0; j < s_p.size(); j++)
                {
                    if (test[i].Index == s_p[j].Index)
                    {
                        break;
                    }
                }
                if (j == s_p.size())
                    temp.push_back(test[i]);
            }
            test.clear();
            test.assign(temp.begin(), temp.end());
        }

        void p_setdiff(int n)
        {
            int i, j;
            vector<MyPoint> temp;
            // sort(s_p.begin(), s_p.end(), LessSort2);
            // sort(PG2i.begin(), PG2i.end(), LessSort2);
            ////出错：没有排序
            // set_difference(PG2i.begin(), PG2i.end(), s_p.begin(), s_p.end(), back_inserter(temp));// back_inserter(temp)
            // test.assign(temp.begin(), temp.end());
            // 自己实现
            temp.clear();
            for (i = 0; i < p_test[n].size(); i++)
            {
                for (j = 0; j < p_s_p[n].size(); j++)
                {
                    if (p_test[n][i].Index == p_s_p[n][j].Index)
                    {
                        break;
                    }
                }
                if (j == p_s_p[n].size())
                    temp.push_back(p_test[n][i]);
            }
            p_test[n].clear();
            p_test[n].assign(temp.begin(), temp.end());
        }

        void my_eval() // Mat MyModel, vector<MyPoint> s_p, vector<MyPoint> test
        {
            // 返回s_new
            // vector<float> KXX = mymodel;// Len*Len
            // vector <float> KXX2, KX2X, KX2X2;
            int i, j;
            int Len = s_p.size(), Wid = test.size();
            float temp = 0.0;
            // KXX2.reserve(Len*Wid);
            // KX2X.reserve(Wid*Len);
            // KX2X2.reserve(Wid*Wid);
            Mat KXX = My_Model; // Len*Len
            Mat KXX2, KX2X, KX2X2, f2_bar, V_f2;
            Mat SP, SP2, Test, Temp;
            vector<MyPoint>::iterator it;
            if ((s_p.size()) && (test.size()) && (KXX.size()))
            {
                KXX2.setZero(Len, Wid);
                KX2X.setZero(Wid, Len);
                KX2X2.setZero(Wid, Wid);
                V_f2.setZero(Wid, Wid);
                SP.setZero(Len, 1);
                SP2.setZero(Len, 1);
                Test.setZero(Wid, 1);
                f2_bar.setZero(Wid, 1);
                for (i = 0; i < Len; i++)
                {
                    SP(i) = s_p[i].x;
                    SP2(i) = s_p[i].z;
                }
                for (i = 0; i < Wid; i++)
                {
                    Test(i) = test[i].x;
                }
                for (i = 0; i < Len; i++)
                {
                    Temp = SP(i) - Test.array();
                    Temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L)) * Temp.array().square()).array().exp()).array());
                    KXX2.row(i) = Temp.adjoint(); // 1*m n行
                }
                for (i = 0; i < Wid; i++)
                {
                    Temp = Test(i) - SP.array();
                    Temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L)) * Temp.array().square()).array().exp()).array());
                    KX2X.row(i) = Temp.adjoint(); // 1*n m行
                    Temp = Test(i) - Test.array();
                    Temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L)) * Temp.array().square()).array().exp()).array());
                    KX2X2.row(i) = Temp.adjoint(); // 1*n m行
                }
                // Temp.setZero(Len, Len);
                Temp = KXX.array() + Sigman;
                Temp = Temp.inverse();
                f2_bar = KX2X * Temp * SP2;                          // m*1
                V_f2 = KX2X2.array() - (KX2X * Temp * KXX2).array(); // m*m
                temp = SP2.mean();
                s_new.clear();
                for (i = 0; i < Wid; i++)
                {
                    if ((V_f2(i, i) < T_model) && (abs((temp - f2_bar(i)) / sqrt(Sigman + V_f2(i, i))) < T_data))
                        s_new.push_back(test[i]);
                }
            }
            else
            {
                // 清空
                s_new.clear();
                return;
            }
        }

        void p_my_eval(int n) // Mat MyModel, vector<MyPoint> s_p, vector<MyPoint> test
        {
            // 返回s_new
            // vector<float> KXX = mymodel;// Len*Len
            // vector <float> KXX2, KX2X, KX2X2;
            int i, j;
            int Len = p_s_p[n].size(), Wid = p_test[n].size();
            float temp = 0.0;
            // KXX2.reserve(Len*Wid);
            // KX2X.reserve(Wid*Len);
            // KX2X2.reserve(Wid*Wid);
            Mat KXX = p_My_Model[n]; // Len*Len
            Mat KXX2, KX2X, KX2X2, f2_bar, V_f2;
            Mat SP, SP2, Test, Test2, Temp;
            vector<MyPoint>::iterator it;
            if (n == 260)
            {
                MyDEBUG("KXX", KXX);
            }
            if ((p_s_p[n].size()) && (p_test[n].size()) && (KXX.size()))
            {
                KXX2.setZero(Len, Wid);
                KX2X.setZero(Wid, Len);
                KX2X2.setZero(Wid, Wid);
                V_f2.setZero(Wid, Wid);
                SP.setZero(Len, 1);
                SP2.setZero(Len, 1);
                Test.setZero(Wid, 1);
                Test2.setZero(Wid, 1);
                f2_bar.setZero(Wid, 1);
                for (i = 0; i < Len; i++)
                {
                    SP(i) = p_s_p[n][i].x;
                    SP2(i) = p_s_p[n][i].z;
                }
                for (i = 0; i < Wid; i++)
                {
                    Test(i) = p_test[n][i].x;
                    Test2(i) = p_test[n][i].z;
                }
                for (i = 0; i < Len; i++)
                {
                    Temp = SP(i) - Test.array();
                    Temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L * L)) * Temp.array().square()).array().exp()).array());
                    KXX2.row(i) = Temp.adjoint(); // 1*n m行
                }
                for (i = 0; i < Wid; i++)
                {
                    Temp = Test(i) - SP.array();
                    Temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L * L)) * Temp.array().square()).array().exp()).array());
                    KX2X.row(i) = Temp.adjoint(); // 1*n m行
                    Temp = Test(i) - Test.array();
                    Temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L * L)) * Temp.array().square()).array().exp()).array());
                    KX2X2.row(i) = Temp.adjoint(); // 1*n m行
                }
                // Temp.setZero(Len, Len);
                Mat eye;
                eye.setIdentity(Len, Len);
                Temp = KXX + Sigman * eye;
                Temp = Temp.inverse();
                f2_bar = KX2X * Temp * SP2;          // m*1
                V_f2 = KX2X2 - (KX2X * Temp * KXX2); // m*m
                temp = SP2.mean();
                p_s_new[n].clear();
                // if (n == 260)
                //{
                //	for (i = 0; i < Wid; i++)
                //	{
                //		cout << V_f2(i, i) << endl;
                //	}
                //	cout << "---------------------" << endl;
                //	cout << f2_bar << endl;
                // }
                for (i = 0; i < Wid; i++) //? 明天改Segment！
                {
                    // if (n == 260)
                    //	cout << abs((Test2(i) - f2_bar(i)) / sqrt(Sigman + V_f2(i, i))) << endl;
                    // if ((V_f2(i, i) < T_model + delta_model*Test(i) / 50) && (abs((Test2(i) - f2_bar(i)) / sqrt(Sigman + V_f2(i, i))) < T_data + delta_data*Test(i) / 50))
                    if ((V_f2(i, i) < T_model) && (abs((Test2(i) - f2_bar(i)) / sqrt(Sigman + V_f2(i, i))) < T_data)) // 11.12
                        p_s_new[n].push_back(p_test[n][i]);
                }
            }
            else
            {
                // 清空
                p_s_new[n].clear();
                return;
            }
        }

        void Segment(vector<MyPoint> Pbi)
        {
            int i, j, k;
            int Len = GP_N, Wid = s_p.size();
            float value = 0.0, Hg = 0.0;
            float ti;
            Mat KXX, KX2X, temp, SP, SP2, Standard_H, EYE;
            EYE.setIdentity(My_Model.rows(), My_Model.rows());
            Standard_H.setZero(GP_N, 1);

            if (Wid && (My_Model.rows()) && (My_Model.cols()))
            {
                KXX = My_Model;
                KX2X.setZero(Len, Wid);
                temp.setZero(Wid, Len);
                SP.setZero(Wid, 1);
                SP2.setZero(Wid, 1);
                for (i = 0; i < Wid; i++)
                {
                    SP(i) = s_p[i].x;
                    SP2(i) = s_p[i].z;
                }
                for (i = 0; i < Len; i++)
                {
                    temp.col(i) = Standard_Op(i) - SP.array(); // wid*1
                }
                temp = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L)) * temp.array().square()).array().exp()).array());
                KX2X = temp.adjoint();
                // for (i = 0; i < Len; i++)
                //{
                //	//temp = Standard_Op(i) - SP.array();//wid*1
                //	temp = (Sigman*delta) + (Sigmaf*(-(1 / (2 * L))*(Standard_Op(i) - SP.array()).array().square()).array().exp()).array();
                //	//temp = (Sigman*delta) + (Sigmaf*temp).array();
                //	KX2X.row(i) = temp.adjoint();//1*m n闁跨喐鏋婚幏锟�
                // }
                temp = KXX + Sigman * EYE;
                Standard_H.noalias() = KX2X * temp.inverse() * SP2; //.noalias()避免产生中间变量
                // cout << "t1 = " << B - A << endl;
                // cout << "t2 = " << C - B << endl;
                // for (i = 0; i < GP_N; i++)
                //	cout << Standard_H(i) << endl;

                // for (i = 0; i < GP_N; i++)//中值滤波
                //{
                //	if (abs(Standard_H(i)) > 0.7)
                //	{
                //		j = (int)(i / 20);
                //		k = (int)(Standard_H(Border[j]) + Standard_H(Border[j + 1])) / 2;
                //		Standard_H(i) = Standard_H(k);
                //	}
                // }   2019.2.14

                // Pbi ith sector的所有点
                for (i = 0; i < Pbi.size(); i++)
                {
                    // temp = Pbi[i].z - Standard_Op.array();
                    // temp.array().abs();
                    // value = temp.minCoeff(&j, &k);//点对应最小Op的行数， 列数
                    j = (int)(Pbi[i].x / binlen) + 1;
                    j = min(j, GP_N - 1);
                    Hg = Standard_H(j); // 对应标准高度
                    if (abs(Pbi[i].z - Hg) <= T_g)
                        GroundFinal.push_back(Pbi[i]);
                    else
                        ObstacleFinal.push_back(Pbi[i]);
                }
            }
            else
            {
                // #pragma omp single// [clauses]
                //		{
                for (i = 0; i < Pbi.size(); i++)
                {
                    ObstacleFinal.push_back(Pbi[i]);
                }
                //		}
            }
        }

        void p_Segment(vector<MyPoint> Pbi, int n)
        {
            int i, j, k;
            int Len = GP_N, Wid = p_s_p[n].size();
            float value = 0.0, Hg = 0.0;
            float ti;
            Mat KXX, KX2X, temp, temp2, SP, SP2, Standard_H, EYE;
            EYE.setIdentity(p_My_Model[n].rows(), p_My_Model[n].rows());
            Standard_H.setZero(GP_N, 1);

            if (Wid && (p_My_Model[n].rows()) && (p_My_Model[n].cols()))
            {
                KXX = p_My_Model[n];
                KX2X.setZero(Len, Wid); // 50*12
                temp.setZero(Wid, 1);

                SP.setZero(Wid, 1);
                SP2.setZero(Wid, 1);
                for (i = 0; i < Wid; i++)
                {
                    SP(i) = p_s_p[n][i].x;
                    SP2(i) = p_s_p[n][i].z;
                }
                if (n == 260)
                    MyDEBUG("SP", SP);
                for (i = 0; i < Len; i++)
                {
                    temp.setZero(Wid, 1);
                    temp = Standard_Op(i) - SP.array(); // wid*1
                    if (n == 260)
                    {
                        MyDEBUG("Standard_Op(i) - SP.array(): ", temp);
                    }
                    temp2.setZero(Wid, 1);
                    temp2 = ((Sigman * delta) + (Sigmaf * (-(1 / (2 * L * L)) * temp.array().square()).array().exp()).array());
                    if (n == 260)
                    {
                        MyDEBUG("exp: ", temp2);
                    }
                    KX2X.row(i) = temp2.adjoint();
                }
                temp = KXX + Sigman * EYE;
                Standard_H.noalias() = KX2X * temp.inverse() * SP2; //.noalias()避免产生中间变量

                // for (i = 0; i < GP_N; i++)//中值滤波
                //{
                //	if (abs(Standard_H(i)) > 0.7)
                //	{
                //		j = (int)(i / 20);
                //		k = (int)(Standard_H(Border[j]) + Standard_H(Border[j + 1])) / 2;
                //		Standard_H(i) = Standard_H(k);
                //	}
                // }   2019.2.14

                // Pbi ith sector的所有点
                for (i = 0; i < Pbi.size(); i++)
                {
                    int idx = 0, dis = 1000;
                    for (j = 0; j < Len; j++)
                    {
                        if (abs(Pbi[i].x - Standard_Op(j)) < dis)
                        {
                            idx = j;
                            dis = abs(Pbi[i].x - Standard_Op(j));
                        }
                    }
                    Hg = Standard_H(idx);          // 对应标准高度
                    if (abs(Pbi[i].z - Hg) <= T_g) // if (abs(Pbi[i].z - Hg) <= T_g + delta_g*Pbi[i].x / 50 || Pbi[i].z < (GP_Bs + delta_bs*(Pbi[i].x / 50)))
                        p_GroundFinal[n].push_back(Pbi[i]);
                    else // if (Pbi[i].z - Hg <= 1.0)
                        p_ObstacleFinal[n].push_back(Pbi[i]);
                }
            }
            else
            {
                // #pragma omp single// [clauses]
                //		{
                for (i = 0; i < Pbi.size(); i++)
                {
                    p_ObstacleFinal[n].push_back(Pbi[i]);
                }
                //		}
            }
        }

        void GetFinalPoint()
        {
            int i = 0;
            float x = 0.0, y = 0.0;
            /*vector<MyPoint>::iterator vector_iterator, it;
            sort(GroundFinal.begin(), GroundFinal.end(), LessSort2);
            vector_iterator = unique(GroundFinal.begin(), GroundFinal.end());
            if (vector_iterator != GroundFinal.end())
                GroundFinal.erase(vector_iterator, GroundFinal.end());

            sort(ObstacleFinal.begin(), ObstacleFinal.end(), LessSort2);
            it = unique(ObstacleFinal.begin(), ObstacleFinal.end());
            if (it != ObstacleFinal.end())
                ObstacleFinal.erase(it, ObstacleFinal.end());*/

            for (i = 0; i < GroundFinal.size(); i++)
            {

                x = GroundFinal[i].x * cos(GroundFinal[i].y * mypi / 180);
                y = GroundFinal[i].x * sin(GroundFinal[i].y * mypi / 180);
                GroundFinal[i].x = x;
                GroundFinal[i].y = y;
                GroundFinal[i].z -= LiDARHeight;
            }
            for (i = 0; i < ObstacleFinal.size(); i++)
            {

                x = ObstacleFinal[i].x * cos(ObstacleFinal[i].y * mypi / 180);
                y = ObstacleFinal[i].x * sin(ObstacleFinal[i].y * mypi / 180);
                ObstacleFinal[i].x = x;
                ObstacleFinal[i].y = y;
                ObstacleFinal[i].z -= LiDARHeight;
            }
        }

        void p_GetFinalPoint()
        {
            int i = 0;
            float x = 0.0, y = 0.0;
            /*vector<MyPoint>::iterator vector_iterator, it;
            sort(GroundFinal.begin(), GroundFinal.end(), LessSort2);
            vector_iterator = unique(GroundFinal.begin(), GroundFinal.end());
            if (vector_iterator != GroundFinal.end())
            GroundFinal.erase(vector_iterator, GroundFinal.end());

            sort(ObstacleFinal.begin(), ObstacleFinal.end(), LessSort2);
            it = unique(ObstacleFinal.begin(), ObstacleFinal.end());
            if (it != ObstacleFinal.end())
            ObstacleFinal.erase(it, ObstacleFinal.end());*/
            for (int j = 0; j < GP_M; j++)
            {
                for (i = 0; i < p_GroundFinal[j].size(); i++)
                {
                    MyPoint temp;
                    x = p_GroundFinal[j][i].x * cos(p_GroundFinal[j][i].y * mypi / 180);
                    y = p_GroundFinal[j][i].x * sin(p_GroundFinal[j][i].y * mypi / 180);
                    temp.x = x;
                    temp.y = y;
                    temp.z = p_GroundFinal[j][i].z;
                    temp.intensity = p_GroundFinal[j][i].intensity;
                    GroundFinal.push_back(temp);
                }
                for (i = 0; i < p_ObstacleFinal[j].size(); i++)
                {
                    MyPoint temp1;
                    x = p_ObstacleFinal[j][i].x * cos(p_ObstacleFinal[j][i].y * mypi / 180);
                    y = p_ObstacleFinal[j][i].x * sin(p_ObstacleFinal[j][i].y * mypi / 180);
                    temp1.x = x;
                    temp1.y = y;
                    temp1.z = p_ObstacleFinal[j][i].z;
                    temp1.intensity = p_ObstacleFinal[j][i].intensity;
                    ObstacleFinal.push_back(temp1);
                }
            }
        }

    } // namespace segmenter
} // namespace autosense