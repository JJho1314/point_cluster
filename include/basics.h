#ifndef _BASIC_H_
#define _BASIC_H_

#pragma once

#include <vector>
#include <Eigen/Dense>
#include <map>
#include <stdio.h>

#include "common/types/object.hpp"
#include "common/types/type.h"

#define PI 3.1415926 // 535897932384626433832795
#define OBJECT_NUM 100
#define RESULT_SIZE GRID_M *GRID_N / 8
#define GRID_M 80 // ���Ƿ��͵Ĵ�С��ÿһ�����޵�դ���ͼ��Ҫ��Ϊ���������ֱ�΄1�720*20�ף�80*80դ��800�ֽڣ�
#define GRID_N 80 //

#define MyMin(X, Y) ((X) < (Y) ? (X) : (Y))
#define MyMax(X, Y) ((X) > (Y) ? (X) : (Y))
#define MyName(X) #X
#define VAR_OUT(X)                             \
    {                                          \
        string xname = MyName(X);              \
        cout.setf(ios::left);                  \
        cout.width(30);                        \
        cout << xname.substr(0, xname.size()); \
        cout.unsetf(ios::left);                \
        cout.fill(' ');                        \
        cout.width(30);                        \
        cout << X << endl;                     \
    }

#define MSG_OUT(head, msg)                                                                 \
    do                                                                                     \
    {                                                                                      \
        fprintf(stderr, "[%s  ]%s %s(Line %d): ", head, __FILE__, __FUNCTION__, __LINE__); \
        fprintf(stderr, msg);                                                              \
        printf("\n");                                                                      \
    } while (0)

#define MyERROR(...) MSG_OUT("ERROR", __VA_ARGS__)

#define MyWARNING(...) MSG_OUT("WARNING", __VA_ARGS__)

#define MyDEBUG(...) // MSG_OUT("DEBUG", __VA_ARGS__)

#define MyINFO(...) printf(__VA_ARGS__) // ����ʱ���԰���һ��printf�����ã��磺MyINFO("%s, %s\n", time, number);

typedef Eigen::Matrix<float, 2, 2, Eigen::DontAlign> Matrix2;
typedef Eigen::Matrix<float, 4, 4, Eigen::DontAlign> Matrix4;
typedef Eigen::Matrix<float, 6, 6, Eigen::DontAlign> Matrix6;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::DontAlign> Mat;

struct Denosing
{
    int index;
    int dis; // �����پ���
};

struct MyPoint
{
    float x;
    float y;
    float z;
    short intensity;
    short type = -1;
    int Index;
    short Isit = -1;
};

struct LShape
{
    int L_index;
    float k;
    std::vector<MyPoint> Line;
};

struct MyBox
{
    MyPoint center;
    LShape Ldata;
    float l;
    float w;
    float angle;
    float vx, vy;
    float ax, ay;
    float boxType;
    float zmax, zmin;
    std::vector<MyPoint> data;
    // int b;
};

// using for saving corner points
struct cornerPoints
{
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
    float x4;
    float y4;
};

// struct ObstacleList{
//	int num;
//	float x[OBJECT_NUM];
//	float y[OBJECT_NUM];
//	float l[OBJECT_NUM];
//	float w[OBJECT_NUM];
//	float angle[OBJECT_NUM];
//	float v[OBJECT_NUM];
//	float a[OBJECT_NUM];
//	int type[OBJECT_NUM];
//	//Obstacle o[OBJECT_NUM];
// };

// union ObstacleUnion{
//	ObstacleList sendobjs;
//	char sendbuff[sizeof(ObstacleList)];
// };

struct ResultData
{
    unsigned char Result[RESULT_SIZE];
    char zone_flag;
};

union GridMapData
{
    ResultData data;
    unsigned char buff[sizeof(ResultData)];
};

typedef struct _EPS
{
    double eps;
    struct _EPS *next;
} Eps;

typedef struct _DATASTRUCT
{
    float x;
    float y;
    float z;
    int grid_x, grid_y;
    int index;
    int cluster_flag;
    Eps *link_head;
    struct _DATASTRUCT *next;
} DataStruct;

struct MyBox_3D
{
    MyPoint center;
    double l, w, h;
    double angle;
    double v, a;
    double vertexes[8][3]; // ������İ˸����ￄ1�7
    double boxType;
};

struct GPSDATA
{
    float x;
    float y;
    float h;
    float theta; // z
    float roll;  // x
    float pitch; // y
    int NumOfSatellites;
    float PositionAccuracy;
    float OrientationAccuracy;
};

typedef struct _MySTRUCT
{
    float x;
    float y;
    float z;
    int grid_x, grid_y, grid_z;
    int cluster_flag;
} MySTRUCT;

// Data Recievement related
struct UDP_Package //? ����64��32��Ϊʲô����ֱ�Ӻ��Ե�ǰ��ￄ1�742��ͷ������ￄ1�7
{
    char FiringData[1200];
    char Status[6];
};

union UDP_Data
{
    char Recvchars[1206];
    UDP_Package package;
};

struct RS_Package
{
    char Head[42];
    char FiringData[1200];
    char Tail[6];
};

union RS_Data
{
    char Recvchars[1248];
    RS_Package package;
};

struct RSDIFOP_Package
{
    char Head[8];
    char FiringData[1238];
    char Tail[2];
};

union RSDIFOP_Data
{
    char Recvchars[1248];
    RSDIFOP_Package package;
};

struct PublicInfo
{
    float x;
    float y;
    float z;
    float heading;
    float roll;
    float pitch;
    float speed;
    float yawrate;
    float steerangle;
};

union Vehicle_Data
{
    char RecvBuff[sizeof(PublicInfo)];
    PublicInfo package;
};

struct Location
{
    float t;             // ʱ��
    float x;             // x����
    float y;             // y����
    float z;             // z����
    float vx;            // x�����ٶ�
    float vy;            // y�����ٶ�
    float vz;            // z�����ٶ�
    float ax;            // x������ٶￄ1�7
    float ay;            // y������ٶￄ1�7
    float az;            // z������ٶￄ1�7
    float roll;          // ��ת�ǣ�Χ��X����ת
    float pitch;         // �����ǣ�Χ��Y����ת
    float yaw;           // ƫ���ǣ�Χ��Z����ת
    float vroll;         // Χ��X����ת���ٶ�
    float vpitch;        // Χ��Y����ת���ٶ�
    float vyaw;          // Χ��Z����ת���ٶ�
    float latstd;        // γ�Ⱦ���
    float lonstd;        // ���Ⱦ���
    float hstd;          // ���θ߶Ⱦ���
    float vnstd;         // �����ٶȾ���
    float vestd;         // �����ٶȾ���
    float vdstd;         // �����ٶȾ���
    float rollstd;       // roll�Ǿ���
    float pitchstd;      // pitch�Ǿ���
    float yawstd;        // yaw�Ǿ���
    int locationType;    // GPS��λ״̬
    int NumOfSatellites; // ���ǵ�����
    int directionType;   // GPS����״̬
    int wheelSpeed;      // ����״̬
};
union Location_Data
{
    char RecvBuff[sizeof(Location)];
    Location package;
};

struct ROIPoint
{
    MyPoint point;
    int index_row;
    int index_col;
};

union TwoCharsInt
{
    char datain[2];
    unsigned short dataout;
};

union OutputPackage
{
    float xydata[83];
    unsigned char cdata[332];
};

// Track module
struct Tracker
{
    float x;
    float y;
    float theta, L, W, vx, vy, ax, ay;
    int index_last, index;
    int tracking_frames;
    int losted_frames;
};

struct State
{
    Eigen::Matrix<float, 6, 1, Eigen::DontAlign> basic_state;
    Matrix6 F, Q, P, H, R;
    int serial_num;
};

struct State2
{
    Eigen::Matrix<float, 4, 1, Eigen::DontAlign> basic_state;
    Matrix4 F, Q, P;
    Eigen::Matrix<float, 2, 4, Eigen::DontAlign> H;
    Matrix2 R;
    // Matrix TR;
    // Eigen::Matrix<float, 2, 1, Eigen::DontAlign> TT;
    int serial_num, id;
    double L, W, angle, zmin, zmax, height;
    // std::vector<MyPoint> data;
    autosense::PointICloudPtr cloud;
    int lost_age;
    int tracked_age;
    std::vector<float> history_length;
    std::vector<float> history_width;
    std::vector<float> history_height;
    std::vector<float> history_yaw;
    int object_type;
    int collion_age;
};

struct LogOfTrack
{
    std::vector<int> frames, serial_nums;
    std::vector<float> x, y, vx, vy, v;
};

struct Pairs
{
    int index_last, index;
    float weight;
    float ICPDistancex;
    float ICPDistancey;
};

struct Track_Tasks
{
    std::vector<MyBox> gBoxVector;
    // std::vector<MyBox> gBoxVectorTracking;
    // std::vector<MyPoint> gPointVector;
    // std::map<int, std::vector<int>> gIndex;
};

struct Draw_Tasks
{
    std::vector<MyBox> gBoxVector;
    std::vector<MyPoint> gPointVector;
};

// Send Info To Plan
struct ObstacleList
{
    short num;
    short x[OBJECT_NUM];
    short y[OBJECT_NUM];
    short l[OBJECT_NUM];
    short w[OBJECT_NUM];
    short angle[OBJECT_NUM];
    short vx[OBJECT_NUM];
    short vy[OBJECT_NUM];
    // short type[OBJECT_NUM];
    // Obstacle o[OBJECT_NUM];
};

union objsunion
{
    ObstacleList sendobjs;
    char sendbuff[sizeof(ObstacleList)];
};

// vision and lidar fusion
// TO DO: need to change to ros's message
struct visionBoxInt
{
    int imgclass;
    int u_left;
    int v_left;
    int u_right;
    int v_right;
};

// struct visionBox{
// 	short imgclass;
// 	short u_left;
// 	short v_left;
// 	short u_right;
// 	short v_right;
// };

// struct vision{
// 	short hour;
// 	short minu;
// 	short sec;
// 	short usec;
// 	short num;
// 	visionBox vision_box[50];
// };

// CVC
struct PointARP
{
    // float x;
    // float y;
    // float z;
    float azimuth;
    float polar_angle;
    float range;
    int voxel_r = -1;
    int voxel_p = -1;
    int voxel_a = -1;
    int index = -1;
};

struct voxel
{
    bool haspoint = false;
    int cluster = -1;
    int point_num = 0;
    std::vector<int> index;
    int voxel_r = -1;
    int voxel_p = -1;
    int voxel_a = -1;
};

#endif
