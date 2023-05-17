/////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2026, by Motovis All Rights Reserved.
//
// 本代码仅用于魔视智能与广州汽车集团股份有限公司汽车工程研究院合作的
// X3V项目（以下简称本项目），魔视智能对本代码及基于本代码开发产生的
// 所有内容拥有全部知识产权，任何人不得侵害或破坏，未经魔视智能授权许可
// 或其他法律认可的方式，任何企业或个人不得随意复制、分发、下载和使用，
// 以及用于非本项目的其他商业用途。
//
// 本代码仅供指定接收人（包括但不限于      ）在魔视智能授权范围内使用，
// 指定接收人必须征得魔视智能授权，才可在软件库中加入本代码。
//
// 本代码是受法律保护的保密信息，如您不是指定接收人，请立即将本代码删除，
// 法律禁止任何非法的披露、或以任何方式使用本代码。指定接收人应对本代码
// 保密信息负有保密义务，未经允许，不得超出本项目约定的披露、复制、传播
// 或允许第三方披露、复制、传播本代码部分或全部信息。
//
//
/////////////////////////////////////////////////////////////////////////
/**
* @file constants.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 为规划模块提供配置参数
* @version V2.0
* @author  xingyu zhang
* @date 2020年2月10日
* @note
*/


#ifndef CONSTANTS
#define CONSTANTS
#include <cmath>
#include "base.h"
#include "math_formula.h"
#include "vector"
#include"cmath"

#undef SMALL_CELL_SEARCH
#define VARIABLE_CELL
#define DEBUG_PRINT
#define CAL_ERROR (1e-5f)
#define FMAX_LIMIT (1e6)
#define NMAX_LIMIT (999)

#ifdef VARIABLE_CELL
#ifdef SMALL_CELL_SEARCH
#define NODE3D_NUM (1845000)
#else
#define NODE3D_NUM (1269000)
#endif
#else
#define NODE3D_NUM (1125000)
//#define NODE3D_NUM (6750000)

#endif
namespace Constants
{
/*最大迭代次数*/
static const int iterations = 50e4;

/*最大次数*/
static const int nMaxNodeNum = 1e5;

/*车辆宽度*/
static const float width = VEHICLE_WID;

/*车辆长度*/
static const float length = VEHICLE_LEN;

/*后轴到车辆尾部纵向距离*/
static const float RearEdgeToCenter = REAR_EDGE2CENTER;

/*后轴到车辆前边纵向距离*/
static const float FrontEdgeToCenter = FRONT_EDGE2CENTER;

/*后轴中心到车辆中心纵向距离*/
static const float RearCenterToVehCenter = length/2 - RearEdgeToCenter;

/*后视镜长度*/
static const float RearViewLength = REAR_VIEW_LENGTH;

/*后视镜宽度*/
static const float RearViewWidth = REAR_VIEW_WIDTH;

/*后视镜到车辆后轴中心纵向距离*/
static const float RearViewMirrorToCenter =  REAR_VIEW_2_FRONT_AXLE - RearViewWidth / 2;

/*轴距*/
static const float  WheelBase = WHEEL_BASE;

/*最小转弯半径*/
static const float  min_steering_radius= MAX(MAX(GetMinRearAxleCenterRadius(1, 1),GetMinRearAxleCenterRadius(1, -1)),
                                             MAX(GetMinRearAxleCenterRadius(-1, 1),GetMinRearAxleCenterRadius(-1, -1)));

/*最大曲率*/
static const float max_steering_curvature = 1 / min_steering_radius;

/*可变半径的步长(m)*/
static const float VarRadiusStepSize = 0.1f;

/*可变半径的数目+1*/
static const int VarRadiusNum = 3 + 1;

/*可变半径的方向：正负*/
static const int VarRadiusDir = 2;

/*Hybrid A*定半径搜索转弯半径*/
static const float r = min_steering_radius + 0.2f + (VarRadiusNum - 1) * VarRadiusStepSize;

/*不是最后一段的RS转弯半径，为了给最后一段RS连接成功留余量*/
static const float RNotInLastCurve = min_steering_radius + 0.4f;

/*车头泊入用到的最大转弯半径*/
static const float MaxROfHeadIn = 50.0f;

/*计算平行出库点的最小转弯半径*/
static const float parking_out_radius = min_steering_radius + 0.3f/*r*/;

/*最大前轴中心等效转角*/
static const float  planning_steering_angle=atanf(WheelBase/r);

/*普通模式车体朝向的离散数量*/
static const int headings = 120;

/*普通模式朝向离散步长倒数(rad-1)*/
static const float reverseDeltaHeadingRad = headings / (2 * PI);

/*普通模式搜索3D网格cell尺寸(m)*/
static const float cellSize = 0.2f;
static const float cellSize2 = 0.1f;

/*普通模式搜索3D网格cell尺寸倒数(m-1)*/
static const float reverseCellSize = 1 / cellSize;
static const float reverseCellSize2 = 1 / cellSize2;

/*普通模式搜索3D网格步长(m)*/
static const float TravelDis = 0.3f;
static const float TravelDis2 = 0.3f;

/*****************可变栅格参数设置Start***********************/
/*可变栅格小朝向数量*/
static const int variableCellheadings1 = 180;

/*可变栅格大朝向数量*/
static const int variableCellheadings2 = 240;

/*可变栅朝向小离散步长倒数(rad-1)*/
static const float reverseVariableCellDeltaHeadingRad1 = variableCellheadings1 / (2 * PI);

/*可变栅朝向大离散步长倒数(rad-1)*/
static const float reverseVariableCellDeltaHeadingRad2 = variableCellheadings2 / (2 * PI);

/*可变栅格最大宽度，任何可变区域操作不可以大于该数量*/
static const int varialeCellWidth = 60;

/*可变栅格最大高度，任何可变区域操作不可以大于该数量*/
static const int varialeCellHeight = 20;

/*****************可变栅格参数设置End***********************/

/*运动基元数量，前面5个，后面5个*/
static const int next_node_num_= 10;

/*各个搜索树对应的前轴偏角*/
static const float fSteering[next_node_num_] =
{
    -planning_steering_angle,
    -planning_steering_angle / 2,
    0,
    planning_steering_angle / 2,
    planning_steering_angle,
    -planning_steering_angle,
    -planning_steering_angle / 2,
    0,
    planning_steering_angle / 2,
    planning_steering_angle
};

/*各个搜索树对应的tan(前轴偏角)*/
static const float fTanSteering[next_node_num_] =
{
    tanf(fSteering[0]),
    tanf(fSteering[1]),
    tanf(fSteering[2]),
    tanf(fSteering[3]),
    tanf(fSteering[4]),
    tanf(fSteering[5]),
    tanf(fSteering[6]),
    tanf(fSteering[7]),
    tanf(fSteering[8]),
    tanf(fSteering[9])
};

///*运动基元数量，前面5个，后面5个*/
//static const int next_node_num_= 14;
///*各个搜索树对应的前轴偏角*/
//static const float fSteering[next_node_num_] =
//{
//    -planning_steering_angle,
//    -planning_steering_angle * 2 / 3,
//    -planning_steering_angle * 1 / 3,
//    0,
//    planning_steering_angle * 1 / 3,
//    planning_steering_angle * 2 / 3,
//    planning_steering_angle,
//    -planning_steering_angle,
//    -planning_steering_angle * 2 / 3,
//    -planning_steering_angle * 1 / 3,
//    0,
//    planning_steering_angle * 1 / 3,
//    planning_steering_angle * 2 / 3,
//    planning_steering_angle
//};

///*各个搜索树对应的tan(前轴偏角)*/
//static const float fTanSteering[next_node_num_] =
//{
//    tanf(fSteering[0]),
//    tanf(fSteering[1]),
//    tanf(fSteering[2]),
//    tanf(fSteering[3]),
//    tanf(fSteering[4]),
//    tanf(fSteering[5]),
//    tanf(fSteering[6]),
//    tanf(fSteering[7]),
//    tanf(fSteering[8]),
//    tanf(fSteering[9]),
//    tanf(fSteering[10]),
//    tanf(fSteering[11]),
//    tanf(fSteering[12]),
//    tanf(fSteering[13])
//};

/*栅格地图cell尺寸(m)*/
static const float obSize = 0.1f;

/*栅格地图cell尺寸倒数(1/m)*/
static const int inverseObSize = 1 / obSize;

/*纵向大圆的个数*/
static const int OuterDiskLNum = 3;

/*横向大圆的个数*/
static const int OuterDiskWNum = 1;

/*碰撞检测用到的大外圆半径*/
static const float OuterDiskRadius = sqrtf(powf((width + 0.2f)/OuterDiskWNum/2,2)+powf((length + 0.2f)/OuterDiskLNum/2,2));

/*大外圆圆心纵向距离*/
static const float OuterDiskLDis = (length + 0.2f)/OuterDiskLNum;

/*大外圆圆心横向距离*/
static const float OuterDiskWDis = (width + 0.2f)/OuterDiskWNum;

/*粗糙圆检测三个大圆的x,y坐标*/
static const float BigCircleCenter[3][2] =
{
    {-(RearEdgeToCenter + 0.1f) + OuterDiskLDis/2 + 0*OuterDiskLDis, 0},
    {-(RearEdgeToCenter + 0.1f) + OuterDiskLDis/2 + 1*OuterDiskLDis, 0},
    {-(RearEdgeToCenter + 0.1f) + OuterDiskLDis/2 + 2*OuterDiskLDis, 0},
};

/*小圆的个数*/
static const int SmallCircleNum = 12;

/*大圆的个数*/
static const int BigCircleNum = 4;

/*后视镜圆的数量*/
static const int RearViewCircleNum = 2;

/*精确圆检测大小圆后视镜圆的总个数*/
static const int CircleNum = SmallCircleNum + BigCircleNum + RearViewCircleNum;

/*包络圆（包括大圆和小圆）和车辆轮廓固定外沿距离*/
static const float CicleAndVehicleContourDis = 0.1f;

/*大圆的半径*/
static const float BigCicleRadius = width / 2 + CicleAndVehicleContourDis;

/*小圆的半径*/
static const float SmallCicleRadius = (2 + sqrtf(2)) * CicleAndVehicleContourDis;

/*后视镜圆的半径*/
static const float RearViewCicleRadius = hypotf(RearViewLength / 2, RearViewWidth / 2);

/*精确圆检测大圆,小圆,后视镜圆的x,y坐标*/
static const float CircleCenter[CircleNum][2] =
{
    {-RearEdgeToCenter + (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
     + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 0 / 3
    },

    {-RearEdgeToCenter + (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
     + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 1 / 3},

    {-RearEdgeToCenter + (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
     + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 2 / 3},

    {-RearEdgeToCenter + (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
    + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 3 / 3},

    {FrontEdgeToCenter - (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
     + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 0 / 3},

    {FrontEdgeToCenter - (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
     + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 1 / 3},

    {FrontEdgeToCenter - (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
    + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 2 / 3},

    {FrontEdgeToCenter - (SmallCicleRadius - CicleAndVehicleContourDis),
     -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)
     + (width - 2 * (SmallCicleRadius - CicleAndVehicleContourDis)) * 3 / 3},

    {-RearEdgeToCenter + (SmallCicleRadius - CicleAndVehicleContourDis) * 3,
    -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)},

    {-RearEdgeToCenter + (SmallCicleRadius - CicleAndVehicleContourDis) * 3,
     width/2 - (SmallCicleRadius - CicleAndVehicleContourDis)},

    {FrontEdgeToCenter - (SmallCicleRadius - CicleAndVehicleContourDis) * 3,
    -width/2 + (SmallCicleRadius - CicleAndVehicleContourDis)},

    {FrontEdgeToCenter - (SmallCicleRadius - CicleAndVehicleContourDis) * 3,
    width/2 - (SmallCicleRadius - CicleAndVehicleContourDis)},

    {-RearEdgeToCenter + 4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2))
     + (length - 2 * (4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2)))) * 0.0f / 3,
     0},

    {-RearEdgeToCenter + 4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2))
     + (length - 2 * (4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2)))) * 1.0f / 3,
     0},

    {-RearEdgeToCenter + 4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2))
     + (length - 2 * (4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2)))) * 2.0f / 3,
    0},

    {-RearEdgeToCenter + 4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2))
     + (length - 2 * (4 * (SmallCicleRadius - CicleAndVehicleContourDis)
     + sqrtf(powf(BigCicleRadius, 2) - powf(width/2, 2)))) * 3.0f / 3,
    0},

    {RearViewMirrorToCenter - RearViewWidth / 2,
     width / 2 + RearViewLength / 2
    },

    {RearViewMirrorToCenter - RearViewWidth / 2,
     -width / 2 - RearViewLength / 2
    }
};

/*障碍物危险距离*/
static const float dangerous_search_obs_dis = 0.1f;

/*危险距离惩罚系数*/
static const float dangerous_search_penalty  = 0.2f;

/*障碍物安全距离*/
static const float normal_search_obs_dis  = 0.4f;

/*安全距离惩罚系数*/
static const float normal_search_penalty  = 0.4f;

/*前向行驶惩罚代价系数*/
static const float traj_forward_penalty = 1.0f;

/*后向行驶惩罚代价系数*/
static const float traj_back_penalty = 1.0f;

/*换挡惩罚代价系数*/
static const float traj_gear_switch_penalty = 10.0f;

/*同向规划惩罚系数*/
static const float traj_same_direc_penalty = 2.5f;

/*转向惩罚代价系数*/
static const float traj_steer_penalty_para = 0.0f;
static const float traj_steer_penalty_obli = 0.3f;

/*转向改变惩罚代价系数*/
static const float traj_steer_change_penalty = 2.0f;

/*不合理转向惩罚代价系数*/
static const float traj_illogical_steer_penalty = 2.0f;

/*连接调用RS连接曲线时是否在距离范围内的阈值*/
static const float ReedsSheepsShotDistance = 10.0f;

/*RS连接曲线离散点的距离*/
static const float ReedsSheepsStepSize = TravelDis;

/*障碍物栅格地图高*/
static const int ob_grid_height = GRID_HEIGHT;

/*障碍物栅格地图宽*/
static const int ob_grid_width = GRID_WIDTH;

/*障碍物栅格地图网格数量*/
static const int ob_grid_num = ob_grid_height * ob_grid_width;

/*搜索网格的3D网格高*/
static const int Node3dHeight = round(ob_grid_height * 0.1f / cellSize);
static const int Node3dHeight2 = round(ob_grid_height * 0.1f / cellSize2);

/*搜索网格的3D网格宽*/
static const int Node3dWidth = round(ob_grid_width * 0.1f / cellSize);
static const int Node3dWidth2 = round(ob_grid_width * 0.1f / cellSize2);

/*最大规划转向角(°)*/
static const int max_steering_angle = MAX_STEERING_ANGLE;

/*最大方向盘角速度(°/s)*/
static const int max_steering_angle_speed = MAX_STEERING_ANGLE_SPEED;

/*平行车位入库点膨胀安全距离*/
static const float paraVehBloating = 0.4f;

/*初始点膨胀安全距离*/
static const float startVehBloating = 0.3f;

/*车辆后倒角纵向长度*/
static const float rear_chamfer_l1 =  REAR_CHAMFER_L1;

/*车辆后倒角横向长度*/
static const float rear_chamfer_l2 =  REAR_CHAMFER_L2;

/*车辆前45度倒角长度*/
static const float chamfer_length = CHAMFER_LENGTH;

/*平行车位的长度*/
static const float para_min_slot_length  = VEHICLE_LEN + 0.7f;

/*平行车位的深度*/
static const float para_min_slot_depth  = VEHICLE_WID;

/*平行车位的长度*/
#ifdef BYD_HAN
static const float obli_min_slot_length  = VEHICLE_WID + 0.15f;
#else
static const float obli_min_slot_length  = VEHICLE_WID + 0.4f;

#endif
/*平行车位的深度*/
static const float obli_min_slot_depth  = VEHICLE_LEN;

/*最小的斜列车位角度*/
static const float obli_min_slot_yaw  = 30*PI/180;

/*最大的斜列车位角度*/
static const float obli_max_slot_yaw  = 150*PI/180;

/*垂直车位区域外的安全碰撞距离(圆检测)*/
static const float obli_out_slot_area_dis = 0.2f; //yangyi

/*垂直车位区域内的安全碰撞距离(圆检测)*/
static const float obli_slot_area_dis = 0.0f;

/*平行车位区域外的安全碰撞距离(圆检测)*/
static const float para_out_slot_area_dis = 0.2f;

/*平行车位区域内的安全碰撞距离(圆检测)*/
static const float para_slot_area_dis = 0.0f;

/*平行车位角点安全距离(线检测)*/
static const float para_slot_corner_safety_dis = 0.15f;

/*插值步长(/m)*/
static const float step_size = 0.05f;

/*额外的最大搜素次数*/
static const int MaxAdditionalSearchTimes = 200;
}
#endif // CONSTANTS

