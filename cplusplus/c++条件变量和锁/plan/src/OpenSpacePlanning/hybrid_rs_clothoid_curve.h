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
* @file hybrid_rs_clothoid_curve.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief ReedSheep曲线衍生曲线函数，为hybrid A*搜索算法提供连接曲线
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该曲线包括CCS,SCS，CS三种曲线形式；三种曲线为ReedsShepp曲线所不具备的，但是却是泊车中重要的连接曲线形式；
* 该曲线的数学推导形式主要参考reedsSheep曲线，其中最重要的是四种曲线变换:timeflip，reflect,timeflip+reflect,backforwards;
* 依据该理论衍生本算法,算法的详细推导参考文档；
* 引用文献 :
* 1 Reeds, J.A. & Shepp, L.A.. (1990). Optimal paths for a car that goes both forwards and backwards. Pacific Journal of Mathematics. 145. 367-393.
* 2 Dubins, L.E.. (1957). On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents. American Journal of Mathematics. 79. 10.2307/2372560.
*/


#ifndef HYBRIDRSCLOTHOIDCURVE_H
#define HYBRIDRSCLOTHOIDCURVE_H

#include <vector>
#include <cmath>
#define NDEBUG
#include <cassert>
#include"base.h"
#if PLATFORM==PC_SYSTEM
#include "QDebug"
#endif
#include"constants.h"
#include"node3d.h"

#define ZERO (1e-6f)
//#define MIN_ARC_LEN (0.1f)
//#define MIN_LINE_LEN (0.0f)
#define  RS_ERR (1e-3f)
namespace HybridAStarPart {

struct VehPos
{//定义点的数据结构
  float x;
  float y;
  float yaw;
  int8_t DrivingDirection;
  int8_t SteeringDirection;
  float steering;
  bool isStart;
  VehPos():x(0), y(0), yaw(0), DrivingDirection(0), SteeringDirection(0), steering(0.0), isStart(false){}
};

/*The Reeds-Shepp path segment types*/
typedef enum
{
    RS_NOP = 0,
    RS_LEFT = 1,
    RS_STRAIGHT = 2,
    RS_RIGHT = 3
} ReedsSheppPathSegmentType;


/*Reeds-Shepp path types*/
static const ReedsSheppPathSegmentType reedsSheppPathType[8][5] =
{
    { RS_LEFT,RS_RIGHT, RS_STRAIGHT, RS_NOP, RS_NOP }, /*0*/
    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_NOP, RS_NOP }, /*1*/
    { RS_STRAIGHT, RS_LEFT, RS_RIGHT,  RS_NOP, RS_NOP }, /*2*/
    { RS_STRAIGHT, RS_RIGHT, RS_LEFT,  RS_NOP, RS_NOP }, /*3*/
    { RS_STRAIGHT,RS_LEFT, RS_STRAIGHT, RS_NOP, RS_NOP }, /*4*/
    { RS_STRAIGHT,RS_RIGHT, RS_STRAIGHT, RS_NOP, RS_NOP }, /*5*/
    { RS_LEFT,RS_STRAIGHT, RS_STRAIGHT, RS_RIGHT, RS_STRAIGHT }, /*6*/
    { RS_RIGHT, RS_STRAIGHT, RS_STRAIGHT, RS_LEFT, RS_STRAIGHT }, /*7*/
};

/*Complete description of a ReedsShepp path*/
typedef struct _ReedsSheppPath
{
    bool validity;
    const ReedsSheppPathSegmentType* type_;
    float length_[5];
    float totalLength_;
    float rho_;
    _ReedsSheppPath()
    {
        validity = false;
        type_ = nullptr;
        totalLength_ =1.0e6;
        rho_ = 0.0;
        length_[0] = 0.;
        length_[1] = 0.;
        length_[2] = 0.;
        length_[3] = 0.;
        length_[4] = 0.;
    }
} ReedsSheppPath;

static float MIN_LINE_LEN = 0.0f;
static float MIN_ARC_LEN = 0.0f;

/**
 * @brief SetReedsSheppPath 设置路径类型和路径长度
 * @param path 路径
 * @param type 路径类型
 * @param t
 * @param u
 * @param v
 * @param w
 * @param x
 */
void SetReedsSheppPath(ReedsSheppPath &path,
                       const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                       float t = 1e6,
                       float u = 0.0,
                       float v = 0.0,
                       float w = 0.0,
                       float x = 0.0);

/**
 * @brief CheckRSCurveIsSameDirection 判断是不是同向的RS曲线
 * @param path
 * @return 1-同向 0-换向
 */
bool CheckRSCurveIsSameDirection(ReedsSheppPath path);

/**
 * @brief AddLineSegInCpCnSnPath 在CpCnSn增加一段缓和直线
 * @param path 路径
 * @return 1-增加成功，0-增加失败
 */
bool AddLineSegInCpCnSnPath(ReedsSheppPath &path);

/**
 * @brief reedsShepp RS曲线
 * @param reverse 0-可以换向的RS曲线; 1-不可以换向的RS曲线
 * @param q0 起点
 * @param q1 终点
 * @param rho_ 比例系数半径）
 * @param path 路径
 */
void reedsShepp(bool reverse, LocationPoint q0, LocationPoint q1, float rho_, ReedsSheppPath& path);

/**
 * @brief reedsSheppParallel 平行泊车曲线连接函数
 * @param q0 起点
 * @param q1 终点
 * @param rho_ 比例系数(半径）
 * @param path 路径
 */
void reedsSheppParallel(LocationPoint q0, LocationPoint q1, float rho_, ReedsSheppPath& path);

/**
 * @brief ReedsShepp_interpolate
 * @param q0 起点
 * @param path 路径
 * @param fUnitArcLength 单位长度
 * @param rho_ 比例系数(半径）
 * @param s 在单位长度位置的插值坐标
 */
void ReedsShepp_interpolate(LocationPoint q0, float fUnitArcLength, float rho_, ReedsSheppPath &path, LocationPoint &s);

/**
 * @brief Generate_ReedsShepp_path 生成RS连接路径
 * @param slotType 车位类型
 * @param reverse 0-不可换挡; 1-可以换挡
 * @param q0 起点
 * @param q1 终点
 * @param rho_ 比例系数(半径）
 * @param step_size 插值点步长
 * @param points 插值点
 * @param path2 输出的RS路径
 * @return 1-曲线有解;0-曲线无解
 */
bool Generate_ReedsShepp_path(int slotType,
                              bool reverse,
                              LocationPoint q0,
                              LocationPoint q1,
                              float rho_,
                              float step_size,
                              std::vector<Node3D> &points,
                              ReedsSheppPath &path2);
}

#endif // HYBRIDRSCLOTHOIDCURVE_H
