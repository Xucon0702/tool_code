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
#ifndef PUBLICFUNPATHPLAN_H
#define PUBLICFUNPATHPLAN_H

#include <vector>
#include "base.h"
#include "vehicle.h"
#include "vector2d.h"
#include "node3d.h"
//#include "helper.h"
#include "Bspline.h"
#include "Knots.h"

#define BSLINE_ORDER            (int)(3)
#define MIN_BSLINE_INTER_NUM (BSLINE_ORDER + 1)

#define INTER_STEP_SIZE (0.05)
#define FORWARD_ACC (0.1f)
#define BACKWARD_ACC (0.1f)

#undef PrintKappa
#define MAX_PLANNING_VELOCITY (0.6f)

#ifdef CHERY_EQ
#define MIN_PLANNING_VELOCITY (0.2f)

#elif defined VEL_JIHE
#define MIN_PLANNING_VELOCITY (0.3f)
#elif defined VEL_BORUI
#define MIN_PLANNING_VELOCITY (0.2f)
#elif defined CHERY_T1D
#define MIN_PLANNING_VELOCITY (0.3f)
#else
#define MIN_PLANNING_VELOCITY (0.3f)
#endif


#define LOW_SPEED_DIS_LIMIT (0.5f)
#define MIN_DIS_VPLAN (0.0f)
#define CHANGE_SPEED_NUM (int(MAX_PLANNING_VELOCITY*10))

#define PLAN_MAX_STEERING_ANGLE_SPEED (700)                       //最大方向盘角速度


enum PlanningStatus
{
    planningOver = 0, /*规划到位*/
    PlanningSucc,     /*规划成功*/
    PlanningFail     /*规划结束*/
};


namespace HybridAStarPart
{
/*定义点的数据结构*/
struct cpoint
{
  float x;
  float y;
  cpoint():x(0), y(0){}
  cpoint(float a, float b):x(a),y(b){}

};

/*直线方程*/
struct LineEq
{
    float A;
    float B;
    float C;
    LineEq():A(0),B(0),C(0){}
    LineEq(float a, float b, float c):A(a),B(b),C(c){}
};

struct LineSeg
{
    cpoint p0;
    cpoint p1;
};

struct slotPosInfor
{
    bool bValidty; /*车位信息有效位*/
    int8_t nSlotSide; /*1-右侧, -1-左侧*/
    int8_t nSlotType; /*车位类型*/
    int8_t nParkingType; /*泊车类型：包括车头泊入,车尾泊入,车头泊出,车尾泊出*/
    int8_t cParkOutDir; /*泊出方向向右,向左*/
    int8_t nDetectType; /*检测类型：超声，环视，自选*/
    int8_t nUssSideType; /*超声边界类型：前单边，后单边，双边*/
    float fXSlotPosInMap; /*车位坐标系在地图坐标系中的坐标x*/
    float fYSlotPosInMap;/*车位坐标系在地图坐标系中的坐标y*/
    float fSlotLength;/*车位长度*/
    float fSlotWidth;/*车位宽度*/
    float fSlotAngle;/*车位角度*/
    float fXFrontSlotCorner;/*前角点x*/
    float fYFrontSlotCorner;/*前角点y*/
    float fXRearSlotCorner; /*后角点x*/
    float fYRearSlotCorner; /*后角点y*/
    float fCrossValueStartAndGoalPoint;/*起点终点叉乘*/
    float fRoadWidth;
    float fGroundpinDepth;/**/
    LocationPoint startPointInSlot;/*车位坐标系中的起点坐标*/
    LocationPoint goalPointInSlot;/*车位坐标系中的终点坐标*/
    LocationPoint startPointInMap;/*地图坐标系中的起点坐标*/
    LocationPoint goalPointInMap;/*地图坐标系中的终点坐标*/
    LocationPoint ParkingInPointInMap;/*地图坐标系中的终点坐标*/
    LocationPoint ParkingInPointInSlot;/*地图坐标系中的终点坐标*/
    LocationPoint safePointParkingIn;/*垂直泊车的安全摆正入库点*/
    LineEq LineEq_Goal; /*地图坐标系目标点的直线方程*/

    LocationPoint targetPointInSlot;
    LocationPoint tFrontSlotCorner;
    LocationPoint tRearSlotCorner;
    bool isAvoidance;
    slotsideType SlotLineType[SlotSideNum]; 
    slotPosInfor():bValidty(false),nSlotSide(0),nSlotType(0), nParkingType(0),cParkOutDir(0),
                   nDetectType(0),nUssSideType(0),fXSlotPosInMap(0.0f), fYSlotPosInMap(0.0f),
                   fSlotLength(0.0f), fSlotWidth(0.0f), fSlotAngle(0.0f), fXFrontSlotCorner(0.0f), fYFrontSlotCorner(0.0f),
                   fXRearSlotCorner(0.0f), fYRearSlotCorner(0.0f), fCrossValueStartAndGoalPoint(0.0f), fRoadWidth(6.0f){}
};

/**
 * @brief The Circle
 */
struct Circle
{
    float x;
    float y;
    float R;
    Circle():x(0),y(0),R(0){}
    Circle(float a, float b, float c):x(a),y(b),R(c){}
};

/**
 * @brief 带方向的线段
 */
struct LineSegment
{
    cpoint s;
    cpoint e;
    Vector2D dir;
    LineSegment(){}
    LineSegment(cpoint start, cpoint end, Vector2D vec)
    {
        s = start;
        e = end;
        dir = vec;
    }
};

struct Polygon
{
    ushort edgsNum;
    std::vector<LineSegment> edgs;
    std::vector<cpoint> vertices;
    Polygon():edgsNum(0){}
};

struct ObsDisInfor
{
    size_t index;
    float fDis;
    ObsDisInfor():index(0),fDis(0.0f){}
};

float normalizeHeadingRad_0_2pi(float x);

float normalizeHeadingRad_Npi_Ppi(float x);

double clamp(double n, double lower, double upper);

double getAcos(double cosTheta);

/**
 * @brief 排斥判断: p1p2为斜边的矩形以及q1q2为斜边的矩形是否重叠
 * @return 1-重叠 0-不重叠
 */
bool IsRectCross(const LocationPoint &p1,
                 const LocationPoint &p2,
                 const LocationPoint &q1,
                 const LocationPoint &q2);

/**
 * @brief  跨立判断: 判断一个线段是否相交
 * @return 1-相交 0-不相交
 */
bool IsLineSegmentCross(const LocationPoint &P1,
                        const LocationPoint &P2,
                        const LocationPoint &Q1,
                        const LocationPoint &Q2);

/**
求线段P1P2与Q1Q2的交点。
先进行快速排斥实验和跨立实验确定有交点再进行计算。
交点（x,y）使用引用返回。
没有验证过
**/
bool GetCrossPoint(const LocationPoint &p1,
                   const LocationPoint &p2,
                   const LocationPoint &q1,
                   const LocationPoint &q2,
                   LocationPoint &interPoint);


/**
 * @brief 圆弧插值
 * @param nDrivingDirection 行驶方向
 * @param nSteeringDirection 转动方向
 * @param kappa 曲率
 * @param fCurveLength 曲线长度
 * @param tPoint 起点
 * @param targetSeg 输出结果
 */
void CircleInterpolate(int8_t nDrivingDirection,
                       int8_t nSteeringDirection,
                       float kappa,
                       float fCurveLength,
                       LocationPoint tPoint,
                       std::vector<CtrlPoint> &targetSeg);

/**
 * @brief 计算离散曲率和速度规划
 * @param targetCtrlPointSets 目标控制点集合,最少三个点
 */
void CalDiscretePointCurvature(std::vector<CtrlPoint>& targetCtrlPointSets);

/**
 * @brief 速度规划
 * @param target_curvature_sets 目标控制点集合
 */
void VelocityPlanning(std::vector<CtrlPoint>& targetCtrlPointSets);


bool JudgeOneVectorIsInMiddleOfOtherTwoVectors(Vector2D vec1, Vector2D vec2, Vector2D vec, int nClockDir);

/**
 * @brief 计算含偏航角点的直线方程点的直线方程
 */
void CalLineEquation(LocationPoint tp, LineEq &LineEq_);

float CalPointAndLineDis(const cpoint &tp, const LineEq &LineEq_);

bool LeftOfLine(LocationPoint tp1,LocationPoint tp2, LocationPoint tCheckPoint);
/**
 * @brief 直线交点
 */
bool CalLineCrossPoint(LineEq  LineEq_1, LineEq  LineEq_2, LocationPoint &tp);

/**
 * @brief 圆和直线的交点Ax + By + C = 0与（x - X）^2 + (y - Y）^2 = R^2交点1和交点2
 */
bool InterPointBetweenCircleAndLine(Circle circle,
                                    LineEq line,
                                    std::vector<LocationPoint> &Points);

/**
 * @brief 计算车辆左右后视镜位置点坐标
 * @param tVehPos 后轴坐标
 * @param tMirror 后视镜外侧点坐标
 */
void CalMirrorPointOfVehicle(const LocationPoint &tVehPos, LocationPoint tMirror[]);
/**
 * @brief 计算后视镜等效直线
 * @param tVehPos 后轴坐标
 * @param mirrorLine 后视镜等效直线
 */
void CalMirrorLineOfVehicle(const LocationPoint &tVehPos, LineSeg mirrorLine[]);
/**
 * @brief 点到点距离
 */
float CalPointDis(LocationPoint p1, LocationPoint p2);

/**
 * @brief 判断点在车辆轮廓内
 */
bool pointInVehicle(const LocationPoint &vehPos, const cpoint &p);

/**
 * @brief 判断点在多边形内
 */
bool pointInPolygon(float x, float y, int polySides, float* polyX, float* polyY);

/**
 * @brief CalSafeDis 计算安全距离
 * @param tR 车位后角点
 * @param tF 车位前角点
 * @param slotCenterLineEq 车位中线方程
 * @return fSafeDis 安全距离
 */
void CalSafeDis(const cpoint &tR, const cpoint &tF, const LineEq &slotCenterLineEq,
                 float &fVehContourSafeDis, float &fMirrorSafeDis);
/**
 * @brief 车位角点和车辆轮廓是否碰撞
 * @param tVehPos 车辆坐标
 * @param tR 车位后角点
 * @param tF 车位前角点
 * @param fSafeDis 安全距离
 * @return 1-碰 0-安全
 */
bool CollisionCheckByVehKeyPointAndSlotCorner(const LocationPoint &tVehPos,
                                              const cpoint &tR, const cpoint &tR2,
                                              const cpoint &tF, const cpoint &tF2,
                                              float fSafeDisOfVehContour, float fSafeDisOfMirror, bool bHasSlotBottomLine = false);
/**
 * @brief 判断点投影在线段上
 * @param LineSeg_ 线段
 * @param point 点
 * @return 1-投影点在线段上， 0-投影点在线段外
 */
bool CheckPointProjectionInLineSegment(const LineSeg &LineSeg_, cpoint point);

/**
 * @brief 点和线段的最近距离
 */
float CalDisBetweenLineSegmentAndPoint(const LineSeg &LineSeg_, const cpoint &point);

/**
 * @brief 线段之间的最小距离
 */
float CalDisBetweenLineSegments(const LineSeg LineSeg_[]);
/**
 * @brief 创建多边形
 */
Polygon CreatePolygon(ushort edgsNum, std::vector<cpoint> vertices);

/**
 * @brief 凸多边形和圆的相交判断
 * @param polygon 凸多边形
 * @param circle 圆
 * @return 1-相交 0-不相交
 */
bool CrossDetectorPolyVsCircle(const Polygon &polygon, const Circle &circle);

bool CrossDetectorPolyVsCircleDEBUG(const std::vector<cpoint> &PolyVertices,
                                    const Circle &circle,
                                    std::vector<cpoint>  &circlePoints);
/**
 * @brief 创建车位区域多边形
 * @param slotPosInfor_
 * @param polygon
 */
void CreatSlotAreaPolygon(const slotPosInfor &slotPosInfor_, Polygon &polygon);


/**
 * @brief 根据车辆位置姿态计算四个角点坐标
 * @param tVehicleRearAxleCenter 后轴中心点位置姿态
 * @param tCorner 角点坐标
 */
void CalVehicleCornerPoint(const LocationPoint &tVehicleRearAxleCenter, LocationPoint tCorner[]);

/**
 * @brief 计算一点绕一点旋转一定角度后的新坐标
 * @param tCenter 中心点
 * @param tRotationPoint 旋转点
 * @param fSinYaw sin旋转角度
 * @param fCosYaw cos旋转角度
 * @param tRotationNewPoint 新坐标
 */
void GetNewPointOfRotatation(const LocationPoint &tCenter,
                             const LocationPoint &tRotationPoint,
                             float fSinYaw,
                             float fCosYaw,
                             LocationPoint& tRotationNewPoint);
}



#endif // PUBLICFUNPATHPLAN_H
