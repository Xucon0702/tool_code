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
* @file hybrid_a_star.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 路径规划的主函数,主要实现垂直泊车，斜列泊车，平行泊车
* hybrid A*搜索路径、轨迹分割处理、共轭梯度轨迹优化算法、控制点发布等主要功能
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该算法参考由Dmitri Dolgov在2007 DARPA Urban Challenge提出的hybrid A*算法主架构，
* 针对狭窄空间的泊车特殊情况,不同之处在于reedsSheep的曲线连接方式，路径的优化目标函数，以及惩罚函数的设计
* 引用文献 :
* 1 Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2010). Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments.
* 2 Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2008). Practical Search Techniques in Path Planning for Autonomous Driving.
* 3 Dolgov, Dmitri & Thrun, Sebastian. (2009). Autonomous driving in semi-structured environments: Mapping and planning.
* 4 KARL KURZER. (2016) . Path Planning in Unstructured Environments : A Real-time Hybrid A' Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle.
* 5 Mizuno, Naoki & Ohno, Kazunori & Hamada, Ryunosuke & Kojima, Hiroyoshi & Fujita, Jun & Amano, Hisanori & Westfechtel, Thomas & Suzuki, Takahiro & Tadokoro, Satoshi. (2019). Enhanced path smoothing based on conjugate gradient descent for firefighting robots in petrochemical complexes.
* 6 Tianyu Gu , Jarrod Snider , John M. Dolan. (2013) .Focused Trajectory Planning for Autonomous On-Road Driving.
* 7 Ziegler, Julius & Stiller, Christoph. (2010). Fast Collision Checking for Intelligent Vehicle Motion Planning.
* 8 Shih, Frank & Wu, Yi-Ta. (2004). Note Fast Euclidean distance transformation in two scans using a 3 3 neighborhood. Computer Vision and Image Understanding.
*/

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <cmath>
#include <queue>
#include <vector>
#include <sys/time.h>
#include<map>

#include"base.h"
#include"math_formula.h"
#include "node3d.h"
#include"vector2d.h"
#include"constants.h"
#include"publicfunpathplan.h"
#include "hybrid_rs_clothoid_curve.h"
#include "collisiondetection.h"
#include "Bspline.h"
#include "Knots.h"
#include"parallelslotpathplanning.h"
#include"smoother.h"
#include"measurestatusintracks.h"
#include"lmf.h"
#include "reeds_shepp.h"

#undef DEGUG_TIME

#define MIN_NOT_OPTIMIZED_POINT_NUM (4)
#define TOTAL_STEPS (5)
#define MIN_X_DIFFER_GOAL (0.05f)
#define MIN_YAW_DIFFER_GOAL (2*PI/180)

#define LONG_TRAJ_POINT_NUM (5)
#define STEEING_ANGLE_LIMIT_OBLI (50)

#define Y_GET_SLOT_INFOR_PRAL (0)
#define UPDATE_X_LIMIT (0.1f)
#define UPDATE_Y_LIMIT (0.2f)
#define UPDATE_YAW_LIMIT (1.5*PI/180)
#define UPDATE_LENGTH_LIMIT (0.06f)

#define UPDATE_X_LIMIT2 (0.15f)
#define UPDATE_Y_LIMIT2 (0.3f)
#define UPDATE_YAW_LIMIT2 (2.5*PI/180)
#define UPDATE_LENGTH_LIMIT2 (0.1f)
#define UPDATE_SWA_LIMIT2 (50)
#define MAX_OBLIQUE_LAST_DRIVING_VELOCITY (0.3f)

#define MAX_SEARCH_TIMES_PARA (4) //yangyi
#define MAX_SEARCH_TIMES_VERTICAL (30)
#define MAX_PARKING_STEP    (16)

#define PARA_MIN_SLOT_YAW (1*PI/180)

#define COLLISION_CHECK_POINT_NUM (1)

#define MIN_STOP_YAW_ERROR (1.5 * PI/180)
#define MIN_STOP_DIS_ERROR (0.15f)

#define PARA_STOP_YAW_ERROR (2*PI/180)
#define OBLI_STOP_YAW_ERROR (2*PI/180)
#define PARA_STOP_DIS_ERROR (0.25f)
#ifdef CHERY_T1D_RUIHU
#define PARA_STOP_X_ERROR (0.15f)
#else
#define PARA_STOP_X_ERROR (0.25f)
#endif
#define PARA_STOP_Y_ERROR (0.5f)
#define OBLI_STOP_DIS_ERROR (0.4f)
#define OBLI_STOP_X_ERROR (0.1f)
#define OBLI_STOP_Y_ERROR (0.4f)

#define LAT_DIS_TO_SLOT_HEAD (2)
#define LONG_DIS_TO_SLOT_HEAD (-1)

#define LAT_DIS_TO_SLOT_TAIL (2)
#define LONG_DIS_TO_SLOT_TAIL (0)

#ifdef CHERY_EQ
#define OBLI_TARGET_MOVE_DIS (-0.4f)
#elif defined CHERY_T1D
#define OBLI_TARGET_MOVE_DIS (0.2f)
#else
#define OBLI_TARGET_MOVE_DIS (-0.2f)//stop Y pos yangyi
#endif

#define USS_MIN_MARGIN (0.1f)
#define USS_DETECT_MARGIN_RATIO (0.6f)

#define VISION_MIN_MARGIN (0.1f)
#define SINGLE_SIDE_SLOT_DIS (0.6f)
#define PARA_SINGLE_SIDE_SLOT_DIS (1)
#define PARA_INSIDE_MIN_DIS    (0.2f)

#define NEW_PLANNING_START_DIS_ERROR (0.3f)
#define NEW_PLANNING_START_YAW_ERROR (3 * PI / 180)
#define NEW_PLANNING_POINT_NUM_ERROR (20)
#define NEW_PLANNING_END_DIS_ERROR (0.05f)
#define NEW_PLANNING_END_YAW_ERROR (0.5 * PI / 180)
#define VEH_MOVE_DIS (0.1f)

#define OBLI_ADJUST_YAW_LIMIT_IN_SLOT (5 * PI / 180) /*c-c-s-曲线连接目标点时，车辆在库内扭动的角度限制，为了不碰柱子*/

#define PLANNING_SAFEY_DIS (0.2f)

#define DIS_MIRROR_ABOVE_SLOT (0)
#define LAST_CURVE_SAFE_POINT_ERROR (0.3f)

#define OBLIQUE_HEAD_OUT_TAR_X  (3.f)
#define OBLIQUE_HEAD_OUT_TAR_Y  (2.f)
namespace HybridAStarPart
{

struct NodeDetect
{
    bool hasChecked;
    bool cFree;
    NodeDetect():hasChecked(false),cFree(false){}
};

enum ParkingPlanningMode
{
    HybridAStarMode = 0, /*hybridA*规划*/
    ParaParkingInMode, /*平行泊入规划*/
    ParaParkingOutMode, /*平行泊出规划*/
    VerticalRemoteInOut
};

typedef Node3D* pNode3D;
typedef std::vector<std::vector<Node3D> > PARTITION_PATH;


class HybridAStar {
public:

    HybridAStar()
    {
        Init();
    }

    void Init(void);

    /**
     * @brief hybridAStar路径规划算法主函数
     * @param start 起点
     * @param goal 终点
     * @param nodes3D 3D节点
     * @param width 3D网格宽度
     * @param height 3D网格高度
     * @param configurationSpace 用于碰撞检测的配置空间
     * @return 终点的3D节点
     */
    bool hybridAStar(Node3D& start,
                        Node3D& goal,
                        Node3D* nodes3D,
                        CollisionDetection& configurationSpace, vector<PARTITION_PATH> &reservedPath);

    /**
     * @brief 计算到目标的行驶代价
     * @param start 起点
     * @param goal 终点
     */
    void updateH(Node3D& start, const Node3D& goal);

    /**
     * @brief 根据扩展树索引号，由当前节点生成下一个节点
     * @param CurrentNode 当前节点
     * @param nNextNodeIndex 生成下一个节点扩展树索引号
     * @return 节点
     */
    Node3D* NextNodeGenerator(Node3D* CurrentNode,
                              int nNextNodeIndex);

    bool NextNodeIsValid(Node3D* NextNode, slotPosInfor &slotPosInfor_);


    /**
    * @brief 不合理的转向判断：
    * @param NextNode 下一个节点
    * @return 1-合理 0-不合理
    */
    bool IllogicalSteeringCheck(Node3D *NextNode);

    /**
     * @brief 计算转向惩罚
     * @param 下一个节点
     * @return 惩罚值
     */
    float CalTrajSteeringPenalty(Node3D *NextNode);

    /**
     * @brief HybridAStar::CalObstacleDisCost
     * @param fDis 障碍物距离
     * @return 障碍物代价
     */
    float CalObstacleDisCost(float fDis);
    /**
     * @brief 总代价：行驶 + 转向 + 转向偏离 + 换挡 + 不合理转向惩罚
     * @param CurrentNode 当前节点
     * @param NextNode 下一个节点
     * @return 总代价
     */
    float TrajCost(Node3D* CurrentNode, Node3D* NextNode);

    /**
     * @brief RS曲线碰撞检测
     * @param configurationSpace 碰撞检测配置空间
     * @param points 输入点
     * @return 1-可行驶; 0-不可行驶
     */
    bool ReedsSheepsCollisionCheck(CollisionDetection &configurationSpace,
                                   const std::vector<Node3D> &points);

    /**
     * @brief 计算车头泊入需要的转弯半径
     * @param tR 后角点
     * @param tF 前角点
     * @param slotCenterLineEq 中线方程
     * @param fSafeDis 安全距离
     * @return fHeadInR 转弯半径
     */
    float CalHeadInParkingRadius(const cpoint &tR,
                                 const cpoint &tF,
                                 const LineEq &slotCenterLineEq,
                                 float fSafeDis);
    /**
     * @brief 变半径RS曲线
     * @param ShottingPoint 起点
     * @param goal 终点
     * @param configurationSpace 碰撞检测配置空间
     * @param slotPosInfor_ 车位
     * @param bReverse 1-可换向 0-不可换向
     * @param nVarRadiusNum 可变半径单方向扩展数目
     * @param nVarRadiusDir 可变半径方向数目
     * @return 1-可行驶; 0-不可行驶
     */
    bool ConnectByVariableRSRadius(const Node3D& ShottingPoint,
                                   const Node3D& goal,
                                   CollisionDetection& configurationSpace,
                                   slotPosInfor &slotPosInfor_,
                                   bool bReverse,
                                   int nVarRadiusNum,
                                   int nVarRadiusDir, PARTITION_PATH &TotalSegs);


    /**
      * @brief RS曲线连接的有效性和是否可行驶
      * @param ShottingPoint 起点
      * @param goal 终点
      * @param configurationSpace 碰撞检测配置空间
      * @param bShotValid 1-连接成功; 0-连接失败
      * @return 终点节点指针
      */
    bool ReedsSheepsShot(const Node3D& ShottingPoint,
                         const Node3D& goal,
                         CollisionDetection& configurationSpace,
                         vector<PARTITION_PATH> &reservedPath);


    /**
     * @brief 判断HyBridASta曲线是否满足要求
     * @param configurationSpace 碰撞检测配置空间
     * @param slotPosInfor_ 车位信息
     * @param TotalSegs  追溯的整条分段轨迹
     * @return bShotValid 1-连接成功; 0-连接失败
     */
    bool JudgeValidityOfHyBridAStarPath(const slotPosInfor &slotPosInfor_,
                                        const PARTITION_PATH &TotalSegs,
                                        float R);

    /**
     * @brief 判断垂直车位HyBridASta曲线是否满足要求
     * @param configurationSpace 碰撞检测配置空间
     * @param slotPosInfor_ 车位信息
     * @param TotalSegs  追溯的整条分段轨迹
     * @return 1-连接成功; 0-连接失败
     */
    bool JudgeObliqueSlotValidityOfHyBridAStarPath(const slotPosInfor &slotPosInfor_,
                                                   const PARTITION_PATH &TotalSegs,
                                                   float R);
    /**
     * @brief 由于轨迹中存在直行的小段轨迹被截断
     * @param TotalSegs 追溯的整条分段轨迹
     * @return 0-连接成功; 1-连接失败
     */
    bool ShootingFailForUnexpectedLineTraj(const PARTITION_PATH &TotalSegs);
    /**
     * @brief 由于不合理的连接转弯半径截断
     * @param TotalSegs 追溯的整条分段轨迹
     * @param R Rs连接转弯半径
     * @return 0-连接成功; 1-连接失败
     */
    bool ShootingFailForUnexpectedRSradius(const PARTITION_PATH &TotalSegs, float R);

    /**
     * @brief 因为rs曲线上一段轨迹为短轨迹，截断连接
     * @param start RS起点
     * @param points  reedsSheep曲线离散点
     * @return 1-连接失败; 0-连接成功
     */
    bool ShootingFailForUnexpectedShortLengthPath(const Node3D &start,
                                            const std::vector<Node3D> &points);


    /**
     * @brief 因为最后三段曲线车辆偏航角改变太少，截断连接
     * @param TotalSegs 追溯的整条分段轨迹
     * @return 1-连接失败; 0-连接成功
     */
    bool ShootingFailForUnexpectedSmallTrajChangedAngle(const PARTITION_PATH &TotalSegs);

    /**
     * @brief 因为后视镜入库时位置不安全，截断连接
     * @param slotPosInfor_ 车位信息
     * @param TotalSegs 追溯的整条分段轨迹
     * @return 1-连接失败,0-连接成功
     */
    bool ShootingFailForBadSafetyofRearMirror(const slotPosInfor &slotPosInfor_,
                                              const PARTITION_PATH &TotalSegs);

    /**
     * @brief 判断是否需要进去后视镜安全检测逻辑
     * @param slotPosInfor_ 车位信息
     * @param TotalSegs 追溯的整条分段轨迹
     * @return 1-需要 0-不需要
     */
    bool JudgeIfNeedCheckBadSafetyofRearMirror(const slotPosInfor &slotPosInfor_,
                                               const PARTITION_PATH &TotalSegs);

    /**
     * @brief 根据Rs曲线连接结果追溯整条hybrid A*轨迹并进行分段
     * @param ShottingPoint RS起点
     * @param points reedsSheep曲线离散点
     * @param TotalSegs 追溯的整条分段轨迹
     */
    void TracePathByRSAndShotPoint(const Node3D &ShottingPoint,
                                   const std::vector<Node3D> &points,
                                   PARTITION_PATH &TotalSegs);

    /**
     * @brief 使含有RS曲线片段的路径点间隔均匀,计算补全RS路径最后一个均匀点多产生的点数
     * @param path 原始路径
     * @param pathInterpolated 均匀路径
     * @return nUniform 补全RS路径最后一个均匀点多产生的点数
     */
    void MakingPathUniform(const std::vector<Node3D>& path,
                          std::vector<Node3D>& pathInterpolated,
                          int &nStartNum,
                          int &nEndNum);
    bool IsFixSteeringPath(const std::vector<Node3D> &path);
    /**
     * @brief 路径后处理:每一片段的路径优化、生成控制需要的控制点
     * @param PartitionedResults 轨迹片段
     * @param partitioned_target_curvature_sets 分段控制点,包含坐标曲率等参数
     * @return 0-优化失败，1-优化成功
     */
    void OriginalHybridAStarTrajPostProcess(const std::vector<std::vector<Node3D> >& PartitionedResults,
                                            std::vector<std::vector<CtrlPoint> >& partitioned_target_curvature_sets);


    /**
     * @brief b样条差值
     * @param 原始优化路径
     * @param 插值步长
     * @param 插值结果
     */
    void BslineInterpolate(const std::vector<Node3D> &path,
                           float bSlineStepSize,
                           std::vector<CtrlPoint>& InterpolatedPath);

    /**
     * @brief 过滤出最优路径
     * @param configurationSpace 配置空间
     * @param reservedPath 搜索的成功路径
     * @param OptimalPath 最优路径
     */
    void filteringOptimalPath(CollisionDetection &configurationSpace,
                              std::vector<PARTITION_PATH> &reservedPath,
                              PARTITION_PATH &OptimalPath);

    /**
     * @brief 计算路径在Y方向的占据范围值
     * @param path路径
     * @return Y方向的占据范围值
     * @note 泊车优先选取Y方向占据最小空间的轨迹，这样可以尽可能减少过道使用
     */
    float CalYRangeOfPath(const PARTITION_PATH &path);
    float CalReservedPathTotalCost(CollisionDetection& configurationSpace,
                                  PARTITION_PATH &reservedPath);

    /**
     * @brief hybrid A Star路径搜索+路径平滑
     * @param start 起点
     * @param goal 终点
     * @param nodes3D node 3D节点
     * @param width 3D网格宽度
     * @param height 3D网格高度
     * @param configurationSpace 配置空间
     * @param HybridAResults hybrid A Star路径搜索结果
     * @param partitioned_target_curvature_sets 分段路径平滑结果
     * @return 1-搜索成功 0-搜索失败
     */
    bool plan(Node3D& start,
              Node3D& goal,
              Node3D* nodes3D,
              CollisionDetection& configurationSpace,
              std::vector<Node3D>& HybridAResults,
              std::vector<std::vector<CtrlPoint> > &partitioned_target_curvature_sets);

    /**
     * @brief 初始化地图和输入信息
     * @param map 地图
     * @param InputParkingIn_ 输入信息
     * @return 1-初始化成功，0-初始化失败
     */
    bool InitMapInforAndVehicleInfor(unsigned char *map,
                                     InputParkingIn &InputParkingIn_);

    float getRoadWidth(const slotPosInfor &slotPosInfor_,
                       CollisionDetection& configurationSpace);
    /**
     * @brief 设置可变网格区域
     * @param slotPosInfor_ 车位信息
     * @param varialeCellBound 可变网格区域
     */
    void SetVarialeCell(const slotPosInfor &slotPosInfor_,
                        VarialeCellBoundary &varialeCellBound);

    /**
     * @brief 判断给出的车位的角点是否在车位两条平行线的外侧
     * @param slotPosInfor_ 车位信息
     * @return
     */
    bool SlotCornerCheck(slotPosInfor &slotPosInfor_);
    /**
     * @brief 输入信息检测
     * @param[in] slotPosInfor 车位信息
     * @return bValidity 0-输入信息不合理 1-输入信息合理
     */
    bool SlotPosInforInputsCheck(slotPosInfor &slotPosInfor_);

    /**
     * @brief 计算垂直车位或者斜列车位的车辆入库必须摆正的坐标点
     * @param slotPosInfor_ 车位信息
     */
    void SetObliSlotSafePoint(slotPosInfor &slotPosInfor_);

    /**
     * @brief 设置垂直车位的终点
     * @param slotPosInfor 车位输入信息
     * @return 1-设置成功 0-设置失败
     */
    bool SetObliSlotGoalPoint(slotPosInfor &slotPosInfor_);

    /**
     * @brief 设置平行车位的终点
     * @param slotPosInfor 车位输入信息
     * @return 1-设置成功 0-设置失败
     */
    bool SetParaSlotGoalPoint(slotPosInfor &slotPosInfor_);

    /**
     * @brief 设置起点和目标点
     * @param slotPosInfor 车位输入信息
     * @return 1-设置成功 0-设置失败
     */
    bool SetStartPointAndGoalPoint(slotPosInfor &slotPosInfor_);

    /**
     * @brief hybrid A*规划的主函数，负责根据初始点和目标点规划一条满足车辆动力学约束、和障碍物无碰撞、曲率连续、速度连续的路径
     * @param[in] hybridAResults hybrid A*的搜索结果
     * @param[in] partitionedCtrlPointSets 对hybrid A*根据行驶方向进行片段分割，优化、插值后的控制点集合
     * @return nPlanningFlag 0-规划结束 1-规划成功 2 -规划失败
     * @note 不包含平行泊车库内调整轨迹搜索
     */
    PlanningStatus PathPlanning(const ulong nTotalGearSwitchTimes,
                      std::vector<Node3D>& hybridAResults,
                      std::vector<std::vector<CtrlPoint> >& partitionedCtrlPointSets);


    /**
     * @brief 给轨迹点赋测量状态
     * @param[in] aTracks 轨迹点
     * @return 0
     * @note
     */
    int AddMeasureStatusInTargetTrack(std::vector<TargetTrack>& aTracks);

    ParaSlotParameters CalParaFrontAndRearMargin(const slotPosInfor &slotPosInfor_);

    /**
     * @brief 平行泊车库内轨迹规划入口
     * @param nDrivingDirection 下一次车辆行驶方向
     * @param startPointInSlot 起点
     * @param goalPointInSlot 终点
     * @param mPartitionedTargetCurvatureSets 规划路径
     * @return nPlanningFlag 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus ParkingInPathPlanningInParaSlot(int8_t nDrivingDirection,
                                                   LocationPoint startPointInSlot,
                                                   LocationPoint goalPointInSlot,
                                                   std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets);

    /**
     * @brief 平行泊车出库规划入口
     * @param nDrivingDirection 下一次车辆行驶方向
     * @param startPointInSlot 起点
     * @param PartitionedTargetCurvatureSets 规划路径
     * @return nPlanningFlag 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus ParkingOutPathPlanningInParaSlot(int8_t nLastDrivingDirection,
                                                    LocationPoint startPointInSlot,
                                                    std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets);

    PlanningStatus ParkingOutPathPlanningInVertSlot(int8_t nLastDrivingDirection,
                                                    LocationPoint startPointInSlot,
                                                    std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets);
    
    PlanningStatus RemoteInOutParkingPathPlanning(int8_t nDrivingDirection,
                                                  LocationPoint startPointInSlot,
                                                  std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets);
    /**
     * @brief 已知坐标系B相对于坐标系A的旋转坐标，将B坐标系中的轨迹转换成A中的轨迹
     * @param RotationCoordinate 坐标系B相对于坐标系A的旋转坐标
     * @param aTracks B坐标系中的轨迹转换成A中的轨迹
     */
    void GetFinalTracks(LocationPoint RotationCoordinate,
                          const std::vector<std::vector<CtrlPoint> >& Segments,
                          std::vector<TargetTrack> &aTracks);



    /**
     * @brief 已知坐标系B相对于坐标系A的旋转坐标，将B坐标系中的轨迹转换成A中的轨迹
     * @param RotationCoordinate 坐标系B相对于坐标系A的旋转坐标
     * @param HybridAResults B坐标系中的轨迹转换成A中的轨迹
     */
    void RotateHybridAResults(LocationPoint RotationCoordinate, std::vector<Node3D>& HybridAResults);

    /**
     * @brief 已知坐标系B相对于坐标系A的旋转坐标，将B坐标系中的轨迹转换成A中的轨迹
     * @param RotationCoordinate 坐标系B相对于坐标系A的旋转坐标
     * @param SegmentSets B坐标系中的轨迹转换成A中的轨迹
     */
    void RotateSegmentSets(LocationPoint RotationCoordinate, std::vector<std::vector<CtrlPoint> >& SegmentSets);


    /**
     * @brief 判断是否能启动平行车库内的轨迹规划
     * @param nLastPlanningPathSegNum 上次规划的hybrid A轨迹段数
     * @param slotPosInfor_ 车位信息
     * @return 1-启动平行库内的轨迹规划，0-不启动
     */
    bool CheckIfStartPathPlanningInParaSlot(bool bParaHybridALastDriving,
                                            const slotPosInfor &slotPosInfor_);

    bool JudgeReachFixedGoalPoint(const slotPosInfor &slotPosInfor_);
    /**
     * @brief 判断车辆当前点坐标是否满足到达目标点的标准
     * @param slotType 车位类型
     * @param startPointInSlot 车辆当前点坐标
     * @param goalPointInSlot 目标点的坐标
     * @return 1-满足 0-不满足
     */
    bool JudgeVehReachGoalPoint(const slotPosInfor &slotPosInfor_);

    LocationPoint GetGoalPointInslot(void){ return m_slotPosInfor.goalPointInSlot;}

    PlanningStatus PathPlanningByMode(ParkingPlanningMode PlanningMode);

    /**
     * @brief 判断泊车规划模式
     * @return   0-hybridA*规划, 1-平行泊入规划, 2-平行泊出规划
     */
    ParkingPlanningMode JudgeParkingPlanningMode(void);

    /**
     * @brief 路径规划后的后处理
     * @param aTracks 发给控制模块的轨迹点
     */
    void PostProcessingTaskAfterPlanningSuccess(std::vector<TargetTrack>& aTracks);

    /**
     * @brief 路径的实时规划
     * @param InputParkingIn_
     * @param aTracks 控制点
     * @return 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus PathPlanningRealTimeState(InputParkingIn &InputParkingIn_,
                                     std::vector<TargetTrack>& aTracks);
    unsigned char* GetMap(){return occupyMap;}

    uint16_t getCurrentStep(){return m_nCurrentStep;}

    uint16_t getTotalStep(){return m_nTotalStep;}

    float getRemainedPathDis(){return m_fResidualDis;}

    std::vector<Node3D> GetHybridAResults(){return m_HybridAResults;}

    std::vector<std::vector<CtrlPoint> > GetSegmentResults(){return m_PartitionedTargetCurvatureSets;}

    VarialeCellBoundary getVarialeCellBound() {return m_varialeCellBound;}
//    VehPos ConvertNode3D2VehPos(Node3D &node)
//    {
//        VehPos tempPoint;
//        tempPoint.x = node.getX();
//        tempPoint.y = node.getY();
//        tempPoint.yaw = node.getT();
//        tempPoint.DrivingDirection = node.GetDirec();
//        tempPoint.steering = node.GetSteer();
//        if(fabsf(tempPoint.steering) < 1e-3f)
//        {
//            tempPoint.SteeringDirection = 0;
//        }
//        else if(tempPoint.steering > 0)
//        {
//            tempPoint.SteeringDirection = STEERING_LEFT;
//        }
//        else
//        {
//            tempPoint.SteeringDirection = STEERING_RIGHT;
//        }
//        tempPoint.isStart = node.isStartPoint();
//        return tempPoint;
//    }

//    Node3D ConvertVehPos2Node3D(VehPos &point)
//    {
//        Node3D node;
//        node.setX(point.x);
//        node.setY(point.y);
//        node.setT(point.yaw);
//        node.SetDirec(point.DrivingDirection);
//        node.setStartPoint(point.isStart);
//        node.SetSteer(point.steering);
//        return node;
//    }
    /**
     * @brief 纯搜索轨迹生成
     * @param node
     * @param TotalSegs
     */
    void TraceSearchPath(const Node3D& node, vector<PARTITION_PATH> &TotalSegs);
    /**
     * @brief 判断是否开始泊出搜索规划
     * @param slotPosInfor_
     * @return
     */
    bool ifStartSearchPlanningParaOut(const slotPosInfor &slotPosInfor);

    bool CheckParkoutSearch(const Node3D* node);

    _planErrCode GetPlanErrCode(void);
private:
    _planErrCode planErrCode;

    struct cmp
    {
        bool operator()(const pNode3D& left, const pNode3D& right) const
        {
            return left->GetCost() >= right->GetCost();
        }
    };

    std::priority_queue<Node3D*, std::vector<Node3D*>, cmp> open_pq;
    std::priority_queue<Node3D*, std::vector<Node3D*>, cmp> close_pq;

    float m_OriginSlotYaw;

    bool m_bLastDriving;
    bool m_ParaHybridALastDriving;

    bool m_bStartPathplanningInParaSLot;
    bool m_bFirstPlan;

    bool m_bUSSHasDetectFrontMargin;
    bool m_bUSSHasDetectRearMargin;

    int8_t m_nDrivingDirection;
    ulong m_nFirstPlanTotalSteps;
    ulong m_TotalGearSwitchTimes;
    ulong mParaSlotSteps;

    slotPosInfor m_slotPosInfor;

    ParkingWorkingStatus m_ParkingCtrlStatus;

    unsigned char occupyMap[Constants::ob_grid_num];

    Node3D* m_pReedsSheepsNodes;

    ParallelSlotPathPlanning ParallelSlotPathPlanning_;
//    HybridAStarPart::Smoother smoother;
    LMF_Optimizer lmfOpter;

    LocationPoint m_RotationCoordinate;

    std::vector<Node3D> m_HybridAResults;
    std::vector<std::vector<CtrlPoint> > m_PartitionedTargetCurvatureSets;
    ulong m_nCurrentStep;
    ulong m_nTotalStep;
    float m_fMovingDisF;
    float m_fMovingDisR;
    float m_deltaRearY;
    float m_deltaRearX;
    float m_deltaFrontY;
    float m_deltaFrontX;
    float m_fHeadInRsRadius;
    float m_fResidualDis;
    float m_fYawOffsetOfOpti;
    bool m_bBestSafeObliParkingInPath;
    bool m_bFindPath;
    ulong m_nAdditionalSearchTimes;
    int m_nCollisonCheckTimes;
    CollisionDetection m_configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞

    VarialeCellBoundary m_varialeCellBound;

    LocationPoint m_lastPlanningPoint;
    bool m_bHitWheelBar;
};
}
#endif // ALGORITHM_H
