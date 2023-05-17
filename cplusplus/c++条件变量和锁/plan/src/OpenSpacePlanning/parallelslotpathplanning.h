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
* @file parallelslotpathplanning.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 平行泊车库内轨迹规划算法,负责计算平行泊车出库点和车库内的轨迹规划
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/

#ifndef PARALLELSLOTPATHPLANNING_H
#define PARALLELSLOTPATHPLANNING_H

#include<vector>
#include"vehicle.h"
#include"base.h"
#include"math_formula.h"
//#include"helper.h"
#include "publicfunpathplan.h"
#include "collisiondetection.h"
#include "constants.h"
#include "clothoid_parkout.h"


/*INT*/
#define MAX_BACK_FORWARD_TIMES (10)            //车位内的最大调整次数
#define MAX_STEP3_YAW (80*PI/180)
#define DELTA_SWA (10)
#define MIN_STEERING_ANGLE_IN_CLOTHOID (100)



/*FLOAT*/
#define MIN_VEH_SIDE_TO_RIGHT_MARGIN (0.3f)
#define MAX_VEH_SIDE_TO_RIGHT_MARGIN (0.4f)
#define MARGIN_C (0.20f)                         //车辆和车位C点最小距离
#define MIN_DRIVING_DIS (0.16f)                 //车辆支持的最小的行车距离
#define CLOTHOID_CURVATURE_FACTOR (0.4f)   /*回旋线曲率变换系数*/
#define ABS_MIN_STOP_YAW (0.5f*PI/180)
#define AHEAD_YAW (0*PI/180)
#define MIN_ALLOW_YAW (-10*PI/180)
#define STEP_SIZE_SEARCH (0.05f)
#define MARGIN_INCREMENT (0.3f)
#define TARGET_X_ERROR (0.1f)
#define MIN_DIS_ONE_TIME (0.1f)
#define Y_ERROR_INCREMENT (0.05f)
#define Y_DOWN_ERROR (-0.15f)
#define Y_UPPER_ERROR (0.25f)
#define TARGET_Y_ERROR (0.1f)
#define BEST_XH (0.5f)
#define MIN_XH (0.0f) /*一次调整可以出库时的车辆出库点H点最小坐标*/
#define MAX_LINE_LENGTH (0.3f)

#define MIN_LAST_PARKING_OUT_LENGTH (0.3f)

#define MAX_YC (0.2f)

#define PARA_PLANNING_RADIUS_ERROR (0.4)
namespace HybridAStarPart {

struct ParaSlotParameters
{
    float fXParaRearMargin;
    float fXParaFrontMargin;
    float fYParaRightMargin;/*翻转坐标系后的内侧车位边界*/
    float fParaTargetY;
    ParaSlotParameters():fXParaRearMargin(0.0), fXParaFrontMargin(0.0),
        fYParaRightMargin(0.0), fParaTargetY(0.0){}
};


struct Step3KeyPointInfor
{
    float startX;
    float fXH;
    int8_t nInitialDrivingDirection;
    ulong nAdjustTimes;
    LocationPoint point;
    Step3KeyPointInfor():startX(0.0),fXH(0.0),nInitialDrivingDirection(0),nAdjustTimes(0){}
};

struct SlotEdgInfor
{
    float fFrontEdg;
    float fRearEdg;
    ulong nAdjustTimes;
    float Y;
    float fPlanningR;
    LocationPoint tPoint;
    std::vector<float> NextDrivingDisSets;
    SlotEdgInfor():fFrontEdg(0), fRearEdg(0), nAdjustTimes(0), Y(0){}
};

enum ParallSlotDrivingMode
{
    InitConditionJudgment,
    SteeringZero,
    LeftBackDriving,
    LeftForwardDriving,
    RightForwardDriving,
    RightBackDriving
};


class ParallelSlotPathPlanning
{
public:
    ParallelSlotPathPlanning()
    {
        Init();
    }
    void Init();
    void InitMapAndSlotInfor(unsigned char *map, const slotPosInfor &slotPosInfor_);
    /**
     * @brief 计算左边界余量和更新车位宽度
     * @param fSlotWidth 车位宽度
     * @param LeftMargin 左边界余量
     * @param NewSlotWidth 更新车位宽度
     */
    void CalLeftMarginAndSlotWidth(float fSlotLength, float fSlotWidth,
                                   float &LeftMargin,
                                   float &NewSlotWidth);

    /**
     * @brief 根据行车状态，计算旋转角度和更新车辆位置
     * @param DrivingMode 行车状态
     * @param tPoint 当前坐标
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @return 旋转角度
     */
    float CalRotationAngleInParaSlot(ParallSlotDrivingMode DrivingMode,
                                     LocationPoint &tPoint, float fRadius,
                                     float fXRearMargin,
                                     float fXFrontEdge,
                                     float fXRightEdge);

    /**
     * @brief 计算右转向前旋转角度和更新车辆位置
     * @param tPoint 当前坐标
     * @param fXFrontMargin 前边界
     * @return 旋转角度
     */
    float CalRightFowardRotationAngleInParaSlot(LocationPoint &tPoint,
                                                float fXFrontMargin,
                                                float fRadius);

    /**
     * @brief 计算右转向后旋转角度和更新车辆位置
     * @param tPoint 当前坐标
     * @param fXRearMargin 后边界
     * @param fYRightMargin 右边界
     * @return 旋转角度
     */
    float CalRightBackRotationAngleInParaSlot(LocationPoint &tPoint,
                                              float fXRearMargin,
                                              float fYRightMargin, float fRadius);

    /**
     * @brief  计算左转向前旋转角度和更新车辆位置
     * @param tPoint 当前坐标
     * @param fXFrontMargin 前边界
     * @return 旋转角度
     */
    float CalLeftForwardRotationAngleInParaSlot(LocationPoint &tPoint,
                                                float fXFrontMargin,
                                                float fRadius);

    /**
     * @brief 计算左转向后旋转角度和更新车辆位置
     * @param tPoint 当前坐标
     * @param fXRearMargin 后边界
     * @param fYRightMargin 右边界
     * @return 旋转角度
     */
    float CalLeftBackRotationAngleInParaSlot(LocationPoint &tPoint,
                                             float fXRearMargin,
                                             float fYRightMargin, float fRadius);
    /**
     * @brief 计算step3入库点
     * @param fXA 平行车位A点X坐标
     * @param fYA 平行车位A点Y坐标
     * @param fXC 平行车位C点X坐标
     * @param fYC 平行车位C点X坐标
     * @param fTargetX 目标停车点X
     * @param fTargetY 目标停车点Y
     * @param bSlotSizeSuccess 1-车位尺寸调整成功 0-车位尺寸调整失败
     * @param tStep3KeyPoint  车辆出库不碰C点的位姿
     * @param TotalSteps 调整次数
     */
    void CalStep3KeyPoint(float fXA,
                          float fYA,
                          float fXC,
                          float fYC, float fTargetX,
                          float fTargetY,
                          bool &bSlotSizeSuccess,
                          LocationPoint &tStep3KeyPoint,
                          ulong &TotalSteps);

    /**
     * @brief 计算车辆F点出车库不碰到车位C点的最小车位C点坐标
     * @param nSlotSide 车位的方向,1-右侧车位 -1-左侧车位
     * @param fXA 平行车位A点X坐标
     * @param fYC 平行车位C点Y坐标
     * @param fTargetY 目标停车点Y
     * @param fXCMin c点最小X坐标
     */
    void CalMinSlotLengthOneTrialParkingIn(float fXA,
                                           float fYC,
                                           float fTargetY,
                                           float &fXCMin);

    /**
     * @brief 计算不同车辆停车位置的调整次数最少的较好的出库点
     * @param fXA 平行车位A点X坐标
     * @param fYA 平行车位A点Y坐标
     * @param fXC 平行车位C点X坐标
     * @param fYC 平行车位C点X坐标
     * @param fTargetX 目标停车点X
     * @param fTargetY 目标停车点Y
     * @param bSlotSizeSuccess 1-车位尺寸调整成功 0-车位尺寸调整失败
     * @param tStep3KeyPoint 车辆出库不碰C点的位姿
     * @param TotalSteps 调整次数
     */
    void CalBestParkingOutPosParkingIn(float fXA,
                                       float fYA,
                                       float fXC,
                                       float fYC, float fTargetX,
                                       float fTargetY,
                                       bool &bSlotSizeSuccess,
                                       LocationPoint &tStep3KeyPoint, ulong &TotalSteps);
#if 0
    void CalBestParkingOutPosParkingIn2(float fXA,
                                        float fYA,
                                        float fXC,
                                        float fYC,
                                        float fTargetX,
                                        float fTargetY,
                                        bool &bSlotSizeSuccess,
                                        LocationPoint &tStep3KeyPoint,
                                        ulong& TotalSteps);
#endif
    /**
     * @brief 计算车辆固定停车位置的出库点
     * @param nInitialDrivingDirection 初始车辆行驶方向
     * @param fXA 平行车位A点X坐标
     * @param fYA 平行车位A点Y坐标
     * @param fXC 平行车位C点X坐标
     * @param fYC 平行车位C点X坐标
     * @param tPoint 车辆在车位里的初始坐标
     * @param bBackForwardAdjustSucesss 调整成功
     * @param nBackForwardTimes 调整次数
     * @param tOutputStep3KeyPoint 车辆出库不碰C点的位姿
     * @param DrivingDisSet 库内调整的行驶轨迹距离集合
     */
    void CalParkingOutPosInfixedInitialPoint(int8_t nInitialDrivingDirection,
                                             float fXA,
                                             float fYA,
                                             float fXC, float fYC,
                                             LocationPoint tPoint, float fRadius,
                                             bool &bBackForwardAdjustSucesss,
                                             ulong &nBackForwardTimes,
                                             LocationPoint &tOutputStep3KeyPoint, std::vector<float> &DrivingDisSet);

    /**
     * @brief 出库检查
     * @param fRadius 出库转弯半径
     * @param fXC C点X坐标
     * @param fYC C点Y坐标
     * @param tStep3KeyPoint 车辆出库位姿
     * @return 0-可以出库 1-不能出库
     */
    bool ParkingOutCheck(float fRadius, float fXA,
                         float fYA,
                         float fXC,
                         float fYC,
                         LocationPoint tStep3KeyPoint);

    /**
     * @brief 用于计算车辆入库后的虚拟前边界/虚拟后边界,使得目标停车点尽量靠近目标停车点
     * @param nSlotPosition 1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆的坐标
     * @param fVehTargety 目标停车点Y
     * @param nInitDrivingDirection 下一次行驶方向
     * @param XFrontMarginNeed 计算的虚拟前边界
     * @param XRearMarginNeed 计算的虚拟后边界
     * @param NextDrivingDisSets 将要行驶的距离
     * @param fVehStopy 预测的最佳停车点Y
     * @return 0-调整失败 1-调整成功
     */
    bool NewPlanningInSlot(float fXRearMargin,
                           float fXFrontMargin,
                           float fYRightMargin,
                           LocationPoint tPoint, float VehTargetX,
                           float fVehTargety,
                           int8_t nInitDrivingDirection, float &fCalRadius,
                           float &XFrontMarginNeed,
                           float &XRearMarginNeed,
                           std::vector<float> &NextDrivingDisSets,
                           float &fVehStopy);

    /**
     * @brief 在已知车辆当前位置和下一次行驶方向，方向盘转角和接下来一次的方向盘转角度数下，预测车辆最终的停车位置。
     * @param nSlotPosition 1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆的坐标
     * @param nInitDrivingDirection 下次行驶方向
     * @param fYStop 预测的最佳停车点
     * @param nBackForwardTimes 预测的最佳停车点需要调整次数
     * @param bBackForwardAdjustSucesss  0-调整失败; 1-调整成功
     */
    void CalStopPosition(float fXRearMargin,
                         float fXFrontMargin,
                         float fYRightMargin,
                         LocationPoint tPoint, float VehTargetX, float fVehTargety,
                         float fRadius,
                         int8_t nInitDrivingDirection,
                         std::vector<float>& NextDrivingDisSets,
                         float &fYStop,
                         ulong &nBackForwardTimes,
                         bool &bBackForwardAdjustSucesss);

    /**
     * @brief 判断最后一把以最大方向盘转角能否保证车辆偏航角回正
     * @param nSlotPosition 1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆的坐标
     * @param NextDrivingDirection 下次行驶方向
     * @param fStopy 车辆偏航角回正的车辆Y坐标
     * @return 1-可以 0-不可以 最后一把以最大方向盘转角能否保证车辆偏航角回正
     */
    bool CheckIfAdjustToZero(float fXRearMargin,
                             float fXFrontMargin,
                             float fYRightMargin,
                             LocationPoint tPoint,
                             float fRadius,
                             int8_t NextDrivingDirection,
                             float &fStopy);


    /**
     * @brief 计算回旋线需要的初始曲率
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param tPoint 当前车辆位置
     * @param nDrivingDirection 行驶方向
     * @param fFindCurvature 回旋线初始曲率
     * @return 1-生成回旋线 0-无法生成回旋线
     */
    bool CalClothoidCurvature(float fXRearMargin,
                                  float fXFrontMargin,
                                  LocationPoint tPoint, float fRadius,
                                  int8_t nDrivingDirection,
                                  float &fFindCurvature);

    /**
     * @brief 计算圆弧回旋线和直线的长度
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param tPoint 当前车辆位置
     * @param fVehTargetX 目标点x
     * @param nDrivingDirection 行驶方向
     * @param fFindCurvature 回旋线需要的初始曲率
     * @param fCircleLen 圆弧的长度
     * @param fClothoidLen 回旋线的长度
     * @param fLineLen 直线的长度
     * @return 1-最后一次 0-不是最后一次
     */
    bool CalHybridCurveLength(float fXRearMargin,
                              float fXFrontMargin,
                              LocationPoint tPoint, float fVehTargetX,
                              int8_t nDrivingDirection,
                              float fFindCurvature,
                              float &fCircleLen,
                              float &fClothoidLen,
                              float &fLineLen);

    /**
     * @brief 生成回旋线
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param tPoint 当前车辆的坐标
     * @param fVehTargetX 目标停车点x
     * @param nDrivingDirection 下一次行驶方向
     * @param targetSeg 输出控制点
     * @return 1-最后一段 0-不是最后一段
     */
    bool ClothoidPathInSlot(float fXRearMargin,
                            float fXFrontMargin,
                            LocationPoint tPoint,
                            float VehTargetX,
                            int8_t nDrivingDirection, float fRadius,
                            std::vector<CtrlPoint> &targetSeg);

    /**
     * @brief 回旋线连接
     * @param bLastDriving 最后一次行驶
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param tPoint 当前车辆坐标
     * @param fFindCurvature 回旋线需要的初始曲率
     * @param nSteeringDirection 方向盘转动方向
     * @param nDrivingDirection  下一次行驶方向
     * @param fCircleLen 圆弧长度
     * @param fClothoidLen 回旋线长度 直线长度
     * @param fLineLen 直线长度
     * @param fVehTargetX 目标x
     * @param TargetSeg 输出控制点
     */
    void ClothoidPathSplicing(bool bLastDriving,
                              float fXRearMargin,
                              float fXFrontMargin,
                              LocationPoint tPoint,
                              float fFindCurvature,
                              int8_t nSteeringDirection,
                              int8_t nDrivingDirection,
                              float fCircleLen,
                              float fClothoidLen,
                              float fLineLen,
                              float fVehTargetX,
                              std::vector<CtrlPoint> &TargetSeg);
    /**
     * @brief 平行泊车库内规划主函数
     * @param nSlotPosition 1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆的坐标
     * @param VehTargetX 目标停车点x
     * @param VehTargety 目标停车点y
     * @param NextDrivingDirection 下次行驶方向
     * @param bLastDriving 1-最后一次规划 0-不是最后一次规划
     * @param TargetSegSet 轨迹结果
     * @return 1-规划成功; 0-规划失败
     * @note 平行泊车库内轨迹计算模块，采用的坐标系左侧和右侧相反
     */
    bool PathPlanningInParallelSlot(int8_t nSlotPosition,
                                    float fXRearMargin,
                                    float fXFrontMargin,
                                    float fYRightMargin,
                                    LocationPoint tPoint, float VehTargetX,
                                    float VehTargety,
                                    int8_t NextDrivingDirection,
                                    std::vector<std::vector<CtrlPoint>> &TargetSegSet);

    /**
     * @brief 平行泊车库内规划主函数
     * @param nSlotPosition 1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆的坐标
     * @param VehTargetX 目标停车点x
     * @param VehTargety 目标停车点y
     * @param NextDrivingDirection 下次行驶方向
     * @param bLastDriving 1-最后一次规划 0-不是最后一次规划
     * @param TargetSegSet 轨迹结果
     * @return 1-规划成功; 0-规划失败
     * @note 平行泊车库内轨迹计算模块，采用的坐标系左侧和右侧相反
     */
    bool PathPlanningInParallelSlotOneStep(float fXRearMargin,
                                           float fXFrontMargin,
                                           float fYRightMargin,
                                           LocationPoint tPoint,
                                           float VehTargetX,
                                           float VehTargety,
                                           int8_t NextDrivingDirection,
                                           bool &bLastDriving,
                                           std::vector<CtrlPoint> &TargetSeg);


    /**
     * @brief 判断是不是可以一次出库,以及计算一次出库的位置
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前点
     * @param OneTimeParkingOutPos 能够一次出库的位姿态
     * @return 1-可以一次出库 0-不可以一次出库
     */
    bool CalOneTimeParkingOutPos(float fXRearMargin,
                                 float fXFrontMargin,
                                 float fYRightMargin,
                                 LocationPoint tPoint, float fRadius,
                                 LocationPoint &OneTimeParkingOutPos);

    /**
     * @brief 计算不同车辆停车位置的调整次数最少的较好的出库点
     * @param nSlotPosition 车位的方向,1-右侧车位 -1-左侧车位
     * @param NextDrivingDirection 下一次行驶方向
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前点
     * @param bSlotSizeSuccess 1-车位尺寸调整成功 0-车位尺寸调整失败
     * @param DrivingSet 行驶距离集合
     */
    void CalBestParkingOutPathParkingOut(int NextDrivingDirection,
                               float fXRearMargin,
                               float fXFrontMargin,
                               float fYRightMargin,
                               LocationPoint tPoint,
                               bool &bSlotSizeSuccess,
                               std::vector <float> &DrivingSet);

    /**
     * @brief 出库检查
     * @param fRadius 出库转弯半径
     * @param fXC C点X坐标
     * @param fYC C点Y坐标
     * @param tStep3KeyPoint 车辆出库位姿
     * @return 0-可以出库 1-不能出库
     */
    bool CheckIfParkingOut(float fRadius,
                           float fXC,
                           float fYC,
                           LocationPoint tStep3KeyPoint);

    /**
     * @brief 生成出库的最后一段圆弧加回旋线路径
     * @param fXFrontMargin 前边界
     * @param tPoint 当前车辆坐标
     * @param SegmentResults 控制点集合
     * @return 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus GenerateLastPathInParkingOut(float fXFrontMargin,
                                                LocationPoint tPoint,
                                                float fRadius,
                                                std::vector<CtrlPoint> &SegmentResults);

    /**
     * @brief 第一次路径规划方向盘打死行驶路径规划
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆坐标
     * @param NextDrivingDirection 下一次行驶方向
     * @return 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus CalFirstPathDirectionParaParkingOut(float fXRearMargin,
                                                       float fXFrontMargin,
                                                       float fYRightMargin,
                                                       LocationPoint tPoint, int8_t &NextDrivingDirection);
    /**
     * @brief 根据当前坐标,方向盘转动方向,车辆行驶方向，进行圆弧 回旋线和直线连接
     * @param tPoint 当前车辆坐标
     * @param nSteeringDirection 转动方向
     * @param nDrivingDirection 行驶方向
     * @param fKappa 圆弧曲率
     * @param fCircleLen 圆弧长度
     * @param fClothoidLen 回旋线长度
     * @param fLineLen 直线长度
     * @param TargetSeg  输出控制点
     */
    void CircleClothoidLinePathSplicing(LocationPoint tPoint,
                                        int8_t nSteeringDirection,
                                        int8_t nDrivingDirection,
                                        float fKappa,
                                        float fCircleLen,
                                        float fClothoidLen,
                                        float fLineLen,
                                        std::vector<CtrlPoint> &TargetSeg);
    /**
     * @brief 平行泊出单步规划
     * @param nSlotPosition 车位的方向,1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆坐标
     * @param NextDrivingDirection 下一次行驶方向
     * @param bFirstPlan 是否第一次规划
     * @param bLastDriving 最后一步
     * @param TargetSeg 目标控制点集合
     * @return 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus ParkingOutPathPlanningOneStep(float fXRearMargin,
                                                 float fXFrontMargin,
                                                 float fYRightMargin,
                                                 LocationPoint tPoint,
                                                 int8_t &NextDrivingDirection,
                                                 bool bFirstPlan,
                                                 bool &bLastDriving,
                                                 std::vector<CtrlPoint> &TargetSeg);

    /**
     * @brief 平行泊出库内轨迹规划
     * @param nSlotPosition 车位的方向,1-右侧车位 -1-左侧车位
     * @param fXRearMargin 后边界
     * @param fXFrontMargin 前边界
     * @param fYRightMargin 右边界
     * @param tPoint 当前车辆坐标
     * @param NextDrivingDirection 下一次行驶方向
     * @param bFirstPlan 是否第一次规划
     * @param bLastDriving 最后一步
     * @param TargetSegSet 目标控制点集合
     * @return 0-规划结束 1-规划成功 2 -规划失败
     */
    PlanningStatus ParkingOutInParallelSlot(int8_t nSlotPosition,
                                            float fXRearMargin,
                                            float fXFrontMargin,
                                            float fYRightMargin,
                                            LocationPoint tPoint,
                                            int8_t NextDrivingDirection,
                                            bool bFirstPlan,
                                            std::vector<std::vector<CtrlPoint> > &TargetSegSet);

    /**
     * @brief 计算角点转弯半径
     */
    int CalCornerPointRadius2(int8_t nSteeringDirection,
                              float fRearAxleCenterRadius,
                              float &fRE1,
                              float &fRE2,
                              float &fRF1,
                              float &fRF2,
                              float &fRG,
                              float &fRH);

    /**
     * @brief 计算角点坐标
     */
    void CalCornerCoordinate2(LocationPoint tVeh,
                              LocationPoint &tE1,
                              LocationPoint &tE2,
                              LocationPoint &tF1,
                              LocationPoint &tF2,
                              LocationPoint &tG,
                              LocationPoint &tH);

    _planErrCode planErrCode;
private:
    CollisionDetection m_configurationSpace;//碰撞检测类实体，用以检测某个配置是否会发生碰撞
    unsigned char occupyMap[Constants::ob_grid_width * Constants::ob_grid_height];

    slotPosInfor m_slotPosInfor;



};
}
#endif // PARALLELSLOTPATHPLANNING_H
