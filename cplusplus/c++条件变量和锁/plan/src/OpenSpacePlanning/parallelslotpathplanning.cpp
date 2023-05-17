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
* @file parallelslotpathplanning.cpp
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 平行泊车库内轨迹规划算法,负责计算平行泊车出库点和车库内的轨迹规划
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/
#include <sys/time.h>

#include "parallelslotpathplanning.h"

/*
 *                       Left Obstacle
 * -----------------------------------------------------
 *
 *
 *
 *                            H  ************* G
 *                               *           *     Vehicle
 *          Y^                E1 *           * F1
 *           |                     *********
 *           |                     E2      F2
 *           |
 *           |
 * ***********----->X           ******************
 *         D *       SLOT       * C
 *           *                  *
 *           ********************
 *           A                   B
 *
 *
 */


namespace HybridAStarPart {

void ParallelSlotPathPlanning::Init()
{
    planErrCode=NO_ERROR;
    m_configurationSpace.Init();
}

void ParallelSlotPathPlanning::InitMapAndSlotInfor(unsigned char *map, const slotPosInfor &slotPosInfor_)
{
    float fSlotCornerSafeDis = 0.0f;
    float fSafeDisOfMirror = 0.0f;
    float fSlotAreaSafeDis = 0.0f;
    float fOutSlotSafeDis = 0.0f;
    cpoint tF, tF2, tR, tR2;
    Polygon polygon;

    Init();
    /*更新碰撞检测地图*/
    memcpy((void*)(occupyMap), (void*)(map), Constants::ob_grid_width * Constants::ob_grid_height* sizeof(unsigned char));
    /*对配置空间进行更新*/
    m_configurationSpace.updateGrid(occupyMap);
    m_slotPosInfor = slotPosInfor_;
    fOutSlotSafeDis = Constants::para_out_slot_area_dis;

    tF.x = m_slotPosInfor.fXFrontSlotCorner + m_slotPosInfor.fXSlotPosInMap;
    tF.y = m_slotPosInfor.fYFrontSlotCorner + m_slotPosInfor.fYSlotPosInMap;

    tF2.x = tF.x - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
            * cosf(m_slotPosInfor.fSlotAngle);
    tF2.y = tF.y - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
            * sinf(m_slotPosInfor.fSlotAngle);

    tR.x = m_slotPosInfor.fXRearSlotCorner + m_slotPosInfor.fXSlotPosInMap;
    tR.y = m_slotPosInfor.fYRearSlotCorner + m_slotPosInfor.fYSlotPosInMap;

    tR2.x = tR.x - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
            * cosf(m_slotPosInfor.fSlotAngle);
    tR2.y = tR.y - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
            * sinf(m_slotPosInfor.fSlotAngle);

    fSlotCornerSafeDis = Constants::para_slot_corner_safety_dis;
    fSlotAreaSafeDis = Constants::para_slot_area_dis;
    fOutSlotSafeDis = Constants::para_out_slot_area_dis;
    fSafeDisOfMirror = 0.01f;
    m_configurationSpace.setSlotConer(tR, tR2, tF, tF2, fSlotCornerSafeDis, fSafeDisOfMirror);
    CreatSlotAreaPolygon(m_slotPosInfor, polygon);
    m_configurationSpace.setAreaSafeDis(polygon, fOutSlotSafeDis, fSlotAreaSafeDis);
}


/******PART I-泊车开始的入库位置规划START********/

/**
 * @brief 计算左边界余量和更新车位宽度
 * @param fSlotWidth 车位宽度
 * @param LeftMargin 左边界余量
 * @param NewSlotWidth 更新车位宽度
 */
void ParallelSlotPathPlanning::CalLeftMarginAndSlotWidth(float fSlotLength,
                                                         float fSlotWidth,
                                                         float &LeftMargin,
                                                         float &NewSlotWidth)
{
    float fMinDis = MIN_VEH_SIDE_TO_RIGHT_MARGIN;
    float fMaxDis = MAX_VEH_SIDE_TO_RIGHT_MARGIN;

//    NewSlotWidth = MIN(fMaxDis + Constants::width, fSlotWidth);
//    LeftMargin = NewSlotWidth - (fMinDis + Constants::width);
    LeftMargin = 0.0f;
    // NewSlotWidth = fSlotWidth-0.15f;
    NewSlotWidth = fSlotWidth;
//    NewSlotWidth = MIN(MAX(fSlotWidth-0.2f, Constants::width+0.3f), fSlotWidth);
}

/**
 * @brief 根据行车状态，计算旋转角度和更新车辆位置
 * @param DrivingMode 行车状态
 * @param tPoint 当前坐标
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param fYRightMargin 右边界
 * @return 旋转角度
 */
float ParallelSlotPathPlanning::CalRotationAngleInParaSlot(ParallSlotDrivingMode DrivingMode,
                                                           LocationPoint &tPoint,
                                                           float fRadius,
                                                           float fXRearMargin,
                                                           float fXFrontMargin,
                                                           float fYRightMargin)
{
    float fTheta = 999.0;

    if(DrivingMode == RightForwardDriving)
    {
        fTheta = CalRightFowardRotationAngleInParaSlot(tPoint, fXFrontMargin, fRadius);
    }
    else if(DrivingMode == RightBackDriving)
    {
        fTheta = CalRightBackRotationAngleInParaSlot(tPoint, fXRearMargin, fYRightMargin, fRadius);
    }
    else if(DrivingMode == LeftForwardDriving)
    {
        fTheta = CalLeftForwardRotationAngleInParaSlot(tPoint, fXFrontMargin, fRadius);
    }
    else if(DrivingMode == LeftBackDriving)
    {
        fTheta = CalLeftBackRotationAngleInParaSlot(tPoint, fXRearMargin, fYRightMargin, fRadius);
    }
    return fTheta;
}

/**
 * @brief 计算右转向前旋转角度和更新车辆位置
 * @param tPoint 当前坐标
 * @param fXFrontMargin 前边界
 * @return 旋转角度
 */
float ParallelSlotPathPlanning::CalRightFowardRotationAngleInParaSlot(LocationPoint &tPoint,
                                                                      float fXFrontMargin,
                                                                      float fRadius)
{
    int8_t nSteeringDirection = STEERING_RIGHT;
    float fE1Radius = 0.0;
    float fF1Radius = 0.0;
    float fE2Radius = 0.0;
    float fF2Radius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float fTheta = 0.0;
    float fThetaF1Contact = FMAX_LIMIT;
    float fThetaF2Contact = FMAX_LIMIT;
    float triValue1 = 0.0;
    float triValue2 = 0.0;

    LocationPoint tE1;
    LocationPoint tF1;
    LocationPoint tE2;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;
    LocationPoint tRotationCenter;

    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);

    /*计算旋转中心*/
    CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                nSteeringDirection,
                                tPoint,
                                fRadius,
                                tRotationCenter);
    /*计算角点转弯半径*/
    CalCornerPointRadius2(nSteeringDirection,
                          fRadius,
                          fE1Radius,
                          fE2Radius,
                          fF1Radius,
                          fF2Radius,
                          fGRadius,
                          fHRadius);

    /*判断F1点的碰撞QIAN边界的工况*/
    if(tRotationCenter.x + Constants::FrontEdgeToCenter < fXFrontMargin)
    {
        fThetaF1Contact = tPoint.yaw;
    }
    else
    {
        triValue1 = (fXFrontMargin - tRotationCenter.x) / fF1Radius;
        triValue2 = (tF1.x - tRotationCenter.x) / fF1Radius;
        if((fabsf(triValue1) <= 1 - CAL_ERROR) && (fabsf(triValue2) <= 1 - CAL_ERROR))
        {
            fThetaF1Contact =  asinf(triValue1) - asinf(triValue2);
        }else{}
    }

    /*判断F2点的碰撞QIAN边界的工况*/
    if(tRotationCenter.x + Constants::FrontEdgeToCenter - Constants::chamfer_length < fXFrontMargin)
    {
        fThetaF2Contact = tPoint.yaw;
    }
    else
    {
        triValue1 = (fXFrontMargin - tRotationCenter.x) / fF2Radius;
        triValue2 = (tF2.x - tRotationCenter.x) / fF2Radius;
        if((fabsf(triValue1) <= 1 - CAL_ERROR) && (fabsf(triValue2) <= 1 - CAL_ERROR))
        {
            fThetaF2Contact =  asinf(triValue1) - asinf(triValue2);
        }else{}
    }
    fTheta = MIN(fThetaF1Contact, fThetaF2Contact);

    /*更新车辆位置*/
    RotateCoordinateOfPoint(tRotationCenter, tPoint, -fTheta, tPoint);
    tPoint.yaw += -fTheta;

    return fTheta;
}

/**
 * @brief 计算右转向后旋转角度和更新车辆位置
 * @param tPoint 当前坐标
 * @param fXRearMargin 后边界
 * @param fYRightMargin 右边界
 * @return 旋转角度
 */
float ParallelSlotPathPlanning::CalRightBackRotationAngleInParaSlot(LocationPoint &tPoint,
                                                                    float fXRearMargin,
                                                                    float fYRightMargin,
                                                                    float fRadius)
{
    int8_t nSteeringDirection  = 0;
    float fThetaE1Contact = FMAX_LIMIT;
    float fThetaE2Contact = FMAX_LIMIT;
    float fThetaHContact = FMAX_LIMIT;
    float fTheta = 0.0;
    float fE1Radius = 0.0;
    float fE2Radius = 0.0;
    float fF1Radius = 0.0;
    float fF2Radius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float triValue1 = 0.0;
    float triValue2 = 0.0;
    float triValue3 = 0.0;
    float triValue4 = 0.0;
    float triValue5 = 0.0;
    float triValue6 = 0.0;
    LocationPoint tRotationCenter;
    LocationPoint tE1;
    LocationPoint tE2;
    LocationPoint tF1;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;

    nSteeringDirection = STEERING_RIGHT;

    /*计算旋转中心*/
    CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                nSteeringDirection,
                                tPoint,
                                fRadius,
                                tRotationCenter);

    /*计算角点转弯半径*/
    CalCornerPointRadius2(nSteeringDirection,
                          fRadius,
                          fE1Radius,
                          fE2Radius,
                          fF1Radius,
                          fF2Radius,
                          fGRadius,
                          fHRadius);

    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);

    triValue1 = (tE1.y - tRotationCenter.y) / fE1Radius;
    triValue2 = (fYRightMargin - tRotationCenter.y) / fE1Radius;

    triValue3 = (tE2.y - tRotationCenter.y) / fE2Radius;
    triValue4 = (fYRightMargin - tRotationCenter.y) / fE2Radius;

    triValue5 = (tRotationCenter.x - tH.x) / fHRadius;
    triValue6 = (tRotationCenter.x - fXRearMargin) / fHRadius;

    /*判断E1点的碰撞右边界的工况*/
    if (fabsf(triValue1) < 1 - CAL_ERROR && (fabsf(triValue2) < 1 - CAL_ERROR))
    {
        fThetaE1Contact = asinf(triValue1) - asinf(triValue2);
    }else{}

    /*判断E2点的碰撞右边界的工况*/
    if ((fabsf(triValue3) < 1 - CAL_ERROR) && (fabsf(triValue4) < 1 - CAL_ERROR))
    {
        fThetaE2Contact = asinf(triValue3) - asinf(triValue4);
    }else{}

    /*判断H点的碰撞后边界的工况*/
    if((fabsf(triValue5) < 1 - CAL_ERROR) && (fabsf(triValue6) < 1 - CAL_ERROR))
    {
        fThetaHContact = acosf(triValue5) - acosf(triValue6);
    }else{}

    fTheta = MIN(MIN(fThetaE1Contact, fThetaE2Contact), fThetaHContact);

    /*更新车辆位置*/
    RotateCoordinateOfPoint(tRotationCenter,
                            tPoint,
                            fTheta,
                            tPoint);
    tPoint.yaw = tPoint.yaw + fTheta;

    return fTheta;
}


/**
 * @brief  计算左转向前旋转角度和更新车辆位置
 * @param tPoint 当前坐标
 * @param fXFrontMargin 前边界
 * @return 旋转角度
 */
float ParallelSlotPathPlanning::CalLeftForwardRotationAngleInParaSlot(LocationPoint &tPoint,
                                                                      float fXFrontMargin,
                                                                      float fRadius)
{
    int8_t nSteeringDirection  = 0;
    float fThetaF1Contact = FMAX_LIMIT;
    float fThetaF2Contact = FMAX_LIMIT;
    float fTheta = 0.0;
    float fE1Radius = 0.0;
    float fE2Radius = 0.0;
    float fF1Radius = 0.0;
    float fF2Radius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float triValue1 = 0.0;
    float triValue2 = 0.0;
    float triValue3 = 0.0;
    float triValue4 = 0.0;
    LocationPoint tRotationCenter;
    LocationPoint tE1;
    LocationPoint tE2;
    LocationPoint tF1;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;
    float fBaseAngle, fYaw1, fYaw2;

    nSteeringDirection = STEERING_LEFT;

    /*计算旋转中心*/
    CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                nSteeringDirection,
                                tPoint,
                                fRadius,
                                tRotationCenter);

    /*计算角点转弯半径*/
    CalCornerPointRadius2(nSteeringDirection,
                          fRadius,
                          fE1Radius,
                          fE2Radius,
                          fF1Radius,
                          fF2Radius,
                          fGRadius,
                          fHRadius);

    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);

    fYaw1 = atanf((tF2.x - tRotationCenter.x) / (tRotationCenter.y - tF2.y));
    fYaw2 = atanf((fXFrontMargin - tRotationCenter.x) / (tRotationCenter.y - 0.0f));
    fBaseAngle = fYaw2 - fYaw1;

    triValue1 = (fXFrontMargin - tRotationCenter.x) / fF1Radius;
    triValue2 = (tF1.x - tRotationCenter.x) / fF1Radius;

    triValue3 = (fXFrontMargin - tRotationCenter.x) / fF2Radius;
    triValue4 = (tF2.x - tRotationCenter.x) / fF2Radius;

    /*判断F1点的碰撞QIAN边界的工况*/
    if(fabsf(triValue1) < 1 - CAL_ERROR && (fabsf(triValue2) < 1 - CAL_ERROR))
    {
        /*计算F1旋转角度*/
        fThetaF1Contact = asinf(triValue1)  - asinf(triValue2);
    }else{}

    /*判断F2点的碰撞QIAN边界的工况*/
    if((fabsf(triValue3) < 1 - CAL_ERROR) && (fabsf(triValue4) < 1 - CAL_ERROR))
    {
        /*计算F2旋转角度*/
        fThetaF2Contact = asinf(triValue3)  - asinf(triValue4);
    }else{}

    fTheta = MIN(fThetaF1Contact, fThetaF2Contact);
    fTheta = MIN(fBaseAngle, fTheta);

    /*更新车辆位置*/
    RotateCoordinateOfPoint(tRotationCenter,
                            tPoint,
                            fTheta,
                            tPoint);
    tPoint.yaw += fTheta;

    return fTheta;
}


/**
 * @brief 计算左转向后旋转角度和更新车辆位置
 * @param tPoint 当前坐标
 * @param fXRearMargin 后边界
 * @param fYRightMargin 右边界
 * @return 旋转角度
 */
float ParallelSlotPathPlanning::CalLeftBackRotationAngleInParaSlot(LocationPoint &tPoint,
                                                                   float fXRearMargin,
                                                                   float fYRightMargin,
                                                                   float fRadius)
{
    LocationPoint tE1;
    LocationPoint tF1;
    LocationPoint tE2;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;
    LocationPoint tRotationCenter;

    bool bE1ContactValidity = false;
    bool bE2ContactValidity = false;

    float fE1Radius = 0.0;
    float fF1Radius = 0.0;
    float fE2Radius = 0.0;
    float fF2Radius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float fDisCenter = 0.0;
    float fTheta = 0.0;
    float fXE1Contact = 0.0;
    float fXE2Contact = 0.0;
    float fThetaE1Contact = FMAX_LIMIT;
    float fThetaE2Contact = FMAX_LIMIT;
    float fThetaHContact = FMAX_LIMIT;
    float triValue1 = 0.0;
    float triValue2 = 0.0;

    int8_t nSteeringDirection = STEERING_LEFT;

    Circle circle;
    LineEq RightEdgeLine;
    std::vector<LocationPoint> Points;

    RightEdgeLine.A = 0.0;
    RightEdgeLine.B = 1.0;
    RightEdgeLine.C = -fYRightMargin;

    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);

    /*计算旋转中心*/
    CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                nSteeringDirection,
                                tPoint,
                                fRadius,
                                tRotationCenter);

    /*计算角点转弯半径*/
    CalCornerPointRadius2(nSteeringDirection,
                          fRadius,
                          fE1Radius,
                          fE2Radius,
                          fF1Radius,
                          fF2Radius,
                          fGRadius,
                          fHRadius);

    fDisCenter = CalPointAndLineDis({tRotationCenter.x, tRotationCenter.y}, RightEdgeLine);
    /*判断E1点的碰撞右边界的工况*/
    if(fE1Radius <= fDisCenter || (tE1.x <= tRotationCenter.x))
    {
        fThetaE1Contact = tPoint.yaw;
    }
    else
    {
        circle.x = tRotationCenter.x;
        circle.y = tRotationCenter.y;
        circle.R = fE1Radius;
        bE1ContactValidity = InterPointBetweenCircleAndLine(circle,
                                                            RightEdgeLine,
                                                            Points);
        if (bE1ContactValidity)
        {
            if(Points.size() == 1)
            {
                fXE1Contact = Points[0].x;
            }
            else if(Points.size() == 2)
            {
                fXE1Contact = MAX(Points[0].x, Points[1].x);
            }else{}
            triValue1 = (tE1.x - tRotationCenter.x) / fE1Radius;
            triValue2 = (fXE1Contact - tRotationCenter.x) / fE1Radius;
            if((fabsf(triValue1) <= 1 - CAL_ERROR) && (fabsf(triValue2) <= 1 - CAL_ERROR))
            {
                fThetaE1Contact = asinf(triValue1) - asinf(triValue2);
            }else{}
        }else{}
    }

    /*判断E2点的碰撞右边界的工况*/
    if(fE2Radius <= fDisCenter || (tE2.x <= tRotationCenter.x))
    {
        fThetaE2Contact = tPoint.yaw;
    }
    else
    {
        circle.x = tRotationCenter.x;
        circle.y = tRotationCenter.y;
        circle.R = fE2Radius;
        bE2ContactValidity = InterPointBetweenCircleAndLine(circle,
                                                            RightEdgeLine,
                                                            Points);
        if (bE2ContactValidity)
        {
            if(Points.size() == 1)
            {
                fXE2Contact = Points[0].x;
            }
            else if(Points.size() == 2)
            {
                fXE2Contact = MAX(Points[0].x, Points[1].x);
            }else{}

            triValue1 = (tE2.x - tRotationCenter.x) / fE2Radius;
            triValue2 = (fXE2Contact - tRotationCenter.x) / fE2Radius;
            if((fabsf(triValue1) <= 1 - CAL_ERROR) && (fabsf(triValue2) <= 1 - CAL_ERROR))
            {
                fThetaE2Contact = asinf(triValue1) - asinf(triValue2);
            }
        }else{}
    }

    /*判断H点的碰撞后边界的工况*/
    if(tRotationCenter.x - Constants::RearEdgeToCenter >= fXRearMargin)
    {
        fThetaHContact = tPoint.yaw;
    }
    else
    {
        triValue1 = (tH.x - tRotationCenter.x) / fHRadius;
        triValue2 = (fXRearMargin - tRotationCenter.x) / fHRadius;
        if((fabsf(triValue1) <= 1 - CAL_ERROR) && (fabsf(triValue2) <= 1 - CAL_ERROR))
        {
            fThetaHContact = asinf(triValue1) - asinf(triValue2);
        }else{}
    }

    fTheta = MIN(fThetaHContact, MIN(fThetaE1Contact, fThetaE2Contact));

    /*更新车辆位置*/
    RotateCoordinateOfPoint(tRotationCenter, tPoint, -fTheta, tPoint);
    tPoint.yaw += -fTheta;

    return fTheta;
}

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

#if 0
void ParallelSlotPathPlanning::CalStep3KeyPoint(float fXA,
                                                float fYA,
                                                float fXC,
                                                float fYC,
                                                float fTargetX,
                                                float fTargetY,
                                                bool &bSlotSizeSuccess,
                                                LocationPoint &tStep3KeyPoint,
                                                ulong& TotalSteps)
{
    bool bBreakValue = false;
    bool bVal = false;

    float fXCMin = 0.0;
    float fMinDis= MIN_DIS_ONE_TIME;
    float fParkingOutRadius= Constants::parking_out_radius;
    float fMaxX = 0.0;
    float fMinX = 0.0;
    float fXInRearEdge = fXA + Constants::RearEdgeToCenter;
    float fDeltaDis = 0.3f;
    int nStatus = 0;
    int num = 0;
    int nIterations = 0;

    bSlotSizeSuccess = false;

    while (nIterations < 10)
    {
        switch (nStatus)
        {
        case 0:
        {
            CalMinSlotLengthOneTrialParkingIn(fXA,
                                              fYC,
                                              fTargetY,
                                              fXCMin);
            if (fXCMin <= fXC && (fTargetX - fDeltaDis >= fXInRearEdge))
            {
                /*计算一次出库且最靠近前边界的出库位置*/
                fMaxX = fXInRearEdge + fMinDis + (fXC - fXCMin);
                /*选择目标停车点和一次出库且最靠近前边界的车辆位置x的最小值,作为最佳入库点*/
                fMinX = MIN(fMaxX, fTargetX - fDeltaDis);
                num = MAX(int((fMinX - fXInRearEdge) / Constants::step_size), 0);
                /*筛选从最佳入库点到最靠近后边界之间能够无碰撞出库的位置*/
                for(int i = 0; i <= num; i++)
                {
                    tStep3KeyPoint.x = fMinX - i * Constants::step_size;
                    tStep3KeyPoint.y = fTargetY;
                    tStep3KeyPoint.yaw = 0;
                    bVal = ParkingOutCheck(fParkingOutRadius,
                                           fYA,
                                           fXC,
                                           fYC,
                                           tStep3KeyPoint);
                    if(bVal)
                    {
                        /*车位大小满足要求*/
                        bSlotSizeSuccess=true;
                        TotalSteps = 1;
                        break;
                    }
                }
                bSlotSizeSuccess = false;
                if(!bSlotSizeSuccess)
                {
                    GAC_LOG_DEBUG("车辆大小不满足, 车辆无法一次进库\n");
                    nStatus = 1;
                }
                else
                {
                    GAC_LOG_DEBUG("车辆可以一次进库\n") ;
                    bBreakValue = true;
                }
            }
            else
            {
                /*如果车位够大，但是目标停车点距离后边界太近，不让一把入库*/
                GAC_LOG_DEBUG( "车辆无法一次进库\n");
                nStatus = 1;
            }
        }
            break;

        case 1:
        {
            /*逆推车辆出库点坐标，判断车位大小尺寸是否符合要求*/
            CalBestParkingOutPos(fXA,
                                 fYA,
                                 fXC,
                                 fYC,
                                 fTargetX,
                                 fTargetY,
                                 bSlotSizeSuccess,
                                 tStep3KeyPoint,
                                 TotalSteps);
            bBreakValue = true;
        }
            break;

        default:
            break;
        }

        nIterations++;

        if (bBreakValue)
        {
            break;
        }else{}
    }
}
#endif

void ParallelSlotPathPlanning::CalStep3KeyPoint(float fXA,
                                                float fYA,
                                                float fXC,
                                                float fYC,
                                                float fTargetX,
                                                float fTargetY,
                                                bool &bSlotSizeSuccess,
                                                LocationPoint &tStep3KeyPoint,
                                                ulong& TotalSteps)
{
    /*该计算入库点的函数不包括一次入库车辆摆正的情况*/
    bSlotSizeSuccess = false;
    /*逆推车辆出库点坐标，判断车位大小尺寸是否符合要求*/
    CalBestParkingOutPosParkingIn(fXA,
                                  fYA,
                                  fXC,
                                  fYC,
                                  fTargetX,
                                  fTargetY,
                                  bSlotSizeSuccess,
                                  tStep3KeyPoint,
                                  TotalSteps);
}


/**
 * @brief 计算车辆F点出车库不碰到车位C点的最小车位C点坐标
 * @param nSlotSide 车位的方向,1-右侧车位 -1-左侧车位
 * @param fXA 平行车位A点X坐标
 * @param fYA 平行车位A点Y坐标
 * @param fYC 平行车位C点Y坐标
 * @param fTargetY 目标停车点Y
 * @param fXCMin c点最小X坐标
 */
void ParallelSlotPathPlanning::CalMinSlotLengthOneTrialParkingIn(float fXA,
                                                                 float fYC,
                                                                 float fTargetY,
                                                                 float &fXCMin)
{
    int8_t nSteeringDirection = STEERING_LEFT;
    float fXRearAxleCenterStop = 0.0;
    float fYRearAxleCenterStop = 0.0;
    float fXRotationCenterStop = 0.0;
    float fYRotationCenterStop = 0.0;
    float fERadius = 0.0;
    float fFRadius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float fStep3Radius = 0.0;
    float fval1 = 0.0;
    float fval2 = 0.0;

    /*车辆一次入库的停车点*/
    fXRearAxleCenterStop = fXA + MIN_DIS_ONE_TIME + Constants::RearEdgeToCenter;
    fYRearAxleCenterStop = fTargetY;

    /*车辆一次入库step3阶段的圆心*/
    fXRotationCenterStop = fXRearAxleCenterStop;
    fStep3Radius= Constants::parking_out_radius;

    fYRotationCenterStop = fYRearAxleCenterStop + fStep3Radius;

    /*计算车辆角点转弯半径*/
    CalCornerPointRadius(RIGHTSIDESLOT,
                         nSteeringDirection,
                         fStep3Radius,
                         fERadius,
                         fFRadius,
                         fGRadius,
                         fHRadius);

    /*计算车辆一次出库的C点最小坐标*/
    fval1 = Constants::para_slot_corner_safety_dis + fFRadius;
    fval2 = fYC - fYRotationCenterStop;
    fXCMin = sqrtf(powf(fval1, 2)- powf(fval2, 2)) + fXRotationCenterStop;
}

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
void ParallelSlotPathPlanning::CalBestParkingOutPosParkingIn(float fXA,
                                                             float fYA,
                                                             float fXC,
                                                             float fYC,
                                                             float fTargetX,
                                                             float fTargetY,
                                                             bool &bSlotSizeSuccess,
                                                             LocationPoint &tStep3KeyPoint,
                                                             ulong& TotalSteps)
{
    bool bBackForwardProcessSuccess = false;
    float SlotSubVehLength = 0.0;
    float StepSize = STEP_SIZE_SEARCH;
    float StartX = 0.0;
    float fRadius = Constants::min_steering_radius + PARA_PLANNING_RADIUS_ERROR;
    int8_t nInitialDrivingDirection = 0;
    int num = 0;
    ulong nDrivingTimes = 0;
    ulong Mintimes = 0;

    LocationPoint tTargetPoint;
    LocationPoint tOutputStep3KeyPoint;
    LocationPoint tE1,tE2,tF1, tF2, tG, tH;
    Step3KeyPointInfor Step3KeyPointInfor_;
    std::vector<Step3KeyPointInfor> Step3KeyPointSet;
    std::vector<Step3KeyPointInfor> Step3KeyPointSet2;
    std::vector<float> DrivingDisSet;

    Step3KeyPointSet2.clear();
    Step3KeyPointSet.clear();
    TotalSteps=0;
    bSlotSizeSuccess = false;
    SlotSubVehLength = fXC - fXA - Constants::length;
    num = int(SlotSubVehLength / StepSize);

    for(int startPointNum = 0; startPointNum <= num; ++startPointNum)
    {
        StartX = fXA + Constants::RearEdgeToCenter + startPointNum * StepSize;
        for(int i = 0; i < 2; i++)
        {
            if (i == 0)
            {
                nInitialDrivingDirection = FORWARD;
            }
            else
            {
                nInitialDrivingDirection = BACKWARD;
            }

            tTargetPoint.x = StartX;
            tTargetPoint.y = fTargetY;
            tTargetPoint.yaw = 0;

            CalParkingOutPosInfixedInitialPoint(nInitialDrivingDirection,
                                                fXA,
                                                fYA,
                                                fXC,
                                                fYC,
                                                tTargetPoint,
                                                fRadius,
                                                bBackForwardProcessSuccess,
                                                nDrivingTimes,
                                                tOutputStep3KeyPoint,
                                                DrivingDisSet);
            if(bBackForwardProcessSuccess)
            {
                Step3KeyPointInfor_.startX = StartX;
                Step3KeyPointInfor_.nInitialDrivingDirection = nInitialDrivingDirection;
                Step3KeyPointInfor_.point = tOutputStep3KeyPoint;
                Step3KeyPointInfor_.nAdjustTimes = nDrivingTimes;
                CalCornerCoordinate2(Step3KeyPointInfor_.point, tE1, tE2, tF1, tF2, tG, tH);
                Step3KeyPointInfor_.fXH = tH.x;
                Step3KeyPointSet.push_back(Step3KeyPointInfor_);
            }
        }
    }
    if(Step3KeyPointSet.empty())
    {
        bSlotSizeSuccess = false;
        TotalSteps = 0;
    }
    else
    {
        /*最小泊车次数*/
        Mintimes = Step3KeyPointSet[0].nAdjustTimes;

        for(size_t i = 0; i < Step3KeyPointSet.size(); ++i)
        {
            if(Step3KeyPointSet[i].nAdjustTimes < Mintimes)
            {
                Mintimes = Step3KeyPointSet[i].nAdjustTimes;
            }
        }
        /*相同最小泊车次数对应的状态加入集合*/
        for(size_t i = 0; i < Step3KeyPointSet.size(); i++)
        {
            if(Step3KeyPointSet[i].nAdjustTimes == Mintimes)
            {
                Step3KeyPointSet2.push_back(Step3KeyPointSet[i]);
            }
        }

        if(Mintimes == 1)
        {
            /*车辆在车位里调整一次*/
            for(size_t i = 0; i < Step3KeyPointSet2.size(); ++i)
            {
                Step3KeyPointSet2[i].startX -= fTargetX - 0.3f;
            }
            sort(Step3KeyPointSet2.begin(), Step3KeyPointSet2.end(),
                 [](const Step3KeyPointInfor &a, const Step3KeyPointInfor &b)
                  {return a.startX < b.startX;});
            tStep3KeyPoint = Step3KeyPointSet2.front().point;
            TotalSteps = Step3KeyPointSet2.front().nAdjustTimes;
        }
        else
        {
            sort(Step3KeyPointSet2.begin(), Step3KeyPointSet2.end(),
                 [](const Step3KeyPointInfor &a, const Step3KeyPointInfor &b)
                  {return a.point.yaw < b.point.yaw;});
            tStep3KeyPoint = Step3KeyPointSet2.front().point;
            TotalSteps = Step3KeyPointSet2.front().nAdjustTimes;
        }


        bSlotSizeSuccess = true;
        CalCornerCoordinate(Step3KeyPointSet2.front().point, tE1, tF1, tG, tH);
        if(m_slotPosInfor.fRoadWidth < 3.3f)
        {
            if(tH.x > 0.3)
            {
                tStep3KeyPoint.x -= 0.1f;
            }
        }

        GAC_LOG_DEBUG("tStep3KeyPoint x %f y %f yaw %f\n",
               tStep3KeyPoint.x,tStep3KeyPoint.y,tStep3KeyPoint.yaw * 57.3f);

        GAC_LOG_DEBUG("xH is %f, xF is %f, yG is %f", tH.x, tF1.x, tG.y);

        GAC_LOG_DEBUG("F and C dis is %f", hypot(tF1.x - fXC, tF1.y - fYC));


        GAC_LOG_DEBUG(" TotalSteps %ld \n",TotalSteps);
    }
}



#if 0
void ParallelSlotPathPlanning::CalBestParkingOutPosParkingIn2(float fXA,
                                                             float fYA,
                                                             float fXC,
                                                             float fYC,
                                                             float fTargetX,
                                                             float fTargetY,
                                                             bool &bSlotSizeSuccess,
                                                             LocationPoint &tStep3KeyPoint,
                                                             ulong& TotalSteps)
{
    bool bVal = false;
    bool bPlanSucc = false;
    float fParkingOutRadius = Constants::parking_out_radius;
    float fTheta = 0.0f;
    float fVehWidth = Constants::width;
    float fRearEdgeToCenter = Constants::RearEdgeToCenter;
    float fDeltaX = 0.0f;
    float fDeltaY = 0.0f;
    float fFinalY = 0.0f;
    float fMinY = 1e5f;

    LocationPoint tPoint;
    std::vector<std::vector<CtrlPoint> > TargetSegSet;

    bSlotSizeSuccess = false;

    for(int j = 0; j < 5; ++j)
    {
        for(int i = 0; i < 90; ++i)
        {
            fTheta = i * 0.5 * PI / 180;
            fDeltaX = fVehWidth * sinf(fTheta);
            fDeltaY = fVehWidth * cosf(fTheta);
            tPoint.x = fXA + fDeltaX / 2 + fRearEdgeToCenter * cosf(fTheta) + 0.05f * j;
            tPoint.y = fYA + fDeltaY / 2 + fRearEdgeToCenter * sinf(fTheta);
            tPoint.yaw = fTheta;
            bVal = ParkingOutCheck(fParkingOutRadius,
                                   fXA,
                                   fYA,
                                   fXC,
                                   fYC,
                                   tPoint);
            if(bVal)
            {
                bPlanSucc = PathPlanningInParallelSlot(RIGHTSIDESLOT,
                                                       fXA,
                                                       fXC,
                                                       fYA,
                                                       tPoint,
                                                       fTargetX,
                                                       fTargetY,
                                                       FORWARD,
                                                       TargetSegSet);
                if(bPlanSucc && !TargetSegSet.empty())
                {
                    fFinalY = fabsf(TargetSegSet.back().back().y - fTargetY);
                    if (fFinalY < fMinY)
                    {
                        fMinY = fFinalY;
                        bSlotSizeSuccess = true;
                        tStep3KeyPoint = tPoint;
                        TotalSteps = TargetSegSet.size();
                    }
                }
            }
        }
    }
    return;
}
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
void ParallelSlotPathPlanning::CalParkingOutPosInfixedInitialPoint(int8_t nInitialDrivingDirection,
                                                                   float fXA,
                                                                   float fYA,
                                                                   float fXC,
                                                                   float fYC,
                                                                   LocationPoint tPoint,
                                                                   float fRadius,
                                                                   bool &bBackForwardAdjustSucesss,
                                                                   ulong& nBackForwardTimes,
                                                                   LocationPoint &tOutputStep3KeyPoint,
                                                                   std::vector<float> &DrivingDisSet)
{
    bool bBreakValue = false;
    bool bVal = false;
    int nIterations = 0;
    float fTheta = 0.0;
    float fMinDis = MIN_XH;/*一次调整可以出库时的车辆出库点H点最小坐标*/
    float fDrivingDis = 0.0;
    float fParkingOutRadius = Constants::parking_out_radius;
    float fXHError = 0.0f;
    LocationPoint tE1;
    LocationPoint tE2;
    LocationPoint tF1;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;
    ParallSlotDrivingMode nStatus = InitConditionJudgment;

    /*初始化开始调整的车辆坐标*/
    DrivingDisSet.clear();
    nBackForwardTimes = 0;
    bBackForwardAdjustSucesss = false;

    if(m_slotPosInfor.nParkingType == HEAD_PARK_OUT)
    {
        fXHError = 1e-2f;
    }else{}

    if(fXC - fXA < 0.8f + Constants::length)
    {
        fMinDis = 0.0f;
    }
    else
    {
        fMinDis = 0.2f;
    }

    while (nIterations < 100)
    {
        switch (nStatus)
        {
        case InitConditionJudgment:
        {
            if (nInitialDrivingDirection == BACKWARD)
            {
                nStatus = RightBackDriving;
            }
            else if (nInitialDrivingDirection == FORWARD)
            {
                nStatus = LeftForwardDriving;
            }
            else
            {
                ;
            }
        }
            break;

        /*右打方向盘后退*/
        case RightBackDriving:
        {
            ++nBackForwardTimes;
            CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);

            /*超过最大泊车次数，车辆或者无法倒退*/
            if(nBackForwardTimes > MAX_BACK_FORWARD_TIMES
               ||(MIN(tE1.y, tE2.y) <= fYA)
               ||(tH.x <= fXA))
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }
            /*下一步行驶的旋转角度和新的车辆位置*/
            fTheta = CalRotationAngleInParaSlot(nStatus, tPoint, fRadius, fXA, fXC, fYA);

            if((fabsf(fTheta) > PI/2 )|| (tPoint.yaw > MAX_STEP3_YAW))
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }

            fDrivingDis = fabsf(fTheta) * fRadius;
            DrivingDisSet.push_back(fDrivingDis);

            CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);

            bVal = ParkingOutCheck(fParkingOutRadius,
                                   fXA,
                                   fYA,
                                   fXC,
                                   fYC,
                                   tPoint); // && (tH.x > fXA + fMinDis - fXHError);
            if (bVal)
            {
                bBackForwardAdjustSucesss = true;
                bBreakValue = true;
            }
            else
            {
                nStatus = LeftForwardDriving;
            }
        }
            break;

        /*左打方向盘前进*/
        case LeftForwardDriving:
        {
            nBackForwardTimes++;
            CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
            /*超过最大泊车次数，车辆或者无法前进*/
            if(nBackForwardTimes > MAX_BACK_FORWARD_TIMES
               ||(MAX(tF1.x, tF2.x) >= fXC))
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }

            /*下一步行驶的旋转角度和新的车辆位置*/
            fTheta = CalRotationAngleInParaSlot(nStatus, tPoint, fRadius, fXA, fXC, fYA);

            if((fabsf(fTheta) > PI/2) || (tPoint.yaw > MAX_STEP3_YAW))
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }
            fDrivingDis = fabsf(fTheta) * fRadius;
            DrivingDisSet.push_back(fDrivingDis);
            nStatus = RightBackDriving;
        }
            break;

        default:
            break;

        }
        nIterations++;
        if (bBreakValue)
        {
            break;
        }
    }

    if(bBackForwardAdjustSucesss)
    {
        tOutputStep3KeyPoint = tPoint;
    }
    else{}
}

/**
 * @brief 出库检查
 * @param fRadius 出库转弯半径
 * @param fXC C点X坐标
 * @param fYC C点Y坐标
 * @param tStep3KeyPoint 车辆出库位姿
 * @return 0-可以出库 1-不能出库
 */
bool ParallelSlotPathPlanning::ParkingOutCheck(float fRadius,
                                               float fXA,
                                               float fYA,
                                               float fXC,
                                               float fYC,
                                               LocationPoint tStep3KeyPoint)
{
    bool bSucc = false;
    bool bFlag = false;
    bool bColl = false;

    int8_t nSteeringDirection = STEERING_LEFT;
    int8_t nDrivingDirection = FORWARD;
    float fLengthOC = 0.0;
    float fERadius = 0.0;
    float fFRadius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float kappa = 1 / fRadius;
    float fCurveLength = 3.0;
    float fYawOF, fYawOC = 0.0f;
    float fMaxYC = 0.0f;
    float fSafeDis = 0.3f;
    Node3D CurNode;
    std::vector<CtrlPoint> targetSeg;
    LocationPoint tRotationCenter;
    LocationPoint tE,tF,tG,tH;
    LocationPoint CurPoint;

    /*计算旋转中心*/
    CalRotationCenterCoordinate(1,
                                nSteeringDirection,
                                tStep3KeyPoint,
                                fRadius,
                                tRotationCenter);
    CalCornerCoordinate(tStep3KeyPoint, tE, tF, tG, tH);

    fYawOF = atan2f(tF.x - tRotationCenter.x, tRotationCenter.y - tF.y);
    fYawOC = atan2f(fXC - tRotationCenter.x, tRotationCenter.y - fYC);
    /*计算角点转弯半径*/
    CalCornerPointRadius(1,
                         nSteeringDirection,
                         fRadius,
                         fERadius,
                         fFRadius,
                         fGRadius,
                         fHRadius);

    fLengthOC = hypotf(fXC - tRotationCenter.x, fYC - tRotationCenter.y);

    bColl = (hypotf(fXC - tF.x, fYC - tF.y) < (Constants::para_slot_corner_safety_dis + 0.05));
    /*判断旋转中心到C点距离和车辆F点转弯半径,是否可以出库*/
    if ((fLengthOC > fFRadius + Constants::para_slot_corner_safety_dis + 0.05f || (tF.y > fYC))
        && (!bColl))
    {
        if(m_slotPosInfor.nParkingType == TAIL_PARK_IN)
        {
            CircleInterpolate(nDrivingDirection,
                              nSteeringDirection,
                              kappa,
                              fCurveLength,
                              tStep3KeyPoint,
                              targetSeg);
            /*翻转左侧坐标系*/
            if(m_slotPosInfor.nSlotSide == LEFTSIDESLOT)
            {
                for(size_t j = 0; j < targetSeg.size(); j++)
                {
                    targetSeg[j].y = -targetSeg[j].y;
                    targetSeg[j].yaw = normalizeHeadingRad_Npi_Ppi(-targetSeg[j].yaw);
                    targetSeg[j].kappa = -targetSeg[j].kappa;
                }
            }

            /*从step3点生成一段轨迹，该轨迹到Y方向车辆角点高出车位角点一定距离时结束,该段轨迹应该无碰撞*/
            for(size_t i = 0; i < targetSeg.size(); i++)
            {
                CurPoint.x = targetSeg[i].x;
                CurPoint.y = targetSeg[i].y;
                CurPoint.yaw = targetSeg[i].yaw;
                CurNode.x = CurPoint.x + m_slotPosInfor.fXSlotPosInMap;
                CurNode.y = CurPoint.y + m_slotPosInfor.fYSlotPosInMap;
                CurNode.t = normalizeHeadingRad_0_2pi(CurPoint.yaw);
                /*这里已经翻转坐标系的四个角点，因此左右车位不一样*/
                CalCornerCoordinate(CurPoint, tE, tF, tG, tH);

                if(m_slotPosInfor.nSlotSide == RIGHTSIDESLOT)
                {
                    fMaxYC = MAX_YC + m_slotPosInfor.fYFrontSlotCorner;
                    if(tF.y > fMaxYC)
                    {
                        bFlag = true;
                        bSucc = true;
                        break;
                    }
                    /*右侧车辆E点和H点离开一定安全距离才开始做碰撞检测*/
                    if((tH.x > fXA + fSafeDis) && (tE.y > fYA + fSafeDis) && (!m_configurationSpace.isTraversable(&CurNode)))
                    {
                        bSucc = false;
                        if(fLengthOC - fFRadius > 0.2f)
                        {
                            GAC_LOG_WARN("平行车位角点可能存在异常情况，请处理！！！");
                        }
                        break;
                    }
                }
                else
                {
                    fMaxYC = -MAX_YC  + m_slotPosInfor.fYFrontSlotCorner;
                    if(tG.y < fMaxYC)
                    {
                        bFlag = true;
                        bSucc = true;
                        break;
                    }
                    /*左侧车辆E点和H点离开一定安全距离才开始做碰撞检测*/
                    if((tE.x > fXA + fSafeDis) && (tH.y < -fYA - fSafeDis) && (!m_configurationSpace.isTraversable(&CurNode)))
                    {
                        bSucc = false;
                        if(fLengthOC - fFRadius > 0.2f)
                        {
                            GAC_LOG_WARN("平行车位角点可能存在异常情况，请处理！！！");
                        }
                        break;
                    }
                }
            }
        }
        else
        {
            bSucc = true;
        }
    }
    else
    {
        bSucc = false;
    }
    return bSucc;
}

/******PART I-泊车开始的入库位置规划END********/


/******PART II-车辆入库后的重新规划START********/

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
bool ParallelSlotPathPlanning::NewPlanningInSlot(float fXRearMargin,
                                                 float fXFrontMargin,
                                                 float fYRightMargin,
                                                 LocationPoint tPoint,
                                                 float VehTargetX,
                                                 float fVehTargety,
                                                 int8_t nInitDrivingDirection,
                                                 float &fCalRadius,
                                                 float &XFrontMarginNeed,
                                                 float &XRearMarginNeed,
                                                 std::vector<float>& NextDrivingDisSets,
                                                 float &fVehStopy)
{
    bool bBackForwardAdjustSucesss = false;
    bool bPlanningSucess = false;

    ulong nBackForwardTimes = 0;
    int nVehRearEdgToSlotRearMargin = 0;
    int nVehFrontEdgToSlotFrontMargin = 0;
    ulong nLastDrivingTimes = NMAX_LIMIT;
    ulong nCurrentDrivingTimes = 0;
    size_t nIndex = 0;
    int nRadiusNum = 0;

    float fYStop = 0.0;
    float fVehRearEdg = 0.0;
    float fVehFrontEdg = 0.0;
    float fIncrement = MARGIN_INCREMENT;
    float fYDownError = Y_DOWN_ERROR;
    float fYUpperError = Y_UPPER_ERROR;
    float fYError = Y_ERROR_INCREMENT;
    float fDetaY = 0.0;
    float fRadius = Constants::min_steering_radius + PARA_PLANNING_RADIUS_ERROR;
    float fRStepSize = 0.1;
    float fTempR = 0.0;
    LocationPoint tE1, tE2, tF1, tF2, tG, tH;
    SlotEdgInfor SlotEdgInfor_;
    std::vector<SlotEdgInfor> SlotEdgInforSets;
    std::vector<SlotEdgInfor> SlotEdgInforSets2;
    XFrontMarginNeed = 0.0;
    XRearMarginNeed = 0.0;

    fVehTargety = -fabsf(fVehTargety);

    nRadiusNum = std::round(PARA_PLANNING_RADIUS_ERROR / fRStepSize) * 2;
    /*计算车辆的前角点X最大值和后角点的X最小值*/
    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
    fVehRearEdg = MIN(MIN(tE1.x, tE2.x), tH.x);
    fVehFrontEdg = MAX(MAX(tF1.x, tF2.x), tG.x);

    /*这里对前后边界和右边界进行更新*/
    fYRightMargin = MIN(fYRightMargin, MIN(tE1.y, tE2.y));
    fXFrontMargin= MAX(fXFrontMargin, MAX(tF1.x, tF2.x));
    fXRearMargin = MIN(fXRearMargin, tH.x);

    /*边界的取值数目*/
    nVehRearEdgToSlotRearMargin = MAX(int((fVehRearEdg - fXRearMargin) / fIncrement)-1, 0);
    nVehFrontEdgToSlotFrontMargin = MAX(int((fXFrontMargin - fVehFrontEdg) / fIncrement)-1, 0);

    for(int nRadius = 0; nRadius <= nRadiusNum; ++nRadius)
    {
        for(int i = 0; i <= nVehFrontEdgToSlotFrontMargin; i++)
        {
            for(int j = 0; j <= nVehRearEdgToSlotRearMargin; j++)
            {
                fTempR = fRadius + (nRadius - nRadiusNum / 2) * fRStepSize;
                /*在一定前后边界下，计算车辆在车库内调整的最终停车点*/
                XFrontMarginNeed = fXFrontMargin - i*fIncrement;
                XRearMarginNeed = fXRearMargin + j*fIncrement;
                CalStopPosition(XRearMarginNeed,
                                XFrontMarginNeed,
                                fYRightMargin,
                                tPoint,
                                VehTargetX,
                                fVehTargety,
                                fTempR,
                                nInitDrivingDirection,
                                NextDrivingDisSets,
                                fYStop,
                                nBackForwardTimes,
                                bBackForwardAdjustSucesss);
                /*1 记录调整成功的边界、调整次数、和最终停车点Y组合*/
                if(bBackForwardAdjustSucesss)
                {
                    SlotEdgInfor_.fFrontEdg = XFrontMarginNeed;
                    SlotEdgInfor_.fRearEdg = XRearMarginNeed;
                    SlotEdgInfor_.nAdjustTimes = nBackForwardTimes;
                    SlotEdgInfor_.Y = fYStop;
                    SlotEdgInfor_.NextDrivingDisSets = NextDrivingDisSets;
                    SlotEdgInfor_.fPlanningR = fTempR;
                    SlotEdgInforSets.push_back(SlotEdgInfor_);
                }
            }
        }
    }

    if(!SlotEdgInforSets.empty())
    {
        /*2 筛选最靠近目标点Y的组合，如果没有，将控制量递增*/
        while(fYError < MAX(fabsf(fYUpperError), fabsf(fYDownError)))
        {
            for(size_t i = 0; i < SlotEdgInforSets.size(); i++)
            {
                fDetaY = SlotEdgInforSets[i].Y - fVehTargety ;
                if(fDetaY < fYUpperError && (fDetaY > fYDownError) /*&& (fabsf(fDetaY) < fYError)*/)
                {
                    SlotEdgInfor_ = SlotEdgInforSets[i];
                    SlotEdgInforSets2.push_back(SlotEdgInfor_);
                }
            }
            if(!SlotEdgInforSets2.empty())
            {
                break;
            }
            fYError += Y_ERROR_INCREMENT;
        }

        if(!SlotEdgInforSets2.empty())
        {
            /*3 筛选调整次数最少的组合*/
            for(size_t i = 0; i < SlotEdgInforSets2.size(); i++)
            {
                nCurrentDrivingTimes = SlotEdgInforSets2[i].nAdjustTimes;

                if(nCurrentDrivingTimes < nLastDrivingTimes)
                {
                    nLastDrivingTimes = nCurrentDrivingTimes;
                    nIndex = i;
                }
                else if(nCurrentDrivingTimes == nLastDrivingTimes )
                {
                    if(fabsf(SlotEdgInforSets2[i].Y - fVehTargety) < fabsf(SlotEdgInforSets2[nIndex].Y - fVehTargety))
                    {
                        nIndex = i;
                    }
                }else{}
            }
            XFrontMarginNeed = SlotEdgInforSets2[nIndex].fFrontEdg;
            XRearMarginNeed = SlotEdgInforSets2[nIndex].fRearEdg;
            NextDrivingDisSets = SlotEdgInforSets2[nIndex].NextDrivingDisSets;
            fVehStopy = SlotEdgInforSets2[nIndex].Y;
            fCalRadius = SlotEdgInforSets2[nIndex].fPlanningR;
            bPlanningSucess = true;
        }else
        {
            /*E20002平行泊车重规划成功，但是车辆较大程度靠里或者靠外*/
            planErrCode=E20002;
        }
    }
    else
    {
        /*E20001平行泊车重规划全部失败*/
        planErrCode=E20001;
    }
    return bPlanningSucess;
}

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
void ParallelSlotPathPlanning::CalStopPosition(float fXRearMargin,
                                               float fXFrontMargin,
                                               float fYRightMargin,
                                               LocationPoint tPoint,
                                               float VehTargetX,
                                               float fVehTargety,
                                               float fRadius,
                                               int8_t nInitDrivingDirection,
                                               std::vector<float>& NextDrivingDisSets,
                                               float &fYStop,
                                               ulong &nBackForwardTimes,
                                               bool &bBackForwardAdjustSucesss)
{
    bool bBreakValue = false;
    int nIterations = 0;
    int nSteeringDirection = 0;
    float fTheta = 0.0;
    float fNextDrivingDis = 0.0;
    float fXError = 0.1;
    ParallSlotDrivingMode status = InitConditionJudgment;
    LocationPoint tE1;
    LocationPoint tF1;
    LocationPoint tE2;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;
    LocationPoint tRotationCenter;

    //Init
    fYStop = 0.0;
    nBackForwardTimes = 0;
    bBackForwardAdjustSucesss = false;
    NextDrivingDisSets.clear();

    while (nIterations < 100)
    {
        switch(status)
        {
        case InitConditionJudgment:
        {
            if(tPoint.yaw > 0)
            {
                if(nInitDrivingDirection == BACKWARD)
                {
                    status=LeftBackDriving;
                }
                else
                {
                    status=RightForwardDriving;
                }
            }
            else if(tPoint.yaw > MIN_ALLOW_YAW)
            {
                 status = SteeringZero;
            }
            else/*车辆偏航角过头，异常情况*/
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
            }
        }
            break;

        case SteeringZero:
        {
            if(nInitDrivingDirection == FORWARD)
            {
                nSteeringDirection = STEERING_LEFT;
            }
            else
            {
                nSteeringDirection = STEERING_RIGHT;
            }
            /*计算旋转中心*/
            CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                        nSteeringDirection,
                                        tPoint,
                                        fRadius,
                                        tRotationCenter);
            fTheta = tPoint.yaw;
            RotateCoordinateOfPoint(tRotationCenter, tPoint, -fTheta, tPoint);
            fNextDrivingDis = fabsf(fTheta) * fRadius;
            NextDrivingDisSets.push_back(fNextDrivingDis);
            bBackForwardAdjustSucesss = true;
            nBackForwardTimes = 1;

            if(nInitDrivingDirection == FORWARD)
            {
                if(tPoint.x > VehTargetX + fXError)
                {
                    /*车辆回正后如果还需要一次换挡才能到达目标点*/
                    nBackForwardTimes++;
                }
            }
            else
            {
                if(tPoint.x < VehTargetX - fXError)
                {
                    /*车辆回正后如果还需要一次换挡才能到达目标点*/
                    nBackForwardTimes++;
                }
            }
            bBreakValue=true;
        }
            break;

        case LeftBackDriving:
        {
            nBackForwardTimes++;
            CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
            /*超过最大泊车次数，车辆或者无法倒退*/
            if(nBackForwardTimes > MAX_BACK_FORWARD_TIMES
               ||(MIN(tE1.y, tE2.y) <= fYRightMargin)
               ||(tH.x <= fXRearMargin))
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }

            /*下一步行驶的旋转角度和新的车辆位置*/
            fTheta = CalRotationAngleInParaSlot(status, tPoint, fRadius, fXRearMargin, fXFrontMargin,fYRightMargin);

            if(fabsf(fTheta) > PI/2)
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }

            fNextDrivingDis = fabsf(fTheta) * fRadius;
            NextDrivingDisSets.push_back(fNextDrivingDis);
            if(fabsf(tPoint.yaw) < CAL_ERROR)
            {
                /*车辆能够调平*/
                if(tPoint.x < VehTargetX - fXError)
                {
                    /*车辆回正后如果还需要一次换挡才能到达目标点*/
                    nBackForwardTimes++;
                }
                bBackForwardAdjustSucesss = true;
                bBreakValue = true;
            }
            else
            {
                status=RightForwardDriving;
            }


        }
            break;

        case RightForwardDriving:
        {
            nBackForwardTimes += 1;
            CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
            /*超过最大泊车次数，车辆或者无法前进*/
            if(nBackForwardTimes > MAX_BACK_FORWARD_TIMES || (MAX(tF1.x, tF2.x) >= fXFrontMargin))
            {
                bBackForwardAdjustSucesss=false;
                bBreakValue=true;
                break;
            }

            /*下一步行驶的旋转角度和新的车辆位置*/
            fTheta = CalRotationAngleInParaSlot(status, tPoint, fRadius, fXRearMargin, fXFrontMargin,fYRightMargin);

            if(fabsf(fTheta) > PI/2)
            {
                bBackForwardAdjustSucesss = false;
                bBreakValue = true;
                break;
            }

            fNextDrivingDis = fabsf(fTheta) * fRadius;
            NextDrivingDisSets.push_back(fNextDrivingDis);
            if(fabsf(tPoint.yaw) < CAL_ERROR)
            {
                /*车辆能够调平*/
                if(tPoint.x > VehTargetX + fXError)
                {
                    /*车辆回正后如果还需要一次换挡才能到达目标点*/
                    nBackForwardTimes++;
                }
                bBackForwardAdjustSucesss = true;
                bBreakValue = true;
            }
            else
            {
                status=LeftBackDriving;
            }

        }
            break;

        default:
            break;
        }
        nIterations++;
        if(bBreakValue)
        {
            break;
        }
    }

    if(bBackForwardAdjustSucesss)
    {
        fYStop = tPoint.y;
    }
    else
    {
        fYStop = 0;
        nBackForwardTimes = 0;
        NextDrivingDisSets.clear();
    }
}

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
bool ParallelSlotPathPlanning::CheckIfAdjustToZero(float fXRearMargin,
                                                   float fXFrontMargin,
                                                   float fYRightMargin,
                                                   LocationPoint tPoint,
                                                   float fRadius,
                                                   int8_t NextDrivingDirection,
                                                   float &fStopy)
{
    LocationPoint tE1;
    LocationPoint tF1;
    LocationPoint tE2;
    LocationPoint tF2;
    LocationPoint tG;
    LocationPoint tH;
    LocationPoint tRotationCenter;
    int8_t nSteeringDirection = STEERING_RIGHT;
    float fE1Radius;
    float fF1Radius;
    float fE2Radius;
    float fF2Radius;
    float fGRadius;
    float fHRadius;
    bool LastDriving=false;

    fStopy = 0.0;

    if(fabsf(tPoint.yaw) < ABS_MIN_STOP_YAW)
    {
        LastDriving = true;
        fStopy = tPoint.y;
    }
    else
    {
        CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
        if(NextDrivingDirection == FORWARD)
        {
            /*这里考虑偏航角的正负值*/
            if(tPoint.yaw > 0)
            {
                nSteeringDirection = STEERING_RIGHT;
            }
            else
            {
                nSteeringDirection = STEERING_LEFT;
            }
            CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                        nSteeringDirection,
                                        tPoint,
                                        fRadius,
                                        tRotationCenter);

            if(tRotationCenter.x + Constants::FrontEdgeToCenter < fXFrontMargin)
            {
                fStopy = tRotationCenter.y + fRadius;
                LastDriving = true;
            }else{}
        }
        else
        {
            if(tPoint.yaw > 0)
            {
                nSteeringDirection = STEERING_LEFT;
            }
            else
            {
                nSteeringDirection = STEERING_RIGHT;
            }
            CalRotationCenterCoordinate(RIGHTSIDESLOT,
                                        nSteeringDirection,
                                        tPoint,
                                        fRadius,
                                        tRotationCenter);
            /*计算角点转弯半径*/
            CalCornerPointRadius2(nSteeringDirection,
                                  fRadius,
                                  fE1Radius,
                                  fE2Radius,
                                  fF1Radius,
                                  fF2Radius,
                                  fGRadius,
                                  fHRadius);

            if((tRotationCenter.x - Constants::RearEdgeToCenter >= fXRearMargin)
               && (MAX(fE1Radius, fE2Radius) < (tRotationCenter.y - fYRightMargin)))
            {
                fStopy = tRotationCenter.y - fRadius;
                LastDriving = true;
            }
        }
    }
    return LastDriving;
}

/**
 * @brief 计算回旋线需要的初始曲率
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param tPoint 当前车辆位置
 * @param nDrivingDirection 行驶方向
 * @param fFindCurvature 回旋线初始曲率
 * @return 1-生成回旋线 0-无法生成回旋线
 */
bool ParallelSlotPathPlanning::CalClothoidCurvature(float fXRearMargin,
                                                    float fXFrontMargin,
                                                    LocationPoint tPoint,
                                                    float fRadius,
                                                    int8_t nDrivingDirection,
                                                    float &fFindCurvature)
{
    bool bClothoidAdjust = false;
    bool bFlag = false;
    float fCircleLen = 0.0;
    float fClothoidLen = 0.0;
    float fStopX = 0.0; /*回旋线结束点的x坐标*/
    float fFrontMarginStopX = 0.0;
    float fRearMarginStopX = 0.0;
    float kappa = 0.0;
    float fMaxR = 20.0; /*m*/
    float fDeltaCurvature = 0.01f;
    int nMaxCurvatureIndex = round(1 / fRadius / fDeltaCurvature);
    int nMinCurvatureIndex = round(1 / fMaxR / fDeltaCurvature);

    fFrontMarginStopX = fXFrontMargin - Constants::FrontEdgeToCenter;
    fRearMarginStopX = fXRearMargin + Constants::RearEdgeToCenter;
    /*判断在该方向盘转角范围内是否有角度，在到达边界时，能够进行方向盘回正*/
    for(int i = nMaxCurvatureIndex; i >= nMinCurvatureIndex; --i)
    {
        kappa = i * fDeltaCurvature;
        /*计算回旋曲线长度和圆弧长度*/
        fClothoidLen = kappa / CLOTHOID_CURVATURE_FACTOR;
        /*计算圆弧长度*/
        fCircleLen = 1 / kappa * (fabsf(tPoint.yaw) - AHEAD_YAW) - fClothoidLen / 2;
        if(fCircleLen < 0.0f)
        {
            fFindCurvature = kappa;
            continue;
        }
        /*计算圆弧加回旋线的车辆终点X坐标*/
        fStopX = tPoint.x + nDrivingDirection * (1 / kappa * sinf(fabsf(tPoint.yaw)) + fClothoidLen / 2);
        if (nDrivingDirection == FORWARD)
        {
            if (fStopX < fFrontMarginStopX)
            {
                bClothoidAdjust = true;
                bFlag = true;
                fFindCurvature = kappa;
            }
        }
        else
        {
            if (fStopX > fRearMarginStopX)
            {
                bClothoidAdjust = true;
                bFlag = true;
                fFindCurvature = kappa;
            }
        }
        if (bFlag) {break;}else{;}
    }
    return bClothoidAdjust;
}

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
bool ParallelSlotPathPlanning::CalHybridCurveLength(float fXRearMargin,
                                                    float fXFrontMargin,
                                                    LocationPoint tPoint,
                                                    float fVehTargetX,
                                                    int8_t nDrivingDirection,
                                                    float fFindCurvature,
                                                    float &fCircleLen,
                                                    float &fClothoidLen,
                                                    float &fLineLen)
{

    float fCircleStopX = 0.0;
    float fStopX = 0.0;
    float R = 1 / fFindCurvature;
    bool bLastDriving = false;
    float fFrontMarginStopX = fXFrontMargin - Constants::FrontEdgeToCenter; /*计算车辆在前后边界的极限位置X坐标*/
    float fRearMarginStopX = fXRearMargin + Constants::RearEdgeToCenter;
    float fXerror = TARGET_X_ERROR;

    /*计算回旋曲线长度和圆弧长度*/
    fClothoidLen = fFindCurvature / CLOTHOID_CURVATURE_FACTOR;
    fCircleLen = R * (fabsf(tPoint.yaw) - AHEAD_YAW) - fClothoidLen / 2;
    /*计算圆弧加回旋线的车辆终点X坐标*/
    fStopX = tPoint.x + nDrivingDirection * (R * sinf(fabsf(tPoint.yaw)) + fClothoidLen / 2);
    /*方向盘转角固定转向,偏航角为0时，车辆位置X*/
    fCircleStopX = tPoint.x + nDrivingDirection * R * (sinf(fabsf(tPoint.yaw)));

    /*当可以用回旋线调整且无法完成到达目标位置时，需要判断用产生回旋线对应的方向盘转角固定转向*/
    /*偏航角为0时，车辆位置X在目标范围0.1m内，这样做的好处是，可以让车辆减少一次调整*/
    if(nDrivingDirection == FORWARD)
    {
        if(fStopX - fVehTargetX > fXerror)
        {
             /*固定转向,偏航角为0时，车辆位置X在目标范围0.1m内*/
            if(fCircleStopX - fVehTargetX < fXerror)
            {
                fCircleLen = R * sinf(fabsf(tPoint.yaw));
                fClothoidLen = 0.0;
                fLineLen = 0.0;
                bLastDriving = true;
            }
            else
            {
                fLineLen = MIN(MAX(fFrontMarginStopX - fStopX, 0.0f), MAX_LINE_LENGTH);
            }
        }
        else if(fStopX - fVehTargetX > -fXerror)
        {
            fLineLen = 0;
            bLastDriving = true;
        }
        else
        {
            fLineLen = MAX(fVehTargetX - fStopX, 0.0f);
            bLastDriving = true;
        }
    }
    else
    {
        if(fStopX - fVehTargetX < -fXerror)
        {
            /*固定转向,偏航角为0时，车辆位置X在目标范围0.1m内*/
            if(fCircleStopX - fVehTargetX > -fXerror)
            {
                fCircleLen = R * sinf(fabsf(tPoint.yaw));
                fClothoidLen = 0.0;
                fLineLen = 0.0;
                bLastDriving = true;
            }
            else
            {
                fLineLen = MIN(MAX(fStopX - fRearMarginStopX, 0), MAX_LINE_LENGTH);
            }
        }
        else if(fStopX - fVehTargetX < fXerror)
        {
            fLineLen = 0.0;
            bLastDriving = true;
        }
        else
        {
            fLineLen = MAX(fStopX - fVehTargetX, 0.0f);
            bLastDriving = true;
        }
    }

    return bLastDriving;
}
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
bool ParallelSlotPathPlanning::ClothoidPathInSlot(float fXRearMargin,
                                                  float fXFrontMargin,
                                                  LocationPoint tPoint,
                                                  float fVehTargetX,
                                                  int8_t nDrivingDirection,
                                                  float fRadius,
                                                   std::vector<CtrlPoint> &targetSeg)
{
    bool bLastDriving = false;
    bool bClothoidAdjust = false;
    bool bCircleAjust = false;
    int8_t nSteeringDirection = 0;
    float fCircleLen = 0.0;
    float fClothoidLen = 0.0;
    float fLineLen = 0.0;
    float fFindCurvature = 0.0;

    targetSeg.clear();

    if (tPoint.yaw > 0.0f)
    {
        nSteeringDirection = -nDrivingDirection;
    }
    else
    {
        nSteeringDirection =  nDrivingDirection;
    }
    if (fabsf(tPoint.yaw) < ABS_MIN_STOP_YAW)
    {
        fFindCurvature = 0.0;
        fCircleLen = 0.0;
        fClothoidLen = 0.0;
        if(nDrivingDirection == FORWARD)
        {
            if(tPoint.x - fVehTargetX > -TARGET_X_ERROR)
            {
                fLineLen = MIN_DRIVING_DIS;
            }
            else
            {
                fLineLen = MAX(fabsf(tPoint.x - fVehTargetX), MIN_DRIVING_DIS);
            }
        }
        else
        {
            if(tPoint.x - fVehTargetX < TARGET_X_ERROR)
            {
                fLineLen = MIN_DRIVING_DIS;
            }
            else
            {
                fLineLen = MAX(fabsf(tPoint.x - fVehTargetX), MIN_DRIVING_DIS);
            }
        }
        bLastDriving = true;
    }
    else
    {
        /*计算回旋线需要的方向盘转角*/
        bClothoidAdjust = CalClothoidCurvature(fXRearMargin,
                                               fXFrontMargin,
                                               tPoint,
                                               fRadius,
                                               nDrivingDirection,
                                               fFindCurvature);
        if (!bClothoidAdjust)
        {
            fFindCurvature = 1 / fRadius;
            fCircleLen = fRadius * fabsf(tPoint.yaw);
            fClothoidLen = 0.0;
            fLineLen = 0.0;
            bLastDriving = false;
        }
        else
        {
            bLastDriving = CalHybridCurveLength(fXRearMargin,
                                                fXFrontMargin,
                                                tPoint,
                                                fVehTargetX,
                                                nDrivingDirection,
                                                fFindCurvature,
                                                fCircleLen,
                                                fClothoidLen,
                                                fLineLen);
        }
    }

    /*圆弧回旋线和直线的拼接*/
    if(fabsf(fCircleLen) > CAL_ERROR && (fabsf(fClothoidLen) < CAL_ERROR))
    {
        bCircleAjust = true;
    }
    else
    {
        bCircleAjust = false;
    }
    /*固定方向盘转向*/
    if(bCircleAjust)
    {
        fCircleLen = MAX(MIN_DRIVING_DIS, fCircleLen);
        CircleInterpolate(nDrivingDirection,
                          nSteeringDirection,
                          fFindCurvature,
                          fCircleLen,
                          tPoint,
                          targetSeg);
    }
    else
    {
        ClothoidPathSplicing(bLastDriving,
                             fXRearMargin,
                             fXFrontMargin,
                             tPoint,
                             fFindCurvature,
                             nSteeringDirection,
                             nDrivingDirection,
                             fCircleLen,
                             fClothoidLen,
                             fLineLen,
                             fVehTargetX,
                             targetSeg);
    }
    return bLastDriving;

}

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
void ParallelSlotPathPlanning::ClothoidPathSplicing(bool bLastDriving,
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
                                                    std::vector<CtrlPoint> &TargetSeg)
{

    int num = 0;
    float fLength = 0.0;
    float fStepSize = Constants::step_size;
    float fLastx = 0.0;
    float fLasty = 0.0;
    float fLastyaw = 0.0;
    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float k = 0.0;
    float kmax = 0.0;
    float s = 0.0;
    float dk = 0.0;
    LocationPoint tempPoint;
    LocationPoint tE1, tF1,tE2, tF2,  tG, tH;
    CtrlPoint CtrlPoint_;

    TargetSeg.clear();
    fLastx= tPoint.x;
    fLasty = tPoint.y;
    fLastyaw = tPoint.yaw;
    fLength = fCircleLen + fClothoidLen + fLineLen;
    num =  MAX(int(fLength / fStepSize), 2);
    fStepSize = fLength / num;

    if(fabsf(fFindCurvature) > CAL_ERROR)
    {
        kmax = fFindCurvature;
        dk = CLOTHOID_CURVATURE_FACTOR ;
    }
    else
    {
        kmax = 0;
        dk = 0;
    }

    CtrlPoint_.x = fLastx;
    CtrlPoint_.y = fLasty;
    if(nDrivingDirection == FORWARD)
    {
        CtrlPoint_.yaw = fLastyaw;
    }
    else
    {
        CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(fLastyaw + PI);
    }
    CtrlPoint_.kappa = nDrivingDirection * nSteeringDirection * kmax;

    CtrlPoint_.driving_direction = nDrivingDirection;
    TargetSeg.push_back(CtrlPoint_);

    for(int i = 1; i <= num; ++i)
    {
        s= i * fStepSize;

        if(s < fCircleLen)
        {
            k = kmax;
        }
        else if(s < fCircleLen + fClothoidLen)
        {
            k = kmax- (s - fCircleLen) * dk;
        }
        else
        {
            k = 0.0;
        }

        x = fLastx + nDrivingDirection * fStepSize * cosf(fLastyaw);
        y = fLasty + nDrivingDirection * fStepSize * sinf(fLastyaw);
        yaw = fLastyaw + nDrivingDirection * nSteeringDirection * fStepSize * k;
        fLastx = x;
        fLasty = y;
        fLastyaw  = yaw;

        CtrlPoint_.x = x;
        CtrlPoint_.y = y;
        if(nDrivingDirection == FORWARD)
        {
            CtrlPoint_.yaw = yaw;
        }
        else
        {
            CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(yaw + PI);
        }
        CtrlPoint_.kappa = nDrivingDirection * nSteeringDirection * k;

        CtrlPoint_.driving_direction = nDrivingDirection;
        TargetSeg.push_back(CtrlPoint_);

        tempPoint.x = x;
        tempPoint.y = y;
        tempPoint.yaw = yaw;
        CalCornerCoordinate2(tempPoint, tE1, tE2, tF1, tF2, tG, tH);

        if(((nDrivingDirection == BACKWARD) && (tH.x < fXRearMargin))
           ||((nDrivingDirection == FORWARD) && (MAX(tF1.x, tF2.x) > fXFrontMargin))
           ||((nDrivingDirection == FORWARD) && (tempPoint.x > fVehTargetX) && bLastDriving)
           ||((nDrivingDirection == BACKWARD) && (tempPoint.x < fVehTargetX) && bLastDriving))
        {
            break;
        }
    }
}

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
 * @param stopflag 1-最后一次规划 0-不是最后一次规划
 * @param TargetSegSet 轨迹结果
 * @return 1-规划成功; 0-规划失败
 * @note 平行泊车库内轨迹计算模块，采用的坐标系左侧和右侧相反
 */
bool ParallelSlotPathPlanning::PathPlanningInParallelSlot(int8_t nSlotPosition,
                                                          float fXRearMargin,
                                                          float fXFrontMargin,
                                                          float fYRightMargin,
                                                          LocationPoint tPoint,
                                                          float VehTargetX,
                                                          float VehTargety,
                                                          int8_t NextDrivingDirection,
                                                          std::vector<std::vector<CtrlPoint> > &TargetSegSet)
{
    bool bTempLastDriving = false;
    int nTimes = 0;
    bool bPlanningSucess = false;/*0-规划结束 1-规划成功 2 -规划失败*/
    bool bTempPlanningSucess = false;/*0-规划结束 1-规划成功 2 -规划失败*/

    std::vector<CtrlPoint> TargetSeg;
    LocationPoint tTempPoint = tPoint;

    TargetSegSet.clear();

    while((!bTempLastDriving) && (nTimes < 100))
    {
        /*循环调用单步库内规划，直到最后一次规划完成*/
        bTempPlanningSucess =  PathPlanningInParallelSlotOneStep(fXRearMargin,
                                                                 fXFrontMargin,
                                                                 fYRightMargin,
                                                                 tTempPoint,
                                                                 VehTargetX,
                                                                 VehTargety,
                                                                 NextDrivingDirection,
                                                                 bTempLastDriving,
                                                                 TargetSeg);
        if(nTimes == 0)
        {
            bPlanningSucess = bTempPlanningSucess;
        }
        if(!bTempPlanningSucess)
        {
            break;
        }

        if(!TargetSeg.empty())
        {
            /*将本段轨迹的终点作为起点继续规划*/
            NextDrivingDirection = -NextDrivingDirection;
            tTempPoint.x = TargetSeg.back().x;
            tTempPoint.y = TargetSeg.back().y;
            if(TargetSeg.back().driving_direction == BACKWARD)
            {
                tTempPoint.yaw = normalizeHeadingRad_Npi_Ppi(TargetSeg.back().yaw + PI);
            }
            else
            {
                tTempPoint.yaw = normalizeHeadingRad_Npi_Ppi(TargetSeg.back().yaw);
            }
            /*翻转左侧坐标系*/
            if(nSlotPosition == LEFTSIDESLOT)
            {
                for(size_t i = 0; i < TargetSeg.size(); i++)
                {
                    TargetSeg[i].y = -TargetSeg[i].y;
                    TargetSeg[i].yaw = normalizeHeadingRad_Npi_Ppi(-TargetSeg[i].yaw);
                    TargetSeg[i].kappa = -TargetSeg[i].kappa;
                }
            }

            if(TargetSeg.size() < 2)
            {
                bPlanningSucess = false;
            }
            TargetSegSet.push_back(TargetSeg);
        }
        ++nTimes;
    }
    return bPlanningSucess;
}

/**
 * @brief 平行泊车库内规划主函数
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param fYRightMargin 右边界
 * @param tPoint 当前车辆的坐标
 * @param VehTargetX 目标停车点x
 * @param VehTargety 目标停车点y
 * @param NextDrivingDirection 下次行驶方向
 * @param stopflag 1-最后一次规划 0-不是最后一次规划
 * @param TargetSegSet 轨迹结果
 * @return 1-规划成功; 0-规划失败
 * @note 平行泊车库内轨迹计算模块，采用的坐标系左侧和右侧相反
 */
bool ParallelSlotPathPlanning::PathPlanningInParallelSlotOneStep(float fXRearMargin,
                                                                 float fXFrontMargin,
                                                                 float fYRightMargin,
                                                                 LocationPoint tPoint,
                                                                 float VehTargetX,
                                                                 float VehTargety,
                                                                 int8_t NextDrivingDirection,
                                                                 bool &bLastDriving,
                                                                 std::vector<CtrlPoint> &TargetSeg)
{
    bool bPlanningSucess = false;
    bool bAdjustToZero = false;
    bool bCircleAdjust = false;

    float XVirtualFrontMargin = 0.0;
    float XVirtualRearMargin = 0.0;
    float fLastDrivingStopY = 0.0;
    float DrivingDis = 0.0;
    float kappa = 0.0;
    float fVehStopy = 0.0;
    float fCalRadius = Constants::min_steering_radius;
    int8_t nSteeringDirection = STEERING_LEFT;
    std::vector<float> NextDrivingDisSets;

    bLastDriving = false;
    TargetSeg.clear();

    if(tPoint.yaw >=0)
    {
        /*根据偏航角偏航角大于0,下一次行驶的方向盘转动方向是*/
        nSteeringDirection = -NextDrivingDirection;
    }
    else
    {
        nSteeringDirection = NextDrivingDirection;
    }

    if(tPoint.yaw >= 0)
    {
        /*以固定方向盘转角筛选最靠近目标点Y坐标的前后边界值*/
        bPlanningSucess = NewPlanningInSlot(fXRearMargin,
                                            fXFrontMargin,
                                            fYRightMargin,
                                            tPoint,
                                            VehTargetX,
                                            VehTargety,
                                            NextDrivingDirection,
                                            fCalRadius,
                                            XVirtualFrontMargin,
                                            XVirtualRearMargin,
                                            NextDrivingDisSets,
                                            fVehStopy);

        if(bPlanningSucess)
        {
            /*判断车辆能否一次打平*/
            bAdjustToZero = CheckIfAdjustToZero(fXRearMargin,
                                                fXFrontMargin,
                                                fYRightMargin,
                                                tPoint,
                                                fCalRadius,
                                                NextDrivingDirection,
                                                fLastDrivingStopY);

            if(bAdjustToZero)
            {
                 /*如果车辆能一次打平，判断打平后的车辆最终停车位置Y和计算的最佳停车位置是否误差在10cm以内*/
                if(fabsf(fVehStopy - fLastDrivingStopY) < TARGET_Y_ERROR)
                {
                    /*如果是，进入回旋线调整函数*/
                    bCircleAdjust = false;
                }
                else
                {
                    /*如果不是，进行圆弧调整*/
                    bCircleAdjust = true;
                }
            }
            else
            {
                /*如果不能一次打平，进行圆弧调整*/
                bCircleAdjust = true;
            }

            if(bCircleAdjust)
            {
                if(!NextDrivingDisSets.empty())
                {
                    DrivingDis = MAX(MIN_DRIVING_DIS, NextDrivingDisSets[0]);

                    /*计算曲率*/
                    kappa = 1 / fCalRadius;

                    /*定曲率轨迹生成*/
                    CircleInterpolate(NextDrivingDirection,
                                      nSteeringDirection,
                                      kappa,
                                      DrivingDis,
                                      tPoint,
                                      TargetSeg);
                }
            }
            else
            {
                /*进入回旋线调整函数,并判断是否是最后一次泊车*/
                bLastDriving = ClothoidPathInSlot(fXRearMargin,
                                                  fXFrontMargin,
                                                  tPoint,
                                                  VehTargetX,
                                                  NextDrivingDirection,
                                                  fCalRadius,
                                                  TargetSeg);
            }

        }
    }
    else
    {
        /*进入回旋线调整函数,并判断是否是最后一次泊车*/
        bLastDriving = ClothoidPathInSlot(fXRearMargin,
                                          fXFrontMargin,
                                          tPoint,
                                          VehTargetX,
                                          NextDrivingDirection,
                                          fCalRadius,
                                          TargetSeg);
        bPlanningSucess = true;
    }

    return bPlanningSucess;

}
/******PART II-车辆入库后的重新规划END********/

/******PART III-平行泊车车辆出库规划Start********/

/**
 * @brief 判断是不是可以一次出库,以及计算一次出库的位置
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param fYRightMargin 右边界
 * @param tPoint 当前点
 * @param OneTimeParkingOutPos 能够一次出库的位姿态
 * @return 1-可以一次出库 0-不可以一次出库
 */
bool ParallelSlotPathPlanning::CalOneTimeParkingOutPos(float fXRearMargin,
                                                       float fXFrontMargin,
                                                       float fYRightMargin,
                                                       LocationPoint tPoint,
                                                       float fRadius,
                                                       LocationPoint &OneTimeParkingOutPos)
{
    float fFRadius = 0.0;
    bool bValid = false;

    fFRadius = hypotf(fRadius + Constants::width / 2, Constants::length - Constants::RearEdgeToCenter);

    OneTimeParkingOutPos.x = fXFrontMargin - sqrt(pow(fFRadius + Constants::para_slot_corner_safety_dis, 2) - pow(tPoint.y + fRadius, 2));

    if(OneTimeParkingOutPos.x - Constants::RearEdgeToCenter > fXRearMargin)
    {
        bValid = true;
    }
    else
    {
        bValid = false;
    }
    return bValid;

}


/**
 * @brief 计算不同车辆停车位置的调整次数最少的较好的出库点
 * @param NextDrivingDirection 下一次行驶方向
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param fYRightMargin 右边界
 * @param tPoint 当前点
 * @param bSlotSizeSuccess 1-车位尺寸调整成功 0-车位尺寸调整失败
 * @param DrivingSet 行驶距离集合
 */
void ParallelSlotPathPlanning::CalBestParkingOutPathParkingOut(int NextDrivingDirection,
                                                     float fXRearMargin,
                                                     float fXFrontMargin,
                                                     float fYRightMargin,
                                                     LocationPoint tPoint,
                                                     bool &bSlotSizeSuccess,
                                                     std::vector <float> &DrivingSet)
{
    bool bBackForwardProcessSuccess = false;
    int nVehRearEdgToSlotRearMargin = 0;
    int nVehFrontEdgToSlotFrontMargin = 0;
    ulong nDrivingTimes = 0;
    ulong nLastDrivingTimes = NMAX_LIMIT;
    ulong nCurrentDrivingTimes = 0;
    size_t nIndex = 0;

    float XFrontMarginNeed = 0.0;
    float XRearMarginNeed = 0.0;
    float fVehRearEdg = 0.0;
    float fVehFrontEdg = 0.0;
    float fIncrement = MARGIN_INCREMENT;
    float fRadius = Constants::min_steering_radius;

    LocationPoint tE1, tE2, tF1, tF2, tG, tH;
    LocationPoint tOutputStep3KeyPoint;
    std::vector <float> DrivingDisSet;
    SlotEdgInfor SlotEdgInfor_;
    std::vector<SlotEdgInfor> SlotEdgInforSets;

    DrivingDisSet.clear();

    /*计算车辆的前角点X最大值和后角点的X最小值*/
    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
    fVehRearEdg = MIN(MIN(tE1.x, tE2.x), tH.x);
    fVehFrontEdg = MAX(MAX(tF1.x, tF2.x), tG.x);

    /*这里对前后边界和右边界进行更新*/
    fYRightMargin = MIN(fYRightMargin, MIN(tE1.y, tE2.y));
    fXFrontMargin = MAX(fXFrontMargin, MAX(tF1.x, tF2.x));
    fXRearMargin = MIN(fXRearMargin, tH.x);

    /*边界的取值数目*/
    nVehRearEdgToSlotRearMargin = MAX(int((fVehRearEdg - fXRearMargin)/fIncrement)-1,0);
    nVehFrontEdgToSlotFrontMargin = MAX(int((fXFrontMargin - fVehFrontEdg)/fIncrement)-1,0);

    for(int i = 0; i <= nVehFrontEdgToSlotFrontMargin; i++)
    {
        for(int j = 0; j <= nVehRearEdgToSlotRearMargin; j++)
        {
            /*在一定前后边界下，计算车辆在车库内调整的最终停车点*/
            XFrontMarginNeed = fXFrontMargin - i*fIncrement;
            XRearMarginNeed = fXRearMargin + j*fIncrement;
            CalParkingOutPosInfixedInitialPoint(NextDrivingDirection,
                                                XRearMarginNeed,
                                                fYRightMargin,
                                                XFrontMarginNeed,
                                                0,
                                                tPoint,
                                                fRadius,
                                                bBackForwardProcessSuccess,
                                                nDrivingTimes,
                                                tOutputStep3KeyPoint,
                                                DrivingDisSet);
            if(bBackForwardProcessSuccess)
            {
                SlotEdgInfor_.fFrontEdg = XFrontMarginNeed;
                SlotEdgInfor_.fRearEdg = XRearMarginNeed;
                SlotEdgInfor_.nAdjustTimes = nDrivingTimes;
                SlotEdgInfor_.tPoint = tOutputStep3KeyPoint;
                SlotEdgInfor_.NextDrivingDisSets = DrivingDisSet;
                SlotEdgInforSets.push_back(SlotEdgInfor_);
            }
        }
    }

    if(SlotEdgInforSets.size() == 0)
    {
        bSlotSizeSuccess = false;
    }
    else
    {
        bSlotSizeSuccess = true;
        /*3 筛选调整次数最少的组合*/
        for(size_t i = 0; i < SlotEdgInforSets.size(); i++)
        {
            nCurrentDrivingTimes = SlotEdgInforSets[i].nAdjustTimes;

            if(nCurrentDrivingTimes < nLastDrivingTimes)
            {
                nLastDrivingTimes = nCurrentDrivingTimes;
                nIndex = i;
            }
            else if(nCurrentDrivingTimes == nLastDrivingTimes )
            {
                if((!SlotEdgInforSets[i].NextDrivingDisSets.empty())
                        && (!SlotEdgInforSets[nIndex].NextDrivingDisSets.empty())
                        && SlotEdgInforSets[i].tPoint.yaw < SlotEdgInforSets[nIndex].tPoint.yaw)
                {
                    nIndex = i;
                }
            }else{}
        }
        DrivingSet = SlotEdgInforSets[nIndex].NextDrivingDisSets;
    }
}


/**
 * @brief 出库检查
 * @param fRadius 出库转弯半径
 * @param fXC C点X坐标
 * @param fYC C点Y坐标
 * @param tStep3KeyPoint 车辆出库位姿
 * @return 0-可以出库 1-不能出库
 */
bool ParallelSlotPathPlanning::CheckIfParkingOut(float fRadius,
                                                 float fXC,
                                                 float fYC,
                                                 LocationPoint tStep3KeyPoint)
{
    float fLengthOC = 0.0;
    float fE1Radius = 0.0;
    float fE2Radius = 0.0;
    float fF1Radius = 0.0;
    float fF2Radius = 0.0;
    float fFRadius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    int8_t nSteeringDirection = 0;
    bool bSucc = false;
    LocationPoint tRotationCenter;

    nSteeringDirection = STEERING_LEFT;
    /*计算旋转中心*/
    CalRotationCenterCoordinate(1,
                                nSteeringDirection,
                                tStep3KeyPoint,
                                fRadius,
                                tRotationCenter);
    /*计算角点转弯半径*/
    CalCornerPointRadius2(nSteeringDirection,
                          fRadius,
                          fE1Radius,
                          fE2Radius,
                          fF1Radius,
                          fF2Radius,
                          fGRadius,
                          fHRadius);
    fLengthOC = hypotf(fXC - tRotationCenter.x, fYC - tRotationCenter.y);

    fFRadius = MAX(fF1Radius, fF2Radius);
    /*判断旋转中心到C点距离和车辆F点转弯半径,是否可以出库*/
    if (fLengthOC > fFRadius + Constants::para_slot_corner_safety_dis)
    {
        bSucc = true;
    }
    else
    {
        bSucc = false;
    }
    return bSucc;
}

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
void ParallelSlotPathPlanning::CircleClothoidLinePathSplicing(LocationPoint tPoint,
                                                              int8_t nSteeringDirection,
                                                              int8_t nDrivingDirection,
                                                              float fKappa,
                                                              float fCircleLen,
                                                              float fClothoidLen,
                                                              float fLineLen,
                                                              std::vector<CtrlPoint> &TargetSeg)
{
    int num = 0;
    float fLength = 0.0;
    float fStepSize = Constants::step_size;
    float fLastx = 0.0;
    float fLasty = 0.0;
    float fLastyaw = 0.0;
    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float k = 0.0;
    float kmax = 0.0;
    float s = 0.0;
    float dk = 0.0;
    CtrlPoint CtrlPoint_;

    fLastx= tPoint.x;
    fLasty = tPoint.y;
    fLastyaw = tPoint.yaw;

    TargetSeg.clear();

    fLength = fCircleLen + fClothoidLen + fLineLen;
    num =  MAX(int(fLength / fStepSize), 2);
    fStepSize = fLength / num;

    if(fabsf(fKappa) > CAL_ERROR)
    {
        kmax = fKappa;
        dk = fKappa / fClothoidLen;
    }
    else
    {
        kmax = 0;
        dk = 0;
    }

    for(int i = 1; i < num + 1; i++)
    {
        s= i * fStepSize;

        if(s < fCircleLen)
        {
            k = kmax;
        }
        else if(s < fCircleLen + fClothoidLen)
        {
            k = kmax- (s - fCircleLen) * dk;
        }
        else
        {
            k = 0;
        }

        x = fLastx + nDrivingDirection * fStepSize * cosf(fLastyaw);
        y = fLasty + nDrivingDirection * fStepSize * sinf(fLastyaw);
        yaw = fLastyaw + nDrivingDirection * nSteeringDirection * fStepSize * k;
        fLastx = x;
        fLasty = y;
        fLastyaw  = yaw;
        CtrlPoint_.x = x;
        CtrlPoint_.y = y;
        if(nDrivingDirection == FORWARD)
        {
            CtrlPoint_.yaw = yaw;
        }
        else
        {
            CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(yaw + PI);
        }
        CtrlPoint_.kappa = nDrivingDirection * nSteeringDirection * k;

        CtrlPoint_.driving_direction = nDrivingDirection;
        TargetSeg.push_back(CtrlPoint_);
    }
}

/**
 * @brief 生成出库的最后一段圆弧加回旋线路径
 * @param fXFrontMargin 前边界
 * @param tPoint 当前车辆坐标
 * @param SegmentResults 控制点集合
 * @return 0-规划结束 1-规划成功 2 -规划失败
 */
PlanningStatus ParallelSlotPathPlanning::GenerateLastPathInParkingOut(float fXFrontMargin,
                                                                      LocationPoint tPoint,
                                                                      float fRadius,
                                                                      std::vector<CtrlPoint> &SegmentResults)
{
    PlanningStatus nPlanningFlag = planningOver;/*0-规划结束 1-规划成功 2 -规划失败*/
    int8_t nSteeringDirection = STEERING_LEFT;
    int8_t nDrivingDirection = FORWARD;

    float fE1Radius = 0.0;
    float fE2Radius = 0.0;
    float fF1Radius = 0.0;
    float fF2Radius = 0.0;
    float fGRadius = 0.0;
    float fHRadius = 0.0;
    float fYaw1 = 0.0;
    float fYaw2 = 0.0;
    float fXC = fXFrontMargin;
    float fYC = 0;
    float fCircleLen = 0.0;
    float fClothoidLen = 0.5;
    float fLineLen  = 0.0;

    LocationPoint tE1, tE2, tF1, tF2, tG, tH, tF;
    LocationPoint tRotationCenter;

    /*计算旋转中心*/
    CalRotationCenterCoordinate(1,
                                nSteeringDirection,
                                tPoint,
                                fRadius,
                                tRotationCenter);
    /*计算角点*/
    CalCornerCoordinate2(tPoint, tE1, tE2, tF1, tF2, tG, tH);
    /*计算角点转弯半径*/
    CalCornerPointRadius2(nSteeringDirection,
                          fRadius,
                          fE1Radius,
                          fE2Radius,
                          fF1Radius,
                          fF2Radius,
                          fGRadius,
                          fHRadius);
    if(fF1Radius > fF2Radius)
    {
        tF = tF1;
    }
    else
    {
        tF = tF2;
    }

    fYaw1 = atanf((tF.x - tRotationCenter.x) / (tRotationCenter.y - tF.y));
    fYaw2 = atanf((fXC - tRotationCenter.x) / (tRotationCenter.y - fYC));
    if(fYaw1 < fYaw2)
    {
        /*此时车辆角点还没有到达距离车位C点最近的位置*/
        fCircleLen = fabsf(fYaw2 - fYaw1) * fRadius;
        fCircleLen = MAX(MIN_LAST_PARKING_OUT_LENGTH, fCircleLen);

        CircleClothoidLinePathSplicing(tPoint,
                                       nSteeringDirection,
                                       nDrivingDirection,
                                       1 / fRadius,
                                       fCircleLen,
                                       fClothoidLen,
                                       fLineLen,
                                       SegmentResults);
        nPlanningFlag = PlanningSucc;
    }
    else
    {
        /*此时车辆角点已经越过距离车位C点最近的位置, 此时不需要规划*/
        nPlanningFlag = planningOver;
    }
    return nPlanningFlag;

}

/**
 * @brief 计算第一次泊出的行驶方向
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param fYRightMargin 右边界
 * @param tPoint 当前车辆坐标
 * @param NextDrivingDirection 下一次行驶方向
 * @return 0-规划结束 1-规划成功 2 -规划失败
 */
PlanningStatus ParallelSlotPathPlanning::CalFirstPathDirectionParaParkingOut(float fXRearMargin,
                                                                             float fXFrontMargin,
                                                                             float fYRightMargin,
                                                                             LocationPoint tPoint,
                                                                             int8_t &NextDrivingDirection)
{
    bool bSlotSizeSuccess = false;
    bool bDrivingForwadValid = false;
    bool bDrivingBackValid = false;
    PlanningStatus nPlanningFlag = planningOver;/*0-规划结束 1-规划成功 2 -规划失败*/
    ulong nDrivingForwadTimes = 0;
    ulong nDrivingBackTimes = 0;

    std::vector<float> DrivingDisSet;
    std::vector<float> FowardDrivingDisSet;
    std::vector<float> BackDrivingDisSet;

    /*进行方向盘打死泊出，首先确定行驶的方向是前进还是后退*/
    for(int i = 0; i < 2 ; i++)
    {
        if(i == 0)
        {
            NextDrivingDirection = FORWARD;
        }
        else
        {
            NextDrivingDirection = BACKWARD;
        }
        CalBestParkingOutPathParkingOut(NextDrivingDirection,
                                        fXRearMargin,
                                        fXFrontMargin,
                                        fYRightMargin,
                                        tPoint,
                                        bSlotSizeSuccess,
                                        DrivingDisSet);
        if(i == 0)
        {
            nDrivingForwadTimes = DrivingDisSet.size();
            bDrivingForwadValid = bSlotSizeSuccess;
            FowardDrivingDisSet = DrivingDisSet;
        }
        else
        {
            nDrivingBackTimes = DrivingDisSet.size();
            bDrivingBackValid = bSlotSizeSuccess;
            BackDrivingDisSet = DrivingDisSet;
        }
    }



    if(bDrivingForwadValid)
    {
        if(bDrivingBackValid)
        {
            if((!FowardDrivingDisSet.empty()) && (!BackDrivingDisSet.empty()))
            {
                if(FowardDrivingDisSet[0] < MIN_DRIVING_DIS)
                {
                    NextDrivingDirection = BACKWARD;
                }
                else
                {
                    if(BackDrivingDisSet[0] < MIN_DRIVING_DIS)
                    {
                        NextDrivingDirection = FORWARD;
                    }
                    else
                    {
                        if(nDrivingBackTimes <= nDrivingForwadTimes)
                        {
                            NextDrivingDirection = BACKWARD;
                        }
                        else
                        {
                            NextDrivingDirection = FORWARD;
                        }
                    }
                }

            }
        }
        else
        {
            if(!FowardDrivingDisSet.empty())
            {
                NextDrivingDirection = FORWARD;
            }
        }
    }
    else
    {
        if(bDrivingBackValid)
        {
            if(!BackDrivingDisSet.empty())
            {
                NextDrivingDirection = BACKWARD;
            }
        }
    }

    if((bDrivingForwadValid && (!FowardDrivingDisSet.empty()))
      || (bDrivingBackValid && (!BackDrivingDisSet.empty())))
    {
        nPlanningFlag = PlanningSucc;
    }
    else
    {
        nPlanningFlag = PlanningFail;
    }
    return nPlanningFlag;

}

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
PlanningStatus ParallelSlotPathPlanning::ParkingOutPathPlanningOneStep(float fXRearMargin,
                                                                       float fXFrontMargin,
                                                                       float fYRightMargin,
                                                                       LocationPoint tPoint,
                                                                       int8_t &NextDrivingDirection,
                                                                       bool bFirstPlan,
                                                                       bool &bLastDriving,
                                                                       std::vector<CtrlPoint> &TargetSeg)
{
    PlanningStatus nPlanningFlag = planningOver;/*0-规划结束 1-规划成功 2 -规划失败*/
    int8_t nSteeringDirection = 0;

    float fDrivingDis = 0.0;
    float fMinRadius = Constants::min_steering_radius;
    float kappa = 1 / fMinRadius;
    float fXC = fXFrontMargin;
    float fYC = 0;
    bool bOneTimeParkingOutValid = false;
    bool bSteering = false;
    bool bParkingOutCheck = false;
    bool bSlotSizeSuccess = false;

    LocationPoint OneTimeParkingOutPos;
    std::vector<float> DrivingDisSet;
    TargetSeg.clear();

    if(bFirstPlan)
    {
        /*判断是不是可以一次出库,以及计算一次出库的位置*/
        bOneTimeParkingOutValid = CalOneTimeParkingOutPos(fXRearMargin,
                                                          fXFrontMargin,
                                                          fYRightMargin,
                                                          tPoint,
                                                          fMinRadius,
                                                          OneTimeParkingOutPos);
        if(bOneTimeParkingOutValid)
        {
            /*第一次泊出路径规划可以一次出库*/
            if(tPoint.x  < OneTimeParkingOutPos.x)
            {
                nPlanningFlag = planningOver;
                return nPlanningFlag;
            }
            else
            {
                fDrivingDis = fabsf(tPoint.x - OneTimeParkingOutPos.x);
                nPlanningFlag = PlanningSucc;
                kappa = 0;
                NextDrivingDirection = BACKWARD;
            }
        }
        else
        {
            nPlanningFlag = CalFirstPathDirectionParaParkingOut(fXRearMargin,
                                                                fXFrontMargin,
                                                                fYRightMargin,
                                                                tPoint,
                                                                NextDrivingDirection);
            /*第一次泊出路径规划不可以一次出库,第一次路径规划方向盘打死行驶*/
            bSteering = true;
        }
    }
    else
    {
        /*不是第一次路径规划方向盘都打死行驶*/
        bSteering = true;
    }


    if(bSteering)
    {
        /*方向盘必须转向时*/
        if(NextDrivingDirection == FORWARD)
        {
            /*下次行驶方向是前进，方向盘左转病判断车辆是否能够出库*/
            nSteeringDirection = STEERING_LEFT;
            bParkingOutCheck = CheckIfParkingOut(fMinRadius, fXC, fYC, tPoint);
        }
        else
        {
            nSteeringDirection = STEERING_RIGHT;
        }

        if(!bParkingOutCheck)
        {
            /*下次行驶车辆无法出库，那么需要规划下次行驶轨迹*/
            CalBestParkingOutPathParkingOut(NextDrivingDirection,
                                            fXRearMargin,
                                            fXFrontMargin,
                                            fYRightMargin,
                                            tPoint,
                                            bSlotSizeSuccess,
                                            DrivingDisSet);
            if(bSlotSizeSuccess && (!DrivingDisSet.empty()))
            {
                fDrivingDis = DrivingDisSet[0];
                nPlanningFlag = PlanningSucc;
            }
            else
            {
                nPlanningFlag = PlanningFail;
            }
        }
        else
        {
            nPlanningFlag = PlanningSucc;
            bLastDriving = true;
        }
    }else{}

    if(!bLastDriving)
    {
        /*不是最后一段轨迹，圆弧插值生成轨迹*/
        if(nPlanningFlag == PlanningSucc)
        {
            /**圆弧插值*/
            fDrivingDis = MAX(MIN_DRIVING_DIS, fDrivingDis);
            CircleInterpolate(NextDrivingDirection,
                              nSteeringDirection,
                              kappa,
                              fDrivingDis,
                              tPoint,
                              TargetSeg);
        }
    }
    else
    {
        /*是最后一段轨迹，圆弧加回旋线生成轨迹*/
//        nPlanningFlag = GenerateLastPathInParkingOut(fXFrontMargin,
//                                                     tPoint,
//                                                     fMinRadius,
//                                                     TargetSeg);
        ClothoidParkout(tPoint, fXFrontMargin, 0.0f, fXRearMargin, 0.0f, TargetSeg);
        if (!TargetSeg.empty())
        {
            nPlanningFlag = PlanningSucc;
        }
    }
    return nPlanningFlag;
}

/**
 * @brief 平行泊出库内轨迹规划
 * @param nSlotPosition 车位的方向,1-右侧车位 -1-左侧车位
 * @param fXRearMargin 后边界
 * @param fXFrontMargin 前边界
 * @param fYRightMargin 右边界
 * @param tPoint 当前车辆坐标
 * @param NextDrivingDirection 下一次行驶方向
 * @param bFirstPlan 是否第一次规划
 * @param TargetSegSet 目标控制点集合
 * @return 0-规划结束 1-规划成功 2 -规划失败
 */
PlanningStatus ParallelSlotPathPlanning::ParkingOutInParallelSlot(int8_t nSlotPosition,
                                                                  float fXRearMargin,
                                                                  float fXFrontMargin,
                                                                  float fYRightMargin,
                                                                  LocationPoint tPoint,
                                                                  int8_t NextDrivingDirection,
                                                                  bool bFirstPlan,
                                                                  std::vector<std::vector<CtrlPoint> > &TargetSegSet)
{
    bool bTempLastDriving = false;
    int nTimes = 0;
    PlanningStatus nPlanningFlag = planningOver;/*0-规划结束 1-规划成功 2 -规划失败*/
    PlanningStatus tempPlanningFlag = planningOver;/*0-规划结束 1-规划成功 2 -规划失败*/

    std::vector<CtrlPoint> TargetSeg;
    LocationPoint tTempPoint = tPoint;

    TargetSegSet.clear();

    while((!bTempLastDriving) && (nTimes < 100))
    {
        /*循环调用单步库内规划，直到最后一次规划完成*/
        tempPlanningFlag = ParkingOutPathPlanningOneStep(fXRearMargin,
                                                         fXFrontMargin,
                                                         fYRightMargin,
                                                         tTempPoint,
                                                         NextDrivingDirection,
                                                         bFirstPlan,
                                                         bTempLastDriving,
                                                         TargetSeg);
        if(nTimes == 0)
        {
            nPlanningFlag = tempPlanningFlag;
        }
        if((tempPlanningFlag == PlanningFail) || (tempPlanningFlag == planningOver))
        {
            break;
        }

        if(!TargetSeg.empty())
        {
            /*将本段轨迹的终点作为起点继续规划*/
            bFirstPlan = false;
            NextDrivingDirection = -NextDrivingDirection;
            tTempPoint.x = TargetSeg.back().x;
            tTempPoint.y = TargetSeg.back().y;
            if(TargetSeg.back().driving_direction == BACKWARD)
            {
                tTempPoint.yaw = normalizeHeadingRad_Npi_Ppi(TargetSeg.back().yaw + PI);
            }
            else
            {
                tTempPoint.yaw = normalizeHeadingRad_Npi_Ppi(TargetSeg.back().yaw);
            }
            /*翻转左侧坐标系*/
            if(nSlotPosition == LEFTSIDESLOT)
            {
                for(size_t i = 0; i < TargetSeg.size(); i++)
                {
                    TargetSeg[i].y = -TargetSeg[i].y;
                    TargetSeg[i].yaw = normalizeHeadingRad_Npi_Ppi(-TargetSeg[i].yaw);
                    TargetSeg[i].kappa = -TargetSeg[i].kappa;
                }
            }

            if(TargetSeg.size() < 2)
            {
                nPlanningFlag = PlanningFail;
            }
            TargetSegSet.push_back(TargetSeg);
        }
        ++nTimes;
    }
    return nPlanningFlag;
}


/******PART III-平行泊车车辆出库规划End********/

/**
 * @brief 计算角点转弯半径
 */
int ParallelSlotPathPlanning::CalCornerPointRadius2(int8_t nSteeringDirection,
                                                    float fRearAxleCenterRadius,
                                                    float &fRE1,
                                                    float &fRE2,
                                                    float &fRF1,
                                                    float &fRF2,
                                                    float &fRG,
                                                    float &fRH)
{
    int nRet = -1;

    if (((nSteeringDirection != STEERING_LEFT) && (nSteeringDirection != STEERING_RIGHT))
         ||(fRearAxleCenterRadius <= 0) )
    {
        ;
    }
    else
    {
        fRE1 = hypotf(fRearAxleCenterRadius + nSteeringDirection * (Constants::width / 2 - Constants::rear_chamfer_l1), Constants::RearEdgeToCenter);
        fRE2 = hypotf(fRearAxleCenterRadius + nSteeringDirection * Constants::width / 2, Constants::RearEdgeToCenter - Constants::rear_chamfer_l2);

        fRF1 = hypotf(fRearAxleCenterRadius + nSteeringDirection * (Constants::width / 2 - Constants::chamfer_length), Constants::FrontEdgeToCenter);
        fRF2 = hypotf(fRearAxleCenterRadius + nSteeringDirection * Constants::width / 2, Constants::FrontEdgeToCenter - Constants::chamfer_length);

        fRG = hypotf(fRearAxleCenterRadius - nSteeringDirection * Constants::width / 2, Constants::length - Constants::RearEdgeToCenter);
        fRH = hypotf(fRearAxleCenterRadius - nSteeringDirection * Constants::width / 2, Constants::RearEdgeToCenter);

        nRet = 0;
    }
    return nRet;
}

/**
 * @brief 计算角点坐标
 */
void ParallelSlotPathPlanning::CalCornerCoordinate2(LocationPoint tVeh,
                                                    LocationPoint& tE1,
                                                    LocationPoint& tE2,
                                                    LocationPoint& tF1,
                                                    LocationPoint& tF2,
                                                    LocationPoint& tG,
                                                    LocationPoint& tH)
{
    LocationPoint tE1Opposite;
    LocationPoint tE2Opposite;
    LocationPoint tF1Opposite;
    LocationPoint tF2Opposite;
    LocationPoint tGOpposite;
    LocationPoint tHOpposite;
    LocationPoint tOriginPoint(0, 0, 0);

    tE1Opposite.x = -Constants::RearEdgeToCenter;
    tE1Opposite.y = -Constants::width / 2 + Constants::rear_chamfer_l1;

    tE2Opposite.x = -Constants::RearEdgeToCenter + Constants::rear_chamfer_l2;
    tE2Opposite.y = -Constants::width / 2;

    tF1Opposite.x = Constants::FrontEdgeToCenter;
    tF1Opposite.y = -(Constants::width / 2 - Constants::chamfer_length);

    tF2Opposite.x = Constants::FrontEdgeToCenter - Constants::chamfer_length;
    tF2Opposite.y = -Constants::width / 2;

    tGOpposite.x = Constants::FrontEdgeToCenter;
    tGOpposite.y = Constants::width / 2;

    tHOpposite.x = -Constants::RearEdgeToCenter;
    tHOpposite.y = Constants::width / 2;

    RotateCoordinateOfPoint(tOriginPoint, tE1Opposite, tVeh.yaw, tE1);
    tE1.x += tVeh.x;
    tE1.y += tVeh.y;

    RotateCoordinateOfPoint(tOriginPoint, tE2Opposite, tVeh.yaw, tE2);
    tE2.x += tVeh.x;
    tE2.y += tVeh.y;

    RotateCoordinateOfPoint(tOriginPoint,tF1Opposite, tVeh.yaw, tF1);
    tF1.x += tVeh.x;
    tF1.y += tVeh.y;

    RotateCoordinateOfPoint(tOriginPoint,tF2Opposite, tVeh.yaw, tF2);
    tF2.x += tVeh.x;
    tF2.y += tVeh.y;

    RotateCoordinateOfPoint(tOriginPoint,tGOpposite, tVeh.yaw, tG);
    tG.x += tVeh.x;
    tG.y += tVeh.y;

    RotateCoordinateOfPoint(tOriginPoint, tHOpposite, tVeh.yaw, tH);
    tH.x += tVeh.x;
    tH.y += tVeh.y;
}

}
