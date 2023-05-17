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
#include"measurestatusintracks.h"
#include "planState.h"


namespace HybridAStarPart
{
bool isPositionUpdateSlotInfo(const slotPosInfor &slotPosInfor_,
                              LocationPoint tCurPos)
{
    bool bRet = false;
    int nSlotType = slotPosInfor_.nSlotType;
    int nParkingType = slotPosInfor_.nParkingType;
    float fSlotCorner2Veh = 0;
    float fGoalYaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.goalPointInMap.yaw);
    LocationPoint OriginPont(0, 0, 0);
    LocationPoint RearCamera(-REAR_EDGE2CENTER, 0, 0);
    
    if (nSlotType == OBLIQUE)
    {
        if(nParkingType ==TAIL_PARK_IN)
        {
            // fSlotCorner2Veh = WIDTH_HALF/cosf(fabs(fabs(tCurPos.yaw) - fabs(fGoalYaw)));
            // if(fabs(fabs(tCurPos.yaw) - fabs(fGoalYaw)) < float(M_PI)/6.f &&
            //         fSlotCorner2Veh + fabs(tCurPos.x - slotPosInfor_.fSlotLength/2) < slotPosInfor_.fSlotLength/2.f)//half of slot width
            // if(fabs(fabs(tCurPos.yaw) - fabs(fGoalYaw)) < float(M_PI)/6.f)
            // {
            //     bRet = true;
            // }
            #if 0
            OriginPont = TransFromWorld2Veh(tCurPos, LocationPoint(0,0,0));
            if(fabs(atan2(RearCamera.y - OriginPont.y, RearCamera.x - OriginPont.x)) < float(M_PI)/4.f
                && OriginPont.x - RearCamera.x < -0.5f
                && OriginPont.x - RearCamera.x > -2.f)
            {
                bRet = true;
            }
            else
            {
                GAC_LOG_INFO("angle %f ", fabs(atan2(RearCamera.y - OriginPont.y, RearCamera.x - OriginPont.x)));
            }
            #endif
            if(fabs(fabs(tCurPos.yaw) - fabs(fGoalYaw)) < float(M_PI)/10.f && tCurPos.y*slotPosInfor_.nSlotSide<-1.25f)
            {
                bRet = true;
            }
        }
    }

    return bRet;
}

/**
 * @brief 遍历整个轨迹，计算视觉需要更新坐标区间状态
 * @param[in] nSlotType 车位类型
 * @param[in] nParkingType 泊车类型
 * @param[in] nSlotSide 车位侧
 * @param[in] tCurPos 坐标
 * @return nUpdatePosition 1,2,3-更新位置
 * @note 只有在视觉泊车最后一段入库轨迹规划结束调用
 */
int StatusCheckWhenLastDriving(const slotPosInfor &slotPosInfor_,
                               LocationPoint tCurPos)
{
    int nUpdatePosition = 0;
    int nSlotType = slotPosInfor_.nSlotType;
    int nParkingType = slotPosInfor_.nParkingType;
    int nSlotSide = slotPosInfor_.nSlotSide;
    LocationPoint tE,tF,tG,tH;
    LocationPoint FrontAxleCenter;
    float fGoalYaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.goalPointInMap.yaw);

    CalCornerCoordinate(tCurPos, tE, tF, tG, tH);

    if (nSlotType == OBLIQUE)
    {
        if(nParkingType ==TAIL_PARK_IN)
        {
            if (nSlotSide == RIGHTSIDESLOT)
            {
                if ((tE.y < Y1_GET_SLOT_INFOR_OBLI)
                        &&(tE.y > Y1_GET_SLOT_INFOR_OBLI-Y_WIDTH_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 1;
                }
                else if((tE.y < Y2_GET_SLOT_INFOR_OBLI)
                        &&(tE.y > Y2_GET_SLOT_INFOR_OBLI-Y_WIDTH_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 2;
                }
                else if((tE.y<Y3_GET_SLOT_INFOR_OBLI)
                        &&(fabsf(fGoalYaw - tCurPos.yaw)<YAW_LIMIT_OBLI))
                {
                    nUpdatePosition = 3;
                }else{}
            }
            else
            {
                if ((tH.y > -Y1_GET_SLOT_INFOR_OBLI)
                        &&(tH.y < -Y1_GET_SLOT_INFOR_OBLI+Y_WIDTH_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 1;
                }
                else if ((tH.y > -Y2_GET_SLOT_INFOR_OBLI)
                         &&(tH.y < -Y2_GET_SLOT_INFOR_OBLI+Y_WIDTH_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 2;
                }
                else if((tH.y > -Y3_GET_SLOT_INFOR_OBLI)
                        &&(fabsf(fGoalYaw - tCurPos.yaw)<YAW_LIMIT_OBLI))
                {
                    nUpdatePosition = 3;
                }else{}
            }

        }
        else
        {
            CalFrontAxleCenterCoordinate(tCurPos, FrontAxleCenter);

            if (nSlotSide== RIGHTSIDESLOT)
            {
                if((tF.y> Y1_GET_SLOT_INFOR_OBLI - Y_WIDTH_GET_SLOT_INFOR_OBLI)
                        &&(tF.y < Y1_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 1;
                }
                else if((tF.y> Y2_GET_SLOT_INFOR_OBLI - Y_WIDTH_GET_SLOT_INFOR_OBLI)
                        &&(tF.y < Y2_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 2;
                }
                else if((tF.y < Y3_GET_SLOT_INFOR_OBLI)
                        &&(fabsf(fGoalYaw - tCurPos.yaw)<YAW_LIMIT_OBLI))
                {
                    nUpdatePosition = 3;
                }else{}
            }
            else
            {
                if((tG.y < -Y1_GET_SLOT_INFOR_OBLI + Y_WIDTH_GET_SLOT_INFOR_OBLI)
                        &&(tG.y > -Y1_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 1;
                }
                else if((tG.y < -Y2_GET_SLOT_INFOR_OBLI + Y_WIDTH_GET_SLOT_INFOR_OBLI)
                        &&(tG.y > -Y2_GET_SLOT_INFOR_OBLI))
                {
                    nUpdatePosition = 2;
                }
                else if((tG.y > -Y3_GET_SLOT_INFOR_OBLI)
                        &&(fabsf(fGoalYaw - tCurPos.yaw)<YAW_LIMIT_OBLI))
                {
                    nUpdatePosition = 3;
                }else{}
            }
        }
    }
    return nUpdatePosition;
}

/**
 * @brief 遍历整个轨迹，计算11和12超声探测的开始结束坐标点
 * @param[in] nSlotSide 车位侧
 * @param[in] fslotWidth 车位宽度
 * @param[in] fslotDepth 车位深度
 * @param[in] aTracks 轨迹点
 * @param[in] U11_start 11超声探测的开始点
 * @param[in] U11_end 11超声探测的结束点
 * @param[in] U12_start 12超声探测的开始点
 * @param[in] U12_end 12超声探测的结束点
 * @return validity 1-计算成功 0-计算失败
 * @note 只有在超声泊车最后一段入库轨迹规划结束调用
 */
bool GetU11AndU12StartAndEndPoint(const slotPosInfor &slotPosInfor_,
                                  std::vector<TargetTrack>& aTracks,
                                  LocationPoint& U11_start,
                                  LocationPoint& U11_end,
                                  LocationPoint& U12_start,
                                  LocationPoint& U12_end)
{

    LocationPoint CurU11;
    LocationPoint CurU11_end;
    LocationPoint U11Inter;
    LocationPoint CurU12;
    LocationPoint CurU12_end;
    LocationPoint U12Inter;
    LocationPoint CurPos;
    LocationPoint Lp0;
    LocationPoint Lp1;
    LocationPoint Rp0;
    LocationPoint Rp1;
    LocationPoint tE;
    LocationPoint tF;
    LocationPoint tG;
    LocationPoint tH;
    LocationPoint startPoint;

    bool bU11Start = false;
    bool bU11End= false;
    bool bU12Start = false;
    bool bU12End= false;
    bool validity = false;
    bool bU11Inter = false;
    bool bU12Inter = false;
    bool bInitCheck = false;

    int8_t nDirection = 0;
    float nDetectDis = 2.0;
    float fXUssPos[12];
    float fYUssPos[12];

    int nSlotSide = slotPosInfor_.nSlotSide;
    float fslotWidth = slotPosInfor_.fSlotLength;
    float fslotDepth = slotPosInfor_.fSlotWidth;
    float fYFrontLine = slotPosInfor_.fYFrontSlotCorner;
    float fYRearLine = slotPosInfor_.fYRearSlotCorner;
    float fUss11StartY = 0.0;
    float fUss11StopY = 0.0;

    float fUss12StartY = 0.0;
    float fUss12StopY = 0.0;
    /* 判断轨迹的第一个点是否在库外 */
    if(aTracks.size() > 0)
    {
        nDirection = SgnCus(aTracks[0].speed);

        startPoint.x = aTracks[0].point.getX();
        startPoint.y = aTracks[0].point.getY();
        CalCornerCoordinate(startPoint, tE, tF, tG, tH);

        if (nSlotSide*tE.y > -1 && nSlotSide*tH.y > -1) /*!!!!*/
        {
            bInitCheck = true;
        }
        else{}
    }
    else{}

   if (bInitCheck)
   {
       /* 左右侧车位，调换11和12号超声探测的车位侧边 */
       if(nSlotSide==RIGHTSIDESLOT)
       {
           Lp0.x = 0;
           Lp0.y = -fslotDepth;
           Lp1.x = 0;
           Lp1.y = fYRearLine;
           Rp0.x = fslotWidth;
           Rp0.y = -fslotDepth;
           Rp1.x = Rp0.x;
           Rp1.y = fYFrontLine;
       }
       else
       {
           Rp0.x = 0;
           Rp0.y = fslotDepth;
           Rp1.x = 0;
           Rp1.y = fYRearLine;
           Lp0.x = fslotWidth;
           Lp0.y = fslotDepth;
           Lp1.x = Lp0.x;
           Lp1.y = fYFrontLine;
       }
       fUss11StartY = Lp1.y;
       fUss11StopY = Lp1.y - nSlotSide * USS_DETECT_DIS;
       fUss12StartY = Rp1.y;
       fUss12StopY = Rp1.y - nSlotSide * USS_DETECT_DIS;

       /* 遍历整个轨迹，计算11和12超声探测的开始结束坐标点 */
       for(size_t i=0; i<aTracks.size(); ++i)
       {
           /*读取坐标点和偏航角归一化*/
           CurPos.x = aTracks[i].point.getX();
           CurPos.y = aTracks[i].point.getY();
           if(nDirection == FORWARD)
           {
               CurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw);
           }
           else
           {
               CurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw + PI);
           }

           /*计算12个超声波的探头位置*/
           CalUSSCoordinate(CurPos, fXUssPos, fYUssPos);

           /*计算11号超声从垂直于车身位置，从车辆安装位置点CurU11到探测的最大距离点CurU11_end构成向量*/
           CurU11.x = fXUssPos[10];
           CurU11.y = fYUssPos[10];
           CurU11_end.x = CurU11.x + nDetectDis*cosf(CurPos.yaw + SENSOR11_ANGLE);
           CurU11_end.y = CurU11.y + nDetectDis*sinf(CurPos.yaw + SENSOR11_ANGLE);

           /*计算12号超声从垂直于车身位置，从车辆安装位置点CurU12到探测的最大距离点CurU12_end构成向量*/
           CurU12.x = fXUssPos[11];
           CurU12.y = fYUssPos[11];
           CurU12_end.x = CurU12.x + nDetectDis*cosf(CurPos.yaw + SENSOR12_ANGLE);
           CurU12_end.y = CurU12.y + nDetectDis*sinf(CurPos.yaw + SENSOR12_ANGLE);

           /*计算两个向量和两侧的车位侧边向量的交点bU11Inter bU12Inter*/
           bU11Inter = GetCrossPoint(CurU11, CurU11_end, Lp0, Lp1, U11Inter);
           bU12Inter = GetCrossPoint(CurU12, CurU12_end, Rp0, Rp1, U12Inter);

           /*根据交点位置，判断11和12号超声开始探测启动位置和结束位置*/

           if((!bU11Start) && bU11Inter && nSlotSide*(U11Inter.y - fUss11StartY) < 0
                   && nSlotSide*(U11Inter.y - fUss11StopY) > 0)
           {
               U11_start = CurPos;
               bU11Start = true;
           }else{}

           if((!bU11End) && bU11Start && bU11Inter && nSlotSide*(U11Inter.y - fUss11StopY) < 0)
           {
               U11_end = CurPos;
               bU11End = true;
           }else{}

           if((!bU12Start) && bU12Inter && nSlotSide*(U12Inter.y - fUss12StartY) < 0
                   && nSlotSide*(U12Inter.y - fUss12StopY) > 0)
           {
               U12_start = CurPos;
               bU12Start = true;
           }else{}

           if((!bU12End) && bU12Start && bU12Inter && nSlotSide*(U12Inter.y - fUss12StopY) < 0)
           {
               U12_end = CurPos;
               bU12End = true;
           }else{}
       }

       if(bU11Start&&bU11End&&bU12Start&&bU12End)
       {
           validity = true;
       }
       else
       {
           validity = false;
       }
   }else{}

   return validity;

}

/**
 * @brief 给超声泊车最后一段轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在超声泊车最后一段入库轨迹规划结束调用
 */
int AddMeasureStatusInObliUSSSlot(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks)
{
    LocationPoint tCurPos;
    LocationPoint U11start;
    LocationPoint U11end;
    LocationPoint U12start;
    LocationPoint U12end;

    bool GetU11U12Succ = false;
    int nSlotSide = slotPosInfor_.nSlotSide;
    int nRet = 0;
    float Ylimit = 0.0;

    /*遍历整个轨迹，计算11和12超声探测的开始结束坐标点*/
    GetU11U12Succ = GetU11AndU12StartAndEndPoint(slotPosInfor_,
                                                 aTracks,
                                                 U11start,
                                                 U11end,
                                                 U12start,
                                                 U12end);

    if (GetU11U12Succ)/*计算成功,给轨迹点赋测量状态*/
    {
        for(size_t i = 0; i < aTracks.size(); ++i)
        {
            /*读取坐标点和偏航角归一化*/
            tCurPos.x = aTracks[i].point.getX();
            tCurPos.y = aTracks[i].point.getY();
            if(aTracks[i].speed < 0)
            {
                tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw + PI);
            }
            else
            {
                tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw);
            }

            if(nSlotSide == RIGHTSIDESLOT)
            {
                Ylimit = MIN(U11end.y, U12end.y);
            }
            else
            {
                Ylimit = MAX(U11end.y, U12end.y);
            }
            /*左侧车位和右侧车位的y轴相反，因此要根据左右侧车位方向判断11.12超声的启动区间，*/
            if(nSlotSide*(U11start.y -  U12start.y ) > 0)
            {
                if(nSlotSide*(tCurPos.y  - U11start.y) <= 0 && nSlotSide*(tCurPos.y  - U12start.y) > 0)
                {
                    aTracks[i].nDetectStates = UssVerticalPushLeftRearPointMode;/*11超声的启动*/
                }
                else if(nSlotSide*(tCurPos.y  - U12start.y) <= 0 && nSlotSide*(tCurPos.y  - Ylimit) > 0 )
                {
                    aTracks[i].nDetectStates = UssVerticalPushRightRearPointMode;/*12超声的启动*/
                }
                else if(nSlotSide*(tCurPos.y  - Ylimit) <= 0
                       && nSlotSide*(tCurPos.y  - Ylimit + nSlotSide*Y_WIDTH_GET_SLOT_INFOR_OBLI) > 0)
                {
                    aTracks[i].nDetectStates = UssVerticalCalLeftRightBoundMode;/*11和12号超声停止收集数据*/
                }else{}

            }
            else if(nSlotSide*(U11start.y - U12start.y) < 0)
            {
                if(nSlotSide*(tCurPos.y  - U12start.y) <= 0 && nSlotSide*(tCurPos.y  - U11start.y) > 0)
                {
                    aTracks[i].nDetectStates = UssVerticalPushRightRearPointMode;
                }
                else if(nSlotSide*(tCurPos.y - U11start.y) <= 0 && nSlotSide*(tCurPos.y  - Ylimit) > 0)
                {
                    aTracks[i].nDetectStates = UssVerticalPushLeftRearPointMode;
                }
                else if(nSlotSide*(tCurPos.y  - Ylimit) <= 0
                       && nSlotSide*(tCurPos.y  - Ylimit + nSlotSide*Y_WIDTH_GET_SLOT_INFOR_OBLI) > 0)
                {
                    aTracks[i].nDetectStates = UssVerticalCalLeftRightBoundMode;
                } else{}

            }
            else //todu yrm
            {
                if(nSlotSide*(tCurPos.y  - U12start.y) < 0)
                {
                    aTracks[i].nDetectStates = UssVerticalPushLeftRearPointMode;/*11和12号同时启动超声收集*/
                    aTracks[i + 1].nDetectStates = UssVerticalPushRightRearPointMode;/*11和12号同时启动超声收集*/
                }
                else if(nSlotSide*(tCurPos.y  - Ylimit) <= 0
                       && nSlotSide*(tCurPos.y  - Ylimit + nSlotSide*Y_WIDTH_GET_SLOT_INFOR_OBLI) > 0)
                {
                    aTracks[i].nDetectStates = UssVerticalCalLeftRightBoundMode;/*11和12号超声停止收集数据*/
                }else{}

            }
            if(i == 0)
            {
                aTracks[i].nDetectStates = 0;
            }
            else if (i >= 1)
            {
                if( aTracks[i].nDetectStates == UssVerticalCalLeftRightBoundMode && aTracks[i-1].nDetectStates == 0)
                {
                    aTracks[i-1].nDetectStates = UssVerticalCalLeftRightBoundMode;
                }else{};
            }
            if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
            {
                if(aTracks.back().point.getY() < -3.0f && aTracks[i].point.getY() < 0.0f)
                {
                    aTracks[i].speed = SgnCus(aTracks[i].speed) * MIN(fabsf(aTracks[i].speed), MAX_VELOCITY_IN_OBLI_SLOT);
                }
            }
            else
            {
                if(aTracks.back().point.getY() > 3.0f && aTracks[i].point.getY() > 0.0f)
                {
                    aTracks[i].speed = SgnCus(aTracks[i].speed) * MIN(fabsf(aTracks[i].speed), MAX_VELOCITY_IN_OBLI_SLOT);
                }
            }
            if(aTracks[i].nDetectStates == UssVerticalCalLeftRightBoundMode)
            {
                size_t nStart = 0;
                if(i >= 5)
                {
                    nStart = i - 5;
                }
                else
                {
                    nStart = 0;
                }
                for(size_t nCurrIndex = nStart; nCurrIndex <= i; ++nCurrIndex)
                {
                    aTracks[nCurrIndex].speed = (MAX_VELOCITY_IN_OBLI_SLOT) * SgnCus(aTracks[i].speed);
                }
            }
        }
    }else{}

    return nRet;
}

/**
 * @brief 给视觉划线车位泊车最后一段轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在视觉划线车位泊车,最后一段入库轨迹规划结束调用
 */
int AddMeasureStatusInObliVisionSlot(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks)
{
    LocationPoint tCurPos;
    int nStatus = 0;
    int nRet = 0;
    size_t nStart = 0;
	planCfg tPlanCfg = {0};

    for(size_t i = 0; i < aTracks.size(); ++i)
    {
        /*读取坐标点和偏航角归一化*/
        tCurPos.x = aTracks[i].point.getX();
        tCurPos.y = aTracks[i].point.getY();
        if(aTracks[i].speed < 0)
        {
            tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw + PI);
        }
        else
        {
            tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw);
        }

        /*遍历整个轨迹，计算视觉需要更新坐标区间状态*/
        nStatus = StatusCheckWhenLastDriving(slotPosInfor_,
                                             tCurPos);

        /*根据更新坐标区间状态，给轨迹点赋状态*/
        if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
        {
            if(/*nStatus == 1 ||*/ nStatus == 2)
            {
            	// aTracks[i].nDetectStates = ViewDynamicUpdatePosNearSlot;
            }
            else if(nStatus == 3)
            {
//                aTracks[i].nDetectStates = ViewDynamicUpdatePosInsideSlot;
            }
            else{}

            //是否动态更新
            MvGetPlanCfg(&tPlanCfg);
            if(1 == tPlanCfg.dynamicProgramming)
            // if(1)
            {
                #if 1  
                if(isPositionUpdateSlotInfo(slotPosInfor_, tCurPos))
                    aTracks[i].nDetectStates = ViewDynamicUpdatePosNearSlot;			
                GAC_LOG_INFO("Dynamic_programming:nStatus %d nDetectStates %d\n", nStatus, aTracks[i].nDetectStates);
                #endif
            }
        }
        else if(slotPosInfor_.nParkingType == HEAD_PARK_IN)
        {
            if(nStatus == 3)
            {
                aTracks[i].nDetectStates = ViewDynamicUpdatePosInsideSlot;
            }
            else{}
        }

        if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
        {
            if(aTracks.back().point.getY() < -3.0f && aTracks[i].point.getY() < 0.0f)
            {
                aTracks[i].speed = SgnCus(aTracks[i].speed) * MIN(fabsf(aTracks[i].speed), MAX_VELOCITY_IN_OBLI_SLOT);
            }
        }
        else
        {
            if(aTracks.back().point.getY() > 3.0f && aTracks[i].point.getY() > 0.0f)
            {
                aTracks[i].speed = SgnCus(aTracks[i].speed) * MIN(fabsf(aTracks[i].speed), MAX_VELOCITY_IN_OBLI_SLOT);
            }
        }

        if(aTracks[i].nDetectStates == ViewDynamicUpdatePosNearSlot || aTracks[i].nDetectStates == ViewDynamicUpdatePosInsideSlot)
        {
            nStart = 0;
            if(i >= 5)
            {
                nStart = i - 5;
            }
            else
            {
                nStart = 0;
            }
            for(size_t nCurrIndex = nStart; nCurrIndex <= i; ++nCurrIndex)
            {
                aTracks[nCurrIndex].speed = (MAX_VELOCITY_IN_OBLI_SLOT) * SgnCus(aTracks[i].speed);
            }
        }
    }

    /*最后10个点发状态确定挡轮杆*/
    if(aTracks.size() >= DETECT_WHEEL_BAR_NUM)
    {
        nStart = aTracks.size() - DETECT_WHEEL_BAR_NUM;
    }
    else
    {
        nStart = 0;
    }
//    for(size_t i = aTracks.size() - 1; i > nStart; --i)
//    {
//        aTracks[i].nDetectStates = DetectWheelBar;
//    }
    return nRet;

}

/**
 * @brief 给超声泊车最后一段hybrid A*轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在超声车位泊车,最后一段入库hybrid A*轨迹规划结束调用
 */
bool AddMeasureStatusInParaUssSlotWhenHybridALastDriving(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks)
{
    bool validity = false;
    LocationPoint tCurPos;
    int nSlotSide = slotPosInfor_.nSlotSide;


    for(size_t i = 0; i < aTracks.size(); ++i)
    {
        /*读取坐标点和偏航角归一化*/
        tCurPos.x = aTracks[i].point.getX();
        tCurPos.y = aTracks[i].point.getY();

        if(aTracks[i].speed < 0)
        {
            tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw + PI);
        }
        else
        {
            tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw);
        }

        /*快到终点提前PARA_POINT_NUM个点开始赋测量状态*/
        if(i > aTracks.size() - PARA_POINT_NUM && nSlotSide*(tCurPos.yaw - nSlotSide*MIN_VEH_YAW_DETECT_REAR_MARGIN) < 0)
        {
            aTracks[i].nDetectStates = UssParallelCalBackBoundMode;
            validity = true;
        }else{}
    }

    return validity;

}

/**
 * @brief 给平行超声泊车在库里向前调整时轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在3号超声Y坐标低于车位外侧前角点的Y坐标,行驶方向为前进调用,平行库内调整调用
 */
bool AddMeasureStatusInParaUssSlotWhenForwardDriving(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks)
{
    float fXUssPos[12];
    float fYUssPos[12];
    float fXFrontMargin = slotPosInfor_.fXFrontSlotCorner;

    int nSlotsSide = slotPosInfor_.nSlotSide;
    bool validity = false;
    LocationPoint tCurPos;

    for(size_t i = 0; i < aTracks.size(); ++i)
    {
        /*读取坐标点和偏航角归一化*/
        tCurPos.x = aTracks[i].point.getX();
        tCurPos.y = aTracks[i].point.getY();
        if(aTracks[i].speed < 0)
        {
            tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw + PI);
        }
        else
        {
            tCurPos.yaw = normalizeHeadingRad_Npi_Ppi(aTracks[i].yaw);
        }

        /*计算12个超声波的探头位置*/
        CalUSSCoordinate(tCurPos, fXUssPos, fYUssPos);

        if(nSlotsSide == RIGHTSIDESLOT)
        {
            /*快到终点提前PARA_POINT_NUM个点,3号超声Y坐标低于车位外侧前角点的Y坐标开始赋测量状态*/
            if(fYUssPos[2] < -Constants::chamfer_length  && fXUssPos[2] > fXFrontMargin - 1)
            {
                aTracks[i].nDetectStates = UssParallelCalFrontBoundMode;
                validity = true;
            }else{}
        }
        else
        {
            /*快到终点提前PARA_POINT_NUM个点,3号超声Y坐标低于车位外侧前角点的Y坐标开始赋测量状态*/
            if(fYUssPos[1] > Constants::chamfer_length && fXUssPos[1] > fXFrontMargin - 1)
            {
                aTracks[i].nDetectStates = UssParallelCalFrontBoundMode;
                validity = true;
            }else{}
        }
    }
    return validity;

}


}
