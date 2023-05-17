#include "math_formula.h"


void TransferRadius2FrontAxleAngle()
{
    float thetaLF;
    float thetaLR;
    float thetaRF;
    float thetaRR;

#ifdef CHERY_EQ
    for(int i = 170; i <=500; i+=30)
    {
        thetaLF = atan(WHEEL_BASE/CherryEQRadius[(i-170)/30][1]);
        thetaLR = atan(WHEEL_BASE/CherryEQRadius[(i-170)/30][2]);
        thetaRF = atan(WHEEL_BASE/CherryEQRadius[(i-170)/30][3]);
        thetaRR = atan(WHEEL_BASE/CherryEQRadius[(i-170)/30][4]);

        GAC_LOG_DEBUG("{%d*PI/180, %f, %f, %f, %f},",i,thetaLF,thetaLR,thetaRF,thetaRR);
    }

#endif

#ifdef VEL_JIHE
    for(int i = 170; i <=500; i+=30)
    {
        thetaLF = atan(WHEEL_BASE/JiHeARadius[(i-170)/30][1]);
        thetaLR = atan(WHEEL_BASE/JiHeARadius[(i-170)/30][2]);
        thetaRF = atan(WHEEL_BASE/JiHeARadius[(i-170)/30][3]);
        thetaRR = atan(WHEEL_BASE/JiHeARadius[(i-170)/30][4]);

        GAC_LOG_DEBUG("{%d*PI/180, %f, %f, %f, %f},",i,thetaLF,thetaLR,thetaRF,thetaRR);
    }

#endif

#ifdef CHERY_T1D_RUIHU
    for(int i = 170; i <= 470; i+=30)
    {
        thetaLF = atan(WHEEL_BASE/T1DRadius[(i-170)/30][1]);
        thetaLR = atan(WHEEL_BASE/T1DRadius[(i-170)/30][2]);
        thetaRF = atan(WHEEL_BASE/T1DRadius[(i-170)/30][3]);
        thetaRR = atan(WHEEL_BASE/T1DRadius[(i-170)/30][4]);
        GAC_LOG_DEBUG("{%d*PI/180, %f, %f, %f, %f},",i,thetaLF,thetaLR,thetaRF,thetaRR);
    }

#endif

#if 0//GAC_A29
    for(int i = 170; i <= 530; i+=30)
    {
        thetaLF = atan(WHEEL_BASE/GAC_A29Radius[(i-170)/30][1]);
        thetaLR = atan(WHEEL_BASE/GAC_A29Radius[(i-170)/30][2]);
        thetaRF = atan(WHEEL_BASE/GAC_A29Radius[(i-170)/30][3]);
        thetaRR = atan(WHEEL_BASE/GAC_A29Radius[(i-170)/30][4]);
        GAC_LOG_DEBUG("{%d*PI/180, %f, %f, %f, %f},",i,thetaLF,thetaLR,thetaRF,thetaRR);
    }

#endif

}




float CalcAngle(const PointSimple &first, const PointSimple &cen, const PointSimple &second)
{
    float dx1, dx2, dy1, dy2;
    float angle;

    dx1 = first.x - cen.x;
    dy1 = first.y - cen.y;

    dx2 = second.x - cen.x;

    dy2 = second.y - cen.y;

    double c = (double)sqrt(dx1 * dx1 + dy1 * dy1) * (double)sqrt(dx2 * dx2 + dy2 * dy2);

    if (c == 0) return -1;

    angle = (double)acos((dx1 * dx2 + dy1 * dy2) / c);

    return angle / PI * 180;
}


float get_slot_Deg(const PointSimple& tP0, const PointSimple& tP1,
                                    const PointSimple& tP2, const PointSimple& tP3,
                                    int  nSide)
{
    float fDeg = -9999.;
    float fR0,fR1,fTmp1,fTmp2;

    // NO_SIDE 0
    // RIGHT_SIDE 1
    // LEFT_SIDE 2
    switch (nSide)
    {
        case RIGHTSIDESLOT:
//            fR0 = get_points_arctan(tP0.x, tP0.y, tP1.x, tP1.y, tP3.x, tP3.y);
            fR0 = CalcAngle(tP1, tP0, tP3);
//            fR1 = get_points_arctan(tP1.x, tP1.y, tP0.x, tP0.y, tP2.x, tP2.y);

//            fR1 = get_points_arctan(tP1.x, tP1.y, tP2.x, tP2.y, tP0.x, tP0.y);
            fR1 = CalcAngle(tP2, tP1, tP0);
            fR1 = 180 - fR1;

            fDeg = (fR0+fR1)*0.5;
            break;

        case LEFTSIDESLOT:
            fR0 = CalcAngle(tP1, tP0, tP3);
            fR0 = 180 - fR0;
            fR1 = CalcAngle(tP2, tP1, tP0);

            fDeg = (fR0+fR1)*0.5;
            break;

//        case NO_SIDE:
//            fR0 = CalcAngle(tP1, tP0, tP3);
//            fR1 = CalcAngle(tP2, tP1, tP0);
//            fR1 = 180 - fR1;

//            fDeg = (fR0+fR1)*0.5;
//            break;

        default:
            break;
    }



    return fDeg;
}



void RadiusCalibrationByFourWheelPulse(int FLCnt, int FRCnt, int RLCnt, int RRCnt, int nSteeringDirection, float &RearAxleCenterRadius,float &RearAxleCenterRadiusByRearAxle, float &RearAxisWheelSpan, float &FrontAxisWheelSpan)
{
    float A,B,C,R_RR;
    float FLWheelDis = FLCnt * FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float FRWheelDis = FRCnt * FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RLWheelDis = RLCnt * RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RRWheelDis = RRCnt * RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    if(nSteeringDirection == 1)
    {
        A = 0.5f * (RRWheelDis / RLWheelDis - 1);
        B =(powf(FRWheelDis / RLWheelDis, 2) - powf(FLWheelDis / RLWheelDis, 2))/ (4.0f * (1+A));
        C = powf(FRWheelDis / RLWheelDis,2) - powf(1 + A + B, 2);
        R_RR = WHEEL_BASE / sqrtf(C);
        RearAxisWheelSpan = A * R_RR * 2;
        FrontAxisWheelSpan = B * R_RR * 2;
        RearAxleCenterRadius = RearAxisWheelSpan / 2 + R_RR;
        RearAxleCenterRadiusByRearAxle = 0.5f * (RLWheelDis+RRWheelDis) / (RRWheelDis-RLWheelDis) * REAR_WHEEL_SPAN;

    }
    else
    {
        A = 0.5f * (RLWheelDis/RRWheelDis - 1);
        B = (powf(FLWheelDis/RRWheelDis, 2) - powf(FRWheelDis / RRWheelDis, 2)) / (4.0f * (1+A));
        C = powf(FLWheelDis/RRWheelDis, 2) - powf(1 + A + B, 2);
        R_RR = WHEEL_BASE / sqrtf(C);
        RearAxisWheelSpan = A * R_RR * 2;
        FrontAxisWheelSpan = B * R_RR * 2;
        RearAxleCenterRadius = RearAxisWheelSpan / 2 + R_RR;
        RearAxleCenterRadiusByRearAxle = 0.5f * (RLWheelDis + RRWheelDis) / (RLWheelDis-RRWheelDis) * REAR_WHEEL_SPAN;
    }
}

void RadiusCalibrationByRearWheelPulse(int RLCnt, int RRCnt,
                                       int nSteeringDirection,
                                       float &RearAxleCenterRadiusByRearAxle)
{
    float RLWheelDis = RLCnt * RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RRWheelDis = RRCnt * RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;

    if(nSteeringDirection == 1)
    {
        RearAxleCenterRadiusByRearAxle = 0.5f * (RLWheelDis + RRWheelDis) / (RRWheelDis - RLWheelDis) * REAR_WHEEL_SPAN;
    }
    else
    {
        RearAxleCenterRadiusByRearAxle = 0.5f * (RLWheelDis + RRWheelDis) / (RLWheelDis - RRWheelDis) * REAR_WHEEL_SPAN;
    }
}


void RadiusCalibrationByFrontWheelPulse(int FLCnt, int FRCnt, int nSteeringDirection,
                                        float &RearAxleCenterRadius)
{
    float A, B, C = 0.0;
    float k = 0.0;
    float FLWheelDis = FLCnt * FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float FRWheelDis = FRCnt * FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    if(nSteeringDirection == 1)
    {
        k = powf(FRWheelDis/FLWheelDis, 2);
    }
    else
    {
        k = powf(FLWheelDis/FRWheelDis, 2);
    }
    A= k -1;
    B= -FRONT_WHEEL_SPAN*(k + 1);
    C= (k - 1) * (WHEEL_BASE * WHEEL_BASE + FRONT_WHEEL_SPAN * FRONT_WHEEL_SPAN / 4);
    RearAxleCenterRadius= (-B + sqrtf(B * B - 4 * A * C)) / (2 * A);
}


void RadiusCalibrationByFrontRearWheelPulse(int FLCnt, int FRCnt, int RLCnt,
                                        int RRCnt, int nSteeringDirection,
                                        float &RearAxleCenterRadius)
{
    float A,B,C = 0.0;
    float k = 0.0;
    float FLWheelDis = FLCnt * FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float FRWheelDis = FRCnt * FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RLWheelDis = RLCnt * RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RRWheelDis = RRCnt * RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    if(nSteeringDirection==1)
    {
        k = powf(FRWheelDis/RLWheelDis, 2);
    }
    else
    {
        k = powf(FLWheelDis/RRWheelDis, 2);
    }
    A = k - 1;
    B = -(k * REAR_WHEEL_SPAN + FRONT_WHEEL_SPAN);
    C = REAR_WHEEL_SPAN * REAR_WHEEL_SPAN * k / 4 - FRONT_WHEEL_SPAN * FRONT_WHEEL_SPAN / 4 - WHEEL_BASE * WHEEL_BASE;
    RearAxleCenterRadius= (-B + sqrtf(B * B - 4 * A * C)) / (2 * A);
}

void RadiusCalibrationByFrontRearWheelPulse2(int FLCnt, int FRCnt, int RLCnt,
                                        int RRCnt, int nSteeringDirection,
                                        float &RearAxleCenterRadius)
{
    float A,B,C = 0.0;
    float k = 0.0;
    float FLWheelDis = FLCnt * FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float FRWheelDis = FRCnt * FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RLWheelDis = RLCnt * RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    float RRWheelDis = RRCnt * RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE;
    if(nSteeringDirection == 1)
    {
        k = powf(FLWheelDis / RLWheelDis, 2);
    }
    else
    {
        k = powf(FRWheelDis / RRWheelDis, 2);
    }
    A= k -1;
    B= -k * REAR_WHEEL_SPAN + FRONT_WHEEL_SPAN;
    C= REAR_WHEEL_SPAN * REAR_WHEEL_SPAN * k / 4 - FRONT_WHEEL_SPAN * FRONT_WHEEL_SPAN / 4 - WHEEL_BASE * WHEEL_BASE;

    RearAxleCenterRadius= (-B + sqrtf(B * B - 4 * A * C)) / (2 * A);
}

bool CalAverageRadius(int FLCnt, int FRCnt, int RLCnt,
                       int RRCnt, int nSteeringDirection,
                       float &RearAxleCenterRadius)
{
    float fR = 0.0f;
    float MinR, MaxR = 0.0f;

    std::vector<float> radiusSet;

    RadiusCalibrationByRearWheelPulse(RLCnt, RRCnt,
                                      nSteeringDirection,
                                      fR);
    radiusSet.push_back(fR);

    RadiusCalibrationByFrontWheelPulse(FLCnt,FRCnt,
                                       nSteeringDirection,
                                       fR);
    radiusSet.push_back(fR);

    RadiusCalibrationByFrontRearWheelPulse(FLCnt,FRCnt,
                                           RLCnt, RRCnt,
                                           nSteeringDirection,
                                           fR);
    radiusSet.push_back(fR);

//    RadiusCalibrationByFrontRearWheelPulse2(FLCnt,FRCnt,
//                                            RLCnt, RRCnt,
//                                           nSteeringDirection,
//                                           fR);
//    radiusSet.push_back(fR);

    std::sort(radiusSet.begin(), radiusSet.end());
    MinR = radiusSet.front();
    MaxR = radiusSet.back();
    if(isnanf(MaxR))
    {
        GAC_LOG_DEBUG("");
    }
    else if(isinff(MaxR))
    {
        GAC_LOG_DEBUG("");
    }

    GAC_LOG_DEBUG("MaxR, MinR is %f, %f,delta is %f", MaxR, MinR, MaxR - MinR);
    if(fabsf(MaxR - MinR) < 0.05f)
    {
        RearAxleCenterRadius =  (MaxR + MinR) / 2;
        return true;
    }
    else
    {
        RearAxleCenterRadius = 0.0;
        return false;
    }

}

/*
*函数名称: CalcCornerPointRadius
*函数功能: 根据车辆方向盘转角/后轴中心点转弯半径/行驶方向，计算车辆四个角点的转弯半径
*输入参数:
*        nSteeringDirection:方向盘转动方向    -1=右转; 1=左转
*        nRunningDirection:行驶方向          -1=后退; 1=前进
*        fRearAxleCenterRadius:后轴中心点转弯半径   (正实数)
*
*输出参数:
*        fRightFrontPointRadius:右前角点转弯半径
*        fRightRearPointRadius:右后角点转弯半径
*        fLeftFrontPointRadius:左前角点转弯半径
*        fLeftRearPointRadius:左后角点转弯半径
*
*
*返回值:
*        -1:无效
*         0:有效
*/
int CalCornerPointRadius(int nSlotPosition, int nSteeringDirection,float fRearAxleCenterRadius,
                           float &fRightRearPointRadius,float &fRightFrontPointRadius,
                           float &fLeftFrontPointRadius,float &fLeftRearPointRadius)
{
    int nRet = -1;

    if ( (nSteeringDirection!= 1)&&(nSteeringDirection!= -1)
         ||(fRearAxleCenterRadius <= 0) )
    {
        ;
    }
    else
    {
            if (nSteeringDirection == 1)
            {
                fRightRearPointRadius = sqrt(pow(fRearAxleCenterRadius+ nSlotPosition*VEHICLE_WID / 2 ,2) + pow(REAR_EDGE2CENTER,2));
                fRightFrontPointRadius = sqrt(pow(fRearAxleCenterRadius + nSlotPosition*VEHICLE_WID / 2,2) + pow(VEHICLE_LEN - REAR_EDGE2CENTER, 2));
                fLeftFrontPointRadius = sqrt(pow(fRearAxleCenterRadius - nSlotPosition*VEHICLE_WID / 2, 2) + pow(VEHICLE_LEN - REAR_EDGE2CENTER, 2));
                fLeftRearPointRadius = sqrt(pow(fRearAxleCenterRadius - nSlotPosition*VEHICLE_WID / 2, 2) + pow(REAR_EDGE2CENTER, 2));
            }
            else
            {
                fRightRearPointRadius = sqrt(pow(fRearAxleCenterRadius - nSlotPosition*VEHICLE_WID / 2, 2) + pow(REAR_EDGE2CENTER,2));
                fRightFrontPointRadius = sqrt(pow(fRearAxleCenterRadius - nSlotPosition*VEHICLE_WID / 2, 2) + pow(VEHICLE_LEN - REAR_EDGE2CENTER, 2));
                fLeftFrontPointRadius = sqrt(pow(fRearAxleCenterRadius + nSlotPosition*VEHICLE_WID / 2, 2) + pow(VEHICLE_LEN - REAR_EDGE2CENTER, 2));
                fLeftRearPointRadius = sqrt(pow(fRearAxleCenterRadius + nSlotPosition*VEHICLE_WID / 2, 2) + pow(REAR_EDGE2CENTER, 2));
            }
        nRet = 0;
    }
    return nRet;
}



int CalFourWheelRadius(int nSteeringDirection,float fRearAxleCenterRadius,
                           float &fRightRearPointRadius,float &fRightFrontPointRadius,
                           float &fLeftFrontPointRadius,float &fLeftRearPointRadius)
{
    int nRet = -1;

    if ( (nSteeringDirection!= 1)&&(nSteeringDirection!= -1)
         ||(fRearAxleCenterRadius <= 0) )
    {
        ;
    }
    else
    {
            if (nSteeringDirection == 1)
            {
                fRightRearPointRadius = fRearAxleCenterRadius+ REAR_WHEEL_SPAN / 2 ;
                fRightFrontPointRadius = sqrt(pow(fRearAxleCenterRadius + REAR_WHEEL_SPAN / 2,2) + pow(WHEEL_BASE, 2));
                fLeftFrontPointRadius = sqrt(pow(fRearAxleCenterRadius - REAR_WHEEL_SPAN / 2, 2) + pow(WHEEL_BASE, 2));
                fLeftRearPointRadius = fRearAxleCenterRadius-REAR_WHEEL_SPAN / 2;
            }
            else
            {
                fRightRearPointRadius = fRearAxleCenterRadius- REAR_WHEEL_SPAN / 2 ;
                fRightFrontPointRadius = sqrt(pow(fRearAxleCenterRadius - REAR_WHEEL_SPAN / 2,2) + pow(WHEEL_BASE, 2));
                fLeftFrontPointRadius = sqrt(pow(fRearAxleCenterRadius + REAR_WHEEL_SPAN / 2, 2) + pow(WHEEL_BASE, 2));
                fLeftRearPointRadius = fRearAxleCenterRadius+REAR_WHEEL_SPAN / 2;
            }
        nRet = 0;
    }
    return nRet;
}



float CalFrontAxleCenterAngleOrSteeringAngle(int nSteeringDirection, int nRunningDirection, float fRequestAngle, int nRequestType)
{
    float Angle;
    float OutputAngle;
    int Index;
    float x1,x2,y1,y2,k;
    int Num;
    Angle=fabs(fRequestAngle);
    if(nSteeringDirection==1)
    {
        if(nRunningDirection==1)
        {
            Num=1;
        }
        else
        {
            Num=2;
        }
    }
    else
    {
        if(nRunningDirection==1)
        {
            Num=3;
        }
        else
        {
            Num=4;
        }
    }
    if(nRequestType==0) //SteeringAngle2FrontAxleCenterAngle;
    {
        for(Index=0;Index<EPS_STEERING_ANGLE_RATIO_SEGMENT;Index++)
        {
            if(Angle>EPSSteeringAngleRatio[Index][0])
            {

            }
            else
            {
                break;
            }
        }
    }
    else//FrontAxleCenterAngle2SteeringAngle;
    {
        for(Index=0;Index<EPS_STEERING_ANGLE_RATIO_SEGMENT;Index++)
        {
            if(Angle>EPSSteeringAngleRatio[Index][Num])
            {

            }
            else
            {
                break;
            }
        }
    }
    if(Index==0)
    {
        y1=EPSSteeringAngleRatio[Index][Num];
        x1=EPSSteeringAngleRatio[Index][0];
        k=y1/x1;
    }
    else if(Index==EPS_STEERING_ANGLE_RATIO_SEGMENT)
    {
        y1=EPSSteeringAngleRatio[Index-1][Num];
        x1=EPSSteeringAngleRatio[Index-1][0];
        k=y1/x1;
    }
    else
    {
        y1=EPSSteeringAngleRatio[Index-1][Num];
        y2=EPSSteeringAngleRatio[Index][Num];
        x1=EPSSteeringAngleRatio[Index-1][0];
        x2=EPSSteeringAngleRatio[Index][0];
        k=(y2-y1)/(x2-x1);
    }
    if(nRequestType==0)
    {
        OutputAngle=k*(Angle-x1)+y1;
    }
    else
    {
        OutputAngle=(Angle-y1)/k+x1;
    }
    return OutputAngle;
}


/*
*函数名称: SteeringWheelAngle2Radius
*函数功能: //根据车辆方向盘转角/方向盘转方向/行驶方向，计算后轴中心点转弯半径
*输入参数:
*        nSteeringDirection:方向盘转动方向    -1=右转; 1=左转
*        nRunningDirection:行驶方向          -1=后退; 1=前进
*        fSteeringAngle:方向盘转角   (1-510)
*
*输出参数:
*
*
*返回值:
*        -1:无效
*        >0:转弯半径
*/

float CalSteeringWheelAngle2Radius(int nSteeringDirection, int nRunningDirection, int fSteeringWheelAngle)
{
    float fRadius = -1;
    float fFrontAxleCenterAngle;
    float fMinLimitRearAxleCenterRadius;
    int nAngle;
    nAngle=abs(fSteeringWheelAngle);
    if ((nSteeringDirection!= 1)&&(nSteeringDirection!= -1)
         ||(nRunningDirection!= 1)&&(nRunningDirection!= -1)&&(nRunningDirection!= 1)&&(nRunningDirection!= 0))
    {
        ;
    }
    else
    {
        fFrontAxleCenterAngle=CalFrontAxleCenterAngleOrSteeringAngle(nSteeringDirection,nRunningDirection,nAngle*PI/180,0);
        if(fabs(fFrontAxleCenterAngle)==0)
        {
            fRadius=99999.f;
        }
        else
        {
            fRadius=WHEEL_BASE/tan(fFrontAxleCenterAngle);
            fMinLimitRearAxleCenterRadius=GetMinLimitRearAxleCenterRadius( nSteeringDirection, nRunningDirection);
            if(fRadius<fMinLimitRearAxleCenterRadius)
            {
                fRadius=fMinLimitRearAxleCenterRadius;
            }
        }
    }
    return fRadius;
}

/*
*函数名称: GetMinRearAxleCenterRadius
*函数功能: 根据车辆方向盘转角/行驶方向，计算后轴中心点最小转弯半径
*输入参数:
*        nSteeringDirection:方向盘转动方向    -1=右转; 1=左转
*        nRunningDirection:行驶方向          -1=后退; 1=前进
*
*输出参数:
*
*
*返回值:
*        -1:无效
*        >0:转弯半径
*/


float GetMinRearAxleCenterRadius(int nSteeringDirection, int nRunningDirection)
{
    float fRadius = -1;
    if ((nSteeringDirection!= 1)&&(nSteeringDirection!= -1)
         ||(nRunningDirection!= 1)&&(nRunningDirection!= -1))
    {
        ;
    }
    else
    {
        fRadius = CalSteeringWheelAngle2Radius(nSteeringDirection, nRunningDirection, MAX_STEERING_ANGLE);
    }
    return fRadius;
}


/*
*函数名称: GetMinLimitRearAxleCenterRadius
*函数功能: 根据车辆方向盘转角/行驶方向，计算后轴中心点最小转弯半径
*输入参数:
*        nSteeringDirection:方向盘转动方向    -1=右转; 1=左转
*        nRunningDirection:行驶方向          -1=后退; 1=前进
*
*输出参数:
*
*
*返回值:
*        -1:无效
*        >0:转弯半径
*/


float GetMinLimitRearAxleCenterRadius(int nSteeringDirection, int nRunningDirection)
{
    float fRadius = -1;
    if ((nSteeringDirection!= 1)&&(nSteeringDirection!= -1)
            ||(nRunningDirection!= 1)&&(nRunningDirection!= -1))
    {
        ;
    }
    else
    {
#ifdef VEL_BORUI
        if (nSteeringDirection==1)
        {
            if (nRunningDirection==1)
            {
                fRadius = 4.15;
            }
            else if (nRunningDirection==-1)
            {
                fRadius = 4.15;
            }
        }
        else
        {
            if (nRunningDirection==1)
            {
                fRadius = 4.15;
            }
            else if (nRunningDirection==-1)
                fRadius = 4.15;
        }
#endif
#ifdef VEL_CHANGAN
        if (nSteeringDirection==1)
        {
            if (nRunningDirection==1)
            {
                fRadius =4.079 ;
            }
            else if (nRunningDirection==-1)
            {
                fRadius = 4.079;
            }
        }
        else
        {
            if (nRunningDirection==1)
            {
                fRadius = 3.933;
            }
            else if (nRunningDirection==-1)
                fRadius = 3.933;
        }
#endif
#ifdef VEL_DONGFENG_E70
        if (nSteeringDirection==1)
        {
            if (nRunningDirection==1)
            {
                fRadius = 3.25+0.9425;
            }
            else if (nRunningDirection==-1)
            {
                fRadius = 3.325+0.9425;
            }
        }
        else
        {
            if (nRunningDirection==1)
            {
                fRadius = 3.31+0.9425;
            }
            else if (nRunningDirection==-1)
                fRadius = 3.37+0.9425;
        }
#endif
    }
    return fRadius;
}

/*
*函数名称: Radius2SteeringWheelAngle
*函数功能: 根据后轴中心点转弯半径/方向盘转向/行驶方向，计算车辆方向盘转角
*输入参数:
*        nSteeringDirection:方向盘转动方向    -1=右转; 1=左转
*        nRunningDirection:行驶方向          -1=后退; 1=前进
*        fRadius:后轴中心点转弯半径   (>0)
*
*输出参数:
*
*
*返回值:
*        -1:无效
*        >0:方向盘转角的绝对值
*/
int CalRadius2SteeringWheelAngle(int nSteeringDirection, int nRunningDirection, float fRadius)
{
    int fSteeringWheelAngle=-1;
    float fFrontAxleCenterAngle;
    if ( (nSteeringDirection!= 1)&&(nSteeringDirection!= -1)
         ||(nRunningDirection!= 1)&&(nRunningDirection!= -1)
         ||(fRadius<=0))
    {
        ;
    }
    else
    {
        fFrontAxleCenterAngle=atan(WHEEL_BASE/fRadius);
        fSteeringWheelAngle=int(CalFrontAxleCenterAngleOrSteeringAngle(nSteeringDirection,nRunningDirection,fFrontAxleCenterAngle,1)*180/PI);
    }
    return fSteeringWheelAngle;
}

/*
*函数名称: RotationCenterCoordinateCalculation
*函数功能: //根据当前车辆位置姿态/方向盘转动方向/后轴中心点转弯半径，计算车辆瞬时转动中心坐标
*输入参数:
*        nSteeringDirection:方向盘转动方向    -1=右转; 1=左转
*        tVehicleRearAxleCenter:车辆位置姿态坐标
*        fRearAxleCenterRadius:后轴中心点转弯半径   (>0)
*输出参数:
*        tRotationCenter:瞬时转动中心坐标
*返回值:
*        -1:无效
*         0:有效
*/
int CalRotationCenterCoordinate(int nSlotPosition, int nSteeringDirection, LocationPoint tVehicleRearAxleCenter,
                                         float fRearAxleCenterRadius, LocationPoint& tRotationCenter)
{
    int nRet = -1;
    if(nSteeringDirection!=1&&nSteeringDirection!=-1)
    {
      ;
    }
    else
    {
        tRotationCenter.x = tVehicleRearAxleCenter.x - nSlotPosition*SgnCus(nSteeringDirection)*fRearAxleCenterRadius * sin(tVehicleRearAxleCenter.yaw);
        tRotationCenter.y = tVehicleRearAxleCenter.y + nSlotPosition*SgnCus(nSteeringDirection)*fRearAxleCenterRadius * cos(tVehicleRearAxleCenter.yaw);
        nRet = 0;
    }
    return nRet;
}


/*
*函数名称: RotationCenterCoordinateCalculation
*函数功能: //根据车辆位置姿态计算四个角点坐标
*输入参数:
*        tVehicleRearAxleCenter:后轴中心点位置姿态
*输出参数:
*        tRightRearCornerPoint:右后角点坐标
*        tRightFrontCornerPoint:右前角点坐标
*        tLeftFrontCornerPoint:左前角点坐标
*        tLeftRearCornerPoint:左后角点坐标
*返回值:
*
*/
void CalCornerCoordinate( LocationPoint tVehicleRearAxleCenter, LocationPoint& tRightRearCornerPoint, LocationPoint& tRightFrontCornerPoint,
                      LocationPoint& tLeftFrontCornerPoint, LocationPoint& tLeftRearCornerPoint)
{
    LocationPoint tRightRearCornerPointOpposite;
    LocationPoint tRightFrontCornerPointOpposite;
    LocationPoint tLeftFrontCornerPointOpposite;
    LocationPoint tLeftRearCornerPointOpposite;
    LocationPoint tOriginPoint;
    tOriginPoint.x=0;
    tOriginPoint.y=0;
    tRightRearCornerPointOpposite.x = -REAR_EDGE2CENTER;
    tRightRearCornerPointOpposite.y = -VEHICLE_WID / 2;
    tRightFrontCornerPointOpposite.x = VEHICLE_LEN - REAR_EDGE2CENTER;
    tRightFrontCornerPointOpposite.y = tRightRearCornerPointOpposite.y ;
    tLeftFrontCornerPointOpposite.x = tRightFrontCornerPointOpposite.x ;
    tLeftFrontCornerPointOpposite.y = VEHICLE_WID / 2;
    tLeftRearCornerPointOpposite.x = tRightRearCornerPointOpposite.x;
    tLeftRearCornerPointOpposite.y =tLeftFrontCornerPointOpposite.y;

    RotateCoordinateOfPoint(tOriginPoint, tRightRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightRearCornerPoint);
    tRightRearCornerPoint.x = tVehicleRearAxleCenter.x + tRightRearCornerPoint.x ;
    tRightRearCornerPoint.y = tVehicleRearAxleCenter.y + tRightRearCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint,tRightFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightFrontCornerPoint);
    tRightFrontCornerPoint.x= tVehicleRearAxleCenter.x + tRightFrontCornerPoint.x;
    tRightFrontCornerPoint.y = tVehicleRearAxleCenter.y + tRightFrontCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint,tLeftFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftFrontCornerPoint);
    tLeftFrontCornerPoint.x = tVehicleRearAxleCenter.x + tLeftFrontCornerPoint.x;
    tLeftFrontCornerPoint.y = tVehicleRearAxleCenter.y + tLeftFrontCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint, tLeftRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftRearCornerPoint);
    tLeftRearCornerPoint.x = tVehicleRearAxleCenter.x + tLeftRearCornerPoint.x;
    tLeftRearCornerPoint.y = tVehicleRearAxleCenter.y + tLeftRearCornerPoint.y;
}


void CalCornerCoordinateChangeSize( LocationPoint tVehicleRearAxleCenter, LocationPoint& tRightRearCornerPoint, LocationPoint& tRightFrontCornerPoint,
                      LocationPoint& tLeftFrontCornerPoint, LocationPoint& tLeftRearCornerPoint,float SmallSizeW,float SmallSizeL)
{
    LocationPoint tRightRearCornerPointOpposite;
    LocationPoint tRightFrontCornerPointOpposite;
    LocationPoint tLeftFrontCornerPointOpposite;
    LocationPoint tLeftRearCornerPointOpposite;
    LocationPoint tOriginPoint;
    tOriginPoint.x=0;
    tOriginPoint.y=0;
    tRightRearCornerPointOpposite.x = -REAR_EDGE2CENTER + SmallSizeL;
    tRightRearCornerPointOpposite.y = -VEHICLE_WID / 2 + SmallSizeW;
    tRightFrontCornerPointOpposite.x = VEHICLE_LEN - REAR_EDGE2CENTER - SmallSizeL;
    tRightFrontCornerPointOpposite.y = tRightRearCornerPointOpposite.y ;
    tLeftFrontCornerPointOpposite.x = tRightFrontCornerPointOpposite.x ;
    tLeftFrontCornerPointOpposite.y = VEHICLE_WID / 2 - SmallSizeW;
    tLeftRearCornerPointOpposite.x = tRightRearCornerPointOpposite.x;
    tLeftRearCornerPointOpposite.y =tLeftFrontCornerPointOpposite.y;

    RotateCoordinateOfPoint(tOriginPoint, tRightRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightRearCornerPoint);
    tRightRearCornerPoint.x = tVehicleRearAxleCenter.x + tRightRearCornerPoint.x ;
    tRightRearCornerPoint.y = tVehicleRearAxleCenter.y + tRightRearCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint,tRightFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightFrontCornerPoint);
    tRightFrontCornerPoint.x= tVehicleRearAxleCenter.x + tRightFrontCornerPoint.x;
    tRightFrontCornerPoint.y = tVehicleRearAxleCenter.y + tRightFrontCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint,tLeftFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftFrontCornerPoint);
    tLeftFrontCornerPoint.x = tVehicleRearAxleCenter.x + tLeftFrontCornerPoint.x;
    tLeftFrontCornerPoint.y = tVehicleRearAxleCenter.y + tLeftFrontCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint, tLeftRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftRearCornerPoint);
    tLeftRearCornerPoint.x = tVehicleRearAxleCenter.x + tLeftRearCornerPoint.x;
    tLeftRearCornerPoint.y = tVehicleRearAxleCenter.y + tLeftRearCornerPoint.y;
}


/*
*函数名称: CalUSSCoordinate
*函数功能: //根据车辆位置姿态计算12个超声波的探头位置
*输入参数:
*        tVehicleRearAxleCenter:后轴中心点位置姿态
*输出参数:
*        fXUssPos:12个超声波的探头X坐标
*        fYUssPos:12个超声波的探头Y坐标
*返回值:
*
*/
void CalUSSCoordinate( LocationPoint tVehicleRearAxleCenter, float fXUssPos[12], float fYUssPos[12])
{
    LocationPoint USS1, NewUss1;
    LocationPoint USS2, NewUss2;
    LocationPoint USS3, NewUss3;
    LocationPoint USS4, NewUss4;
    LocationPoint USS5, NewUss5;
    LocationPoint USS6, NewUss6;
    LocationPoint USS7, NewUss7;
    LocationPoint USS8, NewUss8;
    LocationPoint USS9, NewUss9;
    LocationPoint USS10, NewUss10;
    LocationPoint USS11, NewUss11;
    LocationPoint USS12, NewUss12;
    LocationPoint tOriginPoint;

    tOriginPoint.x=0;
    tOriginPoint.y=0;

    USS1.x = SENSOR1_X;
    USS1.y = SENSOR1_Y;

    USS2.x = SENSOR2_X;
    USS2.y = SENSOR2_Y;

    USS3.x = SENSOR3_X;
    USS3.y = SENSOR3_Y;

    USS4.x = SENSOR4_X;
    USS4.y = SENSOR4_Y;

    USS5.x = SENSOR5_X;
    USS5.y = SENSOR5_Y;

    USS6.x = SENSOR6_X;
    USS6.y = SENSOR6_Y;

    USS7.x = SENSOR7_X;
    USS7.y = SENSOR7_Y;

    USS8.x = SENSOR8_X;
    USS8.y = SENSOR8_Y;

    USS9.x = SENSOR9_X;
    USS9.y = SENSOR9_Y;

    USS10.x = SENSOR10_X;
    USS10.y = SENSOR10_Y;

    USS11.x = SENSOR11_X;
    USS11.y = SENSOR11_Y;

    USS12.x = SENSOR12_X;
    USS12.y = SENSOR12_Y;

    RotateCoordinateOfPoint(tOriginPoint, USS1, tVehicleRearAxleCenter.yaw, NewUss1);
    fXUssPos[0] = tVehicleRearAxleCenter.x + NewUss1.x ;
    fYUssPos[0] = tVehicleRearAxleCenter.y + NewUss1.y;

    RotateCoordinateOfPoint(tOriginPoint, USS2, tVehicleRearAxleCenter.yaw, NewUss2);
    fXUssPos[1] = tVehicleRearAxleCenter.x + NewUss2.x ;
    fYUssPos[1] = tVehicleRearAxleCenter.y + NewUss2.y;

    RotateCoordinateOfPoint(tOriginPoint, USS3, tVehicleRearAxleCenter.yaw, NewUss3);
    fXUssPos[2] = tVehicleRearAxleCenter.x + NewUss3.x ;
    fYUssPos[2] = tVehicleRearAxleCenter.y + NewUss3.y;

    RotateCoordinateOfPoint(tOriginPoint, USS4, tVehicleRearAxleCenter.yaw, NewUss4);
    fXUssPos[3] = tVehicleRearAxleCenter.x + NewUss4.x ;
    fYUssPos[3] = tVehicleRearAxleCenter.y + NewUss4.y;

    RotateCoordinateOfPoint(tOriginPoint, USS5, tVehicleRearAxleCenter.yaw, NewUss5);
    fXUssPos[4] = tVehicleRearAxleCenter.x + NewUss5.x ;
    fYUssPos[4] = tVehicleRearAxleCenter.y + NewUss5.y;

    RotateCoordinateOfPoint(tOriginPoint, USS6, tVehicleRearAxleCenter.yaw, NewUss6);
    fXUssPos[5] = tVehicleRearAxleCenter.x + NewUss6.x ;
    fYUssPos[5] = tVehicleRearAxleCenter.y + NewUss6.y;


    RotateCoordinateOfPoint(tOriginPoint, USS7, tVehicleRearAxleCenter.yaw, NewUss7);
    fXUssPos[6] = tVehicleRearAxleCenter.x + NewUss7.x ;
    fYUssPos[6] = tVehicleRearAxleCenter.y + NewUss7.y;

    RotateCoordinateOfPoint(tOriginPoint, USS8, tVehicleRearAxleCenter.yaw, NewUss8);
    fXUssPos[7] = tVehicleRearAxleCenter.x + NewUss8.x ;
    fYUssPos[7] = tVehicleRearAxleCenter.y + NewUss8.y;

    RotateCoordinateOfPoint(tOriginPoint, USS9, tVehicleRearAxleCenter.yaw, NewUss9);
    fXUssPos[8] = tVehicleRearAxleCenter.x + NewUss9.x ;
    fYUssPos[8] = tVehicleRearAxleCenter.y + NewUss9.y;

    RotateCoordinateOfPoint(tOriginPoint, USS10, tVehicleRearAxleCenter.yaw, NewUss10);
    fXUssPos[9] = tVehicleRearAxleCenter.x + NewUss10.x ;
    fYUssPos[9] = tVehicleRearAxleCenter.y + NewUss10.y;

    RotateCoordinateOfPoint(tOriginPoint, USS11, tVehicleRearAxleCenter.yaw, NewUss11);
    fXUssPos[10] = tVehicleRearAxleCenter.x + NewUss11.x ;
    fYUssPos[10] = tVehicleRearAxleCenter.y + NewUss11.y;

    RotateCoordinateOfPoint(tOriginPoint, USS12, tVehicleRearAxleCenter.yaw, NewUss12);
    fXUssPos[11] = tVehicleRearAxleCenter.x + NewUss12.x ;
    fYUssPos[11] = tVehicleRearAxleCenter.y + NewUss12.y;


}



void CalCornerCoordinatePlus( LocationPoint tVehicleRearAxleCenter,
                              float fRearEdge2Center,
                              float fVehLen,
                              float fVehWid,
                              LocationPoint& tRightRearCornerPoint,
                              LocationPoint& tRightFrontCornerPoint,
                              LocationPoint& tLeftFrontCornerPoint,
                              LocationPoint& tLeftRearCornerPoint)
{
    LocationPoint tRightRearCornerPointOpposite;
    LocationPoint tRightFrontCornerPointOpposite;
    LocationPoint tLeftFrontCornerPointOpposite;
    LocationPoint tLeftRearCornerPointOpposite;
    LocationPoint tOriginPoint;
    tOriginPoint.x=0;
    tOriginPoint.y=0;
    tRightRearCornerPointOpposite.x = -fRearEdge2Center;
    tRightRearCornerPointOpposite.y = -fVehWid / 2;
    tRightFrontCornerPointOpposite.x = fVehLen - fRearEdge2Center;
    tRightFrontCornerPointOpposite.y = tRightRearCornerPointOpposite.y ;
    tLeftFrontCornerPointOpposite.x = tRightFrontCornerPointOpposite.x ;
    tLeftFrontCornerPointOpposite.y = fVehWid / 2;
    tLeftRearCornerPointOpposite.x = tRightRearCornerPointOpposite.x;
    tLeftRearCornerPointOpposite.y =tLeftFrontCornerPointOpposite.y;

    RotateCoordinateOfPoint(tOriginPoint, tRightRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightRearCornerPoint);
    tRightRearCornerPoint.x = tVehicleRearAxleCenter.x + tRightRearCornerPoint.x ;
    tRightRearCornerPoint.y = tVehicleRearAxleCenter.y + tRightRearCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint,tRightFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tRightFrontCornerPoint);
    tRightFrontCornerPoint.x= tVehicleRearAxleCenter.x + tRightFrontCornerPoint.x;
    tRightFrontCornerPoint.y = tVehicleRearAxleCenter.y + tRightFrontCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint,tLeftFrontCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftFrontCornerPoint);
    tLeftFrontCornerPoint.x = tVehicleRearAxleCenter.x + tLeftFrontCornerPoint.x;
    tLeftFrontCornerPoint.y = tVehicleRearAxleCenter.y + tLeftFrontCornerPoint.y;
    RotateCoordinateOfPoint(tOriginPoint, tLeftRearCornerPointOpposite, tVehicleRearAxleCenter.yaw, tLeftRearCornerPoint);
    tLeftRearCornerPoint.x = tVehicleRearAxleCenter.x + tLeftRearCornerPoint.x;
    tLeftRearCornerPoint.y = tVehicleRearAxleCenter.y + tLeftRearCornerPoint.y;
}

/*
*函数名称: RotateCoordinateOfPoint
*函数功能: //计算一点绕一点旋转一定角度后的新坐标
*输入参数:
*        tRotationCenter:旋转中心点
*        tRotationPoint:旋转点
*        fRotationYaw:旋转角度
*输出参数:
*        tRotationNewPoint:右后角点坐标
*返回值:
*
*/
void RotateCoordinateOfPoint(LocationPoint tRotationCenter, LocationPoint tRotationPoint, float fRotationYaw, LocationPoint& tRotationNewPoint)
{
    tRotationNewPoint.x = tRotationCenter.x + (tRotationPoint.x - tRotationCenter.x)*cos(fRotationYaw) - (tRotationPoint.y - tRotationCenter.y)*sin(fRotationYaw);
    tRotationNewPoint.y = tRotationCenter.y + (tRotationPoint.x - tRotationCenter.x)*sin(fRotationYaw) + (tRotationPoint.y - tRotationCenter.y)*cos(fRotationYaw);
}

/*
*函数名称: SgnCus
*函数功能: //返回数字符号的整数
*输入参数:
*        fNum:数字
*输出参数:
*       -1  数字负数
*        1  数字正数
*        0  数字0
*返回值:
*
*/
int SgnCus(float fNum)
{
    if (fNum < 0)
        return -1;
    else if (fNum == 0.)
        return 0;
    else
        return 1;
}

/*
*函数名称: DistanceBetweenLineAndPoint
*函数功能: //计算点和直线距离
*输入参数:
*        fA:直线参数A
*        fB:直线参数B
*        fC:直线参数C
*        fx:点坐标X
*        fy:点坐标Y
*输出参数:
*       fd  点到直线距离
*返回值:
*
*/
float DistanceBetweenLineAndPoint(float fA, float fB, float fC, float fX, float fY)
{
    float fD;
    fD = fabs(fA * fX + fB * fY + fC) / (sqrt(fB * fB + fA* fA));
    return fD;
}

/*
*函数名称: DistanceBetweenPointAndPoint
*函数功能: //计算点和点距离
*输入参数:
*       fX1  点坐标X1
*       fY1  点坐标Y1
*       fX2  点坐标X2
*       fY2  点坐标Y2
*输出参数:
*       fD   点到点距离
*返回值:
*
*/
float DistanceBetweenPointAndPoint(float fX1, float fY1, float fX2, float fY2)
{
    float fD;
    fD = sqrt(pow(fX2 - fX1, 2)+ pow(fY2 - fY1, 2));
    return fD;
}


/*
*函数名称: CalVehRearAxleCenterRealTime
*函数功能: 车辆后轴中心点和四个角点的实时坐标计算
*输入参数:
*         tVeh:车辆的相关的信息参数（转向角，角速度，车速等）
*         fCurrentDis:车辆总的行驶距离（>=0）
*         nDirection:车辆行驶方向 0:车轮向前旋转
*                               1:车轮向后旋转
*                               2:车轮保持静止
*输出参数:
*         m_tVehCoordinateRealTime:车辆后轴中心点和四个角点的实时坐标
*返回值:
*
*/
#if 0
void CalVehRearAxleCenterRealTime(int swa, float TireFL, float TireFR, float TireRL, float TireRR, int nGear, int nPulseDirection,
                                           float &current_angle, float &current_x, float &current_y)

{
//    LocationPoint tVehRearAxleCenterRealTime;

    float fTireFRDeltaDis;
    float fTireFLDeltaDis;
    float fTireRRDeltaDis;
    float fTireRLDeltaDis;
    float fDeltaDis;
    float fDeltaTheta;
    float fVehRRTireCoordinateRealTimeX;
    float fVehRRTireCoordinateRealTimeY;
    float fVehRLTireCoordinateRealTimeX;
    float fVehRLTireCoordinateRealTimeY;
    int nRunningDirectionRealTime;
    static int nWaitTimes = 0;

    static float m_fTireFRLastDis,m_fTireFLLastDis,m_fTireRRLastDis,m_fTireRLLastDis;
    //nPulseDirection 1:Forward  2:Back 0:Static

    if ((TireFL<0.01)&&(TireFR<0.01)&&(TireRL<0.01)&&(TireRR<0.01))//init
    {
        m_fTireFRLastDis = 0.;
        m_fTireFLLastDis = 0.;
        m_fTireRRLastDis = 0.;
        m_fTireRLLastDis = 0.;
    }


    {
//        fDeltaDis = fCurrentDis - m_fLastDis;
        fTireFRDeltaDis = TireFR - m_fTireFRLastDis;
        fTireFLDeltaDis = TireFL - m_fTireFLLastDis;
        fTireRRDeltaDis = TireRR - m_fTireRRLastDis;
        fTireRLDeltaDis = TireRL - m_fTireRLLastDis;
//      m_nRunningDirectionRealTime 1:Forward -1:Back 0:Static
        if(nGear==2&&nPulseDirection!=1)
        {
            nRunningDirectionRealTime=-1;
        }
        else if(nGear==4&&nPulseDirection!=-1)
        {
            nRunningDirectionRealTime=1;
        }
        else
        {
            nRunningDirectionRealTime=0;
        }
        if(fDeltaDis>0.01&&nRunningDirectionRealTime==0)
        {
            GAC_LOG_ERROR("Vehicle dis calculate error!!!");
        }
        if((nGear==2&&nPulseDirection==1)||(nGear==4&&nPulseDirection==2)&&fDeltaDis>0.001)
        {
            GAC_LOG_WARN("车辆溜车！！！%f",fDeltaDis);
        }
//        GAC_LOG_DEBUG("nGear%d,nPulseDirection%d,m_nRunningDirectionRealTime%d",nGear,nPulseDirection,m_nRunningDirectionRealTime);

        if((fTireRLDeltaDis> 5 * 0.0216)||(fTireRRDeltaDis> 5 * 0.0216))
        {
            GAC_LOG_DEBUG("fTireRLDeltaDis %f _____fTireRRDeltaDis %f ",fTireRLDeltaDis ,fTireRRDeltaDis);
        }


//        if(nRunningDirectionRealTime==0)//todo
        if((nRunningDirectionRealTime==0||(fTireRLDeltaDis+fTireRRDeltaDis<0.07))&&(nWaitTimes<15))//todo
        {
            nWaitTimes++;
            return;
        }
//        m_fCurrentDis=fCurrentDis;
//        m_fLastDis = fCurrentDis;
        nWaitTimes = 0;
        m_fTireFRLastDis = TireFR;
        m_fTireFLLastDis = TireFL;
        m_fTireRRLastDis = TireRR;
        m_fTireRLLastDis = TireRL;
        if((fabs(fTireRLDeltaDis-fTireRRDeltaDis)>0.01)&&(abs(swa)>2)&&(nRunningDirectionRealTime!=0))
        {
            fDeltaTheta=-nRunningDirectionRealTime*(fTireRLDeltaDis-fTireRRDeltaDis)/REAR_WHEEL_SPAN;
            current_angle = current_angle +fDeltaTheta;
            fVehRRTireCoordinateRealTimeX =nRunningDirectionRealTime*2*fabs(fTireRRDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*cos(current_angle +fDeltaTheta/2);
            fVehRRTireCoordinateRealTimeY =nRunningDirectionRealTime*2*fabs(fTireRRDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*sin(current_angle +fDeltaTheta/2);
            fVehRLTireCoordinateRealTimeX =nRunningDirectionRealTime*2*fabs(fTireRLDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*cos(current_angle +fDeltaTheta/2);
            fVehRLTireCoordinateRealTimeY =nRunningDirectionRealTime*2*fabs(fTireRLDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*sin(current_angle +fDeltaTheta/2);
            current_x +=(fVehRRTireCoordinateRealTimeX+fVehRLTireCoordinateRealTimeX)/2;
            current_y +=(fVehRRTireCoordinateRealTimeY+fVehRLTireCoordinateRealTimeY)/2;
//            if((fTireRLDeltaDis-fTireRRDeltaDis)>0.00)
//            {
//                tVehRearAxleCenterRealTime.x=m_tVehRRTireCoordinateRealTime.x-REAR_WHEEL_SPAN/2*sin(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);
//                tVehRearAxleCenterRealTime.y=m_tVehRRTireCoordinateRealTime.y+REAR_WHEEL_SPAN/2*cos(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);

//            }
//            else
//            {
//                tVehRearAxleCenterRealTime.x=m_tVehRLTireCoordinateRealTime.x+REAR_WHEEL_SPAN/2*sin(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);
//                tVehRearAxleCenterRealTime.y=m_tVehRLTireCoordinateRealTime.y-REAR_WHEEL_SPAN/2*cos(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);
//            }
        }
        else
        {
            fVehRRTireCoordinateRealTimeX =nRunningDirectionRealTime*fTireRRDeltaDis*cos(current_angle);
            fVehRRTireCoordinateRealTimeY = nRunningDirectionRealTime*fTireRRDeltaDis*sin(current_angle);
            fVehRLTireCoordinateRealTimeX = nRunningDirectionRealTime*fTireRLDeltaDis*cos(current_angle);
            fVehRLTireCoordinateRealTimeY = nRunningDirectionRealTime*fTireRLDeltaDis*sin(current_angle);
            current_x +=(fVehRRTireCoordinateRealTimeX+fVehRLTireCoordinateRealTimeX)/2;
            current_y +=(fVehRRTireCoordinateRealTimeY+fVehRLTireCoordinateRealTimeY)/2;
        }
//        m_fLastYaw = tVehRearAxleCenterRealTime.yaw;
    }

    return;
}
#else

void CalVehRearAxleCenterRealTime(int swa, float TireFL, float TireFR, float TireRL, float TireRR, int nGear, int nPulseDirection,
                                           float &current_angle, float &current_x, float &current_y)

{
//    LocationPoint tVehRearAxleCenterRealTime;

    float fTireFRDeltaDis;
    float fTireFLDeltaDis;
    float fTireRRDeltaDis;
    float fTireRLDeltaDis;
    float fDeltaDis;
    float fDeltaTheta;
    float fVehRRTireCoordinateRealTimeX;
    float fVehRRTireCoordinateRealTimeY;
    float fVehRLTireCoordinateRealTimeX;
    float fVehRLTireCoordinateRealTimeY;
    int nRunningDirectionRealTime;
    static int nWaitTimes = 0;
    static int nLastSwa;
    static int nLastGear;


    static float fcurrent_angle;
    static float fcurrent_x;
    static float fcurrent_y;

    static float fcurrent_angleOut;
    static float fcurrent_xOut;
    static float fcurrent_yOut;

    static float m_fTireFRLastDis,m_fTireFLLastDis,m_fTireRRLastDis,m_fTireRLLastDis;
    static float m_fLastTireFRLastDis,m_fLastTireFLLastDis,m_fLastTireRRLastDis,m_fLastTireRLLastDis;
    //nPulseDirection 0:Forward 1:Back 2:Static

    if ((TireFL<0.01)&&(TireFR<0.01)&&(TireRL<0.01)&&(TireRR<0.01))//init
    {
        m_fTireFRLastDis = 0.;
        m_fTireFLLastDis = 0.;
        m_fTireRRLastDis = 0.;
        m_fTireRLLastDis = 0.;

        m_fLastTireFRLastDis = 0.;
        m_fLastTireFLLastDis = 0.;
        m_fLastTireRRLastDis = 0.;
        m_fLastTireRLLastDis = 0.;

        fcurrent_angle = 0.;
        fcurrent_x = 0.;
        fcurrent_y = 0.;

        fcurrent_angleOut = 0.;
        fcurrent_xOut = 0.;
        fcurrent_yOut = 0.;

        nLastSwa = -9999;
        nLastGear = -9999;

    }


    {
//        fDeltaDis = fCurrentDis - m_fLastDis;
        fTireFRDeltaDis = TireFR - m_fTireFRLastDis;
        fTireFLDeltaDis = TireFL - m_fTireFLLastDis;
        fTireRRDeltaDis = TireRR - m_fTireRRLastDis;
        fTireRLDeltaDis = TireRL - m_fTireRLLastDis;
//      m_nRunningDirectionRealTime 1:Forward -1:Back 0:Static
        if(nGear==2&&nPulseDirection!=1)
        {
            nRunningDirectionRealTime=-1;
        }
//        if(nPulseDirection==1)
//        {
//            nRunningDirectionRealTime=-1;
//        }
//        else if(nPulseDirection==0)
        else if(nGear==4&&nPulseDirection!=-1)
        {
            nRunningDirectionRealTime=1;
        }
        else
        {
            nRunningDirectionRealTime=0;
        }
        if(fDeltaDis>0.01&&nRunningDirectionRealTime==0)
        {
            GAC_LOG_ERROR("Vehicle dis calculate error!!!");
        }
//        if((nGear==2&&nPulseDirection==1)||(nGear==4&&nPulseDirection==2)&&fDeltaDis>0.001)
//        {
//            GAC_LOG_WARN("车辆溜车！！！%f",fDeltaDis);
//        }
//        GAC_LOG_DEBUG("nGear%d,nPulseDirection%d,m_nRunningDirectionRealTime%d",nGear,nPulseDirection,m_nRunningDirectionRealTime);

        if((fTireRLDeltaDis> 5 * 0.0216)||(fTireRRDeltaDis> 5 * 0.0216))
        {
            GAC_LOG_DEBUG("fTireRLDeltaDis %f _____fTireRRDeltaDis %f ",fTireRLDeltaDis ,fTireRRDeltaDis);
        }


//        if(nRunningDirectionRealTime==0)//todo
        if((nRunningDirectionRealTime==0||(fTireRLDeltaDis+fTireRRDeltaDis<0.1))&&(nWaitTimes<15))//todo
        {
            nWaitTimes++;
            return;
        }
//        m_fCurrentDis=fCurrentDis;
//        m_fLastDis = fCurrentDis;
        nWaitTimes = 0;

        if ((abs(nLastSwa-swa)>0)||(nLastGear != nGear))
        {

            fTireFRDeltaDis = TireFR - m_fLastTireFRLastDis;
            fTireFLDeltaDis = TireFL - m_fLastTireFLLastDis;
            fTireRRDeltaDis = TireRR - m_fLastTireRRLastDis;
            fTireRLDeltaDis = TireRL - m_fLastTireRLLastDis;

            m_fTireFRLastDis = TireFR;
            m_fTireFLLastDis = TireFL;
            m_fTireRRLastDis = TireRR;
            m_fTireRLLastDis = TireRL;

            fcurrent_angle = current_angle;
            fcurrent_x = current_x;
            fcurrent_y = current_y;
            nLastSwa = swa;
            nLastGear = nGear;
        }
        else
        {
            fcurrent_angle = fcurrent_angle;
            fcurrent_x = fcurrent_x;
            fcurrent_y = fcurrent_y;
        }

        m_fLastTireFRLastDis = TireFR;
        m_fLastTireFLLastDis = TireFL;
        m_fLastTireRRLastDis = TireRR;
        m_fLastTireRLLastDis = TireRL;


        if((fabs(fTireRLDeltaDis-fTireRRDeltaDis)>0.01)&&(abs(swa)>5)&&(nRunningDirectionRealTime!=0))
        {
            fDeltaTheta=-nRunningDirectionRealTime*(fTireRLDeltaDis-fTireRRDeltaDis)/REAR_WHEEL_SPAN;//1.594;//1.63;//1.605;
            fcurrent_angleOut = fcurrent_angle +fDeltaTheta;
            fVehRRTireCoordinateRealTimeX =nRunningDirectionRealTime*2*fabs(fTireRRDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*cos(fcurrent_angle +fDeltaTheta);
            fVehRRTireCoordinateRealTimeY =nRunningDirectionRealTime*2*fabs(fTireRRDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*sin(fcurrent_angle +fDeltaTheta);
            fVehRLTireCoordinateRealTimeX =nRunningDirectionRealTime*2*fabs(fTireRLDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*cos(fcurrent_angle +fDeltaTheta);
            fVehRLTireCoordinateRealTimeY =nRunningDirectionRealTime*2*fabs(fTireRLDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*sin(fcurrent_angle +fDeltaTheta);
            fcurrent_xOut = fcurrent_x +(fVehRRTireCoordinateRealTimeX+fVehRLTireCoordinateRealTimeX)/2;
            fcurrent_yOut = fcurrent_y +(fVehRRTireCoordinateRealTimeY+fVehRLTireCoordinateRealTimeY)/2;
//            if((fTireRLDeltaDis-fTireRRDeltaDis)>0.00)
//            {
//                tVehRearAxleCenterRealTime.x=m_tVehRRTireCoordinateRealTime.x-REAR_WHEEL_SPAN/2*sin(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);
//                tVehRearAxleCenterRealTime.y=m_tVehRRTireCoordinateRealTime.y+REAR_WHEEL_SPAN/2*cos(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);

//            }
//            else
//            {
//                tVehRearAxleCenterRealTime.x=m_tVehRLTireCoordinateRealTime.x+REAR_WHEEL_SPAN/2*sin(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);
//                tVehRearAxleCenterRealTime.y=m_tVehRLTireCoordinateRealTime.y-REAR_WHEEL_SPAN/2*cos(m_tVehCoordinateRealTime.tVehRearAxleCenterRealTime.yaw);
//            }
        }
        else
        {
            fVehRRTireCoordinateRealTimeX =nRunningDirectionRealTime*fTireRRDeltaDis*cos(fcurrent_angleOut);
            fVehRRTireCoordinateRealTimeY = nRunningDirectionRealTime*fTireRRDeltaDis*sin(fcurrent_angleOut);
            fVehRLTireCoordinateRealTimeX = nRunningDirectionRealTime*fTireRLDeltaDis*cos(fcurrent_angleOut);
            fVehRLTireCoordinateRealTimeY = nRunningDirectionRealTime*fTireRLDeltaDis*sin(fcurrent_angleOut);
            fcurrent_xOut = fcurrent_x +(fVehRRTireCoordinateRealTimeX+fVehRLTireCoordinateRealTimeX)/2;
            fcurrent_yOut = fcurrent_y +(fVehRRTireCoordinateRealTimeY+fVehRLTireCoordinateRealTimeY)/2;
        }
//        m_fLastYaw = tVehRearAxleCenterRealTime.yaw;
    }
    current_y = fcurrent_yOut;
    current_x = fcurrent_xOut;
    current_angle = fcurrent_angleOut;

    return;
}

#endif

void CalVehRearAxleCenterRealTime(float TireFL,float TireFR,float TireRL,float TireRR,int nGear,int nPulseDirection,
                                  float &current_angle, float &current_x, float &current_y,
                                  float &f_ego_car_motion_line_length)

{
//    LocationPoint tVehRearAxleCenterRealTime;

    float fTireFRDeltaDis;
    float fTireFLDeltaDis;
    float fTireRRDeltaDis;
    float fTireRLDeltaDis;
    float fDeltaDis;
    float fDeltaTheta;
    float fVehRRTireCoordinateRealTimeX;
    float fVehRRTireCoordinateRealTimeY;
    float fVehRLTireCoordinateRealTimeX;
    float fVehRLTireCoordinateRealTimeY;
    int nRunningDirectionRealTime;


    static float m_fTireFRLastDis,m_fTireFLLastDis,m_fTireRRLastDis,m_fTireRLLastDis;

    //if ((TireFL<0.01)&&(TireFR<0.01)&&(TireRL<0.01)&&(TireRR<0.01) || b_reset_trajec_static_variable == true)//init
    if ((TireFL < 0.01) && (TireFR < 0.01) && (TireRL < 0.01) && (TireRR < 0.01))
    {
        m_fTireFRLastDis = 0.;
        m_fTireFLLastDis = 0.;
        m_fTireRRLastDis = 0.;
        m_fTireRLLastDis = 0.;
    }

    f_ego_car_motion_line_length = 0;

    {
//        fDeltaDis = fCurrentDis - m_fLastDis;
        fTireFRDeltaDis = TireFR - m_fTireFRLastDis;
        fTireFLDeltaDis = TireFL - m_fTireFLLastDis;
        fTireRRDeltaDis = TireRR - m_fTireRRLastDis;
        fTireRLDeltaDis = TireRL - m_fTireRLLastDis;
//      m_nRunningDirectionRealTime 1:Forward -1:Back 0:Static
        if(nGear==2&&nPulseDirection!=1)
        {
            nRunningDirectionRealTime=-1;
        }
        else if(nGear==4&&nPulseDirection!=-1)
        {
            nRunningDirectionRealTime=1;
        }
        else
        {
            nRunningDirectionRealTime=0;
        }



        if(nRunningDirectionRealTime==0)//todo
        {
            return;
        }
//        m_fCurrentDis=fCurrentDis;
//        m_fLastDis = fCurrentDis;
        m_fTireFRLastDis = TireFR;
        m_fTireFLLastDis = TireFL;
        m_fTireRRLastDis = TireRR;
        m_fTireRLLastDis = TireRL;
        if(fabs(fTireRLDeltaDis-fTireRRDeltaDis)>0.00)
        {
            fDeltaTheta=-nRunningDirectionRealTime*(fTireRLDeltaDis-fTireRRDeltaDis)/REAR_WHEEL_SPAN;
            current_angle = current_angle +fDeltaTheta;
            fVehRRTireCoordinateRealTimeX =nRunningDirectionRealTime*2*fabs(fTireRRDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*cos(current_angle +fDeltaTheta/2);
            fVehRRTireCoordinateRealTimeY =nRunningDirectionRealTime*2*fabs(fTireRRDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*sin(current_angle +fDeltaTheta/2);
            fVehRLTireCoordinateRealTimeX =nRunningDirectionRealTime*2*fabs(fTireRLDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*cos(current_angle +fDeltaTheta/2);
            fVehRLTireCoordinateRealTimeY =nRunningDirectionRealTime*2*fabs(fTireRLDeltaDis/fDeltaTheta*sin(fDeltaTheta/2))*sin(current_angle +fDeltaTheta/2);
            current_x +=(fVehRRTireCoordinateRealTimeX+fVehRLTireCoordinateRealTimeX)/2;
            current_y +=(fVehRRTireCoordinateRealTimeY+fVehRLTireCoordinateRealTimeY)/2;

            f_ego_car_motion_line_length = nRunningDirectionRealTime * fabs(fTireRRDeltaDis + fTireRLDeltaDis) / fDeltaTheta * sin(fDeltaTheta / 2);
        }
        else
        {
            fVehRRTireCoordinateRealTimeX =nRunningDirectionRealTime*fTireRRDeltaDis*cos(current_angle);
            fVehRRTireCoordinateRealTimeY = nRunningDirectionRealTime*fTireRRDeltaDis*sin(current_angle);
            fVehRLTireCoordinateRealTimeX = nRunningDirectionRealTime*fTireRLDeltaDis*cos(current_angle);
            fVehRLTireCoordinateRealTimeY = nRunningDirectionRealTime*fTireRLDeltaDis*sin(current_angle);
            current_x +=(fVehRRTireCoordinateRealTimeX+fVehRLTireCoordinateRealTimeX)/2;
            current_y +=(fVehRRTireCoordinateRealTimeY+fVehRLTireCoordinateRealTimeY)/2;

            f_ego_car_motion_line_length = nRunningDirectionRealTime * (fTireRRDeltaDis + fTireRLDeltaDis) / 2;
        }
    }
    current_angle=mod2pi(current_angle);

    return;
}



int CalVehRearAxleCenterRealTimeInParkingProcess(int nSlotPosition,int nActualSteeringAngle, float fActualVelocity,
                                 float TireFL,float TireFR,
                                 float TireRL,float TireRR,int nGear,int nPulseDirection,
                                 float &current_angle, float &current_x, float &current_y)
{
    float fTireFRDeltaDis;
    float fTireFLDeltaDis;
    float fTireRRDeltaDis;
    float fTireRLDeltaDis;
    float fDeltaTheta;
    static float fTotalDisCalByFourWheel;
    static float fTotalDisCalByRearWheel;
    int nRunningDirectionRealTime;
    static float m_fTireFRLastDis,m_fTireFLLastDis,m_fTireRRLastDis,m_fTireRLLastDis;
    static float m_fLastTireFRLastDis,m_fLastTireFLLastDis,m_fLastTireRRLastDis,m_fLastTireRLLastDis;
    //nPulseDirection 0:Static 1:Forward  2:Back
    // nRunningDirectionRealTime 1:Forward -1:Back 0:Static
    if ((TireFL<0.01)&&(TireFR<0.01)&&(TireRL<0.01)&&(TireRR<0.01))//init
    {
        fTotalDisCalByFourWheel=0;
        fTotalDisCalByRearWheel=0;
        m_fTireFRLastDis = 0.;
        m_fTireFLLastDis = 0.;
        m_fTireRRLastDis = 0.;
        m_fTireRLLastDis = 0.;
        m_fLastTireFRLastDis = 0.;
        m_fLastTireFLLastDis = 0.;
        m_fLastTireRRLastDis = 0.;
        m_fLastTireRLLastDis = 0.;
    }
    fTireFRDeltaDis = TireFR - m_fTireFRLastDis;
    fTireFLDeltaDis = TireFL - m_fTireFLLastDis;
    fTireRRDeltaDis = TireRR - m_fTireRRLastDis;
    fTireRLDeltaDis = TireRL - m_fTireRLLastDis;
    if(nGear==2&&nPulseDirection!=1)
    {
        nRunningDirectionRealTime=-1;
    }
    else if(nGear==4&&nPulseDirection!=2)
    {
        nRunningDirectionRealTime=1;
    }
    else
    {
        nRunningDirectionRealTime=0;
    }


    if(nGear==2&&nPulseDirection==1)
    {
        GAC_LOG_WARN("车辆挂倒挡时向前遛车!!!");
    }
    if(nGear==4&&nPulseDirection==2)
    {
        GAC_LOG_WARN("车辆挂前进挡时向后遛车!!!");
    }
//    GAC_LOG_DEBUG("nRunningDirectionRealTime!!!%d;fTireFRDeltaDis%f",nRunningDirectionRealTime,fTireFRDeltaDis);

    m_fTireFRLastDis = TireFR;
    m_fTireFLLastDis = TireFL;
    m_fTireRRLastDis = TireRR;
    m_fTireRLLastDis = TireRL;
    if(fabs(nActualSteeringAngle)!=0)
    {
        int nSteeringDirection;
        float fRearAxleCenterRadius;
        float fFLWheelRadius;
        float fFRWheelRadius;
        float fRLWheelRadius;
        float fRRWheelRadius;
        nSteeringDirection=SgnCus(nActualSteeringAngle);
        fRearAxleCenterRadius=CalSteeringWheelAngle2Radius(nSteeringDirection, nRunningDirectionRealTime, abs(nActualSteeringAngle));
        CalFourWheelRadius(nSteeringDirection, fRearAxleCenterRadius,
                                   fRRWheelRadius, fFRWheelRadius, fFLWheelRadius, fRLWheelRadius);
       // 当车位方向为左侧车位时，需要将坐标系以X轴镜像，因此需要将车辆左右测车轮行驶距离和转弯半径以及方向盘转动方向调换
        fDeltaTheta= nRunningDirectionRealTime*nSteeringDirection*nSlotPosition*(fTireFLDeltaDis/fFLWheelRadius+fTireFRDeltaDis/fFRWheelRadius
                                                                  +fTireRLDeltaDis/fRLWheelRadius+fTireRRDeltaDis/fRRWheelRadius)/4;
        fTotalDisCalByFourWheel+=fabs(fDeltaTheta*fRearAxleCenterRadius);
        fTotalDisCalByRearWheel+=(fTireRRDeltaDis+fTireRLDeltaDis)/2;
        current_x += nRunningDirectionRealTime*2*fabs(fRearAxleCenterRadius*sin(fDeltaTheta/2))*cos(current_angle +fDeltaTheta/2);
        current_y += nRunningDirectionRealTime*2*fabs(fRearAxleCenterRadius*sin(fDeltaTheta/2))*sin(current_angle +fDeltaTheta/2);
        current_angle += fDeltaTheta;
    }
    else
    {
        float fDeltaAverageDis=(fTireFLDeltaDis+fTireFRDeltaDis+fTireRLDeltaDis+fTireRRDeltaDis)/4;
        current_x += nRunningDirectionRealTime*fDeltaAverageDis*cos(current_angle);
        current_y += nRunningDirectionRealTime*fDeltaAverageDis*sin(current_angle);
        fTotalDisCalByFourWheel+=fDeltaAverageDis;
        fTotalDisCalByRearWheel+=fDeltaAverageDis;
    }
//    GAC_LOG_DEBUG("fTotalDisCalByFourWheel%f,fTotalDisCalByRearWheel%f",fTotalDisCalByFourWheel,fTotalDisCalByRearWheel);
    current_angle=mod2pi(current_angle);
    return 0;
}



int CalVehRearAxleCenterRealTimeInParkingProcessAnother(int nSlotPosition,int nActualSteeringAngle, float fActualVelocity,
                                 float TireFL,float TireFR,
                                 float TireRL,float TireRR,int nGear,int nPulseDirection,
                                 float &current_angle, float &current_x, float &current_y)
{
    float fTireFRDeltaDis;
    float fTireFLDeltaDis;
    float fTireRRDeltaDis;
    float fTireRLDeltaDis;
    float fDeltaTheta;
    static float fTotalDisCalByFourWheel;
    static float fTotalDisCalByRearWheel;
    int nRunningDirectionRealTime;
    static float m_fTireFRLastDis,m_fTireFLLastDis,m_fTireRRLastDis,m_fTireRLLastDis;
    static float m_fLastTireFRLastDis,m_fLastTireFLLastDis,m_fLastTireRRLastDis,m_fLastTireRLLastDis;
    //nPulseDirection 0:Static 1:Forward  2:Back
    // nRunningDirectionRealTime 1:Forward -1:Back 0:Static
    if ((TireFL<0.01)&&(TireFR<0.01)&&(TireRL<0.01)&&(TireRR<0.01))//init
    {
        fTotalDisCalByFourWheel=0;
        fTotalDisCalByRearWheel=0;
        m_fTireFRLastDis = 0.;
        m_fTireFLLastDis = 0.;
        m_fTireRRLastDis = 0.;
        m_fTireRLLastDis = 0.;
        m_fLastTireFRLastDis = 0.;
        m_fLastTireFLLastDis = 0.;
        m_fLastTireRRLastDis = 0.;
        m_fLastTireRLLastDis = 0.;
    }
    fTireFRDeltaDis = TireFR - m_fTireFRLastDis;
    fTireFLDeltaDis = TireFL - m_fTireFLLastDis;
    fTireRRDeltaDis = TireRR - m_fTireRRLastDis;
    fTireRLDeltaDis = TireRL - m_fTireRLLastDis;
    if(nGear==2&&nPulseDirection!=1)
    {
        nRunningDirectionRealTime=-1;
    }
    else if(nGear==4&&nPulseDirection!=2)
    {
        nRunningDirectionRealTime=1;
    }
    else
    {
        nRunningDirectionRealTime=0;
    }
    if(nGear==2&&nPulseDirection==1)
    {
        GAC_LOG_WARN("车辆挂倒挡时向前遛车!!!");
    }
    if(nGear==4&&nPulseDirection==2)
    {
        GAC_LOG_WARN("车辆挂前进挡时向后遛车!!!");
    }
//    GAC_LOG_DEBUG("nRunningDirectionRealTime!!!%d;fTireFRDeltaDis%f",nRunningDirectionRealTime,fTireFRDeltaDis);

    m_fTireFRLastDis = TireFR;
    m_fTireFLLastDis = TireFL;
    m_fTireRRLastDis = TireRR;
    m_fTireRLLastDis = TireRL;
    if(fabs(nActualSteeringAngle)!=0)
    {
        int nSteeringDirection;
        float fRearAxleCenterRadius;
        float fFLWheelRadius;
        float fFRWheelRadius;
        float fRLWheelRadius;
        float fRRWheelRadius;
        nSteeringDirection=SgnCus(nActualSteeringAngle);
        fRearAxleCenterRadius=CalSteeringWheelAngle2Radius(nSteeringDirection, nRunningDirectionRealTime, abs(nActualSteeringAngle));
        CalFourWheelRadius(nSteeringDirection, fRearAxleCenterRadius,
                                   fRRWheelRadius, fFRWheelRadius, fFLWheelRadius, fRLWheelRadius);
       // 当车位方向为左侧车位时，需要将坐标系以X轴镜像，因此需要将车辆左右测车轮行驶距离和转弯半径以及方向盘转动方向调换
        fDeltaTheta= nRunningDirectionRealTime*nSteeringDirection*nSlotPosition*(fTireFLDeltaDis/fFLWheelRadius+fTireFRDeltaDis/fFRWheelRadius
                                                                  +fTireRLDeltaDis/fRLWheelRadius+fTireRRDeltaDis/fRRWheelRadius)/4;

        fTotalDisCalByFourWheel+=fabs(fDeltaTheta*fRearAxleCenterRadius);
        fTotalDisCalByRearWheel+=(fTireRRDeltaDis+fTireRLDeltaDis)/2;
        current_x += nRunningDirectionRealTime*2*fabs(fRearAxleCenterRadius*sin(fDeltaTheta/2))*cos(current_angle +fDeltaTheta/2);
        current_y += nRunningDirectionRealTime*2*fabs(fRearAxleCenterRadius*sin(fDeltaTheta/2))*sin(current_angle +fDeltaTheta/2);
        current_angle += fDeltaTheta;
    }
    else
    {
        float fDeltaAverageDis=(fTireFLDeltaDis+fTireFRDeltaDis+fTireRLDeltaDis+fTireRRDeltaDis)/4;
        current_x += nRunningDirectionRealTime*fDeltaAverageDis*cos(current_angle);
        current_y += nRunningDirectionRealTime*fDeltaAverageDis*sin(current_angle);
        fTotalDisCalByFourWheel+=fDeltaAverageDis;
        fTotalDisCalByRearWheel+=fDeltaAverageDis;

    }
//    GAC_LOG_DEBUG("fTotalDisCalByFourWheel%f,fTotalDisCalByRearWheel%f",fTotalDisCalByFourWheel,fTotalDisCalByRearWheel);
    current_angle=mod2pi(current_angle);
    return 0;
}

int round_double(double number)
{
    return (number > 0.0) ? (number + 0.5) : (number - 0.5);
}

int CalPointsOfLine(std::pair<int, int> tP0, std::pair<int, int> tP1, std::vector<std::pair<int, int>>& vLine)
{

    int nRet = -1;
    if ((tP0.first <0)||(tP1.first <0)||(tP0.second <0)||(tP1.second <0))
    {
        ;
    }
    else
    {
        vLine.clear();
        std::pair <int,int> p;
        double fLen = hypot(tP0.first-tP1.first, tP0.second-tP1.second)/0.7;
        int n = round_double(fLen);
        for (int i=1; i<n+1; i++)
        {

            p.first = round_double( tP0.first + (tP1.first - tP0.first)*i/n);
            p.second = round_double( tP0.second + (tP1.second - tP0.second)*i/n);

            vLine.push_back(p);

        }

        p.first = round_double(tP0.first);
        p.second = round_double(tP0.second);

        vLine.push_back(p);

        p.first = round_double(tP1.first);
        p.second = round_double(tP1.second);

        vLine.push_back(p);


    }


    return nRet;
}





int CalVehRearAxleCenterRealTimeSimulation(int nActualSteeringAngle, float fActualVelocity,
                                           int nRunningDirectionRealTime,
                                           float &current_angle, float &current_x, float &current_y)
{
    float detaT=0.02;
    float fDeltaTheta;
    //nPulseDirection 0:Static 1:Forward  2:Back
    if(fabs(nActualSteeringAngle)!=0)
    {
        int nSteeringDirection;
        float fRearAxleCenterRadius;
        nSteeringDirection=SgnCus(nActualSteeringAngle);
        fRearAxleCenterRadius=CalSteeringWheelAngle2Radius(nSteeringDirection, nRunningDirectionRealTime, abs(nActualSteeringAngle));
       // 当车位方向为左侧车位时，需要将坐标系以X轴镜像，因此需要将车辆左右测车轮行驶距离和转弯半径以及方向盘转动方向调换
        current_x += nRunningDirectionRealTime*fabs(fActualVelocity*detaT)*cos(current_angle);
        current_y += nRunningDirectionRealTime*fabs(fActualVelocity*detaT)*sin(current_angle);
        fDeltaTheta= nRunningDirectionRealTime*nSteeringDirection*fActualVelocity*detaT/fRearAxleCenterRadius;
        current_angle += fDeltaTheta;
    }
    else
    {
        float fDeltaAverageDis=fActualVelocity*detaT;
        current_x += nRunningDirectionRealTime*fDeltaAverageDis*cos(current_angle);
        current_y += nRunningDirectionRealTime*fDeltaAverageDis*sin(current_angle);
    }
    current_angle=mod2pi(current_angle);
    return 0;
}

/*
*函数名称: CalVehicleInitPositionBasedOnGroundCoordinateSystem
*函数功能: 建立车位大地坐标系，并给出车辆在大地坐标系下后轴中心的坐标
*输入参数:
*        nSlotPosition:车位方向:1右侧，-1：左侧
*        p0,p1车位角点
*
*输出参数:
*        VehRearAxleCenterPosition:车辆后轴中心坐标
*
*
*返回值:
*
*
*/
void CalVehicleInitPositionBasedOnGroundCoordinateSystem(int nSlotPosition,
                                                         LocationPoint p0,
                                                         LocationPoint p1,
                                                         LocationPoint &VehRearAxleCenterPosition)
{
    float yaw;
    LocationPoint Newp0;
    LocationPoint Newp1;
    LocationPoint tRotationCenter;
    if(nSlotPosition==RIGHTSIDESLOT)
    {
        yaw=atan2(p0.y-p1.y,p0.x-p1.x);
        tRotationCenter.x=0;
        tRotationCenter.y=0;
        RotateCoordinateOfPoint(tRotationCenter, p0, -yaw, Newp0);
        RotateCoordinateOfPoint(tRotationCenter, p1, -yaw, Newp1);
        VehRearAxleCenterPosition.x=0-Newp1.x;
        VehRearAxleCenterPosition.y=0-Newp1.y;
        VehRearAxleCenterPosition.yaw=-yaw;
    }
    else
    {
        yaw=atan2(p1.y-p0.y,p1.x-p0.x);
        tRotationCenter.x=0;
        tRotationCenter.y=0;
        RotateCoordinateOfPoint(tRotationCenter, p0, -yaw, Newp0);
        RotateCoordinateOfPoint(tRotationCenter, p1, -yaw, Newp1);
        VehRearAxleCenterPosition.x=0-Newp0.x;
        VehRearAxleCenterPosition.y=Newp0.y;
        VehRearAxleCenterPosition.yaw=yaw;
    }

}

void CalUpdatedVehiclePositionBasedOnGroundCoordinateSystem(int nSlotPosition,
                                                         LocationPoint p0,
                                                         LocationPoint p1,
                                                         LocationPoint &CurrentVehRearAxleCenterPosition)
{
    float yaw;
    LocationPoint Newp0;
    LocationPoint Newp1;
    LocationPoint OppNewp0;
    LocationPoint OppNewp1;

    LocationPoint tRotationCenter;
    LocationPoint NewCurrentVehRearAxleCenterPosition;
    if(nSlotPosition==RIGHTSIDESLOT)
    {
        yaw=CurrentVehRearAxleCenterPosition.yaw;
        tRotationCenter.x=0;
        tRotationCenter.y=0;
        RotateCoordinateOfPoint(tRotationCenter, p0, -yaw, Newp0);
        RotateCoordinateOfPoint(tRotationCenter, p1, -yaw, Newp1);
        RotateCoordinateOfPoint(tRotationCenter, CurrentVehRearAxleCenterPosition, -yaw, NewCurrentVehRearAxleCenterPosition);

        OppNewp0.x=Newp0.x-NewCurrentVehRearAxleCenterPosition.x;
        OppNewp0.y=Newp0.y-NewCurrentVehRearAxleCenterPosition.y;

        OppNewp1.x=Newp1.x-NewCurrentVehRearAxleCenterPosition.x;
        OppNewp1.y=Newp1.y-NewCurrentVehRearAxleCenterPosition.y;

        CalVehicleInitPositionBasedOnGroundCoordinateSystem(nSlotPosition,
                                                             OppNewp0,
                                                             OppNewp1,
                                                             CurrentVehRearAxleCenterPosition);
    }
    else
    {
        CurrentVehRearAxleCenterPosition.y = -CurrentVehRearAxleCenterPosition.y;
        CurrentVehRearAxleCenterPosition.yaw = -CurrentVehRearAxleCenterPosition.yaw;
        p0.y = -p0.y;
        p1.y = -p1.y;

        yaw=CurrentVehRearAxleCenterPosition.yaw;
        tRotationCenter.x=0;
        tRotationCenter.y=0;
        RotateCoordinateOfPoint(tRotationCenter, p0, -yaw, Newp0);
        RotateCoordinateOfPoint(tRotationCenter, p1, -yaw, Newp1);
        RotateCoordinateOfPoint(tRotationCenter, CurrentVehRearAxleCenterPosition, -yaw, NewCurrentVehRearAxleCenterPosition);

        OppNewp0.x=Newp0.x-NewCurrentVehRearAxleCenterPosition.x;
        OppNewp0.y=NewCurrentVehRearAxleCenterPosition.y-Newp0.y;

        OppNewp1.x=Newp1.x-NewCurrentVehRearAxleCenterPosition.x;
        OppNewp1.y=NewCurrentVehRearAxleCenterPosition.y-Newp1.y;

        CalVehicleInitPositionBasedOnGroundCoordinateSystem(nSlotPosition,
                                                             OppNewp0,
                                                             OppNewp1,
                                                             CurrentVehRearAxleCenterPosition);
        CurrentVehRearAxleCenterPosition.y = -CurrentVehRearAxleCenterPosition.y;
        CurrentVehRearAxleCenterPosition.yaw = -CurrentVehRearAxleCenterPosition.yaw;
    }

}



LocationPoint TransFromWorld2Veh(const LocationPoint& tCurVhPose,
		 									const LocationPoint& tSrcPoint)
{
 	LocationPoint tTargPoint;
	 tTargPoint.x = (tSrcPoint.x - tCurVhPose.x) * cos(tCurVhPose.yaw)
				 + (tSrcPoint.y - tCurVhPose.y) * sin(tCurVhPose.yaw);
	 tTargPoint.y = (tSrcPoint.y - tCurVhPose.y) * cos(tCurVhPose.yaw)
				  - (tSrcPoint.x - tCurVhPose.x) * sin(tCurVhPose.yaw);
 
	 return tTargPoint;

}



/*
*函数名称: CalSlotCoordinateBasedOnVehicleCoordinateSystem
*函数功能: 建立车辆坐标系，并给出车位四个角点p0,p1,p2,p3相对于车辆坐标系的坐标
*输入参数:
*        nSlotPosition:车位方向:1右侧，-1：左侧
*        fParkingLength:原始车位长度
*        fParkingWidth:原始车位宽度
*        VehRearAxleCenterPosition:车辆后轴中心坐标
*
*输出参数:
*        p0,p1,p2,p3:车位四个角点
*
*
*返回值:
*
*
*/
void CalSlotCoordinateBasedOnVehicleCoordinateSystem(int nSlotPosition,
                                                     float fParkingDepth,
                                                     float fFrontEdge,
                                                     float fRearEdge,
                                                     LocationPoint tVeh,
                                                     LocationPoint &p0,
                                                     LocationPoint &p1,
                                                     LocationPoint &p2,
                                                     LocationPoint &p3)
{

    LocationPoint tP0;
    LocationPoint tP1;
    LocationPoint tP2;
    LocationPoint tP3;
    LocationPoint tRotationCenter(0,0,0);
    LocationPoint tNewVeh;

    if(nSlotPosition== RIGHTSIDESLOT)
    {
        tP0.x = fFrontEdge;
        tP0.y = 0;

        tP1.x = fRearEdge;
        tP1.y = 0;

        tP2.x = tP1.x;
        tP2.y = tP1.y - fParkingDepth;

        tP3.x = tP0.x;
        tP3.y = tP2.y;
    }
    else
    {
        tP0.x = fRearEdge;
        tP0.y = 0;

        tP1.x = fFrontEdge;
        tP1.y = 0;

        tP2.x = tP1.x;
        tP2.y = tP1.y + fParkingDepth;

        tP3.x = tP0.x;
        tP3.y = tP2.y;
    }

    RotateCoordinateOfPoint(tRotationCenter, tP0, -tVeh.yaw, tP0);
    RotateCoordinateOfPoint(tRotationCenter, tP1, -tVeh.yaw, tP1);
    RotateCoordinateOfPoint(tRotationCenter, tP2, -tVeh.yaw, tP2);
    RotateCoordinateOfPoint(tRotationCenter, tP3, -tVeh.yaw, tP3);
    RotateCoordinateOfPoint(tRotationCenter, tVeh, -tVeh.yaw, tNewVeh);

    p0.x = tP0.x - tNewVeh.x;
    p0.y = tP0.y - tNewVeh.y;

    p1.x = tP1.x - tNewVeh.x;
    p1.y = tP1.y - tNewVeh.y;

    p2.x = tP2.x - tNewVeh.x;
    p2.y = tP2.y - tNewVeh.y;

    p3.x = tP3.x - tNewVeh.x;
    p3.y = tP3.y - tNewVeh.y;
}

float mod2pi(float x)
{
    double v = fmod(x, 2*PI);
    if (v < -PI)
        v += 2*PI;
    else
        if (v > PI)
            v -= 2*PI;
    return v;
}
float mod2twopi(float x)
{
    double v = fmod(x, 2*PI);
    if (v < 0)
        v += 2*PI;
    else
        ;
    return v;
}


Line GetLineThetaOfManyPoints(LocationPoint *p)
{
    float x0=p->x;
    float y0=p->y;
    float x;
    float y;
    static float xLast = -99999.;
    static float yLast = -99999.;

    float sum_x2 = 0.0;
    float sum_y2 = 0.0;
    float sum_y = 0.0;
    float sum_x = 0.0;
    float sum_xy = 0.0;
    double mx=0;
    double my=0;
    int nCountPoint;

	if (p != NULL) 
	{
	    while(!p){
	        if ((fabs(p->x-xLast)>0.001)||(fabs(p->y-yLast)>0.001))
	        {
	            x = p->x;
	            y = p->y;
	            sum_x2 += x * x;
	            sum_y += y;
	            sum_x += x;
	            sum_xy += x * y;
	            sum_y2+=y*y;
	            nCountPoint++;
	        }else{
	            xLast = p->x;
	            yLast = p->y;
	        }
	        p++;
	    }
	}

    mx = sum_x / nCountPoint;
    my = sum_y / nCountPoint;
    float Lxx=sum_x2-sum_x*sum_x/nCountPoint;
    float Lxy=sum_xy-sum_x*sum_y/nCountPoint;
    float Lyy=sum_y2-sum_y*sum_y/nCountPoint;

    float a=Lxy;
    float b=-Lxx;
    float c=Lxx*my-Lxy*mx;
    float r=(Lxx<1e-5||Lyy<1e-5)?0:fabs(Lxy)/sqrt(Lxx*Lyy);//0-1

    float theta=atan2(-a,b);
    float theta1=(theta<0)?theta+PI:theta-PI;

    float ref=atan2(y-y0,x-x0);


//    return fabs(theta-ref)<fabs(theta1-ref)?theta:theta1;
    Line ans;
    ans.a=a;
    ans.b=b;
    ans.c=c;
    ans.r=r;
//    ans.theta=fabs(theta-ref)<fabs(theta1-ref)?theta:theta1;

    return ans;
}



bool pointInPolygon(float x, float y, int polySides, float* polyX, float* polyY)
{

  int   i,j=polySides-1;
  bool  oddNodes=false;

  for (i=0;i<polySides; i++) {
    if((polyY[i]< y && polyY[j]>=y
    ||   polyY[j]<y && polyY[i]>=y)
    && (polyX[i]<=x || polyX[j]<=x)) {
      oddNodes^=(polyX[i]+(y-polyY[i])/(polyY[j]-polyY[i])*(polyX[j]-polyX[i])<x);}
    j=i;}

  return oddNodes;
}

bool intersection(float l1x1, float l1y1, float l1x2, float l1y2,
                  float l2x1, float l2y1, float l2x2, float l2y2)
{
    //快速排斥实验
    if ((l1x1 > l1x2 ? l1x1 : l1x2) < (l2x1 < l2x2 ? l2x1 : l2x2) ||
        (l1y1 > l1y2 ? l1y1 : l1y2) < (l2y1 < l2y2 ? l2y1 : l2y2) ||
        (l2x1 > l2x2 ? l2x1 : l2x2) < (l1x1 < l1x2 ? l1x1 : l1x2) ||
        (l2y1 > l2y2 ? l2y1 : l2y2) < (l1y1 < l1y2 ? l1y1 : l1y2))
    {
        return false;
    }
    //跨立实验
    if ((((l1x1 - l2x1)*(l2y2 - l2y1) - (l1y1 - l2y1)*(l2x2 - l2x1))*
        ((l1x2 - l2x1)*(l2y2 - l2y1) - (l1y2 - l2y1)*(l2x2 - l2x1))) > 0 ||
        (((l2x1 - l1x1)*(l1y2 - l1y1) - (l2y1 - l1y1)*(l1x2 - l1x1))*
        ((l2x2 - l1x1)*(l1y2 - l1y1) - (l2y2 - l1y1)*(l1x2 - l1x1))) > 0)
    {
        return false;
    }
    return true;
}


bool lineCrossPolygon(float lx1, float ly1,float lx2, float ly2, int polySides, float* polyX, float* polyY)//线段和多边形是否相交
{

    int   i,j=polySides-1;
    bool  oddNodes=false;


    for (i=0;i<polySides; i++)
    {
        oddNodes = intersection(lx1,ly1, lx2,ly2,
                                polyX[i], polyY[i], polyX[j], polyY[j]);
        j=i;
        if (oddNodes)//相交退出
        {
            break;
        }
    }



    return oddNodes;
}


void CalFrontAxleCenterCoordinate( LocationPoint tVehicleRearAxleCenter, LocationPoint& tFrontAxleCenter)
{
    tFrontAxleCenter.x = tVehicleRearAxleCenter.x+WHEEL_BASE*cos(tVehicleRearAxleCenter.yaw);
    tFrontAxleCenter.y = tVehicleRearAxleCenter.x+WHEEL_BASE*sin(tVehicleRearAxleCenter.yaw);
}
