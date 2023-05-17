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
#include "clothoid_parkout.h"
#include "constants.h"
#include <cmath>
// #define printf(format, ...)

namespace HybridAStarPart
{

//namespace Constants
//{
//    static const float rear_chamfer_l1 =  0.2f;
//    static const float rear_chamfer_l2 =  0.2f;
//    static const float min_steering_radius = 1.0f/0.521447f;
//    static const float para_slot_corner_safety_dis = CORNER_SAFETY_DIS;
//    static const float step_size = 0.05f;
//};

//参数
static float feasible_corner_safety_dis = 0.0f;
static std::vector<LocationPoint> PoseSeg(6);
static std::vector<LocationPoint> ClothoidSeg; //回旋线查表

//左转向
static float min_steering_radius1 = 0.0f;
static float ClothoidAngle1 = 0.0f;
static float ClothoidForwardX1 = 0.0f;
static float ClothoidForwardY1 = 0.0f;
static float ClothoidBackwardX1 = 0.0f;
static float ClothoidBackwardY1 = 0.0f;
static float fTangentDist1 = 0.0f;
static float fHalfCurvDist1 = 0.0f;

//右转向
static float min_steering_radius2 = 0.0f;
static float ClothoidAngle2 = 0.0f;
static float ClothoidForwardX2 = 0.0f;
static float ClothoidForwardY2 = 0.0f;
static float ClothoidBackwardX2 = 0.0f;
static float ClothoidBackwardY2 = 0.0f;
static float fTangentDist2 = 0.0f;
static float fHalfCurvDist2 = 0.0f;

static float Sign(float fValue)
{
    if (fValue > 0.0f)
        return 1.0f;
    else if (fValue < 0.0f)
        return -1.0f;
    else
        return 0.0f;
}

void InitClothoid()
{
    int nNum = CLOTHOID_LENGTH;
    float fSigma;
    float fSk, fSkh, fSkp;
    LocationPoint TmpPoint(0.0f,0.0f,0.0f);

    fSigma = 1/Constants::min_steering_radius/(CLOTHOID_LENGTH*Constants::step_size);

    ClothoidSeg.clear();
    ClothoidSeg.push_back(TmpPoint);
    for (int i=0; i<nNum; ++i)
    {
        fSk = i*Constants::step_size;
        fSkh = fSk + 0.5f*Constants::step_size;
        fSkp = fSk + Constants::step_size;

        TmpPoint.x += Constants::step_size/6*(cosf(0.5f*fSigma*powf(fSk,2)) + 4.0f*cosf(0.5f*fSigma*powf(fSkh,2)) + cosf(0.5f*fSigma*powf(fSkp,2)));
        TmpPoint.y += Constants::step_size/6*(sinf(0.5f*fSigma*powf(fSk,2)) + 4.0f*sinf(0.5f*fSigma*powf(fSkh,2)) + sinf(0.5f*fSigma*powf(fSkp,2)));
        TmpPoint.yaw = 0.5f*fSigma*powf(fSkp,2);
        ClothoidSeg.push_back(TmpPoint);
    }
    ClothoidAngle1 = ClothoidAngle2 = ClothoidSeg.back().yaw;
    ClothoidForwardX1 = ClothoidForwardX2 = ClothoidSeg.back().x;
    ClothoidForwardY1 = ClothoidForwardY2 = ClothoidSeg.back().y;
    ClothoidBackwardX1 = ClothoidBackwardX2 = ClothoidForwardX1*cosf(ClothoidAngle1) + ClothoidForwardY1*sinf(ClothoidAngle1);
    ClothoidBackwardY1 = ClothoidBackwardY2 = ClothoidForwardX1*sinf(ClothoidAngle1) - ClothoidForwardY1*cosf(ClothoidAngle1);
    fTangentDist1 = fTangentDist2 = Constants::min_steering_radius*cosf(ClothoidAngle1) + ClothoidForwardY1;
    fHalfCurvDist1 = fHalfCurvDist2 = ClothoidForwardX1 - Constants::min_steering_radius*sinf(ClothoidAngle1);
}

float CalcMidAngle(LocationPoint tPoint, float fCX, float fCY)
{
    float fCenterX, fCenterY;
    float fSecondCenterX, fSecondCenterY;
    float fMinAngle, fMinAngle0;
    float fTmpCenterX, fTmpCenterY;
    float fTmpAngle = 0.0f;
    float fCornerMinDist, fSafeLateralDist, fMinCenterDist;
    float fCutCoef;
    float a, b, c;

    min_steering_radius1 = min_steering_radius2 = Constants::min_steering_radius;
    feasible_corner_safety_dis = std::min(Constants::para_slot_corner_safety_dis, Constants::min_steering_radius -
                                          VEHICLE_WID*0.5f - REAR_VIEW_LENGTH);

    fCenterX = tPoint.x - min_steering_radius1*sinf(tPoint.yaw);
    fCenterY = tPoint.y + min_steering_radius1*cosf(tPoint.yaw);

    fCornerMinDist = VEHICLE_WID*0.5f + REAR_VIEW_LENGTH + feasible_corner_safety_dis;
    fSafeLateralDist = std::max(LATERAL_SAFETY_DIS, REAR_VIEW_LENGTH + feasible_corner_safety_dis +
                                fCY + fTangentDist1 - min_steering_radius1) + VEHICLE_WID*0.5f;
    fMinCenterDist = sqrtf(powf(fTangentDist1,2) + powf(fHalfCurvDist1,2));

    //默认出库方式
    a = tanf(RECOMMEND_ANGLE); b = -1.0f;
    c = fCenterY - tanf(RECOMMEND_ANGLE)*fCenterX - fTangentDist1/cosf(RECOMMEND_ANGLE);
    fSecondCenterY = fSafeLateralDist - fTangentDist1;
    fSecondCenterX = -(b*fSecondCenterY + c)/a + fTangentDist1*sqrtf(powf(a,2)+powf(b,2))/fabs(a);
    if (sqrtf(powf(fSecondCenterX-fCenterX,2) + powf(fSecondCenterY-fCenterY,2)) < 2*fMinCenterDist)
    {
        fTmpAngle = asinf((fSecondCenterY - fCenterY)/(2*fMinCenterDist));
        fSecondCenterX = fCenterX + 2*fMinCenterDist*cosf(fTmpAngle);
    }
    fMinAngle = atan2f(fSecondCenterY - fCenterY, fSecondCenterX - fCenterX) +
        asinf(fTangentDist1/(0.5f*sqrtf(powf(fSecondCenterX-fCenterX,2) + powf(fSecondCenterY-fCenterY,2))));
    fMinAngle = std::max(fMinAngle, 0.0f);
    if ((fCX > fCenterX && fCY + (fCornerMinDist + fTangentDist1 - min_steering_radius1) < fCenterY - fTangentDist1 + ZERO_LENGTH) ||
        (fCX < fCenterX && fCY - (fCornerMinDist + fTangentDist1 - min_steering_radius1) < fCenterY + fTangentDist1 + ZERO_LENGTH))
    {
        return fMinAngle; //保证避障角度为正
    }

    //出库角度限制(库角)
    if (sqrtf(powf(fCX-fCenterX,2) + powf(fCY-fCenterY,2)) > fTangentDist1 + fCornerMinDist+fTangentDist1-min_steering_radius1)
    {
        fCutCoef = fTangentDist1/(fTangentDist1 + fCornerMinDist+fTangentDist1-min_steering_radius1);
        fTmpAngle = atan2(fCY - fCenterY, fCX - fCenterX) +
            asinf(fTangentDist1/(fCutCoef*sqrtf(powf(fCX-fCenterX,2) + powf(fCY-fCenterY,2))));
        fTmpCenterY = fCY - (min_steering_radius1 - fCornerMinDist) * cosf(fTmpAngle); //阈值
        if (fSafeLateralDist - fTangentDist1 > fTmpCenterY)
        {
            a = sinf(fTmpAngle);
            b = -cosf(fTmpAngle);
            c = fCenterY*cosf(fTmpAngle) - sinf(fTmpAngle)*fCenterX - fTangentDist1;
            fTmpCenterY = fSafeLateralDist - fTangentDist1;
            fTmpCenterX = -(b*fTmpCenterY + c)/a + fTangentDist1*sqrtf(powf(a,2)+powf(b,2))/fabs(a);
        }
        else
        {
            fTmpCenterY = fSafeLateralDist - fTangentDist1;
            a = fCY - fTmpCenterY;
            b = min_steering_radius1 - fCornerMinDist;
            c = sqrtf(fabs(powf(b,2) - powf(a,2)));
            fTmpCenterX = fCX + c;
        }

        if (sqrtf(powf(fTmpCenterY-fCenterY,2) + powf(fTmpCenterX-fCenterX,2)) > 2*fMinCenterDist)
        {
            //横向距离可达
            if (fabs(fTmpCenterY-fCenterY) < 2*fMinCenterDist)
            {
                a = fTmpCenterY - fCenterY;
                b = 2*fMinCenterDist;
                c = sqrtf(fabs(powf(b,2) - powf(a,2)));
                fSecondCenterX = std::max(fSecondCenterX, fCenterX + c);
            }
            fSecondCenterX = std::min(fSecondCenterX, fTmpCenterX);
            fMinAngle0 = atan2f(fSecondCenterY - fCenterY, fSecondCenterX - fCenterX) +
                asinf(fTangentDist1/(0.5f*sqrtf(powf(fSecondCenterX-fCenterX,2) + powf(fSecondCenterY-fCenterY,2))));
        }
        else
        {
            //增大横向距离
            a = min_steering_radius1 - fCornerMinDist;
            b = 2*fMinCenterDist;
            c = sqrtf(powf(fCX-fCenterX,2) + powf(fCY-fCenterY,2));
            fTmpAngle = atan2f(fCY - fCenterY, fCX - fCenterX) - acosf((powf(b,2)+powf(c,2)-powf(a,2))/(2*b*c));
            fSecondCenterX = fCenterX + b*cosf(fTmpAngle);
            fSecondCenterY = fCenterY + b*sinf(fTmpAngle);
            fMinAngle0 = atan2f(fSecondCenterY - fCenterY, fSecondCenterX - fCenterX) +
                asinf(fTangentDist1/(0.5f*sqrtf(powf(fSecondCenterX-fCenterX,2) + powf(fSecondCenterY-fCenterY,2))));
        }

        //强制转向
        fCutCoef = min_steering_radius1/(min_steering_radius1+0.5f*VEHICLE_WID+feasible_corner_safety_dis);
        fTmpAngle = atan2f(fCY - fCenterY, fCX - fCenterX) +
            asinf(min_steering_radius1/(fCutCoef*sqrtf(powf(fCX-fCenterX,2) + powf(fCY-fCenterY,2)))) + ClothoidAngle1;
        fMinAngle0 = std::max(fMinAngle0, fTmpAngle);
    }
    else
    {
        //距离过近
        fTmpAngle = atan2f(fCY-fCenterY, fCX-fCenterX) + 0.5f*static_cast<float>(M_PI) + ClothoidAngle1;
    }

    fMinAngle = std::max(fMinAngle, fMinAngle0);
    while (fMinAngle < -M_PI)
        fMinAngle += 2.0f*static_cast<float>(M_PI);
    while (fMinAngle > M_PI)
        fMinAngle -= 2.0f*static_cast<float>(M_PI);
    return fMinAngle;
}

void CalcRotationCenter(LocationPoint tPoint, float fCX, float fCY, float fDX, float fDY,
                        float fTargetAngle)
{
    float fTmpS, fCutCoef, fSigma;
    float fTurnDir;
    float fCenterX, fCenterY;
    float A, B, C, CoefA, CoefB, a, b, c;
    float fTmpCenterX, fTmpCenterY;
    float fRE1, fRE2, fRE;
    float fCornerMinDist, fSafeLateralDist, fCornerDist;
    int nIndex;

    min_steering_radius1 = min_steering_radius2 = Constants::min_steering_radius;
    fSigma = 1/Constants::min_steering_radius/(CLOTHOID_LENGTH*Constants::step_size);

    if (fabs(fTargetAngle - tPoint.yaw) < ZERO_ANGLE)
    {
        PoseSeg[0].x = PoseSeg[1].x = tPoint.x;
        PoseSeg[0].y = PoseSeg[1].y = tPoint.y;
        PoseSeg[0].yaw = PoseSeg[1].yaw = fTargetAngle;
    }
    else if (fabs(fTargetAngle - tPoint.yaw) < ClothoidAngle1)
    {
        PoseSeg[0].x = tPoint.x;
        PoseSeg[0].y = tPoint.y;
        PoseSeg[0].yaw = tPoint.yaw;

        fTurnDir = Sign(fTargetAngle - tPoint.yaw);
        ClothoidAngle1 = fabs(fTargetAngle - tPoint.yaw); //改变第一段转弯半径
        fTmpS = sqrtf(2*ClothoidAngle1/fSigma);
        min_steering_radius1 = 1.0f/(fSigma*fTmpS);
        nIndex = floorf(fTmpS/Constants::step_size);
        fCutCoef = (fTmpS - Constants::step_size*nIndex)/Constants::step_size;
        ClothoidForwardX1 = ClothoidSeg[nIndex].x + fCutCoef*(ClothoidSeg[nIndex+1].x - ClothoidSeg[nIndex].x);
        ClothoidForwardY1 = ClothoidSeg[nIndex].y + fCutCoef*(ClothoidSeg[nIndex+1].y - ClothoidSeg[nIndex].y);
        ClothoidBackwardX1 = ClothoidForwardX1*cosf(ClothoidAngle1) + ClothoidForwardY1*sinf(ClothoidAngle1);
        ClothoidBackwardY1 = ClothoidForwardX1*sinf(ClothoidAngle1) - ClothoidForwardY1*cosf(ClothoidAngle1);
        fTangentDist1 = min_steering_radius1*cosf(ClothoidAngle1) + ClothoidForwardY1;
        fHalfCurvDist1 = ClothoidForwardX1 - min_steering_radius1*sinf(ClothoidAngle1);

        PoseSeg[1].x = tPoint.x + ClothoidBackwardX1*cosf(tPoint.yaw) - fTurnDir*ClothoidBackwardY1*sinf(tPoint.yaw);
        PoseSeg[1].y = tPoint.y + ClothoidBackwardX1*sinf(tPoint.yaw) + fTurnDir*ClothoidBackwardY1*cosf(tPoint.yaw);
        PoseSeg[1].yaw = tPoint.yaw + fTurnDir*ClothoidAngle1;
    }
    else
    {
        fTurnDir = Sign(fTargetAngle - tPoint.yaw);
        PoseSeg[0].x = tPoint.x + fTurnDir*min_steering_radius1*(sinf(fTargetAngle - fTurnDir*ClothoidAngle1) - sinf(tPoint.yaw));
        PoseSeg[0].y = tPoint.y + fTurnDir*min_steering_radius1*(cosf(tPoint.yaw) - cosf(fTargetAngle - fTurnDir*ClothoidAngle1));
        PoseSeg[0].yaw = fTargetAngle - fTurnDir*ClothoidAngle1;
        PoseSeg[1].x = PoseSeg[0].x + ClothoidBackwardX1*cosf(PoseSeg[0].yaw) - fTurnDir*ClothoidBackwardY1*sinf(PoseSeg[0].yaw);
        PoseSeg[1].y = PoseSeg[0].y + ClothoidBackwardX1*sinf(PoseSeg[0].yaw) + fTurnDir*ClothoidBackwardY1*cosf(PoseSeg[0].yaw);
        PoseSeg[1].yaw = PoseSeg[0].yaw + fTurnDir*ClothoidAngle1;
    }

    //已经平行
    if (PoseSeg[1].yaw < ZERO_ANGLE)
    {
        PoseSeg[2].x = PoseSeg[3].x = PoseSeg[4].x = PoseSeg[5].x = PoseSeg[1].x;
        PoseSeg[2].y = PoseSeg[3].y = PoseSeg[5].y = PoseSeg[5].y = PoseSeg[1].y;
        PoseSeg[2].yaw = PoseSeg[3].yaw = PoseSeg[4].yaw = PoseSeg[5].yaw = 0.0f;
        return;
    }

    //改变第二段转弯半径
    if (PoseSeg[1].yaw < 2*ClothoidAngle2)
    {
        ClothoidAngle2 = 0.5f*PoseSeg[1].yaw;
        fTmpS = sqrtf(2*ClothoidAngle2/fSigma);
        min_steering_radius2 = 1.0f/(fSigma*fTmpS);
        nIndex = floorf(fTmpS/Constants::step_size);
        fCutCoef = (fTmpS - Constants::step_size*nIndex)/Constants::step_size;
        ClothoidForwardX2 = ClothoidSeg[nIndex].x + fCutCoef*(ClothoidSeg[nIndex+1].x - ClothoidSeg[nIndex].x);
        ClothoidForwardY2 = ClothoidSeg[nIndex].y + fCutCoef*(ClothoidSeg[nIndex+1].y - ClothoidSeg[nIndex].y);
        ClothoidBackwardX2 = ClothoidForwardX2*cosf(ClothoidAngle2) + ClothoidForwardY2*sinf(ClothoidAngle2);
        ClothoidBackwardY2 = ClothoidForwardX2*sinf(ClothoidAngle2) - ClothoidForwardY2*cosf(ClothoidAngle2);
        fTangentDist2 = min_steering_radius2*cosf(ClothoidAngle2) + ClothoidForwardY2;
        fHalfCurvDist2 = ClothoidForwardX2 - min_steering_radius2*sinf(ClothoidAngle2);
    }

    //直接转弯
    PoseSeg[2].x = PoseSeg[1].x;
    PoseSeg[2].y = PoseSeg[1].y;
    PoseSeg[2].yaw = PoseSeg[1].yaw;
    PoseSeg[3].x = PoseSeg[2].x + ClothoidForwardX2*cosf(PoseSeg[2].yaw) + ClothoidForwardY2*sinf(PoseSeg[2].yaw);
    PoseSeg[3].y = PoseSeg[2].y + ClothoidForwardX2*sinf(PoseSeg[2].yaw) - ClothoidForwardY2*cosf(PoseSeg[2].yaw);
    PoseSeg[3].yaw = PoseSeg[2].yaw - ClothoidAngle2;
    PoseSeg[4].x = PoseSeg[3].x + min_steering_radius2*(sinf(PoseSeg[3].yaw) - sinf(ClothoidAngle2));
    PoseSeg[4].y = PoseSeg[3].y + min_steering_radius2*(cosf(ClothoidAngle2) - cosf(PoseSeg[3].yaw));
    PoseSeg[4].yaw = ClothoidAngle2;
    PoseSeg[5].x = PoseSeg[4].x + ClothoidBackwardX2*cosf(PoseSeg[4].yaw) + ClothoidBackwardY2*sinf(PoseSeg[4].yaw);
    PoseSeg[5].y = PoseSeg[4].y + ClothoidBackwardX2*sinf(PoseSeg[4].yaw) - ClothoidBackwardY2*cosf(PoseSeg[4].yaw);
    PoseSeg[5].yaw = 0.0f;
    fCenterX = PoseSeg[5].x - fHalfCurvDist2;
    fCenterY = PoseSeg[5].y - fTangentDist2;

    feasible_corner_safety_dis = std::min(Constants::para_slot_corner_safety_dis, min_steering_radius2 -
                                          VEHICLE_WID*0.5f - REAR_VIEW_LENGTH);
    fCornerMinDist = VEHICLE_WID*0.5f + REAR_VIEW_LENGTH + feasible_corner_safety_dis;
    fSafeLateralDist = std::max(LATERAL_SAFETY_DIS, REAR_VIEW_LENGTH + feasible_corner_safety_dis +
                                fCY + fTangentDist2 - min_steering_radius2) + VEHICLE_WID*0.5f;


    //延长线
    A = sinf(PoseSeg[1].yaw);
    B = -cosf(PoseSeg[1].yaw);
    C = PoseSeg[1].y*cosf(PoseSeg[1].yaw) - PoseSeg[1].x*sinf(PoseSeg[1].yaw);

    //出库距离限制
    fTmpCenterY = fSafeLateralDist - fTangentDist2;
    fTmpCenterX = -(B*fTmpCenterY + C)/A + fTangentDist2/fabs(A);
    if (fCenterY < fTmpCenterY)
    {
        fCenterX = fTmpCenterX;
        fCenterY = fTmpCenterY;
    }

    //内转角碰撞
    fCornerDist = -(A*fCX+B*fCY+C-fTangentDist2)/sqrtf(powf(A,2)+powf(B,2));
    if (fabs(fCornerDist) < fabs(min_steering_radius2-fCornerMinDist))
    {
        CoefA = -B/A;
        CoefB = (fTangentDist2 - C) / A;
        a = powf(CoefA,2) + 1;
        b = 2*CoefA*(CoefB-fCX) - 2*fCY;
        c = powf(CoefB-fCX,2) + powf(fCY,2) - powf(min_steering_radius2-fCornerMinDist,2);
        fTmpCenterY = (-b - sqrtf(fabs(powf(b,2) - 4*a*c)))/(2*a);
        fTmpCenterX = CoefA*fTmpCenterY + CoefB;
        if (fTmpCenterX > fCX && fCenterY < fTmpCenterY)
        {
            fCenterX = fTmpCenterX;
            fCenterY = fTmpCenterY;
        }
    }
    else if (fabs(fCornerDist) < min_steering_radius2+fCornerMinDist)
    {
        fTmpCenterX = fCX + fCornerDist*sinf(PoseSeg[1].yaw);
        fTmpCenterY = fCY - fCornerDist*cosf(PoseSeg[1].yaw);
        if (fTmpCenterX > fCX && fCenterY < fTmpCenterY)
        {
            fCenterX = fTmpCenterX;
            fCenterY = fTmpCenterY;
        }
    }

    //外转角碰撞
    fCornerDist = fabs(A*fDX+B*fDY+C-fTangentDist2)/sqrtf(powf(A,2)+powf(B,2));
    fRE1 = hypotf(fTangentDist2 + 0.5f*VEHICLE_WID - Constants::rear_chamfer_l1, REAR_EDGE2CENTER);
    fRE2 = hypotf(fTangentDist2 + 0.5f*VEHICLE_WID, REAR_EDGE2CENTER - Constants::rear_chamfer_l2);
    fRE = std::max(fRE1, fRE2);
    if (fCornerDist < fRE + feasible_corner_safety_dis)
    {
        CoefA = -B/A;
        CoefB = (fTangentDist2 - C) / A;
        a = powf(CoefA,2) + 1;
        b = 2*CoefA*(CoefB-fDX) - 2*fDY;
        c = powf(CoefB-fDX,2) + powf(fDY,2) - powf(fRE + feasible_corner_safety_dis,2);
        fTmpCenterY = (-b + sqrtf(fabs(powf(b,2) - 4*a*c)))/(2*a);
        fTmpCenterX = CoefA*fTmpCenterY + CoefB;
        if (fTmpCenterX > fDX && fCenterY < fTmpCenterY)
        {
            fCenterX = fTmpCenterX;
            fCenterY = fTmpCenterY;
        }
    }

    //补齐端点
    PoseSeg[5].x = fCenterX + fHalfCurvDist2;
    PoseSeg[5].y = fCenterY + fTangentDist2;
    PoseSeg[5].yaw = 0.0f;

    PoseSeg[4].x = PoseSeg[5].x - ClothoidForwardX2*cosf(PoseSeg[5].yaw) + ClothoidForwardY2*sinf(PoseSeg[5].yaw);
    PoseSeg[4].y = PoseSeg[5].y - ClothoidForwardX2*sinf(PoseSeg[5].yaw) - ClothoidForwardY2*cosf(PoseSeg[5].yaw);
    PoseSeg[4].yaw = PoseSeg[5].yaw + ClothoidAngle2;

    PoseSeg[3].x = PoseSeg[4].x - min_steering_radius2*(sinf(PoseSeg[3].yaw) - sinf(PoseSeg[4].yaw));
    PoseSeg[3].y = PoseSeg[4].y - min_steering_radius2*(cosf(PoseSeg[4].yaw) - cosf(PoseSeg[3].yaw));
    PoseSeg[3].yaw = PoseSeg[3].yaw; //不变

    PoseSeg[2].x = PoseSeg[3].x - ClothoidBackwardX2*cosf(PoseSeg[3].yaw) + ClothoidBackwardY2*sinf(PoseSeg[3].yaw);
    PoseSeg[2].y = PoseSeg[3].y - ClothoidBackwardX2*sinf(PoseSeg[3].yaw) - ClothoidBackwardY2*cosf(PoseSeg[3].yaw);
    PoseSeg[2].yaw = PoseSeg[3].yaw + ClothoidAngle2;

}

void CalcTrajectory(LocationPoint tPoint, std::vector<CtrlPoint> &TargetSeg)
{
    float fAngleStep, fSigma;
    float fTurnDir;
    float fTmpX, fTmpY, fTmpYaw;
    int nNum;
    CtrlPoint TmpPoint;

    fSigma = 1/Constants::min_steering_radius/(CLOTHOID_LENGTH*Constants::step_size);
    TargetSeg.clear();

    //1.左转圆弧
    fTurnDir = Sign(PoseSeg[0].yaw - tPoint.yaw);
    fAngleStep = Constants::step_size / min_steering_radius1;
    nNum = floorf(fabs(PoseSeg[0].yaw - tPoint.yaw) / fAngleStep); //包含起点
    if (fabs(PoseSeg[0].yaw - tPoint.yaw)*min_steering_radius1 > ZERO_LENGTH)
    {
        for (int i=0; i<nNum+1; ++i)
        {
            TmpPoint.x = tPoint.x + fTurnDir*min_steering_radius1*(sinf(tPoint.yaw + i*fTurnDir*fAngleStep) - sinf(tPoint.yaw));
            TmpPoint.y = tPoint.y + fTurnDir*min_steering_radius1*(cosf(tPoint.yaw) - cosf(tPoint.yaw + i*fTurnDir*fAngleStep));
            TmpPoint.yaw = tPoint.yaw + i*fTurnDir*fAngleStep;
            TmpPoint.kappa = fTurnDir*1.0f/min_steering_radius1;
            TmpPoint.driving_direction = 1;
            TargetSeg.push_back(TmpPoint);
        }
    }

    //2.左回正
    fTurnDir = Sign(PoseSeg[1].yaw - PoseSeg[0].yaw);
    nNum = floorf(1.0f/min_steering_radius1/fSigma/Constants::step_size);
    if (sqrtf(2.0f*fabs(PoseSeg[1].yaw - PoseSeg[0].yaw)/fSigma) > ZERO_LENGTH)
    {
        TmpPoint.x = PoseSeg[0].x;
        TmpPoint.y = PoseSeg[0].y;
        TmpPoint.yaw = PoseSeg[0].yaw;
        TmpPoint.kappa = fTurnDir*1.0f/min_steering_radius1;
        TmpPoint.driving_direction = 1;
        TargetSeg.push_back(TmpPoint);
        for (int i=MIN(nNum,ClothoidSeg.size()-2); i>=1; --i)
        {
            fTmpYaw = ClothoidAngle1 - ClothoidSeg[i].yaw;
            fTmpX = ClothoidForwardX1 - ClothoidSeg[i].x;
            fTmpY = ClothoidForwardY1 - ClothoidSeg[i].y;
            ClothoidBackwardX1 = fTmpX*cosf(ClothoidAngle1) + fTmpY*sinf(ClothoidAngle1);
            ClothoidBackwardY1 = fTmpX*sinf(ClothoidAngle1) - fTmpY*cosf(ClothoidAngle1);

            TmpPoint.x = PoseSeg[0].x + ClothoidBackwardX1*cosf(PoseSeg[0].yaw) - fTurnDir*ClothoidBackwardY1*sinf(PoseSeg[0].yaw);
            TmpPoint.y = PoseSeg[0].y + ClothoidBackwardX1*sinf(PoseSeg[0].yaw) + fTurnDir*ClothoidBackwardY1*cosf(PoseSeg[0].yaw);
            TmpPoint.yaw = PoseSeg[0].yaw + fTurnDir*fTmpYaw;
            TmpPoint.kappa = fTurnDir*fSigma*i*Constants::step_size;
            TmpPoint.driving_direction = 1;
            TargetSeg.push_back(TmpPoint);
        }
    }

    //3.直行
    nNum = floorf(sqrtf(powf(PoseSeg[2].x-PoseSeg[1].x,2) + powf(PoseSeg[2].y-PoseSeg[1].y,2))/Constants::step_size);
    if (sqrtf(powf(PoseSeg[2].x-PoseSeg[1].x,2) + powf(PoseSeg[2].y-PoseSeg[1].y,2)) > ZERO_LENGTH)
    {
        for (int i=0; i<nNum+1; ++i)
        {
            TmpPoint.x = PoseSeg[1].x + (i*Constants::step_size)*cosf(PoseSeg[1].yaw);
            TmpPoint.y = PoseSeg[1].y + (i*Constants::step_size)*sinf(PoseSeg[1].yaw);
            TmpPoint.yaw = PoseSeg[1].yaw;
            TmpPoint.kappa = 0;
            TmpPoint.driving_direction = 1;
            TargetSeg.push_back(TmpPoint);
        }
    }

    //4.右打死
    nNum = floorf(1.0f/min_steering_radius2/fSigma/Constants::step_size); //包含终点
    if ((PoseSeg[2].yaw - PoseSeg[3].yaw)*min_steering_radius2 > ZERO_LENGTH)
    {
        for (int i=0; i<nNum+1 && i<ClothoidSeg.size()-1; ++i)
        {
            fTmpYaw = ClothoidSeg[i].yaw;
            fTmpX = ClothoidSeg[i].x;
            fTmpY = ClothoidSeg[i].y;

            TmpPoint.x = PoseSeg[2].x + fTmpX*cosf(PoseSeg[2].yaw) + fTmpY*sinf(PoseSeg[2].yaw);
            TmpPoint.y = PoseSeg[2].y + fTmpX*sinf(PoseSeg[2].yaw) - fTmpY*cosf(PoseSeg[2].yaw);
            TmpPoint.yaw = PoseSeg[2].yaw - fTmpYaw;
            TmpPoint.kappa = -fSigma*i*Constants::step_size;
            TmpPoint.driving_direction = 1;
            TargetSeg.push_back(TmpPoint);
        }
    }

    //5.右转
    fAngleStep = -Constants::step_size / min_steering_radius2;
    nNum = floorf((PoseSeg[4].yaw - PoseSeg[3].yaw) / fAngleStep); //包含起点
    if ((PoseSeg[3].yaw - PoseSeg[4].yaw)*min_steering_radius2 > ZERO_LENGTH)
    {
        for (int i=0; i<nNum+1; ++i)
        {
            TmpPoint.x = PoseSeg[3].x - min_steering_radius2*(sinf(PoseSeg[3].yaw + i*fAngleStep) - sinf(PoseSeg[3].yaw));
            TmpPoint.y = PoseSeg[3].y - min_steering_radius2*(cosf(PoseSeg[3].yaw) - cosf(PoseSeg[3].yaw + i*fAngleStep));
            TmpPoint.yaw = PoseSeg[3].yaw + i*fAngleStep;
            TmpPoint.kappa = -1.0f/min_steering_radius2;
            TmpPoint.driving_direction = 1;
            TargetSeg.push_back(TmpPoint);
        }
    }

    //6.右回正
    nNum = floorf(1.0f/min_steering_radius2/fSigma/Constants::step_size);
    if (sqrtf(2.0f*fabs(PoseSeg[5].yaw - PoseSeg[4].yaw)/fSigma) > ZERO_LENGTH)
    {
        TmpPoint.x = PoseSeg[4].x;
        TmpPoint.y = PoseSeg[4].y;
        TmpPoint.yaw = PoseSeg[4].yaw;
        TmpPoint.kappa = -1.0f/min_steering_radius2;
        TmpPoint.driving_direction = 1;
        TargetSeg.push_back(TmpPoint);
        for (int i=MIN(nNum,ClothoidSeg.size()-2); i>=0; --i)
        {
            fTmpYaw = ClothoidAngle2 - ClothoidSeg[i].yaw;
            fTmpX = ClothoidForwardX2 - ClothoidSeg[i].x;
            fTmpY = ClothoidForwardY2 - ClothoidSeg[i].y;
            ClothoidBackwardX2 = fTmpX*cosf(ClothoidAngle2) + fTmpY*sinf(ClothoidAngle2);
            ClothoidBackwardY2 = fTmpX*sinf(ClothoidAngle2) - fTmpY*cosf(ClothoidAngle2);

            TmpPoint.x = PoseSeg[4].x + ClothoidBackwardX2*cosf(PoseSeg[4].yaw) + ClothoidBackwardY2*sinf(PoseSeg[4].yaw);
            TmpPoint.y = PoseSeg[4].y + ClothoidBackwardX2*sinf(PoseSeg[4].yaw) - ClothoidBackwardY2*cosf(PoseSeg[4].yaw);
            TmpPoint.yaw = PoseSeg[4].yaw - fTmpYaw;
            TmpPoint.kappa = -fSigma*i*Constants::step_size;
            TmpPoint.driving_direction = 1;
            TargetSeg.push_back(TmpPoint);
        }
    }

}

void ClothoidParkout(LocationPoint tPoint,
                     float fCX, float fCY,
                     float fDX, float fDY,
                     std::vector<CtrlPoint> &TargetSeg)
{
    float fTargetAngle;
    InitClothoid();
    fTargetAngle = CalcMidAngle(tPoint, fCX, fCY);
    CalcRotationCenter(tPoint, fCX, fCY, fDX, fDY, fTargetAngle);
    CalcTrajectory(tPoint, TargetSeg);
}

void ClothoidVerticalParkout(LocationPoint tPoint,
                             float fCX, float fCY,
                             float fDX, float fDY,
                             float fSlotYaw,
                             std::vector<CtrlPoint> &TargetSeg)
{
    InitClothoid();
    CalcRotationCenter(tPoint, fCX, fCY, fDX, fDY, fSlotYaw);
    CalcTrajectory(tPoint, TargetSeg);
}


}
