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
#ifndef CLOTHOID_PARKOUT_H
#define CLOTHOID_PARKOUT_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include "base.h"
#include "vehicle.h"

#define ZERO_LENGTH (1e-4f)
#define ZERO_ANGLE  (1e-4f)

#define LATERAL_SAFETY_DIS  (0.7f) //横向目标距离
#define CLOTHOID_LENGTH     (15)   //回旋线长度
#define RECOMMEND_ANGLE     static_cast<float>(25.0/180.0*M_PI) //默认出库角度

namespace HybridAStarPart
{

void ClothoidParkout(LocationPoint tPoint,
                     float fCX, float fCY,
                     float fDX, float fDY,
                     std::vector<CtrlPoint> &TargetSeg);

void ClothoidVerticalParkout(LocationPoint tPoint,
                             float fCX, float fCY,
                             float fDX, float fDY,
                             float fSlotYaw,
                             std::vector<CtrlPoint> &TargetSeg);

void InitClothoid();

float CalcMidAngle(LocationPoint tPoint, float fCX, float fCY);

void CalcRotationCenter(LocationPoint tPoint, float fCX, float fCY, float fDX, float fDY,
                        float fTargetAngle);

void CalcTrajectory(LocationPoint tPoint, std::vector<CtrlPoint> &TargetSeg);

}

#endif // CLOTHOID_PARKOUT_H
