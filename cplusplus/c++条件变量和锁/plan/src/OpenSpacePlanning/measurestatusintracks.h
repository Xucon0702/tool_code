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
#ifndef MEASURESTATUSINTRACKS_H
#define MEASURESTATUSINTRACKS_H

#include <vector>
#include "base.h"
#include "vehicle.h"
#include "vector2d.h"
#include "node3d.h"
#include "publicfunpathplan.h"
#define Y1_GET_SLOT_INFOR_OBLI   (0.5f)
#define Y2_GET_SLOT_INFOR_OBLI   (-1.0f)
#define Y3_GET_SLOT_INFOR_OBLI (-3.0f)
#define YAW_LIMIT_OBLI (5*PI/180)
#define Y_WIDTH_GET_SLOT_INFOR_OBLI  (0.1f)

#define DETECT_WHEEL_BAR_NUM (20) /*wheel bar*/
#define USS_DETECT_DIS (1)


#define PARA_POINT_NUM (5)
#define MIN_VEH_YAW_DETECT_REAR_MARGIN (35*PI/180)
#define MAX_VELOCITY_IN_OBLI_SLOT (0.4f)

namespace HybridAStarPart
{

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
                               LocationPoint tCurPos);

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
                                  std::vector<TargetTrack> &aTracks,
                                  LocationPoint& U11_start,
                                  LocationPoint& U11_end,
                                  LocationPoint& U12_start,
                                  LocationPoint& U12_end);



/**
 * @brief 给超声泊车最后一段轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在超声泊车最后一段入库轨迹规划结束调用
 */
int AddMeasureStatusInObliUSSSlot(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks);

/**
 * @brief 给视觉划线车位泊车最后一段轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在视觉划线车位泊车,最后一段入库轨迹规划结束调用
 */
int AddMeasureStatusInObliVisionSlot(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks);

/**
 * @brief 给超声泊车最后一段hybrid A*轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在超声车位泊车,最后一段入库hybrid A*轨迹规划结束调用
 */
bool AddMeasureStatusInParaUssSlotWhenHybridALastDriving(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks);

/**
 * @brief 给平行超声泊车在库里向前调整时轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note 只有在3号超声Y坐标低于车位外侧前角点的Y坐标,行驶方向为前进调用,平行库内调整调用
 */
bool AddMeasureStatusInParaUssSlotWhenForwardDriving(const slotPosInfor &slotPosInfor_, std::vector<TargetTrack>& aTracks);


}


#endif // MEASURESTATUSINTRACKS_H
