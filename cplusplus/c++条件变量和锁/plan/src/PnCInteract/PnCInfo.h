/*
 * PnCInfo.h
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#ifndef PNC_INFO_H_
#define PNC_INFO_H_

#include "pps_send_mcu.h"
#include "pps_recv_mcu.h"
#include "base.h"


#define GAC_PORT

namespace DDS {
#if defined(GAC_PORT) && !defined(GAC_A58)
typedef enum _GearStatus
{
    None = 0,
    Drive,
    Neutral,
    Reverse,
    Parking
} GearStatus;
#else
typedef enum _GearStatus
{
    None = 0,
    Parking,
    Reverse,
    Neutral,
    Drive
} GearStatus;
#endif

typedef enum {
  NoError = 0,
  StaticFailed = 1,
  OverParkStep = 2,
  DynamicFailed = 4,
  ParkOutDectctFailed = 5
} PlanError;

typedef enum _ControlState
{
    NoErr = 0,
    NoPoints,
    TrackErr,
    ObstacleErr,
    NoMoveErr,
    GearErr
} ControlState;


#ifdef GAC_A58
/*
typedef struct _MvPpsAPAPath
{
	struct {
		UINT8 Path_Gear;	//规划档位
		UINT8 Path_ID;		//规划轨迹ID
		UINT8 Path_type;	//路径类型
		UINT16 PathSize;	//路径点有效个数
		UINT8 PlanComplete; //是否到达终点
		UINT8 PlanError;	//是否规划错误
		float PathLen;		//总路径长
		float RmPathLen;	//剩余路径长
	} PathAttribute;

	struct {
		float Vmax; 		//速度
		float Curv; 		//曲率
		float X;			//X坐标
		float Y;			//Y坐标
		float Yaw;			//航向角
		UINT8 Lamp; 		//转向灯请求
	} PathPoint[40]; 		//路点结构体数组
}MvPpsApaPathData;
*/
extern MvPpsApaPathData  _ApaPathData;
#endif

//发送
extern MvPpsPathPointArr40   _PathPoints;
extern MvPpsApaPathAttribute _PathAttribute;

extern UINT8                _APAfusion_PosReset;
extern MvApafusionPosValue  _APAfusion_PosValue;

extern UINT8        _WarningType;

//接收
extern UINT8        _Control_State;
extern UINT8        _Brk_Flag;
extern MvPpsCtrlOut _CtrlOut;
}

void getControlFeedback(DDS::ControlState &Control_State);

#endif /* PNC_INFO_H_ */
