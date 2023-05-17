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

/**********************Tips:decision moudle<-->HMI**********************/

#ifndef _PPS_APA_DECISION_H_
#define _PPS_APA_DECISION_H_

#if 1
#ifdef __cplusplus
extern "C" {
#endif
#endif

#include <stdio.h>
#include <stdint.h>


#include "typedefs.h"
// #include "pps_common.h"
#include  "pps_apa_common.h"

#define	PPS_DECISION_PRT	0

typedef struct _HdmiApaSearchSelectSloInfo 
{ 
	UINT8 IsSelected; //1-->有选中的车位
	UINT8 SlotType;
	UINT8 ParkType;	
	UINT8 ParkOutDir;
	UINT32 SlotID;
	UINT8 researve[4];	
}HdmiApaSearchSelectSloInfo;

typedef struct _HdmiToApaPpsInfo 
{ 
	UINT32 uCmd;

   	MoudleStatue ApaCmd[MoudleMax];
	UINT32	uSoltType;		//0：视觉车位 1：自选车位             2:超声车位 其他无效 
	UINT32	uTurn;			//0：无/1水平左侧/2水平右侧/3垂直左侧/4垂直右侧/5遥控前行/6遥控后退 
	UINT32	uSenSor;		//0：无 1：蓝牙
	HdmiApaSearchSelectSloInfo HmiApaSelSlot; //巡库时高亮点选的车位信息
	MvSlotData_pps	tSelectSlot;//选择的车位信息(泊入使用)
	PpsPose			userSelInsideWorldCoord[4];			/*自选车位发送的显示在屏幕点位*/ 
}HdmiToApaInfo;


typedef struct _PlanToHdmi
{ 
   	UINT32	uCmd;			//0:无效 1:泊入 2:泊出
   	UINT32	uStatue;		//[b23 - b16]泊车总状态       0:泊车成功 1:失败
   							//[b15 - b 8]规划状态：		 0:泊车到位 1:规划成功 2:规划失败
   							//[b 7 - b 0]控制状态：  	 0:处于运动状态 1:静止状态 2:泊车完成 3:无跟踪目标点数据 4:目标点索引超限 5:跟踪偏离轨迹超限 6:障碍物逼停等待超时
   							//7:障碍物前后逼停 8:减速带停止 9:控制模式错误
   	UINT32	uTotalStep;		//总进度
	UINT32	uCurrentStep;	//当前进度
	UINT32	uDisStep;		//小进度
	UINT32	uPlanOutDetectSts;//泊出道路通畅
	FLOAT px[4];
	FLOAT py[4];
}PlanToHdmi; 


typedef struct _ApaToHdmiInfo 
{ 
	UINT8			isTargetSlotDataValid; //for parkIn
	UINT64    		lTimeMsec;     //视频帧的时间戳,用于同步
	UINT64			lVisonSlotTimeMsec; 	//视频帧的时间戳,用于同步
	PlanToHdmi 		tPlanToHdmi;
	UINT32			tSoltNum;
	MvSlotData_pps 	tSolt[20];//决策车位信息
	unsigned char 	reserved[4096-84-192*20];
}ApaToHdmiInfo; 



 

INT32 MvPpsGetHdmiToApaInfo(HdmiToApaInfo *pHdmiToApaInfo);
INT32 MvPpsGetApaToHdmiInfo(ApaToHdmiInfo *pApaToHdmiInfo);
INT32 MvPpsSendHdmiToApaInfo(HdmiToApaInfo *pHdmiToApaInfo);
INT32 MvPpsSendApaToHdmiInfo(ApaToHdmiInfo *pApaToHdmiInfo);

INT32 MvPpsRecvHdmiToApaInfoInit();
INT32 MvPpsSendHdmiToApaInfoInit();
INT32 MvPpsRecvApaToHdmiInfoInit();
INT32 MvPpsSendApaToHdmiInfoInit();




#if 1
#ifdef __cplusplus
}
#endif
#endif

#endif

