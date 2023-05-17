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
// APA决策/融合/规划使用的头文件 
/////////////////////////////////////////////////////////////////////////


#ifndef _PPS_APA_COMMON_H_
#define _PPS_APA_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>

#include "typedefs.h"
#include "MvApaCommon.h"


#define OBSTACLE_NAG_SIDE_RST         (23)    /* 车位对面过道的宽度，x=x=[-1 : 0.5 : 10], total = 23 */


#if 0
typedef enum _ApaMoudle
{ 
	UltrMoudle = 0,
	FusionMoudle,
	MixSlotMoudle,
	PlanInMoudle,
	HeadInMoudle,
	ParaOutMoudle,
	VerOutMoudle,
	ControlMoudle,
	AvidanceMoudle,
	PdcMoudle,
	MoudleMax
}ApaMoudle;


typedef enum _MoudleStatue
{ 
	ApaMoudleOff = 0,
	ApaMoudleStart,
	ApaMoudleRun,
	ApaMoudleStop,
	ApaMoudleMax
}MoudleStatue;
#endif

/* 车位类型 */
typedef enum 
{
    PPSNoSuitablePS = 0,   /* 无效车位 */
    PPSParallelSpace,      /* 平行车位 */
    PPSVerticalSpace,      /* 垂直车位 */
    PPSObliqueSpace        /* 斜列车位 */
}PpsParkType;






//规划模块和控制模块的交互接口
typedef enum 
{
    freeStauts_pps=0, //0 停车规划
    updated_pps, //1 停车规划完成
    perparing_pps, //2 原地方向盘
    working_pps, //3 车辆正常行驶
    parkingErr_pps, //4 异常停车
    parkingOver_pps, //5 泊车结束
    RunningFreeStauts_pps, //6 垂直最后一段动态规划
    RunningUpdated_pps, //7 垂直最后一段动态规划完成
    Breaking_pps, //8 刹车
    ParkingSucc_pps, //9 泊车成功
    waiting_pps
}ParkingWorkingStatus_pps;

#if 0
enum PlanningStatus_pps
{
    planningOver = 0, /*规划到位*/
    PlanningSucc,     /*规划成功*/
    PlanningFail     /*规划结束*/
};
#endif


/* 左/右侧障碍物 */
typedef enum 
{
    PPSNOOBS = 0,  /* 无障碍物 */
    PPSLEFTOBS,    /* 左侧障碍物 */
    PPSRIGHTOBS,    /* 右侧障碍物 */
    PPSLeftRightObs  /*左右侧障碍物*/
}PpsObstacleSide;



typedef struct _ArmPointInt32_pps
{
	INT32 	x;															//X坐标
	INT32 	y;															//Y坐标	
}ArmPointInt32_pps;

typedef struct _ArmPointFloat_pps
{
	FLOAT 	x;															//X坐标
	FLOAT 	y;															//Y坐标	
}ArmPointFloat_pps;

typedef struct _MvSlotPoint_pps			
{
	ArmPointInt32_pps tImagePoint;										//车位图像坐标
	ArmPointFloat_pps tWorldPoint;										//车位世界坐标
} MvSlotPoint_pps;

typedef struct _MvSlotData_pps										//车位信息
{
	MvSlotPoint_pps  	tPoint0;										//车位点1坐标（图像+世界）
	MvSlotPoint_pps  	tPoint1;										//车位点2坐标（图像+世界）
	MvSlotPoint_pps  	tPoint2;										//车位点3坐标（图像+世界）
	MvSlotPoint_pps  	tPoint3;										//车位点4坐标（图像+世界）
	MvSlotPoint_pps		tGroundPoint0;									//第一个挡轮杆外侧顶点
	MvSlotPoint_pps		tGroundPoint1;									//第二个挡轮杆外侧顶点
	UINT32				nSlotId;										//车位ID
	UINT32				nDirection;										//车位方向 0:左 1:右 2:前 3:后	(这个只表示是哪个探头探测到的)		
	UINT32 				nType;											//车位类型
	UINT32 				nAvailableState;								//可用状态（占用情况）：1=可用 0=不可用	
	FLOAT				fAngle;											//与x轴正方向夹角，车位航向角
	FLOAT				fSlotTheta;										//置信度
	UINT32  			nParkSlot34ptFlag;								//车位34点有效性,1有效,0无效
	UINT32  			tGroundPinFlag;									//档轮杆有效性,1可信,0无效信
	ArmPointFloat_pps 	tIconPoint[4];									//车位图标的图像坐标
	UINT8				aReserved[192 - 6*16 - 4*8-4*8]; 
} MvSlotData_pps;

typedef struct 
{
    float x;
    float y;
    float yaw;
}LocationPoint_pps;

typedef struct 
{
	float x;
    float y;
}PpsPose;



/* 车位参数 */
typedef struct _SlotsParm
{
    PpsObstacleSide slotside;      /* 左, 右侧 */
    PpsPose start_close_point;     /* 车位离主车近端的起始点 */
    PpsPose end_close_point;       /* 车位离主车近端的结束点 */
    PpsPose start_far_point;       /* 车位离主车远端的起始点 */
    PpsPose end_far_point;         /* 车位离主车远端的结束点 */

    PpsPose obsBeforePoint;        /* 靠近车位近端开始点的真实障碍物坐标 */
    PpsPose obsAfterPoint;         /* 靠近车位近端结束点的真实障碍物坐标 */

    float theta;                /* 车位在世界坐标系下的角度，(-PI, PI) */
    float depth;               /* 车位深度 */
    float length;                /* 车位宽度(平行车位的长度) */
    float wallPosbltyInSlot;    /* 车位内墙还是路沿的置信度, 0<-:路沿, ->1:墙 */

    PpsParkType park_type;         /* 车位类型 */
    LocationPoint_pps curVhPose;      /* 当前车辆位置 */
    float obsOtherSide[OBSTACLE_NAG_SIDE_RST];   /* 车位对面过道的宽度，x=[-1, 10] */
    
}SlotsParm;






#ifdef __cplusplus
}
#endif

#endif

