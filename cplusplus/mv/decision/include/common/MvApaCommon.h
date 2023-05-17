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
// APA决策/AVM共同使用的头文件 
/////////////////////////////////////////////////////////////////////////


#ifndef _MV_APA_COMMON_H_
#define _MV_APA_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "typedefs.h"

#define VehTrackNum				(20)

#define USS_RADAR_MAX_NUM		12

#define DISTANCE2 	  0

#define	SlotSideNumber  4

//guangqi:stateMachine
typedef enum 
{ 
	Off = 0,
	STANDBY,
	SEARCHING,
	GUIDANCE,
	COMPLETE,
	FAILURE,
	TRTMINATE,
	SUSPEND
}ApaState;


//enum
#if 1
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

typedef enum _MapBuildingMode
{ 
	ApaSearching = 0,//build map by freespace and uss
	ApaParkingIn,
	HpaCruise,
	HapToApa
}MapBuildingMode; 

//GAC泊出方向
enum ParkOutMode_E
{
	ParkOutModeDefault_E = 0,
	PARALLEL_LEFT_OUT_E,
	PARALLEL_RIGHT_OUT_E,
	VERTICAL_LEFT_OUT_E,
	VERTICAL_RIGHT_OUT_E,
	VERTICAL_REMOTE_FORWARD_E,
	VERTICAL_REMOTE_BACKWARD_E
};

typedef enum
{
    NotCutE = 0,
    HadCutE
}SlotSideCutE;

typedef enum _ppsSlotsideType  //边的属性
{
    side_real_pps = 0,			//0：真实
    side_virtual_pps = 1,		//1：虚拟
    side_curb_pps = 2,			//2：路沿
    side_wall_pps = 3,			//3：墙
    side_treelawn_pps = 4,		//4：绿化带
    side_entrance_pps = 5,		//5：车位入口
    side_pillar_pps = 6			//6：柱子
}ppsSlotsideType;


//plan
//传给decision的，用于mvplayer显示错误
typedef enum _ppsPlanErrCode{
    NO_ERROR_E=0,
    N_10001_E, //车位尺寸不够
    N_10002_E, //平行泊车入库点规划失败
    N_10003_E, //垂直车位目标点和车位干涉
    N_10004_E, //可变栅格数量计算错误
    N_10005_E, //搜索失败
    N_10006_E, //超出最大轨迹段数
    N_10007_E, //两次规划轨迹段数差距太大
    N_20001_E=21, //平行泊车重规划全部失败
    N_20002_E, //平行泊车重规划成功，但是车辆较大程度靠里或者靠外
    N_20003_E, //平行泊车库内重规划输入y大于0
    N_30001_E	//泊出空间探测失败
}ppsPlanErrCodeEnum;

//传给mcu3_0
typedef enum {
  NoError_E = 0,
  StaticFailed_E = 1,
  DynamicFailed_E = 4,
  ParkOutDectctFailed_E = 5
} PlanErrorEum;


//fusion
typedef enum _ObstByFreespaceEum  //融合车位要过滤的freespace类型
{
    MV_PEOPLE_INSLOT_E = 0, //行人
    MV_NOMOTOR_INSLOT_E = 1, //非机动车二轮车
    MV_MOTOR_INSLOT_E = 2, //机动车  
    MV_PILLAR_INSLOT_E  = 3, //柱子  
    MV_GROUNDLOCK_INSLOT_E = 4, //地锁开 
    MV_NOSTOPSIGN_INSLOT_E = 5, //禁停牌子 
    MV_WARNINGPOLE_INSLOT_E = 6,//警示柱 
    MV_WARNINGCONE_INSLOT_E = 7, //警示锥 
    MV_OBSTYPENUMBER_INSLOT_E = 8
}PpsObstByFreespaceEum;


/************************************/

//APA  参数设置
typedef struct apa_para_cfg{
	int apaLevelParkStep;  	  //平行泊车步数上限 默认16次
	int apaVerticalParkStep;  //垂直泊车步数上限 默认16次
	int apaFrontRearWheelDis; //前后轮距离障碍车外侧边连线的距离 默认0cm 超声平行
	int apaFrontRearDis;      //车辆前后距离  文档为居中 平行车位
	int apaCarBarrierDis;    //车辆车头距离障碍车车头距离 默认0cm 超声车位
	int apaFrontRearWheelInsideDis; //前后轮外沿离线内侧距离 平行车位
	int apaRearInsideDis;    //车尾离后线内侧的距离 垂直车位车头与参考线距离
	int apaCorDriveBackDis;  //遥控前进后退行驶距离 一个车身长度
}APA_PARA_CFG;

#endif


//struct
#if 1
typedef struct _ApaUltrData
{
	float Distance1;	//一次回波
	float Distance2;	//二次回波
	float Level;		//回波强度
	float Width;		//回波宽度
	UInt8 status;		//状态
	UInt8	nReserved0[3];
}ApaUltrData;

#if 1
typedef struct _MV_Apa_CAN_CAR_INFO_256
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号
	UINT64 	lTimeMsec;												//时间戳(ms)
	INT32 	nBrake;													//刹车  1，刹车  0，未刹车 
	INT32 	nLLight;												//左转向灯 1、点亮 0、不亮
	INT32 	nRLight;												//右转向灯 1、点亮 0、不亮
	FLOAT 	fVelocity;												//车速	 km/h
	INT32 	nBrakeValue;  											//刹车力度 用于制动
	FLOAT 	fAccelPedalPosition;									//油门力度 
	FLOAT 	fLongiAccel;											//纵向加速度
	FLOAT 	fLateralAccel;											//横向加速度
	FLOAT 	fYawRate;												//偏航角速度
	INT32 	nTransmissionStatus;									//档位 0无效 1P 2R 3N 4D
	FLOAT 	fAlpha;													//车轮转向角 
	FLOAT 	fSteeingWheelAngle;										//方向盘转向角，顺时针为正，角度
	FLOAT 	fSteeingWheelRate;										//方向盘转向速率
	INT32   nParkBrake ;											//手刹 0 手刹未起作用 1 手刹起作用 2 错误 3 无效
	INT32   nWipeWasherSwitch;										//雨刮 0不工作 1低档 2中档 3高档 4间歇档
	INT32   nHazardLightSwitch;										//危险报警灯开关 0无效 1关 2开
	INT32	nHighLowBeamSwitch;										//近光远光灯开关 0无效 1近光 2远光 3错误 3无效
	UINT16  nFLWheelSpeedRC;										//左前轮速脉冲信号
	UINT16  nFRWheelSpeedRC;										//右前轮速脉冲信号
	UINT16  nRLWheelSpeedRC;										//左后轮速脉冲信号
	UINT16  nRRWheelSpeedRC;										//右后轮速脉冲信号
	UINT8  	nDoorStat[4];											//[0-4]左前,右前,左后,右后,0关门 1开门
	INT32   nPulseDirection;										//轮速脉冲方向
	//add 11-29
	FLOAT   fWheelSpd;                     							//估计轮速
    FLOAT	x;
    FLOAT	y;
    FLOAT	yaw;                                                    //yaw
	UINT8 	nBonnetSts;								// 引擎盖 0关闭 1打开
	UINT8 	nTrunkSts;								// 后备箱 0关闭 1打开
	UINT8 	nSRFSts;								// 天窗 0关闭 1打开
	UINT8 	nReserve;	
	
	UINT8 	nReserved[256-4*30];	
}MvApaCanCarInfo;
#else
typedef struct _MV_Apa_CAN_CAR_INFO_256
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号
	UINT64 	lTimeMsec;												//时间戳(ms)
	INT32 	nBrake;													//刹车  1，刹车  0，未刹车 
	INT32 	nLLight;												//左转向灯 1、点亮 0、不亮
	INT32 	nRLight;												//右转向灯 1、点亮 0、不亮
	FLOAT 	fVelocity;												//车速	 km/h
	INT32 	nBrakeValue;  											//刹车力度 用于制动
	FLOAT 	fAccelPedalPosition;									//油门力度 
	FLOAT 	fLongiAccel;											//纵向加速度
	FLOAT 	fLateralAccel;											//横向加速度
	FLOAT 	fYawRate;												//偏航角速度
	INT32 	nTransmissionStatus;									//档位 0无效 1P 2R 3N 4D
	FLOAT 	fAlpha;													//车轮转向角 
	FLOAT 	fSteeingWheelAngle;										//方向盘转向角，顺时针为正，角度
	FLOAT 	fSteeingWheelRate;										//方向盘转向速率
	INT32   nParkBrake ;											//手刹 0 手刹未起作用 1 手刹起作用 2 错误 3 无效
	INT32   nWipeWasherSwitch;										//雨刮 0不工作 1低档 2中档 3高档 4间歇档
	INT32   nHazardLightSwitch;										//危险报警灯开关 0无效 1关 2开
	INT32	nHighLowBeamSwitch;										//近光远光灯开关 0无效 1近光 2远光 3错误 3无效
	UINT16  nFLWheelSpeedRC;										//左前轮速脉冲信号
	UINT16  nFRWheelSpeedRC;										//右前轮速脉冲信号
	UINT16  nRLWheelSpeedRC;										//左后轮速脉冲信号
	UINT16  nRRWheelSpeedRC;										//右后轮速脉冲信号
	UINT8  	nDoorStat[4];											//[0-4]左前,右前,左后,右后,0关门 1开门
	INT32   nPulseDirection;										//轮速脉冲方向
	UINT8 	nReserved[256-4*25];	
}MvApaCanCarInfo;
#endif

typedef struct _UssObsCoordCalProcessMap_merge {
	INT8 		nObsMx;
    INT8 		nObsMy;
    INT8 		nObsMz;
	INT8		reserved;
    UINT32 		nObsMsg;
} UssObsCoordCalProcessMap_merge;

typedef struct _MvPpsUssObsCoordCalProcessOut_merge {
	UINT64 lRecvTimeMs;
	UINT64 lMCU21STimeMs;
	UINT64 lMCU30RTimeMs;
	UINT64 lMCU30STimeMs;
	UINT64 lMCUSocTimeMs;
	UssObsCoordCalProcessMap_merge tObsR[20];
	UssObsCoordCalProcessMap_merge tObsF[20];
	UssObsCoordCalProcessMap_merge tObsSR[90];
	UssObsCoordCalProcessMap_merge tObsSL[90];
} MvPpsUssObsCoordCalProcessOut_merge;

typedef struct _UssSlotSearhApsProcessCoord_merge{
	float	fSlotUssAMx;
    float	fSlotUssBMx;
    float	fSlotUssCMx;
    float	fSlotUssDMx;
    float	fSlotUssAMy;
    float	fSlotUssBMy;
    float	fSlotUssCMy;
    float	fSlotUssDMy;
} UssSlotSearhApsProcessCoord_merge;

typedef struct _MvPpsUssSlotSearchApsProcessOut_merge {
	UINT64 lRecvTimeMs;
    UINT64 lMCU21STimeMs;
    UINT64 lMCU30RTimeMs;
    UINT64 lMCU30STimeMs;
    UINT64 lMCUSocTimeMs;
    UssSlotSearhApsProcessCoord_merge tLeftParaSlot[5];
    UssSlotSearhApsProcessCoord_merge tLeftVertSlot[5];
    UssSlotSearhApsProcessCoord_merge tRightParaSlot[5];
    UssSlotSearhApsProcessCoord_merge tRightVertSlot[5];
} MvPpsUssSlotSearchApsProcessOut_merge;

typedef struct _ApaUssInfo
{
	UINT64 msgCnt;
	MvPpsUssObsCoordCalProcessOut_merge   pUssObsInfo;
	MvPpsUssSlotSearchApsProcessOut_merge pUssSlotsInfo;
}ApaUssInfo;


typedef struct _MvVehPont_
{
	float		fx;
	float		fy;
	float		fyaw;
}MvVehPont;

typedef struct _MvTMVehPont_
{
	UINT64	lTimeMsec;     //时间戳(ms)
	MvVehPont tMvVehPont;
}MvTMVehPont;


#endif




#if 0
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
	UINT32				nDirection;										//车位方向 0:左 1:右 2:前 3:后			
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
#endif

//原视觉车位类型
typedef enum 
{
    OrgPerceptionParallelSlot = 0,  /* 平行车位 */
    OrgPerceptionVerticalSlot,      /* 垂直车位 */
    OrgPerceptionObliqueSlot        /* 斜列车位 */
}OrgSlotType;




#ifdef __cplusplus
}
#endif

#endif

