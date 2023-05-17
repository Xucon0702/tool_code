#ifndef _APACOMMONSTRUCT_H_
#define _APACOMMONSTRUCT_H_
#include "typedefs.h"
#include "MvPlatformCommonStruct.h"
#include "vehicle.h"


#if 0

#define VehTrackNum		(20)



#define USS_RADAR_MAX_NUM		12


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

typedef struct _ApaUltrData
{
	float Distance1;	//一次回波
	float Distance2;	//二次回波
	float Level;		//回波强度
	float Width;		//回波宽度
	UInt8 status;		//状态
	UInt8	nReserved0[3];
}ApaUltrData;


typedef struct _PcHdmiToApaInfo 
{ 
	UINT32 uCmd;
   	UINT32 ApaCmd[MoudleMax];
	UINT32	uSoltType;		//0：视觉车位 1：自选车位             2:超声车位 其他无效 
	UINT32	uTurn;			//0：无效 1左侧泊车/前  2右侧泊车/后 3:两侧 其他无效 
	UINT32	uSenSor;		//0：无 1：蓝牙
	SlotData	tSelectSlot;//选择的车位信息
}PcHdmiToApaInfo;




typedef struct _HdmiToApaInfo 
{ 
	UINT32 uCmd;

   	MoudleStatue ApaCmd[MoudleMax];
	UINT32	uSoltType;		//0：视觉车位 1：自选车位             2:超声车位 其他无效 
	UINT32	uTurn;			//0：无效 1左侧泊车/前  2右侧泊车/后 3:两侧 其他无效 
	UINT32	uSenSor;		//0：无 1：蓝牙
	MvSlotData	tSelectSlot;//选择的车位信息
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
}PlanToHdmi; 

typedef struct _ApaToHdmiInfo 
{ 
	PlanToHdmi 		tPlanToHdmi;
	uint32_t		tSoltNum;
	MvSlotData 		tSolt[20];//选择的车位信息
}ApaToHdmiInfo; 


typedef struct _VehCtrInfo_
{
	uint8_t		CtrlModeBack;
	uint8_t		nReserved[3];
}PlanCtrl;



typedef struct _TrackPoint_
{
	float    	x;
	float    	y;
    float    	yaw;
    float    	curvature;
    float    	speed;
    int32_t 	nDetectStates;
}MvTrackPoint;


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

//Struct for slot info and VEL start point for calculation
typedef struct _InputParkingInDebug
{
     float fVehCurX;
    float fVehCurY;
    float fVehCurYaw;
    float fSlotDepth;
    float fSlotWidth;
    float fSlotYaw;
    float fAisleObstacleY;
    float fYFrontLine;
    float fYRearLine;
    float fXRearEdge;//
    float fXFrontEdge;//
    float fXSlotPosInGrid;
    float fYSlotPosInGrid;
    int8_t cSlotPosition;
    int8_t cSlotType;
    int8_t cParkType;
    int8_t cDetectType;
    int8_t cARV_On;
    int8_t cParkOutDir;
    int nUssSideType;


    //1111111
    int m_nDrivingDirection;
    float m_fMarginLeft;
    float m_fMarginRear;
    float m_fMarginFront;

    float m_fXRearMargin;
    float m_fXFrontMargin;
    float m_fYRightMargin;

    float m_VehTargety;
    float m_VehTargetx;
    float m_slotAngle;


    bool m_LastDriving;
    bool m_ParaHybridALastDriving;
    bool m_bStartPathplanningInParaSLot;
    bool m_bFirstPlan;
    bool m_bHasSetGoalPoint;


    int m_nFirstPlanTotalSteps;
    int m_TotalGearSwitchTimes;
    int mParaSlotSteps;
    bool m_bNotNeedParkingOut;
    bool m_bSlotFrontEdgeObstacle;
    bool m_bSlotRearEdgeObstacle;

    LocationPoint m_StartPoint;//3D
//    LocationPoint m_GoalPoint;//3D

    ParkingWorkingStatus m_ParkingCtrlStatus;

//    LocationPoint m_StartPointOffset;
    LocationPoint m_goalPointOffset;

    bool m_bUSSHasDetectFrontMargin;
    bool m_bUSSHasDetectRearMargin;
    LocationPoint RotationCoordinate;

	int m_nCurrentStep;
    int m_nTotalStep;
    //=====================================================208===============
    UINT32 uSaveID;
    INT32  nWarningFlag;
    LocationPoint tSaveP0;
    LocationPoint tSaveP1;
    //=====================================================32===============

	char Reserved[256-208-32 ];
	
    //std::vector<std::vector<CtrlPoint> > m_LastPlanningTargetCtrlPointsSet;
}InputParkingInDebug;


#endif


#endif


