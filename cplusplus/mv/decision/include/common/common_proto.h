#ifndef COMMON_PROTO_H
#define COMMON_PROTO_H

#include<stdint.h>

#include  "pps_apa_common.h"

typedef uint8_t UINT8;
typedef uint32_t UINT32;
typedef uint64_t UINT64;
typedef float FLOAT;


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



#endif