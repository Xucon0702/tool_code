/***********************************************************************************************
 * 版权所有 2015 魔视智能科技(上海)有限公司
 * Copyright (c) 2015,Motovis Intelligent Technologies (Shanghai) Co.,Ltd
 * 魔视智能公司秘密
 * Motovis Intelligent Confidential Proprietary
 *
 * FileName: 			   
 * Author:				   taotao
 * Version:				   v3.2.0.1
 * Date: 				   2020/03/06
 * Encode：					UTF-8
 * Description:			   嵌入式发送平台的结构体 （从第三代网络开始）  
 *						   用于和平台交互使用，双方需要保持一致。
 * 							一、
 *								3个enum列表
 *								ModeAppE 		平台维护
 *								PrivateDataType	平台维护
 *								NetSendIndex	嵌入式维护
 *							二、
 *								发送顺序
 *								————————————————————————————————————————————————————————————————————————————————
 *								|主题数据			|图像数据		|定长数据(公+私)			|变长Seg数据		|变长Dump数据		|
 *								————————————————————————————————————————————————————————————————————————————————
 *								|ContentData	|VideoData	|FixData			|VarSegData		|VarDumpData	|
 *								————————————————————————————————————————————————————————————————————————————————
 *							
 * History:				   
 *		<Author>	  <Date>	  		<Version> 	 				<Description>
 *		taotao		2020/3/6			3.2.0.0						新建3.2版本结构体。从此版本开始，所有子结构体的版本号都从1开始。
 *		taotao		2020/3/9			3.2.0.1						1.增加5种图像类型；2.增加部分注释；3.远近灯和报警灯信息放入车身can结构体；4.修改slot；5.提出共有结构体类似point；6.删除车辆区域数据MvBusControlArea（公交项目），和 报警区域MvAlarmCtlArea（市政项目） ，移入私有库
 *										3.2.0.2						1.跟踪目标属性增加融合类型(for 7020版本)；2.把小于8字节的结构体都补到8字节对齐；3.所有外发结构体都加一个帧号字段；4.合入吉利最新车位信息结构体；5.把轮速脉冲移入车身结构体
 *										3.2.0.3						1.所有结构体8字节对齐; 2.Can车身结构体根据会议讨论修改; 3.slog的相关移入私有结构体（如 栅格）; 4.所有的结构体的帧号字段统一命名为 nFrameId; 5.删除eyewatch等不用结构体; 6.根据增减的结构体更新ENUM值（从这版本后，Enum不再修改已有的，一律只做添加）; 7. TSR数据RoadMark数据分开,不再塞到一个结构体里
 *		taotao		2020/3/10			3.2.0.4						1.扩大slot; 2.增加报警事项; 3.mainobj结构体改名，和二代重复了 4.增加环视相关耗时统计 5.修改OD结构体
 *		taotao		2020/3/12			3.3.0.0						1.版本大改动,content中增加描述发送顺序、数量的功能
 *		taotao		2020/3/13			3.3.0.1						1.为了吉利项目这种多核发送，需要平台进行组合的机制，保留nVideoMode和nSubVideoMode字段
 *		taotao		2020/3/13			3.3.0.2						1.增加enum值
 *		taotao		2020/3/18			3.3.0.3						1.增加多路的deviceNo这一项;2.将公交项目的_MvTrackImgPos_属性移入跟踪目标属性
					2020/3/20			3.3.0.4						1.增加对算法不同权重的seg名称的兼容;2.NetSendContentInfo修改顺序到64字节对齐;3.删除MvSlotInfo结构体,目前发送结构体里不需要
 *		taotao		2020/3/25			3.3.0.5						1.增加算法的车道线类型说明
 *		taotao		2020/3/27			3.3.1.0						1.修改私有数据的发送机制，改为在工程文件中声明各自的enum
 *		taotao		2020/3/30			3.3.1.1						1.修复MvSlotOutput结构体预留位bug
 *		taotao		2020/3/31			3.3.1.2						1.chDataEnum改为16位的，否则无法支持众多项目enum范围
 *					2020/4/1			3.3.1.3						1.增加了俯视的DeviceNoEnum，用来标识俯视2D相关数据
 *		zwj			2020/4/2			3.3.1.4						1.can增加时间戳和车门信息
 *		taotao		2020/4/3			3.3.1.5						1.光流结构体修改为8字节对齐
 *		zwj			2020/4/7			3.3.1.6						1.增加组合模式COMBINED_MODE_7(16 17)
 *		taotao		2020/4/13			3.3.1.7						1.增加车位的nOccupyFlag，华体项目上位机需要这个数据
 *		taotao		2020/4/16			3.3.1.8						1.删除nOccupyFlag
 *		taotao		2020/4/29			3.3.1.9						1.增加CanInfo结构体
 *		taotao		2020/5/6			3.3.1.10					1.新增seg通道；2.增加128个点 freespace拟合（MvArmFreeSpaceRegionInfoBV）3.增加栅格摘要 
 *      huping      2020/5/19           3.3.1.11                    1.Added ModeAppE Enum "TWO_YUV422_1280x720". For 2 channels YUV422.
		taotao		2020/6/6			3.3.1.12					1.Added ModeAppE Enum "C71_VISION". "FOUR_G_640x360".
***********************************************************************************************/

#ifndef _MV_PLATFORM_COMMON_STRUCT_H_
#define _MV_PLATFORM_COMMON_STRUCT_H_
	
#include "typedefs.h"											//用于数据类型定义

#define NET_SEND_STRUCT_VER "3.3.1.12"							//此文件版本定义

//===========================================================================================================
//===========================================================================================================
//=========================    宏定义       ====================================================================
//===========================================================================================================
//===========================================================================================================

//约定的Seg 通道 map表
//SEGKEY：各个通道定义对应的key值，对应SegChannelInfo.sChannelKey里的数据，这个嵌入式和平台统一就好。平台拿到参数知道是什么含义，怎么画图就行。
//SEGNAME：各个通道定义的对应的名称，这个需要嵌入式和算法一致，是在算法发布的weight.ini文件中注明的
#define MAX_SEG_CHANNAL_NUM			32							//	最多32个通道
#define MAX_SEG_KEY_NUM				32							//	最多的key数，注意这个值和MAX_SEG_CHANNAL_NUM不一样，最多32通道，但是key对应关系可能大于32,。比如这次1通道是车，下次1通道是人，这样占用一个通道，却是占用2个key
#define SEGKEY_EMPTY				0
#define SEGNAME_EMPTY				"empty"						//
#define SEGKEY_Lane					1
#define SEGNAME_Lane				"lane"						//车道线
#define SEGKEY_Freespace			2
#define SEGNAME_Freespace			"freespace"					//可行驶区域
#define SEGKEY_Curb					3
#define SEGNAME_Curb				"curb"						//路沿
#define SEGKEY_Zebra				4
#define SEGNAME_Zebra				"zebracross"				//斑马线
#define SEGKEY_People				5
#define SEGNAME_People				"people"					//人
#define SEGKEY_Car					6
#define SEGNAME_Car					"car"						//小汽车
#define SEGKEY_Bike					7
#define SEGNAME_Bike				"bike"						//自行车
#define SEGKEY_Guideline			8
#define SEGNAME_Guideline			"guideline"					//导向线
#define SEGKEY_Wheel				9
#define SEGNAME_Wheel				"wheel"						//车轮
#define SEGKEY_whitebroken			10
#define SEGNAME_whitebroken			"whitebroken"				//白虚线
#define SEGKEY_yellowbroke			11
#define SEGNAME_yellowbroke			"yellowbroke"				//黄虚线
#define SEGKEY_whiteline			12
#define SEGNAME_whiteline			"whiteline"					//白实线
#define SEGNAME_whiteline2			"white_lines"				//白实线，算法不同人出的权重,这个通道名字起的不一样，需要做兼容
#define SEGKEY_yellowline			13
#define SEGNAME_yellowline			"yellowline"				//黄实线
#define SEGNAME_yellowline2			"yellow_lines"				//黄实线
#define SEGKEY_LaneMark				14
#define SEGNAME_LaneMark			"landmark"					//地面标志
#define SEGNAME_LaneMark2			"land_mark"					//地面标志
#define SEGKEY_groundpin			15
#define SEGNAME_groundpin			"groundpin"					//挡杆
#define SEGKEY_obstacle				16
#define SEGNAME_obstacle			"obstacle"					//障碍物
#define SEGKEY_pillar				17
#define SEGNAME_pillar				"pillar"					//柱子
#define SEGKEY_deceleraion_hump		18
#define SEGNAME_deceleraion_hump	"deceleraion_hump"			//减速带
#define SEGKEY_parkingline			19
#define SEGNAME_parkingline			"parkingline"
#define SEGKEY_carlight				20
#define SEGNAME_carlight			"car_light"					//车灯
#define SEGKEY_NoMotor				21
#define SEGNAME_NoMotor				"nomotor"
#define SEGKEY_Motor				22					
#define SEGNAME_Motor				"motor"						//摩托车





//约定的TSR  类别 map表    
//2019.12.11 将TSR的index改成和算法神经网络输出值一致的，对照表来自算法内网资料，需要和算法一致
#define UNKNOWN_TYPE		(0)
#define SPEED5_MAP_INDEX (104)
#define SPEED10_MAP_INDEX (94)
#define SPEED15_MAP_INDEX (98)
#define SPEED20_MAP_INDEX (99)
#define SPEED25_MAP_INDEX (100)
#define SPEED30_MAP_INDEX (101)
#define SPEED35_MAP_INDEX (102)
#define SPEED40_MAP_INDEX (103)
#define SPEED50_MAP_INDEX (105)
#define SPEED60_MAP_INDEX (106)
#define SPEED70_MAP_INDEX (107)
#define SPEED80_MAP_INDEX (108)
#define SPEED90_MAP_INDEX (109)
#define SPEED100_MAP_INDEX (95)
#define SPEED110_MAP_INDEX (110)
#define SPEED120_MAP_INDEX (97)
//以下是算法网络没有指定的,临时自行指定（TSR网络没有地面标线）
#define STRAIGHT_LINE_MAP_INDEX (4)
#define STRAIGHT_LEFT_LINE_MAP_INDEX (6)
#define STRAIGHT_RIGHT_LINE_MAP_INDEX (5)
#define RIGHT_LINE_MAP_INDEX (1)
#define LEFT_LINE_MAP_INDEX (3)
#define LEFT_RIGHT_LINE_MAP_INDEX (2)

//===========================================================================================================
//===========================================================================================================
//=========================    ENUM       ====================================================================
//===========================================================================================================
//===========================================================================================================

//=============================================================================================
// 传输图像类型
// 此enum由平台组维护并提供新的ENUM
// 如果项目需要一种新的传输方式，请到平台组维护此enum的人员处申请一个新的enum值使用.
// 请不要自行添加，避免与其他项目出现冲突
//=============================================================================================
enum ModeAppE
{
	NO_VIDEO 					= 0,					//无图像数据
	FRONT_G_640x360 			= 1,					//前视G分量小图,大小640x360
	FRONT_Y_1280x720 			= 2,					//前视Y分量大图,大小1280x720
	FRONT_YUV422_640x360 		= 3,					//前视YUV422小图,大小640x360x2
	FRONT_YUV422_1280x720 		= 4,					//前视YUV422大图,大小1280x720x2
	FRONT_RGB888_640x360 		= 5,					//前视RGB888小图,大小640x360x4
	FRONT_RGB888_1280x720 		= 6,					//前视RGB888大图,大小1280x720x4
	FRONT_H264_1280x720 		= 7,					//前视h264
	LEFT_H264_1280x720 			= 8,					//左视h264
	RIGHT_H264_1280x720 		= 9,					//右视h264
	INNER_H264_1280x720 		= 10,					//内视h264
	BUS_3G_640x360 				= 11,					//公交三种G分量小图,大小640x360
	FRONT_H264_1280x1080 		= 12,					//沃尔顿h264,大小1280x1080
	PARKING_2D_RGBA_640x720 	= 13,					//2D车位拼接图,大小640*720*4(2D RGBA拼接图+2D seg+2D G拼接图)
	LEFT_RIGHT_FISH_640x360 	= 14,					//左右鱼眼G
	FRONT_REAR_FISH_640x360 	= 15,					//前后鱼眼G
	FOURE_SEG 					= 16,					//四路seg 
	PARKING_2D_G_800x1000 		= 17,					//G分量 2D拼接图,大小800x1000
	LINE_GRAPH 					= 18,					//BSD线性图
	COMBINED_MODE_1 			= 19,					//组合模式1-吉利(13 16 17 +融合规划)	
	COMBINED_MODE_2 			= 20,					//组合模式2( 13 14 16)	
	COMBINED_MODE_3 			= 21,					//组合模式3( 13 15 16)	
	COMBINED_MODE_4 			= 22,					//组合模式4(13  16  17)
	COMBINED_MODE_5 			= 23,					//组合模式5( 13 16 17 18)
	COMBINED_MODE_6 			= 24,					//组合模式6(17 18)
	LEFT_FISH_640x360 			= 25,					//左路鱼眼小图,大小640x360
	RIGHT_FISH_640x360 			= 26,	
	FRONT_FISH_640x360 			= 27,					//前后鱼眼小图,大小640x360
	REAR_FISH_640x360 			= 28,	
	RGBA_YUV 					= 29,					//YUV(1280*720*2) + 2D RGBA(640*720*4) 
	LEFT_YUV422_1280x720 		= 30,					//左路YUV	
	RIGHT_YUV422_1280x720 		= 31,					//右路YUV
	REAR_YUV422_1280x720 		= 32,					//后路YUV
	FOUR_YUV422_1280x720 		= 33,					//4路YUV
	COMBINED_MODE_7 			= 34,					//组合模式6(16 17)
	TWO_YUV422_1280x720         = 35,                   //2 CH YUV422: (1280*720*2)*2
	C71_VISION 					= 36,					//C71 环视 所有结果 环视-NetSendContentInfo + 640*720*4 + 160*160*36 + 640*360*4 + 256*160*20*4 + +MvArmDetectInfo+MvArmTrackInfo+ MvCanCarInfo+MvSlotOutput + MvLaneLineInfo+ MvFreeSpaceRegionInfo
	FOUR_G_640x360				= 37,					//C71 环视 4路小图+鱼眼结果 4路小图- NetSendContentInfo + 640*360*4+MvArmDetectInfo+MvArmTrackInfo
    C71_GRID                    = 38,                   //C71 栅格图 400*500 
    C71_GRID_RTE                = 39,                   //C71 栅格图数据RTE接口
    INVALID_MODE_APP = -1
};

//=============================================================================================
// 定长数据结构体的Index
// 此enum由嵌入式维护
// 必须保持向下兼容，可以添加新的，但不要修改已有的
//=============================================================================================
enum NetSendIndex
{
	NS_NO_DATA				= 0,						//
	NS_CONTENT_DATA			= 1,						//主题数据
	NS_VIDEO_DATA			= 2,						//图像数据
	NS_RESO_RATIO_INFO		= 3,						//分辨率信息，MvArmResoRatioInfo
	NS_DETECT_OBJ			= 4,						//检测目标，MvArmDetectInfo
	NS_TRACK_OBJ			= 5,						//跟踪目标，MvArmTrackInfo
	NS_FREESPACE_REGION		= 6,						//可行驶区域拟合64点，MvArmFreeSpaceRegionInfo
	NS_TIME_INFO			= 7,						//时间统计，MvArmTimeInfo
	NS_TSR_INFO				= 8,						//交通标志，MvArmTsrInfo
	NS_ROADMARK_INFO		= 9,						//交通标志，MvArmLaneMarkInfo
	NS_IHBC_INFO			= 10,						//IHBC信息，MvArmIhbcInfo
	NS_ALG_LOG				= 11,						//算法调试日志，MvAlgLogInfo
	NS_ARM_LOG				= 12,						//嵌入式调试日志，MvArmLogInfo
	NS_ROAD_INFO			= 13,						//车道线信息，MvArm2ndRoadOutputInfo	
	NS_CAN_CAR_INFO			= 14,						//Can车身信息，MvCanCarInfo
	NS_CAN_FUNC_INFO		= 15,						//Can功能信息，MvCanFuncInfo
	NS_SYSTEM_INFO			= 16,						//系统状态，MvSystemInfo
	NS_CAN_OBJ_ID			= 17,						//调整ID后的目标信息，MvArmCanObjID
	NS_MAIN_OBJ_INFO		= 18,						//主目标信息，MvArmMainObjInfo
	NS_WARN_INFO			= 19,						//报警信息，MvWarnInfo
	NS_IMU_INFO				= 20,						//IMU信息，MvImuDataPack
	NS_OPT_INFO				= 21,						//光流信息，MvOpticalFlowInfo
	NS_SLOT_INFO			= 22,						//车位信息，MvSlotOutput
	NS_AVWARN_INFO			= 23,						//环视报警信息，MvAroundViewWarnStruct
	//NS_PRIVATE_DATA			= 24,					//私有数据，放在所有共有结构体后面发。(已取消此机制)
	NS_SEG_SAMMARY			= 25,						//Seg数据摘要	，MvArmSegSummaryInfo		
	NS_SEG_DATA				= 26,						//Seg数据
	NS_DUMP_SAMMATY			= 27,						//Dump数据摘要，MvDumpDataSummary
	NS_DUMP_DATA			= 28,						//Dump数据
	NS_CAN_INFO				= 29,						//Can完整信息，MvCanInfo
	NS_FREESPACE_REGION_BV	= 30,						//可行驶区域拟合128点（2D图），MvArmFreeSpaceRegionInfoBV
	NS_GRID_SAMMARY			= 31,						//栅格数据摘要，MvGridDataSummary
	NS_GRID_DATA			= 32,						//栅格数据
    NS_MULTISLOT_INFO		= 33,						//车位信息，MvMultiSlotOutput
    NS_ULTRAPARK		    = 34,						//原始超声车位，ApaUssInfo
    NS_GRIDMAP		        = 35,						//融合栅格地图数据，MvFusionGridMap
    NS_DECISION_PARK        = 36,                       //决策车位
    NS_UPDATE_SLOT		    = 38,					    //闭环更新数据，MvFusionGridMap
    NS_DECISION_ERRORCODE   = 39,					    //决策错误信息，MvDecisionE
    NS_PLANNING_OUTPUT      = 40,					    //规划输出，MvPlanningOutput
	/*
		add new struct index here.
	*/
	NS_NetSendIndex_MAX_NUM
};

enum NetGeelyIndex
{
    NS_GEELY_MIN            = 1100,
    NS_GEELY_LANE_LINE      = 1101,
    NS_GEELY_FREESPACE_POINT= 1102,
    NS_GEELY_FREESPACE_GRID = 1103,
	NS_FISHEYE_DETECT      	= 1108,      //鱼眼检测
    NS_OPTICAL              = 1109,      //光流
    NS_POINT_CLOUD          = 1110,      //点云
    NS_TRACK_OBSTACLE       = 1111,      //动静态障碍物
};

//=============================================================================================
//设备enum,为了兼容最早的环视版本，这个是按算法的顺序来的
//=============================================================================================
enum DeviceNoEnum
{
	DN_LEFT					=	0,					//左视
	DN_FRONT				=	1,					//前视
	DN_RIGHT				=	2,					//右视
	DN_REAR					=	3,					//后视
	DN_VERTICAL				=	10,					//俯视,用于俯视图2D相关数据的标识			
	DN_EMPTY				=	255,				//空，此通道无效
};

//===========================================================================================================
//===========================================================================================================
//=========================    结构体       ====================================================================
//===========================================================================================================
//===========================================================================================================
#pragma pack(push,1)

//===========================================================================================================
//======================== 公用结构体数据     ===================================================================
//===========================================================================================================
typedef struct _ArmPointInt16_
{
	INT16 	x;															//X坐标
	INT16 	y;															//Y坐标	
}ArmPointInt16;

typedef struct _ArmPointInt32_
{
	INT32 	x;															//X坐标
	INT32 	y;															//Y坐标	
}ArmPointInt32;

typedef struct _ArmPointFloat_
{
	FLOAT 	x;															//X坐标
	FLOAT 	y;															//Y坐标	
}ArmPointFloat;


#ifndef MV_FREESPACE_INFO
#define MV_FREESPACE_INFO

/**********************************************************
栅格图以车辆中心为中心，第一个字节对应位置为车辆左前方水平10米，垂直12.5米处。障碍物以凸边形方式输出。
***********************************************************/
typedef struct _MvFreeSpaceInfo_
{
    UINT64      lTimeMsec;		//时间戳，单位ms
    UINT32		nFrameIndex;	//帧序号
    UINT32 		nResvered;		//保留
    CHAR        chFreeSpace[200*250];	//20*25米，精度10cm*10cm;
                                        //0=freespace, 1=curb ,2=减速带
                                        //11=一般障碍物, 12=行人, 13=二轮车，14=多轮车,15=柱子,16=限位杆 ,17=车位锁开 18=车位锁关
} MvFreeSpaceInfo;

#endif

//===========================================================================================================
//======================== 分辨率数据     ===================================================================
//===========================================================================================================
typedef struct Resolution_ratio_32_										//分辨率信息，遵循变换顺序是先crop再resize
{
	UINT16		nChannel;									
	UINT16 		nVideoWidth;											//视频图像宽，eg 1280
	UINT16		nVideoHeight;											//视频图像高，eg 720
	UINT16		nVideoCropWidth;										//Crop之后的图像宽
	UINT16		nVideoCropHight;										//Crop之后的图像高
	UINT16		nVideoCropWidthOffset;									//Crop上面的偏移量
	UINT16		nVideoCropHightOffset;									//Crop左边的偏移量
	UINT16		nCnnWidth;												//Crop之后resize之后的CNN图像宽
	UINT16		nCnnHeight;												//Crop之后resize之后的图像高
	UINT16		nSegWidth;												//Seg图像宽
	UINT16		nSegHeight;												//Seg图像高
	UINT16		nRes[5];												//对齐到8字节
}ResolutionRatioInfo;

typedef struct Mv_ARM_RESORATION_RATIO_256_											//分辨率信息，用于16801命令通道一次传输
{
	UINT32 					nStructLen;									//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32      			nStructVersion;                             //此结构体"MvArmResoRatioInfo"的版本
	UINT32      			nChannelNum;                             	//包含几路分辨率信息
	ResolutionRatioInfo		tResolutionRatioInfo[4];					//4路的分辨率信息
	INT8 		cReserved[256 - 4*3 - 32*4];							//对齐到8字节
}MvArmResoRatioInfo;												
//===========================================================================================================
//======================== 检测目标 ===============================================================
//===========================================================================================================
typedef struct ARM_DETECT_OBJ_INFO_128_									//检测目标结果
{
	INT16 	nDetectType;			 									//目标类型:小类目标 0：人，1：小汽车/SUV，2：大巴，3：卡车，4：中巴，5：斑马线，6：两轮车，7：三轮车，10：交通标志，13：障碍物
	INT16	cDeviceNo;													//所属哪一路,在DeviceNoEnum里选
	FLOAT	fTop;														//检测框坐标
	FLOAT	fLeft;														//检测框坐标
	FLOAT	fRight;														//检测框坐标
	FLOAT	fBottom;													//检测框坐标
	FLOAT	fWeight;													//检测置信度
	ArmPointInt32	Boxviewpt[8];										//3D box检测结果
	FLOAT	fBoxDirection;												//3D box车头航向
	INT8	cRes[128 - 2*2 - 4*5 - 8*8 - 4];							//对齐到8字节
}ArmDetObjInfo;

typedef struct Detect_Info_15k_											//检测目标结果
{
	UINT32 				nStructLen;										//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 				nFrameId;										//用来表示本包检测数据属于哪一帧图像
	INT32				nDetectNum;										//检测目标个数
	ArmDetObjInfo		mDetectArrObj[80];								//检测目标属性
	INT8 				cReserved[15360-4*3-128*80];					//对齐到8字节
}MvArmDetectInfo;
//===========================================================================================================
//======================== 跟踪目标 ===============================================================
//===========================================================================================================
typedef struct _MV_ARM_THREE_DIMEMSION_SIZE_12							//"MvArmThreeDimensionSize" is changed from Alg struct "MvThreeDimensionSize"
{
	FLOAT 	fLength;
	FLOAT 	fWidth;
	FLOAT 	fHeight;
}MvArmThreeDimensionSize;


typedef struct _MvTrackImgPos_20
{
	UINT8 nDeviceNo;
	UINT8 nRes[3];
	FLOAT nTop;															//目标框位置左上点纵坐标
	FLOAT nLeft;                    									//目标框位置左上点横坐标
	FLOAT nRight;                   									//目标框位置右下点横坐标
	FLOAT nBottom;                  									//目标框位置右下点纵坐标
}MvArmTrackImgPos;



typedef struct ARM_TRACK_OBJ_INFO_320_									//"ArmTrackObjInfo" is changed form Alg struct "MvOutObject"							
{
	INT16 	nType;               										//目标类型:大类目标 0：人/两轮车，1：机动车，2：其他，3：不需要跟踪的目标，4：地面标记
	INT16 	nDetectType;			 									//目标类型:小类目标 0：人，1：小汽车/SUV，2：大巴，3：卡车，4：中巴，5：斑马线，6：两轮车，7：三轮车，10：交通标志，13：障碍物
	INT32 	nId;														//目标ID				
	FLOAT 	fTop;														//坐标						
	FLOAT 	fLeft; 														//坐标
	FLOAT 	fRight;														//坐标
	FLOAT 	fBottom; 													//坐标
	FLOAT 	fWeight;													//如果为负数，表明此结果是推算出来的结果，目标置信度，最大值1，最小值0
	FLOAT 	fDist;														//每个目标的距离	单位m，最大值150，最小值-150
	FLOAT 	fVelo;														//相对运动目标的相对速度 m/s最大100			
	FLOAT 	fTTC;														//ttc时间 单位s，最小值0
	INT16	sFcwMark;													//FCW输出标志
	INT16	sReserved;													//输出标志恢复为保留
	FLOAT 	fXDist;                   									//水平方向距离(m)
	FLOAT 	fXVelo;														//水平方向速度(m/s)
	INT32 	nIncomingState;												//-255 未初始化，1 是  0否
	FLOAT 	fAcc;														//加速度
	INT32 	nObjVaild;   												//1=new;  2 =Measured in this cycle;  3=Not Measured in this cycle;  4 = tracker;
	INT32 	nLanePos;													//返回目标车道信息 -255 未初始化 -2：靠左 -1：靠左，左车道，0：当前车道，1靠右,右车道  2 靠右    3 cut-in（在旁边车道）  4-cut out（在当前车道）
	INT32 	nIsMainObj;                 								//是否是主目标：1是 0否
	FLOAT 	fObjWidth;													//目标的宽(世界坐标)单位m，最大值10，最小值0
	FLOAT 	fObjHeight;													//目标的高度(世界坐标) 单位m，最大值10，最小值0
	FLOAT 	fXAcc;														//目标横向加速度 单位m/s2，最大值10，最小值0
	INT32 	nHisLen;													//跟踪历史长度 连续帧数，最大值255，最小值0
	INT8 	chFcwTTCValid;             									//1=valid 0=invalid
	INT8 	cDeviceNo;													//设备号,在DeviceNoEnum里选，单摄像头视觉目标填写
	INT16 	sRangeTTCScore;												//距离、TTC的打分, 从0到10表示不可靠到可靠
    INT8 	nFusionState;              									//0 纯视觉目标， 1视觉雷达融合目标， 2纯雷达目标
	INT8 	nStatus;													//目标状态，第一位表示是否静止(增加部分)
	INT8 	chReserved[2];	
	ArmPointInt32 Boxviewpt[8];											//上汽项目 for 3D box
	FLOAT 	fBoxDirection;    											//上汽项目 车头朝向
	MvArmThreeDimensionSize BoxSize;
	INT8 	nBetweenLane;												//北汽项目，是否在车道线内
	INT8	nIfInFreespace;												//是否在freespace内
	INT8 	chReserved1[2];												//对齐到8字节
	MvArmTrackImgPos tImgPos[4];										//目标在左前右后4路摄像头的坐标，视觉融合目标填写
	FLOAT 	fAngle;     												//目标和本车夹角
	FLOAT 	fObjLength;													//目标的长度(世界坐标) 单位m
	INT8 	chReserved2[320-4*24-8*8-4-12-4-20*4-4*2];					//对齐到8字节
}ArmTrackObjInfo;

typedef struct Track_Info_25k_											//跟踪目标结果
{
	UINT32 				nStructLen;										//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32				nFrameId;										//用来表示本包检测数据属于哪一帧图像
	INT32				nTrackNum;										//跟踪目标个数
	ArmTrackObjInfo		mTrackArrObj[64];								//跟踪目标属性
	INT8 				cReserved[25600-4*3-320*64];
}MvArmTrackInfo;
//===========================================================================================================
//======================== Freespace多边形拟合数据 ==========================================================
//===========================================================================================================
typedef struct _ArmFreeSpacePoint_24								//"ArmFreeSpacePoint" is changed form Alg struct "MvFreeSpacePoint"
{
	ArmPointInt32 		pt;											//图像坐标，相对于1280 720的图像
	ArmPointFloat 	ptWorld;										//世界坐标系位置
	UINT8 			uType;											//属性40：路沿，0人，1车
	UINT8 			nFlag;                                  		//是不是最远距离标记
	INT8			chReserved[24-8*2-2-4];
	FLOAT		  	fRadius;	 		 							//每一个点到车辆中心的绝对距离
}ArmFreeSpacePoint;

typedef struct FV_FreeSpaceRegionInfo_2k							//front view 前视FreeSpace拟合固定64个点
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 		nFrameId;											//帧号
	ArmFreeSpacePoint pt[64];										//坐标，相对于1280 720的图像
	INT32 numbers;													//点数
	UINT8	nDeviceNo;												//当前是第几路视频，在DeviceNoEnum里选
	UINT8	nRes[3];
	INT8 chReserved[2048-4*2-24*64-4-4];
}MvArmFreeSpaceRegionInfo;


typedef struct BV_FreeSapceRegionInfo_4k							//bird view 俯视Freespace拟合固定128个点,2D图
{
	UINT32 					nStructLen;								//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32					nFrameIndex;							//帧序号
	UINT32 					nPointNum;								//点的个数
	ArmFreeSpacePoint		tPoint[128];							//freespace顶点信息			
	UINT64      			lTimeMsec;								//时间戳，单位ms	
	INT8					chRes[4096 - 12 - 24*128 - 8];
}MvArmFreeSpaceRegionInfoBV;


typedef struct _MvGeelyFreeSpacePoint_
{
    ArmPointInt32 	tImagePoint;	 //图像坐标系，对应于200*250的图像
    ArmPointFloat 	tWorldPoint;   //世界坐标系位置
    FLOAT		  	fRadius;	 	 //每一个点到车辆中心的绝对距离
    UINT8		  	nType;		 //顶点对应障碍物类型  10=路沿Yellow 3=限位杆purple (11 16)=障碍物light blue 14=多轮车red 12=行人green 15=柱子blue 13=二轮车black
    UINT8   	  	chReserved[3];
} MvGeelyFreeSpacePoint;


typedef struct _MvGeelyFreeSpaceRegionInfo_
{
    UINT64      			lTimeMsec;			//时间戳，单位ms
    UINT32					nFrameIndex;		//帧序号
    UINT32 					nPointNum;			//点的个数
    MvGeelyFreeSpacePoint	tPoint[128];		//freespace顶点信息
} MvGeelyFreeSpaceRegionInfo;


//===========================================================================================================
//======================== 时间数据 =========================================================================
//===========================================================================================================
typedef struct Time_Info_256
{
	UINT32 		nStructLen;											//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 		nFrameId;											//帧号
	Uint64 		lVideoTimeStampUs;									//本帧图像时间戳(从FPGA获取)
	Uint64		lCnnCalcStampUs;									//开始Can计算的时间戳
	Uint64		lTrackStampUs;										//开始跟踪
	Uint64   	lNetSendStampUs;									//本帧网络发送时间戳(linux系统获取)
	FLOAT		fCnnCalcMs;											//Cnn后处理计算耗时
	FLOAT		fOptCacheMs;										//光流刷cache耗时
	FLOAT		fFcCalcMs;											//FC后处理计算耗时
	FLOAT		fSegDecodeMs;										//Seg解码耗时
	FLOAT		fSegEncodeMs;										//Seg编码耗时
	FLOAT		fTrackObjMs;										//跟踪算法耗时
	FLOAT		fRoadMs;											//车道线拟合耗时
	FLOAT		fFreespaceRegionMs;									//多边型拟合耗时
	FLOAT		fSelfStudyTimeMs;									//自学习耗时
	FLOAT		fTsrMs;												//TSR后处理耗时
	FLOAT		fRoadmarkMs;										//地面标线后处理耗时
	FLOAT		fWarnMs;											//报警算法处理耗时
	FLOAT		fPriMs;		
	FLOAT		fPriMs2;												
	Uint64		lSlotDetectStampUs;									//车位检测耗时
	Uint64		lParkStampUs;										//泊车跟踪耗时
	Uint64   	lBSDStampUs;										//BSD耗时
	Uint64   	lMODStampUs;										//MOD耗时	
	Uint64   	lDOWStampUs;										//DOW耗时
	
	INT8		cReserved[256-4*2-8*9-4*14];						
}MvArmTimeInfo;
//===========================================================================================================
//======================== Tsr数据 ==========================================================================
//===========================================================================================================
typedef struct _MvTsrInfo_28
{
	INT16 nTop;														//TSR检测框坐标
	INT16 nLeft;													//TSR检测框坐标
	INT16 nRight;													//TSR检测框坐标
	INT16 nBottom;													//TSR检测框坐标
	INT16 sFirstSliVeloType;										//存放第一限速类别
	INT16 sFirstSliScore;											//存放第一限速置信度
	INT16 sSecondSliVeloType;										//存放第二限速类别
	INT16 sSecondSliScore;											//存放第二限速置信度
	INT16 sThirdSliVeloType;										//存放第三限速类别
	INT16 sThirdSliScore;											//存放第三限速置信度
	FLOAT fWorldX;													//世界坐标系X
	FLOAT fWorldY;													//世界坐标系Y
}MvTsrInfo;	

typedef struct _MvTsrInfoArray_512									//TSR标志 eg 限速
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	INT32 nFrameId;													//TSR检测帧ID
	INT32 nTsrNum;													//TSR检测目标数量
	MvTsrInfo mTsrInfo[16];											//TSR检测目标属性
	INT8	nRes[512 - 4*3 - 28*16];								//8字节对齐
}MvArmTsrInfo;

typedef struct _MvLaneMarkInfoArray_512				  				//same as "MvTsrInfoArray"   地面标志目标，eg 左转，右转...
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异

	INT32 nFrameId;													//地面标线检测帧ID
	INT32 nLaneMarkNum;												//地面标线检测目标数据
	MvTsrInfo mLaneMarkInfo[16];									//地面标线检测目标属性
	INT8	nRes[512 - 4*3 - 28*16];								//8字节对齐

}MvArmLaneMarkInfo;


//===========================================================================================================
//======================== IHBC数据 =========================================================================
//===========================================================================================================
typedef struct _MvIhbcInfo_8
{
	INT16  sFirstIhbcType;											//存放第一ihbc类别
	INT16  sFirstIhbcScore;											//存放第一ihbc置信度
	INT16  sSecondIhbcType;											//存放第二ihbc类别
	INT16  sSecondIhbcScore;										//存放第二ihbc置信度
}MvIhbcInfo;

typedef struct Ihbc_Info_128
{
	UINT32 		nStructLen;											//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异=
	INT32 		nIhcStatus;											//算法建议 近光=1 远光=2
	INT32 		nIhbcID;											//ihbc检测帧帧号
	INT32 		nIhbcNum;											//Ihbc本帧检测数量
	MvIhbcInfo 	mIhbcInfo[2];										//IHBC目标信息
	UINT32 		nIhbcFrameId;										//Ihbc检测帧号
	INT8 		chReserved[128-4*4-8*2-4];							
}MvArmIhbcInfo;
//===========================================================================================================
//======================== log数据 ==========================================================================
//===========================================================================================================
typedef struct ALG_LOG_2048											//算法调试日志
{
	INT8 AlgLog[2048];
}MvAlgLogInfo;

typedef struct Arm_LOG_2048											//嵌入式调试日志
{
	INT8 ArmLog[2048];
}MvArmLogInfo;
//===========================================================================================================
//======================== 车道线数据 =======================================================================
//===========================================================================================================
#if 0
typedef enum _MvArmLANETYPE											//From ALG "AlgCommonStructDef.h"
{
	MV_WHITE_SOLID_LINE = 10,										//白实线
	MV_YELLOW_SOLID_LINE,											// = 11,//黄实线
	MV_WHITE_DASHED_LINE,											// = 12,//白虚线
	MV_YELLOW_DASHED_LINE,											//13//黄虚线	
	MV_NOTYPE_LINE,													// 14不区分线性及颜色
	MV_DOUBLE_YELLOW_LINE,											//双黄线
	MV_DOUBLE_WHITE_LINE,											//双白线
	MV_SOLID_LINE = 20,												//单实现
	MV_DASHED_LINE,													//单虚线
	MV_DOUBLE_SOLID_LINE,											//双实线
	MV_LEFT_DEFICIENCY_RIGHT_EXCESS,								//左虚右实
	MV_LEFT_EXCESS_RIGHT_DEFICIENCY,								//左实右虚
	MV_CURB
}MvArm_LANE_TYPE;
#endif

typedef struct __ARM_LANE_OUTPUT_INFO_128
{
	INT32		nLaneID;											//车道线ID用于跟踪，-1 ：未检测到车道线，最大值10，最小值-1
	FLOAT		fAxisAngle;											//车道与车轴线角度 单位度，最大值180，最小值-180
	FLOAT		fCurveCoff[4];										//世界坐标，计算公式 x=fCurveCoff[3]*y^3+fCurveCoff[2]*y^2+fCurveCoff[1]*y+fCurveCoff[0]
	FLOAT		fScore;												//置信度，最大值1，最小值0
	INT32		nCurveMark;											//0:直线  1:曲线
	INT32	 	nStartX;  											//起始点X坐标，与原图分辨率有关
	INT32  		nStartY;											//起始点Y坐标，与原图分辨率有关
	INT32	 	nEndX;												//终点X坐标，与原图分辨率有关
	INT32   	nEndY;   											//终点Y坐标，与原图分辨率有关
	FLOAT		fSrcCurveCoff[4];									//图像坐标，计算公式 x=fSrcCurveCoff[3]*y^3+fSrcCurveCoff[2]*y^2+fSrcCurveCoff[1]*y+fSrcCurveCoff[0]
	FLOAT		fDx;												//velo: at x direction	靠近车道线的速度,>0表示靠右，<0表示靠左, 单位m/s，最大值100，最小值-100
	INT32		nLineType;											//车道线类型：见MvArm_LANE_TYPE的enum
	FLOAT		fLaneLineWidth;										//车道线宽度 单位米,最小值0,最大值1
	FLOAT		fLaneLineLength;									//车道线长度 单位米,最小值0,最大值255
	FLOAT		fShadowScore;										//阴影判断（内部用）
	INT32     	nRealLaneStartX;
	INT32     	nRealLaneStartY;
	INT32     	nRealLaneEndX;
	INT32     	nRealLaneEndY;
	INT32     	nLaneLineColor;										//车道线颜色（0：未知，1：白色，2：黄色，3：未知）
	INT8		pReserve[128 - 4*21-20];	
}ArmLaneOutputInfo;


typedef struct __MV_ARM_LANE_SRC_INFO_2K5							//原图车道线基本信息//modified From "MvLaneSrcInfo" of ALG
{
	UINT16  pSrcPointX[4][128];										//车道线数据点
	UINT16  pSrcPointY[4][128];										//车道线数据点
	UINT16  nPointCounters[4];										//车道线数据点
	INT8	pReserve[2560-2*4*128-2*4*128-2*4];	
}MvArmLaneSrcInfo;   

typedef struct __ARM_SECOND_ROAD_INFO_4k
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号
	ArmLaneOutputInfo pLaneInfo[4];
	INT32	nState;													//内部用
	FLOAT	fLaneWidth;												//车道线宽度单位米，最大值5，最小值0
	FLOAT	fLeftDistance[2];										//摄像机到左车道线宽度单位米，最大值5，最小值//fLeftDistance[0]:(perpendicular to lane) //fLeftDistance[1]:(perpendicular to axis)
	FLOAT	fRightDistance[2];	 
	FLOAT   fTTCx;              									//TTC: at x direction 单位秒，最大值1024，最小值0
	FLOAT	fDx;													//velo in x direction 靠近车道线的速度,>0表示靠右，<0表示靠左, 单位m/s，最大值100，最小值-100
	FLOAT   fTyreToLeftLane;										//左轮胎到左车道线的距离,绝对值大于5表示无效。单位m，最大值5，最小值-5
	FLOAT   fTyreToRightLane;   									//右轮胎到右车道线的距离,绝对值大于5表示无效。单位m，最大值5，最小值-5
	INT8 	pReserve[1024-128*4-4*10-4];
	FLOAT   fCameraRTMatrix[25];            						//相机的rt矩阵，用于在客户端端上显示车道线
	MvArmLaneSrcInfo tLaneSrcPointInfo;   							//原图的点坐标信息，供客户端使用
	INT8 	pReserve2[4096-4-1024-4*25-2560];			
}MvArm2ndRoadOutputInfo;


//===========================================================================================================
//======================== Can数据 ==========================================================================
//===========================================================================================================
typedef struct _MV_CAN_CAR_INFO_256
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
    INT32   nPulseDirection;										//轮速脉冲方向     1 向前,2向后,0 静止   
    FLOAT   fWheelSpd;                     							//估计轮速
    FLOAT	x;
    FLOAT	y;
    FLOAT	yaw;                                                    //yaw
	
    UINT8	nRcta;													//RCTA 			b0:左报警 b1:右报警  b2-b7：保留    0：无 1：有报警
    UINT8	nDrResetFlage;											//1:重置,0:默认
	UINT8 	nReserved1[2];
	
	INT32   nLowBeamSt;												//近光灯   0关 1开
	INT32   nHighBeamSt;											//远光灯 0关 1开
	INT32	nReverseLightSt;										//倒车灯 0关 1开
	INT32   nBrakeLightSt;											//刹车灯 0关 1开
	INT32	nFLDCM_MirrorFoldUnfoldSt;								//左后视镜 1折叠 2展开
	INT32   nFRDCM_MirrorFoldUnfoldSt;								//右后视镜 1折叠 2展开
	INT32   nParkingLampSt;										    //位置灯 0关 1开
	INT32   nDTRLSt;												//日行灯 0关 1开
	
	UINT8 	nBonnetSts;								// 引擎盖 0关闭 1打开
	UINT8 	nTrunkSts;								// 后备箱 0关闭 1打开
	UINT8 	nSRFSts;								// 天窗 0关闭 1打开
	UINT8 	nReserve;

	UINT8 	nReserved[256-4*30-4*9];	
}MvCanCarInfo;

typedef struct _MV_CAN_FUNCTION_INFO_256
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号
	INT32	nSysEnableCmd;											//报警系统使能开关
	INT32   nLdwSensitivity;	 									//LDW灵敏度 1为低       2为高   //20180519会议确定，应BYD需求添加
	INT32   nFcwWarn;           									//FCW报警   1有   0无
	INT32   nPreFcwWarn;        									//PREFCW报警   1有   0无	
	INT32   nLDWSwitch;												//LDW开关
	INT32   nPCWSwitch;												//PCW开关
	INT32   nIHBCSwitch;											//IHBC开关
	INT32   nSLISwitch;												//SLI开关
	INT32	nEmsCount; 												//EMS3循环计数    0正常  1丢包
	INT32	nAbsCount; 												//Abs循环计数       0正常  1丢包
	INT32	nEspCount; 												//Esp循环计数       0正常  1丢包
	INT32	nSasCount; 												//Sas循环计数          
	INT32	nIcuCount; 												//Icu循环计数
	INT32	nBcmCount; 												//Bcm循环计数
	INT32   nTargetGear;           									//期望档位
	FLOAT 	fTargetAlpha;											//期望转向角 
	FLOAT	fAVPRealtimeVehicleLocationX0;							//北汽 车辆实时位置 X0
	FLOAT	fAVPRealtimeVehicleLocationY0;							//北汽 车辆实时位置 Y0 
	UINT16	uAVPState1; 											//北汽 车辆AVP状态1（是否可以泊车/取车）
	UINT16	uAVPState2; 											//北汽 车辆AVP状态2（泊车/取车状态）
	UINT16	uAVPState3; 											//北汽 车辆AVP状态3（上下电状态）
	UINT16	uAVPState4; 											//北汽 车辆AVP状态4（过程控制状态）
	UINT8 	Reserved[256-4*22];	
}MvCanFuncInfo;

#if 0  //和GPU部分opengl头文件CAN结构体定义冲突
typedef struct _MV_CAN_INFO_512
{
	MvCanCarInfo	tCar;
	MvCanFuncInfo	tFunc;
}MvCanInfo;
#endif

typedef struct _MV_SYSTEM_INFO_128									//系统状态(自检)
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异	
	UINT32 	nFrameId;												//帧号
	INT8 	nFaultStatus;											//系统状态
	INT8    nVanishingPointStatus;									//灭点状态
	INT8	nCailFinishStatus;										//状态 1=in progress 0=error	2=success
	INT8 	nRes;
	INT32	nCailFinishTime;										//目前完成次数 0~2000
	INT8 	nReserved[128-4*4];
}MvSystemInfo;
//===========================================================================================================
//======================== Can 目标数据 =====================================================================
//===========================================================================================================
typedef struct _CanObjID_32											//转算法ID使用
{
	INT32 nIsObjId;													//用于记录CAN输出OBJID是否已放入目标ID
	INT32 nObjId;													//用于存储对应算法的ID(依次累加)
	INT32 nObjNo;													//用于输出对应算法的ID在Track队列中的序号
	INT32 nCanObjId;												//用于输出CANID对应的ID 0-63
	INT32 nReserved[4];												//保留字段
}CanObjID;

typedef struct _MvCanObjID_10k
{
	UINT32 		nStructLen;											//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 		nFrameId;											//帧号
	INT32		nstartID;											//用于记录从那里开始查找
	CanObjID 	tCanObjID[256];										//最多对应256个目标ID
	INT8 		chReserved[10240-4*3-32*256];
}MvArmCanObjID;   														//总长度8256;
//===========================================================================================================
//======================== 报警事件数据  ====================================================================
//===========================================================================================================
typedef struct _MainObjInfo_32_										//主目标信息，只保留了ID等基本信息，其他信息都根据ID去跟踪结构体取
{
	INT32 nMainObjID; 												//主目标ID(算法数组的ID)
	INT32 nType; 													//主目标类型
	INT32 EventType;												//报警事件类型
	INT32 nDangerLevel;												//危险等级
	INT32 nRes[4];
}ArmMainObjInfo;

typedef struct _ARM_MAIN_OBJ_INFO_
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 nFrameId;												//视频帧序号
	UINT32 nMainObjNum;												//主目标个数，预留给以后存在多个主目标的场景
	ArmMainObjInfo tMainObjInfo[6];									//主目标属性
	INT8	nRes[256 - 4*3 - 32*6];									//保留	
}MvArmMainObjInfo;

typedef struct _MvWarnInfo_											//watch
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号

	INT8 chAdasSwitch;												//系统总开关 0=关闭告警系统失效 1= 打开 正常工作
	INT8 chRes[3];
	//LDW
	INT8 chLdwSwitch;               								//LDW告警开关 0 =关闭 1 =打开 正常工作
	INT8 chLeftLaneFlag;											//左车道线是否检测到标志 0 未检测到左侧车道线 1 检测到
	INT8 chRightLaneFlag;											//右车道线是否检测到标志 0 未检测到右侧车道线 1 检测到
	INT8 chLdwWarn;               									//车道偏离告警 0=无告警  1= 左偏离  2= 右偏离
	INT8 chDepartureDir;        									//偏离方向 1向右 0 向左 2 未偏离
	INT8 chLdwBuzzingFlag;											//LDW蜂鸣器声音报警标志
	INT8 chLdwMiddleWarn;               							//车道中线偏离告警 0=无告警  1= 左中线偏离  2= 右中线偏离
	INT8 chResLdw[5];
	//FCW
	INT8 chFcwSwitch;                								//FCW告警开关 0 =关闭 1 =打开 正常工作
	INT8 chFcwWarn;                									//前车碰撞告警 0=未检测到车辆 1=无碰撞告警 2=前车碰撞预告警  3 =前车碰撞告警
	INT8 chFcwBuzzingFlag;											//FCW蜂鸣器声音报警标志
	INT8 chResFcw[5];
	//PCW
	INT8 chPcwSwitch;                								//PCW告警开关 0 =关闭 1=打开 正常工作
	INT8 chPcwWarn;                									//行人碰撞告警  0=未识别到行人 1=安全区域行人 2=行人碰撞预告警  3=行人碰撞告警
	INT8 chPcwBuzzingFlag;											//PCW蜂鸣器声音报警标志
	INT8 chResPcw[5];
	//HMW
	INT8 chHmwSwitch;            									//HMW告警开关 0= 关闭 1= 打开 正常工作
	INT8 chHmwWarn;            										//车距检测告警 0= 未识别到车辆 1= 安全车距  2= 危险车距
	INT8 chHmwBuzzingFlag;											//HMW蜂鸣器声音报警标志
	INT8 chResHmw;
	FLOAT fHmwTTC;                									//车距检测距离 0.1--2.5s 用时间表示 0.0为无效值
	//Camera相关信息
	INT8 chCameraCover;												//摄像头是否被遮挡 0=未被遮挡 1=已被遮挡
	INT8 chResCamera[7];
	//Malfunction故障
	INT8 chMalfunctionStatus;										//故障检测   1=正常 0=故障
	INT8 chResMal[7];
	//uFCW
	INT8 chUFcwSwitch;                								//uFcw告警开关 0 =关闭 1=打开 正常工作
	INT8 chResUFcw[7];
	//SLI
	INT8 chSLISwitch;                								//SLI告警开关 0 =关闭 1=打开 正常工作
	INT8 chResSLI[7];
	//IHC
	INT8 chIHCSwitch;                								//IHC告警开关 0 =关闭 1=打开 正常工作
	INT8 chResIHC[7];
	//AEB
	INT8	chAebSwitch;											//AEB告警开关 0 =关闭 1=打开 正常工作
	INT8	chPreBrake;
	INT8	chFullBrake;
	INT8	chResAEB[5];
	//BSD
	INT8 chBSDSwitch;                								//BSD告警开关 0 =关闭 1=打开 
	INT8 chBSDSpeed;                								//BSD触发速度,大于触发速度开启BSD功能.
	INT8 chBSDWarn;                									//BSD告警  0=无告警 1=左边 2=右边
	INT8 chBSDWarnLevel;											//BSD告警等级  0=无告警 1=1级告警 2=2级告警
	INT8 chResBsd[4];
	//MOD
	INT8 chMODSwitch;                								//MOD告警开关 0 =关闭 1=打开 
	INT8 chMODSpeed;                								//MOD触发速度,小于触发速度开启MOD功能.
	INT8 chMODWarn;                									//MOD告警  0=无告警 3=前边 4=后边
	INT8 chResMOD[5];
	//DOW
	INT8 chDOWSwitch;                								//DOW告警开关 0 =关闭 1=打开 
	INT8 chDOWWarn;                									//DOW告警  0=无告警 1=左边 2=右边
	INT8 chDOWWarnLevel;											//DOW告警等级  0=无告警 1=1级告警 2=2级告警
	INT8 chResDow[5];	
	//PD
	INT8 chPDSwitch;                								//PD 告警开关 0 =关闭 1=打开 正常工作
	INT8 chResPD[7];	
	//LDW
	INT8 chLDW2Switch;                								//环视LDW 告警开关 0 =关闭 1=打开 正常工作
	INT8 chLDW2Res[7];
	INT8 chReserved[256-4*24-5*8];									//预留 用于后续扩展
}MvWarnInfo;
//===========================================================================================================
//======================== IMU数据  =========================================================================
//===========================================================================================================
typedef struct _IMU_data_64
{
	UINT64 		Time; 												//时间戳、毫秒
	FLOAT	 	AccX;												//X方向加速度(m/s2)(原始数据，不经过任何算法，直接从传感器读出来的值)
	FLOAT	 	AccY;												//Y方向加速度(m/s2)
	FLOAT	 	AccZ;												//Z方向加速度(m/s2)
	FLOAT		GyroX; 												//X方向角速度(度/s)
	FLOAT		GyroY; 												//Y方向角速度(度/s)
	FLOAT		GyroZ; 												//Z方向角速度(度/s)
	FLOAT		MagX;												//X方向磁强(uT)
	FLOAT		MagY;												//Y方向磁强(uT)
	FLOAT		MagZ;												//Z方向磁强(uT)
	FLOAT		Yaw;												//偏航角(度)
	FLOAT		Pitch; 												//俯仰角(度)
	FLOAT		Roll;												//横滚角(度)
	FLOAT		Temperature;										//温度
	UINT8 Reserved[4];												//预留位
}ImuData;

typedef struct  _IMU_Data_Pack_1k5
{
	UINT32 		nStructLen;											//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 		nFrameId;											//帧号
	UINT32 		ImuDataNum;											//80s的数据包中有多少组有效数据，最大16组
	ImuData		tImuData[16];
	UINT8 chReserved[1536-4*3-64*16];
}MvImuDataPack;

//===========================================================================================================
//======================== 光流数据  ========================================================================
//===========================================================================================================
typedef struct _MV_OPTICAL_FLOW_INFO_8k_
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号
	INT32 	nCurDet;												//804
	UINT16 	curDetX[200];											//算法定义的光流点保存
	UINT16 	curDetY[200];											//算法定义的光流点保存
	UINT32 	nPreFlow;												//1028
	UINT16 	preFlowX[256];											//算法定义的光流点保存
	UINT16 	preFlowY[256];											//算法定义的光流点保存
	UINT32 	nCurFlow;												//1028
	UINT16 	curFlowX[256];											//算法定义的光流点保存
	UINT16 	curFlowY[256];											//算法定义的光流点保存
	UINT8 chReserved[8192-4*5-2*1424];	
}MvOpticalFlowInfo;

//===========================================================================================================
//======================== 车位数据  ========================================================================
//===========================================================================================================
typedef struct _MvSlotPoint			
{
	ArmPointInt32 tImagePoint;										//车位图像坐标
	ArmPointFloat tWorldPoint;										//车位世界坐标
} MvSlotPoint;

typedef struct _MvSlotData											//车位信息
{
	MvSlotPoint  	tPoint0;										//车位点1坐标（图像+世界）
	MvSlotPoint  	tPoint1;										//车位点2坐标（图像+世界）
	MvSlotPoint  	tPoint2;										//车位点3坐标（图像+世界）
	MvSlotPoint  	tPoint3;										//车位点4坐标（图像+世界）
	MvSlotPoint		tGroundPoint0;									//第一个挡轮杆外侧顶点
	MvSlotPoint		tGroundPoint1;									//第二个挡轮杆外侧顶点
	UINT32			nSlotId;										//车位ID
	UINT32			nDirection;										//车位方向 0:左 1:右 2:前 3:后			
	UINT32 			nType;											//车位类型
	UINT32 			nAvailableState;								//可用状态（占用情况）：1=可用 0=不可用	
	FLOAT			fAngle;											//与x轴正方向夹角，车位航向角
	FLOAT			fSlotTheta;										//置信度
	UINT32  		nParkSlot34ptFlag;								//车位34点有效性,1有效,0无效
	UINT32  		tGroundPinFlag;									//档轮杆有效性,1可信,0无效信
	ArmPointFloat 	tIconPoint[4];									//车位图标的图像坐标
	float 			fBestSlotScore;									//最佳位置得分, 二次更新使用
	UINT8			aReserved[192 - 6*16 - 4*8-4*8-4]; 
} MvSlotData;

#if 0
typedef struct _SlotData											//车位信息
{
	MvSlotPoint  	tPoint0;										//车位点1坐标（图像+世界）
	MvSlotPoint  	tPoint1;										//车位点2坐标（图像+世界）
	MvSlotPoint  	tPoint2;										//车位点3坐标（图像+世界）
	MvSlotPoint  	tPoint3;										//车位点4坐标（图像+世界）
	MvSlotPoint		tGroundPoint0;									//第一个挡轮杆外侧顶点
	MvSlotPoint		tGroundPoint1;									//第二个挡轮杆外侧顶点
	UINT32			nSlotId;										//车位ID
	UINT32			nDirection;										//车位方向 0:左 1:右 2:前 3:后			
	UINT32 			nType;											//车位类型//0水平,1垂直,2斜		|0 无效车位,|0x10 最优车位,|0x20 备选环视车位,|0x30 不可用车位,|0x40 超声车位,|0x50 自选车位
	UINT32 			nAvailableState;								//可用状态：1=可用 0=不可用	
	FLOAT			fAngle;											//与x轴正方向夹角，车位航向角
	FLOAT			fSlotTheta;										//最优车位置信度, 靠近1最优
	UINT32  		nParkSlot34ptFlag;								//车位34点有效性,1有效,0无效
	UINT32  		tGroundPinFlag;									//档轮杆有效性,1可信,0无效信
	ArmPointFloat 	tIconPoint[4];									//车位图标的图像坐标
	UINT8			aReserved[192 - 6*16 - 4*8-4*8]; 
} SlotData;
#endif


typedef struct _MvSlotOutput
{
	UINT32 		nStructLen;											//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32      nFrameId;											//帧号
	UINT64		lTimeMsec;											//同步用时间戳
	UINT16 		nCarX0;												//拼接图中小车左上角的X坐标 	
	UINT16 		nCarY0;												//拼接图中小车左上角的Y坐标 
	UINT16 		nCarX1;												//拼接图中小车右下角的X坐标
	UINT16 		nCarY1;												//拼接图中小车右下角的Y坐标
	UINT32 		nDetectSlotNum;										//检测车位个数
    UINT32 		nTrackSlotNum;										//跟踪车位个数
    UINT32 		nParkSlotNum;										//跟踪车位			
    UINT32      nManageSlotNum;										//管理车位个数
	MvSlotData  tDetectSlot[16];									//检测车位信息
	MvSlotData  tTrackSlot[16];										//跟踪车位信息
	MvSlotData  tParkSlot[1];										//泊车车位信息
	MvSlotData  tManageSlot[20];									//管理车位信息
  UINT64		lVisionSlotSendMs;
	UINT8 		chRerseved[10240 - 4*10 -  192*16 -192*16 -192*1 - 192*20 - 8];
} MvSlotOutput;

typedef struct _MvAroundViewWarnStruct_								//环视报警信息
{
	UINT32 nObjID;    												//目标ID
	UINT32 nObjType;  												//1:左边 2:右边  3:前 4:后
	ArmPointInt32	tPoint1;    									//点0坐标
	ArmPointInt32	tPoint2;										//点1坐标
	ArmPointInt32	tPoint3;										//点2坐标   
	ArmPointInt32	tPoint4;										//点3坐标   
	UINT8 chRerseved[64 - 8 - 8*4];    
} MvAroundViewWarnStruct;

typedef struct _MvAroundViewWarnInfo_								//环视报警输出
{
	UINT64	lTimeMsec;												//时间戳ms
	UINT32  nFrameId;												//帧号
	UINT32  nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32	nRes;													//8字节对齐
	UINT32 nBSDTargetNum;											//BSD目标个数
	UINT32 nMODTargetNum; 											//MOD目标个数
	UINT32 nDOWTargetNum; 											//DOW目标个数
	MvAroundViewWarnStruct 	 tBSDInfo[16];							//BSD目标属性	
	MvAroundViewWarnStruct	 tMODInfo[16];							//MOD目标属性
	MvAroundViewWarnStruct	 tDOWInfo[16];							//DOW目标属性
	UINT8 chRerseved[5120 - 4*8  - 64*16*3]; 
} MvAroundViewWarnInfo;

//===========================================================================================================
//======================== MCU交互数据 (512字节)  ===========================================================
//===========================================================================================================



//===========================================================================================================
//======================== 私有数据摘要  ====================================================================
//===========================================================================================================
typedef struct _MV_PRIVATE_DATA_SUMMARY_64
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32  nFrameId;												//帧号
	UINT32	nPriDataEnum;											//用于标识私有数据的发送的结构体，从ENUM中取值(PrivateDataType)
	UINT32 	nPriDataLen;											//私有数据的总大小
	UINT32 	nPriDataNum;											//私有数据的结构体个数
	UINT8 	chReserved[64-4*5];
}MvPrivateDataSummary;

//===========================================================================================================
//======================== Seg分割数据 ======================================================================
//===========================================================================================================
typedef struct MvArmEncodePoint
{
	INT16 nStart;													//Seg编码点信息
	INT16 nMarky;													//Seg编码点信息
	INT16 nLenght;													//Seg编码点信息
}MvArmEncodePoint;

typedef struct Seg_Info_Every_Channel_info_40
{
	UINT16 	sChannelEncodeNum;										//32个通道每个通道编码的点数
	INT8 	sChannelName[32];										//32个通道,每个通道的名称
	UINT8 	sChannelKey; 											//32个通道,每个通道的定义(需要维护一张对应表 KEY：名称)	类似：(1:curb)(2:lane)(3:freespace)...eg:sChannalKey[0]=2，表示通道0是lane
	UINT8 	chReserved[40-2-32-1];
}SegChannelInfo;

typedef struct Seg_Summary_Info_1k5
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//帧号
	UINT8	nDeviceNo;												//当前是第几路视频，在DeviceNoEnum里选
	UINT8	nReserved;													
	UINT16	nSegWidth;												//seg宽
	UINT16	nSegHeight; 											//seg高
	UINT16	nSegChannalNum; 										//每路视频里的seg通道数,暂定最多32路
	SegChannelInfo  tSegChannelInfo[32];							//每一路seg通道的信息
	UINT32  nSegDataLen;											//Seg数据总长度
	INT8 cReserved[1536-4*2-2*4-40*32-4];
}MvArmSegSummaryInfo;
//===========================================================================================================
//======================== DUMP数据摘要  ====================================================================
//===========================================================================================================
typedef struct _MV_DUMP_DATA_SUMMARY_64
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//dump数据帧号
	INT8 	chDumpDataName[32];										//dump数据名称
	UINT32 	nDumpdataLen;											//dump数据长度
	UINT8 	chReserved[64-4*2-32-4];
}MvDumpDataSummary;


//===========================================================================================================
//======================== 栅格数据摘要  ====================================================================
//===========================================================================================================
#if 0																//栅格定义，当前第一版
typedef enum MvClassType
{
	MV_NO_FREESPACE,
	MV_FREESPACE = 1,
	Mv_LIMIT_SPEED = 2,
	MV_GROUNDPIN = 3, 				//档栏杆
	MV_LANEMARK = 4, 				//车道线
	MV_PARKINGLINE = 5,				//车位线
	MV_CURB = 10, 					//路沿
	MV_OBSTACLE = 11, 				//一般障碍物
	MV_PEOPLE = 12, 				//行人
	MV_NOMOTOR = 13, 				//非机动车二轮车
	MV_MOTOR = 14, 					//机动车
	MV_PILLAR  = 15, 				//柱子
	MV_GROUNDLOCK = 16, 			//地锁开
	
	NAtype  = 255                 //255：默认标签，初始化时设置为此标签   
}MvClassType;
#endif

typedef struct _MV_GRID_DATA_SUMMARY_64
{
	UINT32 	nStructLen;												//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32 	nFrameId;												//dump数据帧号
	UINT32  nLength;												//栅格长多少点
	UINT32  nWidth;													//栅格宽多少点
	FLOAT	fLengthPerGrid;											//单个栅格的真实长度，单位m
	FLOAT	fWidthPerGrid;											//单个栅格的真实宽度，单位m
	UINT8	nClassVer;												//栅格定义版本，对应MvClassType的版本
	UINT8 	chRes[64-4*6-1];
}MvGridDataSummary;
//===========================================================================================================
//头部结构体，用来记录哪些结构体存在哪些不存在
//x:存在,且版本号为x		   0:不存在
//===========================================================================================================

typedef struct _DATA_EXIST_
{
	UINT16	chDataEnum;											//指明是哪种数据,对应NetSendIndexModeAppE的enum
	UINT8	chVersion;											//数据填写版本号，图像对应ModeAppE的enum
	UINT8	chRes[1];											
}DataExist;


#define MAX_SEND_DATASEG (32)
typedef struct Net_Send_to_MvPlayer_Content_Info_512_
{
	//类型一：全局配置
	UINT32  	nHeadMark;										//0xDCBAABCD
	UINT32      nStructVersion;                                 //此结构体"NetSendContentInfo"的版本
	UINT32 		nStructLen;										//本结构体的大小，用于上位机校验是否有兼容性问题的平台差异
	UINT32  	nVideoMode;										//发送的图像类型，可选值可用ModeAppE中的enum值。
	UINT32  	nSubVideoMode;									//组合模式，用于多核分别发送数据，到平台软件上进行组合使用
	UINT32  	nVdmaFrameId;									//本次网络发送的图像帧号
	UINT64		lVdmaFrameTime;									//图像时间戳
	UINT32  	nMdmaFrameId;									//本次网络发送的数据帧号
	UINT8 		chExistData;									//本帧是否有数据
	UINT8 		chReserved0[3];
	UINT32 		nNetDataLen;									//网络发送 的 content+图像+定长数据(包括私有数据)+SEG数据+DUMP数据 的 总长度	(不包含32字节头)
	UINT32      nVideoLen;										//图像数据的长度
	UINT32  	nFixTotalLen;									//定长数据(包括私有数据)的长度	
	UINT32  	nVariableTotalLen;								//变长SEG数据长度
	UINT32   	nDumpTotalLen;									//变长DUMP数据长度
	UINT32		nDataNum;										//描述tDataExist字段的有效数量
	//类型二：数据存在标志
	DataExist	tDataExist[MAX_SEND_DATASEG];									//数据的顺序，类型，版本说明

	UINT64		lFreespaceTime;
	UINT64		lOptFlowTime;
	UINT8		chReserved[512 - 4*16 - 4* MAX_SEND_DATASEG - 16];
}NetSendContentInfo;

#if 0
typedef struct
{
	float driving_veh_x;
	float driving_veh_y;
}DrivingVehicleLastPoseStruct;
#endif


#pragma pack(pop)


#endif

