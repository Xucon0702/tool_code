/*
* 版权所有 2015 魔视智能科技(上海)有限公司
* Copyright (c) 2015,Motovis Intelligent Technologies (Shanghai) Co.,Ltd
* 魔视智能智能公司秘密
* Motovis Intelligent Confidential Proprietary
* 文件名称：AlgCommonStructDef.h
* 摘要：算法公用结构体和宏定义
* 版本：v1.0.0.0
* 作者：於锋
* 完成日期：2019年3月8日
*/

#ifndef _ALG_COMMON_STRUCT_DEF_H_
#define _ALG_COMMON_STRUCT_DEF_H_

#ifndef WIN32
#ifdef __cplusplus
extern "C" {
#endif
#endif

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#ifndef _MV_INT64
#define _MV_INT64
typedef signed long long  Mv_int64;
//typedef	unsigned long long	uint64_t;

#endif

#ifndef _MV_BOOL_
typedef unsigned int MV_Bool;//bool/Bool,conflict!
#define _MV_BOOL_
#endif

#ifdef __cplusplus
#else
//typedef int bool;
#define true (1)
#define false (0)
#endif

#define	MAX_COUNT_OBJ_DETECT_PERFRAME			80
#ifdef PSP_PSROIPOOLING_NOTSR_1824_948
#define MAX_AD_HEIGHT							136//320//474
#define MAX_AD_WIDTH							448//576//912
#elif defined PSP_PSROIPOOLING_NOTSR_2048_768
#define MAX_AD_HEIGHT							384//320//474
#define MAX_AD_WIDTH							768//576//912
#elif defined PSP_PSROIPOOLING_1920_1080
#define MAX_AD_HEIGHT							540//320//474
#define MAX_AD_WIDTH							960//576//912
#else
#define MAX_AD_HEIGHT							360
#define MAX_AD_WIDTH							640
#endif
#define POINTNUMPERFREESPACE					64   //输出的freespace点数

#ifndef MV_MAX
#define MV_MAX(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef MV_MIN
#define MV_MIN(a,b)            (((a) < (b)) ? (a) : (b))
#endif

#define		MV_PI						3.1415926535897932384626433832795
#define		MV_CONST_RA2AN				57.295779513082320876798154814105
#define		MV_CONST_AN2RA				0.01745329251994329576923690768489

////////////////////////////////////////////////
////多相机使用
#ifdef MV_BUS_FUSION_PROJ
#define NEARDISTANCE (15000)
#define FARDISTANCE  (30000)
#else
#define NEARDISTANCE (30)
#define FARDISTANCE  (50)
#endif

typedef enum _CAMERA_ID
{
	MV_LEFT_CAMERA = 0,	
	MV_FRONT_CAMERA,// = 1,
	MV_RIGHT_CAMERA,// = 2,
	MV_LEFT_REAR_CAMERA,	// 3
	MV_RIGHT_REAR_CAMERA,  // 4
	MV_LEFT_BACK_CAMERA,	// 5
	MV_RIGHT_BACK_CAMERA,  // 6
	MV_BACK_CAMERA,  // 7
	MV_NOTYPE_CAMERA,  // 8
	MV_DMS_CAMERA, //9
	MV_FRONT_BSD_CAMERA, //10
	MV_BELT_CAMERA, //11
	MV_OVERBOARD_CAMERA //12
}MV_CAMERA_ID;
////////////////////////////////////////////

typedef enum MvClassType
{
	pedestrian = 0,
	car,//1
	bus,//2
	truck,//3
	midbus,//4
	zebra,
	bike,
	tricycle,
	MV_LANE_POINT,
	NTypeObj,										//不确定目标类型
	MV_TSR,                  						//10交通标志
	MV_IHC,											//11定制需求：车大灯
	MV_STOP_LINE,       							//12停止线
	MV_GENERAL_OBJ,									//13
	MV_CORNERS,										//14邊緣
	MV_WHEELS,										//15车轮
	roadmask,										//16路面标识
	obstacle,										//17障碍物
	curb,											//18路沿
	_truck_front,									//19
	_car_front,										//20							
	_minibus_front,									//21
	_bus_front,										//22	
	//add
	car_chece,
	bus_chece,
	truck_chece,
	midelbus_chece,
	mainpeople,
	mainbicycle,
	speedsign,										//add 限速
	trafficlight,									//add 交通灯
	MV_LIGHT_CAR = 50,                              //50
	_car_back = 51,							  	    //车头车尾区分，车尾。
	MV_POLE = 52,									//警示柱
	//fisheye add
	mv_vehicle = 53,								//车，鱼眼大类
	mv_warning_cone = 54,							//警示锥
	mv_lifting_gate = 55,							//路闸闸杆子
	mv_no_parking_sign = 56,						//禁停牌
	mv_plate = 57,									//车牌
	mv_guideline = 58,								//导向线
	mv_wheel_stops = 59,							//挡轮杆
	mv_ground_anchor = 60,							//地锁开
	mv_text = 61,									//文本
	mv_tuiche_nomotor = 62,							//推车婴儿车购物车等非机动车
	_bus_back = 63,
	_truck_back = 64,
	_midbus_back = 65,
	mv_ground_anchor_close = 66,					//地锁关
	mv_lifting_pillar = 67,							//路闸闸杆的柱子
	mv_turnningsign = 68,							//转向标识牌
	mv_exitsign = 69,								//出口标识牌
	mv_speedsign = 70,								//限速标识牌
	mv_othersign = 71,								//其他标识牌


	NAtype	= 80					         		//40：默认标签，初始化时设置为此标签
}MvClassType;

//新增分割类别
typedef enum MvClassType_Segment
{
	MV_seg_freespace = 0, 							//可行驶区域
	MV_seg_curb = 1,  								//路沿
	MV_seg_laneMark = 2,  							//车道线
	MV_seg_parkingLine = 3,  						//车位线
	MV_seg_decelerationHump = 4,  					//减速带
	MV_seg_fence = 5,  								//栏杆
	MV_seg_warningTape = 6,  						//警示带
	MV_seg_motor_car = 7,  							//小汽车
	MV_seg_motor_bus = 8,  							//公交车
	MV_seg_motor_truck = 9,  						//卡车
	MV_seg_nomotor_bicycle = 10, 					//自行车
	MV_seg_people = 11, 							//行人
	MV_seg_warningPole = 12,						//警示柱
	MV_seg_warningCone = 13,						//警示锥桶
	MV_seg_liftingPole = 14,						//道闸闸杆
	MV_seg_noParkingSign = 15,						//禁停标识牌
	MV_seg_obstacle = 16,							//其它障碍物
	MV_seg_wheel = 17,								//车轮
	MV_seg_plate = 18,								//车牌
	MV_seg_guideLine = 19,							//导向线
	MV_seg_groundPin = 20,							//挡轮杆
	MV_seg_groundLock = 21,							//地锁
	MV_seg_pillar = 22,								//柱子
	MV_seg_zebraCrossing = 23,						//斑马线
	MV_seg_waterHorseBarrier = 24,					//水马围栏
	MV_seg_fireHydrant = 25,						//消防栓
	MV_seg_nomotor = 26,							//非机动车,自行车已单独出去
	MV_seg_liftingPillar = 27,						//道闸闸柱
	MV_seg_noStopArea = 28,							//禁停区
	MV_seg_stopLine = 29,							//停止线
	MV_seg_slowArea = 30,							//减速区
	MV_seg_cornerCurb = 31,							//墙角沿               
														
														   		
	MV_seg_ignore_region= 80,						//训练使用：忽略区
	MV_seg_ignore_plate= 81							//训练使用：忽略牌	

}MvClassType_Segment;

typedef enum _MvTsrClassType
{
	///*1000+: 北汽c71&7020定制非完整版本,背景0+72类*/
#define ETSR_START_INDEX 1000
	ETSR_NAtype = ETSR_START_INDEX										+ 0,									//hardnegative,背景和其他类
	ETSR_GUIDE_SIGN_TURN_RIGHT = ETSR_START_INDEX						+ 1,													//指示标志 - 向右转弯
	ETSR_GUIDE_SIGN_TURN_LEFT_RIGHT = ETSR_START_INDEX					+ 2,													//指示标志 - 向左和向右转弯
	ETSR_GUIDE_SIGN_TURN_LEFT = ETSR_START_INDEX						+ 3,													//指示标志 - 向左转弯
	ETSR_GUIDE_SIGN_STRAIGHT = ETSR_START_INDEX							+ 4,													//指示标志 - 直行
	ETSR_GUIDE_SIGN_STRAIGHT_RIGHT = ETSR_START_INDEX					+ 5,													//指示标志 - 直行和向右转弯
	ETSR_GUIDE_SIGN_STRAIGHT_LEFT = ETSR_START_INDEX					+ 6,													//指示标志 - 直行和向左转弯
	ETSR_GUIDE_SIGN_TURN_AROUND = ETSR_START_INDEX						+ 7,													//指示标志 - 允许掉头
	ETSR_GUIDE_SIGN_KEEP_LEFT_RIGHT = ETSR_START_INDEX					+ 8,													//指示标志 - 靠左靠右行驶
	ETSR_GUIDE_SIGN_TURN_LEFT_AROUND = ETSR_START_INDEX					+ 9,													//指示标志 - 向左转弯和掉头
	ETSR_GUIDE_SIGN_ROUNDABOUT = ETSR_START_INDEX						+ 10,													//指示标志 - 环岛行驶
	ETSR_GUIDE_SIGN_KEEP_RIGHT_LANE = ETSR_START_INDEX					+ 11,													//指示标志 - 靠右侧道路行驶
	ETSR_GUIDE_SIGN_KEEP_LEFT_LANE = ETSR_START_INDEX					+ 12,													//指示标志 - 靠左侧道路行驶
	ETSR_PROHIBIT_NO_THROUGH = ETSR_START_INDEX							+ 13,													//禁令标志 - 禁止通行
	ETSR_PROHIBIT_MAX_SPEED_10KM = ETSR_START_INDEX						+ 14,									//禁令标志 - 限制速度10km / h，立牌红白圆圈
	ETSR_PROHIBIT_MAX_SPEED_100KM = ETSR_START_INDEX					+ 15,									//禁令标志 - 限制速度100km / h
	ETSR_PROHIBIT_MAX_SPEED_110KM = ETSR_START_INDEX					+ 16,									//禁令标志 - 限制速度110km / h
	ETSR_PROHIBIT_MAX_SPEED_120KM = ETSR_START_INDEX					+ 17,									//禁令标志 - 限制速度120km / h
	ETSR_PROHIBIT_MAX_SPEED_15KM = ETSR_START_INDEX						+ 18,									//禁令标志 - 限制速度15km / h
	ETSR_PROHIBIT_MAX_SPEED_20KM = ETSR_START_INDEX						+ 19,									//禁令标志 - 限制速度20km / h
	ETSR_PROHIBIT_MAX_SPEED_25KM = ETSR_START_INDEX						+ 20,									//禁令标志 - 限制速度25km / h
	ETSR_PROHIBIT_MAX_SPEED_30KM = ETSR_START_INDEX						+ 21,									//禁令标志 - 限制速度30km / h
	ETSR_PROHIBIT_MAX_SPEED_35KM = ETSR_START_INDEX						+ 22,									//禁令标志 - 限制速度35km / h
	ETSR_PROHIBIT_MAX_SPEED_40KM = ETSR_START_INDEX						+ 23,									//禁令标志 - 限制速度40km / h
	ETSR_PROHIBIT_MAX_SPEED_5KM = ETSR_START_INDEX						+ 24,									//禁令标志 - 限制速度5km / h
	ETSR_PROHIBIT_MAX_SPEED_50KM = ETSR_START_INDEX						+ 25,									//禁令标志 - 限制速度50km / h
	ETSR_PROHIBIT_MAX_SPEED_60KM = ETSR_START_INDEX						+ 26,									//禁令标志 - 限制速度60km / h
	ETSR_PROHIBIT_MAX_SPEED_70KM = ETSR_START_INDEX						+ 27,									//禁令标志 - 限制速度70km / h
	ETSR_PROHIBIT_MAX_SPEED_80KM = ETSR_START_INDEX						+ 28,									//禁令标志 - 限制速度80km / h
	ETSR_PROHIBIT_MAX_SPEED_90KM = ETSR_START_INDEX						+ 29,									//禁令标志 - 限制速度90km / h
	ETSR_EPROHIBIT_MAX_SPEED_100KM = ETSR_START_INDEX					+ 30,									//禁令电子标志 - 限制速度100km / h
	ETSR_EPROHIBIT_MAX_SPEED_40KM = ETSR_START_INDEX					+ 31,									//禁令电子标志 - 限制速度40km / h
	ETSR_EPROHIBIT_MAX_SPEED_5KM = ETSR_START_INDEX						+ 32,									//禁令电子标志 - 限制速度5km / h
	ETSR_EPROHIBIT_MAX_SPEED_50KM = ETSR_START_INDEX					+ 33,									//禁令电子标志 - 限制速度50km / h
	ETSR_EPROHIBIT_MAX_SPEED_60KM = ETSR_START_INDEX					+ 34,									//禁令电子标志 - 限制速度60km / h
	ETSR_EPROHIBIT_MAX_SPEED_70KM = ETSR_START_INDEX					+ 35,									//禁令电子标志 - 限制速度70km / h
	ETSR_EPROHIBIT_MAX_SPEED_80KM = ETSR_START_INDEX					+ 36,									//禁令电子标志 - 限制速度80km / h
	ETSR_PROHIBIT_PARK_ANY_TIME = ETSR_START_INDEX						+ 37,									//禁令标志 - 禁止车辆临时或长时停放
	ETSR_PROHIBIT_ENTRY = ETSR_START_INDEX								+ 38,													//禁令标志 - 禁止驶入
	ETSR_PROHIBIT_PARK_LONG_TIME = ETSR_START_INDEX						+ 39,													//禁令标志 - 禁止车辆长时停放
	ETSR_PROHIBIT_OTHERS = ETSR_START_INDEX								+ 40,													//禁令标志 - 其他
	ETSR_PROHIBIT_NO_LIMIT_SPEED_100KM = ETSR_START_INDEX				+ 41,									//禁令标志 - 解除限制速度100km / h，，立牌绿白圆圈
	ETSR_PROHIBIT_NO_LIMIT_SPEED_60KM = ETSR_START_INDEX				+ 42,									//禁令标志 - 解除限制速度60km / h
	ETSR_PROHIBIT_NO_LIMIT_SPEED_80KM = ETSR_START_INDEX				+ 43,									//禁令标志 - 解除限制速度80km / h
	ETSR_PROHIBIT_PARK = ETSR_START_INDEX								+ 44,													//禁令标志 - 禁止停车
	ETSR_ROAD_MAX_SPEED_35KM = ETSR_START_INDEX							+ 45,													//限速35，路面刷白圆圈
	ETSR_ROAD_MAX_SPEED_5KM = ETSR_START_INDEX							+ 46,													//限速5
	ETSR_LIGHT_GREEN_ARROW = ETSR_START_INDEX							+ 150,													//指示标志 - 箭头绿色交通灯
	ETSR_LIGHT_GREEN_CIRCLE = ETSR_START_INDEX							+ 151,													//指示标志 - 圆形绿色交通灯
	ETSR_LIGHT_RED_ARROW = ETSR_START_INDEX								+ 152,													//指示标志 - 箭头红色交通灯
	ETSR_LIGHT_RED_CIRCLE = ETSR_START_INDEX							+ 153,													//指示标志 - 圆形红色交通灯
	ETSR_LIGHT_YELLOW_ARROW = ETSR_START_INDEX							+ 154,													//指示标志 - 箭头黄色交通灯
	ETSR_LIGHT_YELLOW_CIRCLE = ETSR_START_INDEX							+ 155,													//指示标志 - 圆形黄色交通灯
	ETSR_GUIDE_LINE_TURN_LEFT = ETSR_START_INDEX						+ 200,													//导向线 - 左转
	ETSR_GUIDE_LINE_STRAIGHT = ETSR_START_INDEX							+ 201,													//导向线 - 直行
	ETSR_GUIDE_LINE_TURN_RIGHT = ETSR_START_INDEX						+ 202,													//导向线 - 右转
	ETSR_GUIDE_LINE_LEFT_STRAIGHT = ETSR_START_INDEX					+ 203,													//导向线 - 左转直行
	ETSR_GUIDE_LINE_RIGHT_STRAIGHT = ETSR_START_INDEX					+ 204,													//导向线 - 右转直行
	ETSR_GUIDE_LINE_TURN_LEFT_RIGHT = ETSR_START_INDEX					+ 205,													//导向线 - 左转右转
	ETSR_GUIDE_LINE_LEFT_RIGHT_STRAIGHT = ETSR_START_INDEX				+ 206,													//导向线 - 左右转直行
	ETSR_GUIDE_LINE_TURN_AROUND = ETSR_START_INDEX						+ 207,													//导向线 - 掉头
	ETSR_GUIDE_LINE_NO_TURN_AROUND = ETSR_START_INDEX					+ 208,													//导向线 - 禁止掉头
	ETSR_GUIDE_LINE_TURN_LEFT_AROUND = ETSR_START_INDEX					+ 209,													//导向线 - 左转掉头
	ETSR_GUIDE_LINE_STRAIGHT_TURN_AROUND = ETSR_START_INDEX				+ 210,													//导向线 - 直行掉头
	ETSR_ROAD_DIAMOND_SQUARE = ETSR_START_INDEX							+ 211,													//菱形标志
	ETSR_ROAD_TRIANGLE_INVERTED = ETSR_START_INDEX						+ 212,													//倒三角
	ETSR_ROAD_CIRCLE_IN_CROSS = ETSR_START_INDEX						+ 213,													//中心圈
	ETSR_ROAD_MAX_SPEED_20KM = ETSR_START_INDEX							+ 214,													//限速20
	ETSR_ROAD_MAX_SPEED_40KM = ETSR_START_INDEX							+ 215,													//限速40
	ETSR_ROAD_MAX_SPEED_60KM = ETSR_START_INDEX							+ 216,													//限速60
	ETSR_ROAD_MAX_SPEED_70KM = ETSR_START_INDEX							+ 217,													//限速70
	ETSR_ROAD_MAX_SPEED_80KM = ETSR_START_INDEX							+ 218,													//限速80
	ETSR_ROAD_MAX_SPEED_100KM = ETSR_START_INDEX						+ 219,													//限速100

	///*2000+: TSR扩展完整类别版本:0背景+153类,枚举前缀加EXT_XXX !!!*/
	 
#define EXT_ETSR_START_INDEX 2000		
#ifdef MIT_TSR_MORE_241		//柳汽新增至241类，且顺序变化。
	EXT_ETSR_NAtype = EXT_ETSR_START_INDEX + 0,			//hardnegative,背景和其他类	
	EXT_ETSR_i10 = EXT_ETSR_START_INDEX + 1,   //指示标志-向右转弯
	EXT_ETSR_i11 = EXT_ETSR_START_INDEX + 2,   //指示标志-向左和向右转弯
	EXT_ETSR_i12 = EXT_ETSR_START_INDEX + 3,   //指示标志-向左转弯
	EXT_ETSR_i13 = EXT_ETSR_START_INDEX + 4,   //指示标志-直行
	EXT_ETSR_i14 = EXT_ETSR_START_INDEX + 5,   //指示标志-直行和向右转弯
	EXT_ETSR_i15 = EXT_ETSR_START_INDEX + 6,   //指示标志-直行和向左转弯
	EXT_ETSR_i16 = EXT_ETSR_START_INDEX + 7,   //指示标志-允许掉头
	EXT_ETSR_i17 = EXT_ETSR_START_INDEX + 8,   //指示标志-靠左靠右行驶
	EXT_ETSR_i18 = EXT_ETSR_START_INDEX + 9,   //指示标志-向左转弯和掉头
	EXT_ETSR_i19 = EXT_ETSR_START_INDEX + 10,   //指示标志-停车
	EXT_ETSR_i3 = EXT_ETSR_START_INDEX + 11,   //指示标志-环岛行驶
	EXT_ETSR_i5 = EXT_ETSR_START_INDEX + 12,   //指示标志-靠右侧道路行驶
	EXT_ETSR_i5b = EXT_ETSR_START_INDEX + 13,   //指示标志-非机动车靠右侧行驶
	EXT_ETSR_i5c = EXT_ETSR_START_INDEX + 14,   //指示标志-机动车靠右侧行驶
	EXT_ETSR_i6 = EXT_ETSR_START_INDEX + 15,   //指示标志-靠左侧道路行驶
	EXT_ETSR_i6c = EXT_ETSR_START_INDEX + 16,   //指示标志-机动车靠左侧行驶
	EXT_ETSR_il100 = EXT_ETSR_START_INDEX + 17,   //指示标志-最低限速100
	EXT_ETSR_il110 = EXT_ETSR_START_INDEX + 18,   //指示标志-最低限速110
	EXT_ETSR_il40 = EXT_ETSR_START_INDEX + 19,   //指示标志-最低限速40
	EXT_ETSR_il50 = EXT_ETSR_START_INDEX + 20,   //指示标志-最低限速50
	EXT_ETSR_il60 = EXT_ETSR_START_INDEX + 21,   //指示标志-最低限速60
	EXT_ETSR_il70 = EXT_ETSR_START_INDEX + 22,   //指示标志-最低限速70
	EXT_ETSR_il80 = EXT_ETSR_START_INDEX + 23,   //指示标志-最低限速80
	EXT_ETSR_il90 = EXT_ETSR_START_INDEX + 24,   //指示标志-最低限速90
	EXT_ETSR_io = EXT_ETSR_START_INDEX + 25,   //指示标志-其他
	EXT_ETSR_ip = EXT_ETSR_START_INDEX + 26,   //指示标志-人行横道线
	EXT_ETSR_ir1 = EXT_ETSR_START_INDEX + 27,   //指示标志-机动车车道
	EXT_ETSR_ir2 = EXT_ETSR_START_INDEX + 28,   //指示标志-公交线路专用车道
	EXT_ETSR_ir3 = EXT_ETSR_START_INDEX + 29,   //指示标志-自行车道
	EXT_ETSR_lg = EXT_ETSR_START_INDEX + 30,   //指示标志-绿色交通灯
	EXT_ETSR_lr = EXT_ETSR_START_INDEX + 31,   //指示标志-红色交通灯
	EXT_ETSR_ly = EXT_ETSR_START_INDEX + 32,   //指示标志-黄色交通灯
	EXT_ETSR_p1 = EXT_ETSR_START_INDEX + 33,   //禁令标志-禁止超车
	EXT_ETSR_p10 = EXT_ETSR_START_INDEX + 34,   //禁令标志-禁止机动车通行
	EXT_ETSR_p11 = EXT_ETSR_START_INDEX + 35,   //禁令标志-禁止鸣喇叭
	EXT_ETSR_p12 = EXT_ETSR_START_INDEX + 36,   //禁令标志-禁止二轮摩托车通行
	EXT_ETSR_p13 = EXT_ETSR_START_INDEX + 37,   //禁令标志-禁止某两种车通行
	EXT_ETSR_p14 = EXT_ETSR_START_INDEX + 38,   //禁令标志-禁止直行
	EXT_ETSR_p15 = EXT_ETSR_START_INDEX + 39,   //禁令标志-禁止人力车通行
	EXT_ETSR_p16 = EXT_ETSR_START_INDEX + 40,   //禁令标志-禁止人力货运三轮车通行
	EXT_ETSR_p17 = EXT_ETSR_START_INDEX + 41,   //禁令标志-禁止人力客运三轮车通行
	EXT_ETSR_p18 = EXT_ETSR_START_INDEX + 42,   //禁令标志-禁止拖拉机通行
	EXT_ETSR_p19 = EXT_ETSR_START_INDEX + 43,   //禁令标志-禁止向右转弯
	EXT_ETSR_p2 = EXT_ETSR_START_INDEX + 44,   //禁令标志-禁止畜力车进入
	EXT_ETSR_p20 = EXT_ETSR_START_INDEX + 45,   //禁令标志-禁止向左向右转弯
	EXT_ETSR_p21 = EXT_ETSR_START_INDEX + 46,   //禁令标志-禁止直行和向右转弯
	EXT_ETSR_p22 = EXT_ETSR_START_INDEX + 47,   //禁令标志-禁止三轮机动车通行
	EXT_ETSR_p23 = EXT_ETSR_START_INDEX + 48,   //禁令标志-禁止向左转弯
	EXT_ETSR_p24 = EXT_ETSR_START_INDEX + 49,   //禁令标志-禁止向右转弯(客车)
	EXT_ETSR_p25 = EXT_ETSR_START_INDEX + 50,   //禁令标志-禁止小型客车驶入标志
	EXT_ETSR_p26 = EXT_ETSR_START_INDEX + 51,   //禁令标志-禁止载货汽车通行
	EXT_ETSR_p27 = EXT_ETSR_START_INDEX + 52,   //禁令标志-禁止运输危险物品车辆驶入
	EXT_ETSR_p28 = EXT_ETSR_START_INDEX + 53,   //禁令标志-禁止直行和向左转弯
	EXT_ETSR_p29 = EXT_ETSR_START_INDEX + 54,   //禁令标志-禁止自行车和行人通行
	EXT_ETSR_p3 = EXT_ETSR_START_INDEX + 55,   //禁令标志-禁止大型客车驶入
	EXT_ETSR_p30 = EXT_ETSR_START_INDEX + 56,   //禁令标志-禁止两轮摩托车和自行车通行
	EXT_ETSR_p4 = EXT_ETSR_START_INDEX + 57,   //禁令标志-禁止电动三轮车驶入
	EXT_ETSR_p5 = EXT_ETSR_START_INDEX + 58,   //禁令标志-禁止掉头
	EXT_ETSR_p6 = EXT_ETSR_START_INDEX + 59,   //禁令标志-禁止非机动车通行
	EXT_ETSR_p8 = EXT_ETSR_START_INDEX + 60,   //禁令标志-禁止汽车拖、挂车通行
	EXT_ETSR_p9 = EXT_ETSR_START_INDEX + 61,   //禁令标志-禁止行人通行
	EXT_ETSR_pa10 = EXT_ETSR_START_INDEX + 62,   //禁令标志-限制轴重10t
	EXT_ETSR_pa12 = EXT_ETSR_START_INDEX + 63,   //禁令标志-限制轴重12t
	EXT_ETSR_pa13 = EXT_ETSR_START_INDEX + 64,   //禁令标志-限制轴重13t
	EXT_ETSR_pa14 = EXT_ETSR_START_INDEX + 65,   //禁令标志-限制轴重14t
	EXT_ETSR_pa8 = EXT_ETSR_START_INDEX + 66,   //禁令标志-限制轴重8t
	EXT_ETSR_pb = EXT_ETSR_START_INDEX + 67,   //禁令标志-禁止通行
	EXT_ETSR_pc = EXT_ETSR_START_INDEX + 68,   //禁令标志-停车检查标志
	EXT_ETSR_pg = EXT_ETSR_START_INDEX + 69,   //禁令标志-减速让行
	EXT_ETSR_ph1_5 = EXT_ETSR_START_INDEX + 70,   //禁令标志-限制高度1.5m
	EXT_ETSR_ph1_6 = EXT_ETSR_START_INDEX + 71,   //禁令标志-限制高度1.6m
	EXT_ETSR_ph1_7 = EXT_ETSR_START_INDEX + 72,   //禁令标志-限制高度1.7m
	EXT_ETSR_ph1_8 = EXT_ETSR_START_INDEX + 73,   //禁令标志-限制高度1.8m
	EXT_ETSR_ph1_9 = EXT_ETSR_START_INDEX + 74,   //禁令标志-限制高度1.9m
	EXT_ETSR_ph2 = EXT_ETSR_START_INDEX + 75,   //禁令标志-限制高度2m
	EXT_ETSR_ph2_1 = EXT_ETSR_START_INDEX + 76,   //禁令标志-限制高度2.1m
	EXT_ETSR_ph2_2 = EXT_ETSR_START_INDEX + 77,   //禁令标志-限制高度2.2m
	EXT_ETSR_ph2_3 = EXT_ETSR_START_INDEX + 78,   //禁令标志-限制高度2.3m
	EXT_ETSR_ph2_4 = EXT_ETSR_START_INDEX + 79,   //禁令标志-限制高度2.4m
	EXT_ETSR_ph2_5 = EXT_ETSR_START_INDEX + 80,   //禁令标志-限制高度2.5m
	EXT_ETSR_ph2_6 = EXT_ETSR_START_INDEX + 81,   //禁令标志-限制高度2.6m
	EXT_ETSR_ph2_7 = EXT_ETSR_START_INDEX + 82,   //禁令标志-限制高度2.7m
	EXT_ETSR_ph2_8 = EXT_ETSR_START_INDEX + 83,   //禁令标志-限制高度2.8m
	EXT_ETSR_ph2_9 = EXT_ETSR_START_INDEX + 84,   //禁令标志-限制高度2.9m
	EXT_ETSR_ph3 = EXT_ETSR_START_INDEX + 85,   //禁令标志-限制高度3m
	EXT_ETSR_ph3_1 = EXT_ETSR_START_INDEX + 86,   //禁令标志-限制高度3.1m
	EXT_ETSR_ph3_2 = EXT_ETSR_START_INDEX + 87,   //禁令标志-限制高度3.2m
	EXT_ETSR_ph3_3 = EXT_ETSR_START_INDEX + 88,   //禁令标志-限制高度3.3m
	EXT_ETSR_ph3_4 = EXT_ETSR_START_INDEX + 89,   //禁令标志-限制高度3.4m
	EXT_ETSR_ph3_5 = EXT_ETSR_START_INDEX + 90,   //禁令标志-限制高度3.5m
	EXT_ETSR_ph3_6 = EXT_ETSR_START_INDEX + 91,   //禁令标志-限制高度3.6m
	EXT_ETSR_ph3_7 = EXT_ETSR_START_INDEX + 92,   //禁令标志-限制高度3.7m
	EXT_ETSR_ph3_8 = EXT_ETSR_START_INDEX + 93,   //禁令标志-限制高度3.8m
	EXT_ETSR_ph3_9 = EXT_ETSR_START_INDEX + 94,   //禁令标志-限制高度3.9m
	EXT_ETSR_ph4 = EXT_ETSR_START_INDEX + 95,   //禁令标志-限制高度4m
	EXT_ETSR_ph4_1 = EXT_ETSR_START_INDEX + 96,   //禁令标志-限制高度4.1m
	EXT_ETSR_ph4_2 = EXT_ETSR_START_INDEX + 97,   //禁令标志-限制高度4.2m
	EXT_ETSR_ph4_3 = EXT_ETSR_START_INDEX + 98,   //禁令标志-限制高度4.3m
	EXT_ETSR_ph4_4 = EXT_ETSR_START_INDEX + 99,   //禁令标志-限制高度4.4m
	EXT_ETSR_ph4_5 = EXT_ETSR_START_INDEX + 100,   //禁令标志-限制高度4.5m
	EXT_ETSR_ph4_6 = EXT_ETSR_START_INDEX + 101,   //禁令标志-限制高度4.6m
	EXT_ETSR_ph4_7 = EXT_ETSR_START_INDEX + 102,   //禁令标志-限制高度4.7m
	EXT_ETSR_ph4_8 = EXT_ETSR_START_INDEX + 103,   //禁令标志-限制高度4.8m
	EXT_ETSR_ph4_9 = EXT_ETSR_START_INDEX + 104,   //禁令标志-限制高度4.9m
	EXT_ETSR_ph5 = EXT_ETSR_START_INDEX + 105,   //禁令标志-限制高度5m
	EXT_ETSR_ph5_1 = EXT_ETSR_START_INDEX + 106,   //禁令标志-限制高度5.1m
	EXT_ETSR_ph5_2 = EXT_ETSR_START_INDEX + 107,   //禁令标志-限制高度5.2m
	EXT_ETSR_ph5_3 = EXT_ETSR_START_INDEX + 108,   //禁令标志-限制高度5.3m
	EXT_ETSR_ph5_4 = EXT_ETSR_START_INDEX + 109,   //禁令标志-限制高度5.4m
	EXT_ETSR_ph5_5 = EXT_ETSR_START_INDEX + 110,   //禁令标志-限制高度5.5m
	EXT_ETSR_pl10 = EXT_ETSR_START_INDEX + 111,   //禁令标志-限制速度10km/h
	EXT_ETSR_pl100 = EXT_ETSR_START_INDEX + 112,   //禁令标志-限制速度100km/h
	EXT_ETSR_pl110 = EXT_ETSR_START_INDEX + 113,   //禁令标志-限制速度110km/h
	EXT_ETSR_pl120 = EXT_ETSR_START_INDEX + 114,   //禁令标志-限制速度120km/h
	EXT_ETSR_pl15 = EXT_ETSR_START_INDEX + 115,   //禁令标志-限制速度15km/h
	EXT_ETSR_pl20 = EXT_ETSR_START_INDEX + 116,   //禁令标志-限制速度20km/h
	EXT_ETSR_pl25 = EXT_ETSR_START_INDEX + 117,   //禁令标志-限制速度25km/h
	EXT_ETSR_pl30 = EXT_ETSR_START_INDEX + 118,   //禁令标志-限制速度30km/h
	EXT_ETSR_pl35 = EXT_ETSR_START_INDEX + 119,   //禁令标志-限制速度35km/h
	EXT_ETSR_pl40 = EXT_ETSR_START_INDEX + 120,   //禁令标志-限制速度40km/h
	EXT_ETSR_pl5 = EXT_ETSR_START_INDEX + 121,   //禁令标志-限制速度5km/h
	EXT_ETSR_pl50 = EXT_ETSR_START_INDEX + 122,   //禁令标志-限制速度50km/h
	EXT_ETSR_pl60 = EXT_ETSR_START_INDEX + 123,   //禁令标志-限制速度60km/h
	EXT_ETSR_pl70 = EXT_ETSR_START_INDEX + 124,   //禁令标志-限制速度70km/h
	EXT_ETSR_pl80 = EXT_ETSR_START_INDEX + 125,   //禁令标志-限制速度80km/h
	EXT_ETSR_pl90 = EXT_ETSR_START_INDEX + 126,   //禁令标志-限制速度90km/h
	EXT_ETSR_pm10 = EXT_ETSR_START_INDEX + 127,   //禁令标志-限制质量 10t
	EXT_ETSR_pm11 = EXT_ETSR_START_INDEX + 128,   //禁令标志-限制质量 11t
	EXT_ETSR_pm12 = EXT_ETSR_START_INDEX + 129,   //禁令标志-限制质量 12t
	EXT_ETSR_pm13 = EXT_ETSR_START_INDEX + 130,   //禁令标志-限制质量 13t
	EXT_ETSR_pm14 = EXT_ETSR_START_INDEX + 131,   //禁令标志-限制质量 14t
	EXT_ETSR_pm15 = EXT_ETSR_START_INDEX + 132,   //禁令标志-限制质量 15t
	EXT_ETSR_pm16 = EXT_ETSR_START_INDEX + 133,   //禁令标志-限制质量 16t
	EXT_ETSR_pm17 = EXT_ETSR_START_INDEX + 134,   //禁令标志-限制质量 17t
	EXT_ETSR_pm18 = EXT_ETSR_START_INDEX + 135,   //禁令标志-限制质量 18t
	EXT_ETSR_pm19 = EXT_ETSR_START_INDEX + 136,   //禁令标志-限制质量 19t
	EXT_ETSR_pm2 = EXT_ETSR_START_INDEX + 137,   //禁令标志-限制质量 2t
	EXT_ETSR_pm2_5 = EXT_ETSR_START_INDEX + 138,   //禁令标志-限制质量 2.5t
	EXT_ETSR_pm20 = EXT_ETSR_START_INDEX + 139,   //禁令标志-限制质量 20t
	EXT_ETSR_pm21 = EXT_ETSR_START_INDEX + 140,   //禁令标志-限制质量 21t
	EXT_ETSR_pm22 = EXT_ETSR_START_INDEX + 141,   //禁令标志-限制质量 22t
	EXT_ETSR_pm23 = EXT_ETSR_START_INDEX + 142,   //禁令标志-限制质量 23t
	EXT_ETSR_pm24 = EXT_ETSR_START_INDEX + 143,   //禁令标志-限制质量 24t
	EXT_ETSR_pm25 = EXT_ETSR_START_INDEX + 144,   //禁令标志-限制质量 25t
	EXT_ETSR_pm26 = EXT_ETSR_START_INDEX + 145,   //禁令标志-限制质量 26t
	EXT_ETSR_pm27 = EXT_ETSR_START_INDEX + 146,   //禁令标志-限制质量 27t
	EXT_ETSR_pm28 = EXT_ETSR_START_INDEX + 147,   //禁令标志-限制质量 28t
	EXT_ETSR_pm29 = EXT_ETSR_START_INDEX + 148,   //禁令标志-限制质量 29t
	EXT_ETSR_pm30 = EXT_ETSR_START_INDEX + 149,   //禁令标志-限制质量 30t
	EXT_ETSR_pm31 = EXT_ETSR_START_INDEX + 150,   //禁令标志-限制质量 31t
	EXT_ETSR_pm32 = EXT_ETSR_START_INDEX + 151,   //禁令标志-限制质量 32t
	EXT_ETSR_pm33 = EXT_ETSR_START_INDEX + 152,   //禁令标志-限制质量 33t
	EXT_ETSR_pm34 = EXT_ETSR_START_INDEX + 153,   //禁令标志-限制质量 34t
	EXT_ETSR_pm35 = EXT_ETSR_START_INDEX + 154,   //禁令标志-限制质量 35t
	EXT_ETSR_pm36 = EXT_ETSR_START_INDEX + 155,   //禁令标志-限制质量 36t
	EXT_ETSR_pm37 = EXT_ETSR_START_INDEX + 156,   //禁令标志-限制质量 37t
	EXT_ETSR_pm38 = EXT_ETSR_START_INDEX + 157,   //禁令标志-限制质量 38t
	EXT_ETSR_pm39 = EXT_ETSR_START_INDEX + 158,   //禁令标志-限制质量 39t
	EXT_ETSR_pm40 = EXT_ETSR_START_INDEX + 159,   //禁令标志-限制质量 40t
	EXT_ETSR_pm41 = EXT_ETSR_START_INDEX + 160,   //禁令标志-限制质量 41t
	EXT_ETSR_pm42 = EXT_ETSR_START_INDEX + 161,   //禁令标志-限制质量 42t
	EXT_ETSR_pm43 = EXT_ETSR_START_INDEX + 162,   //禁令标志-限制质量 43t
	EXT_ETSR_pm44 = EXT_ETSR_START_INDEX + 163,   //禁令标志-限制质量 44t
	EXT_ETSR_pm45 = EXT_ETSR_START_INDEX + 164,   //禁令标志-限制质量 45t
	EXT_ETSR_pm46 = EXT_ETSR_START_INDEX + 165,   //禁令标志-限制质量 46t
	EXT_ETSR_pm47 = EXT_ETSR_START_INDEX + 166,   //禁令标志-限制质量 47t
	EXT_ETSR_pm48 = EXT_ETSR_START_INDEX + 167,   //禁令标志-限制质量 48t
	EXT_ETSR_pm49 = EXT_ETSR_START_INDEX + 168,   //禁令标志-限制质量 49t
	EXT_ETSR_pm5 = EXT_ETSR_START_INDEX + 169,   //禁令标志-限制质量  5t
	EXT_ETSR_pm50 = EXT_ETSR_START_INDEX + 170,   //禁令标志-限制质量 50t
	EXT_ETSR_pm51 = EXT_ETSR_START_INDEX + 171,   //禁令标志-限制质量 51t
	EXT_ETSR_pm52 = EXT_ETSR_START_INDEX + 172,   //禁令标志-限制质量 52t
	EXT_ETSR_pm53 = EXT_ETSR_START_INDEX + 173,   //禁令标志-限制质量 53t
	EXT_ETSR_pm54 = EXT_ETSR_START_INDEX + 174,   //禁令标志-限制质量 54t
	EXT_ETSR_pm55 = EXT_ETSR_START_INDEX + 175,   //禁令标志-限制质量 55t
	EXT_ETSR_pm56 = EXT_ETSR_START_INDEX + 176,   //禁令标志-限制质量 56t
	EXT_ETSR_pm57 = EXT_ETSR_START_INDEX + 177,   //禁令标志-限制质量 57t
	EXT_ETSR_pm58 = EXT_ETSR_START_INDEX + 178,   //禁令标志-限制质量 58t
	EXT_ETSR_pm59 = EXT_ETSR_START_INDEX + 179,   //禁令标志-限制质量 59t
	EXT_ETSR_pm6 = EXT_ETSR_START_INDEX + 180,   //禁令标志-限制质量  6t
	EXT_ETSR_pm60 = EXT_ETSR_START_INDEX + 181,   //禁令标志-限制质量 60t
	EXT_ETSR_pm7 = EXT_ETSR_START_INDEX + 182,   //禁令标志-限制质量  7t
	EXT_ETSR_pm8 = EXT_ETSR_START_INDEX + 183,   //禁令标志-限制质量  8t
	EXT_ETSR_pm9 = EXT_ETSR_START_INDEX + 184,   //禁令标志-限制质量  9t
	EXT_ETSR_pn = EXT_ETSR_START_INDEX + 185,   //禁令标志-禁止车辆临时或长时停放
	EXT_ETSR_pne = EXT_ETSR_START_INDEX + 186,   //禁令标志-禁止驶入
	EXT_ETSR_pnl = EXT_ETSR_START_INDEX + 187,   //禁令标志-禁止车辆长时停放
	EXT_ETSR_po = EXT_ETSR_START_INDEX + 188,   //禁令标志-其他
	EXT_ETSR_pr10 = EXT_ETSR_START_INDEX + 189,   //禁令标志-解除限制速度5km/h
	EXT_ETSR_pr100 = EXT_ETSR_START_INDEX + 190,   //禁令标志-解除限制速度100km/h
	EXT_ETSR_pr20 = EXT_ETSR_START_INDEX + 191,   //禁令标志-解除限制速度20km/h
	EXT_ETSR_pr30 = EXT_ETSR_START_INDEX + 192,   //禁令标志-解除限制速度30km/h
	EXT_ETSR_pr40 = EXT_ETSR_START_INDEX + 193,   //禁令标志-解除限制速度40km/h
	EXT_ETSR_pr45 = EXT_ETSR_START_INDEX + 194,   //禁令标志-解除限制速度45km/h
	EXT_ETSR_pr5 = EXT_ETSR_START_INDEX + 195,   //禁令标志-解除限制速度5km/h
	EXT_ETSR_pr50 = EXT_ETSR_START_INDEX + 196,   //禁令标志-解除限制速度50km/h
	EXT_ETSR_pr60 = EXT_ETSR_START_INDEX + 197,   //禁令标志-解除限制速度60km/h
	EXT_ETSR_pr70 = EXT_ETSR_START_INDEX + 198,   //禁令标志-解除限制速度70km/h
	EXT_ETSR_pr80 = EXT_ETSR_START_INDEX + 199,   //禁令标志-解除限制速度80km/h
	EXT_ETSR_ps = EXT_ETSR_START_INDEX + 200,   //禁令标志-停车让行
	EXT_ETSR_psl = EXT_ETSR_START_INDEX + 201,   //禁令标志-减速慢行
	EXT_ETSR_pss = EXT_ETSR_START_INDEX + 202,   //禁令标志-禁止停车
	EXT_ETSR_pw1_5 = EXT_ETSR_START_INDEX + 203,   //禁令标志-限制宽度1.5m
	EXT_ETSR_pw1_6 = EXT_ETSR_START_INDEX + 204,   //禁令标志-限制宽度1.6m
	EXT_ETSR_pw1_7 = EXT_ETSR_START_INDEX + 205,   //禁令标志-限制宽度1.7m
	EXT_ETSR_pw1_8 = EXT_ETSR_START_INDEX + 206,   //禁令标志-限制宽度1.8m
	EXT_ETSR_pw1_9 = EXT_ETSR_START_INDEX + 207,   //禁令标志-限制宽度1.9m
	EXT_ETSR_pw2 = EXT_ETSR_START_INDEX + 208,   //禁令标志-限制宽度2m
	EXT_ETSR_pw2_1 = EXT_ETSR_START_INDEX + 209,   //禁令标志-限制宽度2.1m
	EXT_ETSR_pw2_2 = EXT_ETSR_START_INDEX + 210,   //禁令标志-限制宽度2.2m
	EXT_ETSR_pw2_3 = EXT_ETSR_START_INDEX + 211,   //禁令标志-限制宽度2.3m
	EXT_ETSR_pw2_4 = EXT_ETSR_START_INDEX + 212,   //禁令标志-限制宽度2.4m
	EXT_ETSR_pw2_5 = EXT_ETSR_START_INDEX + 213,   //禁令标志-限制宽度2.5m
	EXT_ETSR_pw2_6 = EXT_ETSR_START_INDEX + 214,   //禁令标志-限制宽度2.6m
	EXT_ETSR_pw2_7 = EXT_ETSR_START_INDEX + 215,   //禁令标志-限制宽度2.7m
	EXT_ETSR_pw2_8 = EXT_ETSR_START_INDEX + 216,   //禁令标志-限制宽度2.8m
	EXT_ETSR_pw2_9 = EXT_ETSR_START_INDEX + 217,   //禁令标志-限制宽度2.9m
	EXT_ETSR_pw3 = EXT_ETSR_START_INDEX + 218,   //禁令标志-限制宽度3m
	EXT_ETSR_pw3_1 = EXT_ETSR_START_INDEX + 219,   //禁令标志-限制宽度3.1m
	EXT_ETSR_pw3_2 = EXT_ETSR_START_INDEX + 220,   //禁令标志-限制宽度3.2m
	EXT_ETSR_pw3_3 = EXT_ETSR_START_INDEX + 221,   //禁令标志-限制宽度3.3m
	EXT_ETSR_pw3_4 = EXT_ETSR_START_INDEX + 222,   //禁令标志-限制宽度3.4m
	EXT_ETSR_pw3_5 = EXT_ETSR_START_INDEX + 223,   //禁令标志-限制宽度3.5m
	EXT_ETSR_pw3_6 = EXT_ETSR_START_INDEX + 224,   //禁令标志-限制宽度3.6m
	EXT_ETSR_pw3_7 = EXT_ETSR_START_INDEX + 225,   //禁令标志-限制宽度3.7m
	EXT_ETSR_pw3_8 = EXT_ETSR_START_INDEX + 226,   //禁令标志-限制宽度3.8m
	EXT_ETSR_pw3_9 = EXT_ETSR_START_INDEX + 227,   //禁令标志-限制宽度3.9m
	EXT_ETSR_pw4 = EXT_ETSR_START_INDEX + 228,   //禁令标志-限制宽度4m
	EXT_ETSR_pw4_1 = EXT_ETSR_START_INDEX + 229,   //禁令标志-限制宽度4.1m
	EXT_ETSR_pw4_2 = EXT_ETSR_START_INDEX + 230,   //禁令标志-限制宽度4.2m
	EXT_ETSR_pw4_3 = EXT_ETSR_START_INDEX + 231,   //禁令标志-限制宽度4.3m
	EXT_ETSR_pw4_4 = EXT_ETSR_START_INDEX + 232,   //禁令标志-限制宽度4.4m
	EXT_ETSR_pw4_5 = EXT_ETSR_START_INDEX + 233,   //禁令标志-限制宽度4.5m
	EXT_ETSR_pw4_6 = EXT_ETSR_START_INDEX + 234,   //禁令标志-限制宽度4.6m
	EXT_ETSR_pw4_7 = EXT_ETSR_START_INDEX + 235,   //禁令标志-限制宽度4.7m
	EXT_ETSR_pw4_8 = EXT_ETSR_START_INDEX + 236,   //禁令标志-限制宽度4.8m
	EXT_ETSR_pw4_9 = EXT_ETSR_START_INDEX + 237,   //禁令标志-限制宽度4.9m
	EXT_ETSR_pw5 = EXT_ETSR_START_INDEX + 238,   //禁令标志-限制宽度5m
	EXT_ETSR_wo = EXT_ETSR_START_INDEX + 239,   //警告标志
	EXT_ETSR_zp1 = EXT_ETSR_START_INDEX + 240      //禁令标志-解除禁止超车
#else
    
	EXT_ETSR_NAtype = EXT_ETSR_START_INDEX									+ 0  ,			//hardnegative,背景和其他类		
	EXT_ETSR_GUIDE_SIGN_TURN_RIGHT = EXT_ETSR_START_INDEX					+ 1  ,			//	i10     指示标志-向右转弯
	EXT_ETSR_GUIDE_SIGN_TURN_LEFT_RIGHT = EXT_ETSR_START_INDEX				+ 2  ,			//	i11     指示标志-向左和向右转弯
	EXT_ETSR_GUIDE_SIGN_TURN_LEFT = EXT_ETSR_START_INDEX					+ 3  ,			//	i12     指示标志-向左转弯
	EXT_ETSR_GUIDE_SIGN_STRAIGHT = EXT_ETSR_START_INDEX						+ 4  ,			//	i13     指示标志-直行
	EXT_ETSR_GUIDE_SIGN_STRAIGHT_RIGHT = EXT_ETSR_START_INDEX				+ 5  ,			//	i14     指示标志-直行和向右转弯
	EXT_ETSR_GUIDE_SIGN_STRAIGHT_LEFT = EXT_ETSR_START_INDEX				+ 6  ,			//	i15     指示标志-直行和向左转弯
	EXT_ETSR_GUIDE_SIGN_TURN_AROUND = EXT_ETSR_START_INDEX					+ 7  ,			//	i16     指示标志-允许掉头
	EXT_ETSR_GUIDE_SIGN_KEEP_LEFT_RIGHT = EXT_ETSR_START_INDEX				+ 8  ,			//	i17     指示标志-靠左靠右行驶
	EXT_ETSR_GUIDE_SIGN_TURN_LEFT_AROUND = EXT_ETSR_START_INDEX				+ 9  ,			//	i18     指示标志-向左转弯和掉头
	EXT_ETSR_GUIDE_SIGN_PARK = EXT_ETSR_START_INDEX 						+ 10 ,			//	i19     指示标志-停车
	EXT_ETSR_GUIDE_SIGN_ROUNDABOUT = EXT_ETSR_START_INDEX					+ 11 ,			//	i3      指示标志-环岛行驶
	EXT_ETSR_GUIDE_SIGN_KEEP_RIGHT_LANE = EXT_ETSR_START_INDEX				+ 12 ,			//	i5      指示标志-靠右侧道路行驶，通用箭头
	EXT_ETSR_GUIDE_SIGN_KEEP_RIGHT_LANE_NON_MOTOR = EXT_ETSR_START_INDEX	+ 13 , 			//	i5b     指示标志-非机动车靠右侧行驶，骑行下箭头
	EXT_ETSR_GUIDE_SIGN_KEEP_RIGHT_LANE_MOTOR = EXT_ETSR_START_INDEX		+ 14 ,			//	i5c     指示标志-机动车靠右侧行驶，车辆下箭头
	EXT_ETSR_GUIDE_SIGN_KEEP_LEFT_LANE = EXT_ETSR_START_INDEX				+ 15 ,			//	i6      指示标志-靠左侧道路行驶
	EXT_ETSR_GUIDE_SIGN_KEEP_LEFT_LANE_MOTOR = EXT_ETSR_START_INDEX			+ 16 ,			//	i6c     指示标志-机动车靠左侧行驶
	EXT_ETSR_GUIDE_SIGN_MIN_100KM = EXT_ETSR_START_INDEX					+ 17 ,			//	il100   指示标志-最低限速100
	EXT_ETSR_GUIDE_SIGN_MIN_110KM = EXT_ETSR_START_INDEX					+ 18 ,			//	il110   指示标志-最低限速110
	EXT_ETSR_GUIDE_SIGN_MIN_40KM = EXT_ETSR_START_INDEX						+ 19 ,			//	il40    指示标志-最低限速40
	EXT_ETSR_GUIDE_SIGN_MIN_50KM = EXT_ETSR_START_INDEX						+ 20 ,			//	il50    指示标志-最低限速50
	EXT_ETSR_GUIDE_SIGN_MIN_60KM = EXT_ETSR_START_INDEX						+ 21 ,			//	il60    指示标志-最低限速60
	EXT_ETSR_GUIDE_SIGN_MIN_70KM = EXT_ETSR_START_INDEX						+ 22 ,			//	il70    指示标志-最低限速70
	EXT_ETSR_GUIDE_SIGN_MIN_80KM = EXT_ETSR_START_INDEX						+ 23 ,			//	il80    指示标志-最低限速80
	EXT_ETSR_GUIDE_SIGN_MIN_90KM = EXT_ETSR_START_INDEX						+ 24 ,			//	il90    指示标志-最低限速90
	EXT_ETSR_GUIDE_SIGN_OTHERS = EXT_ETSR_START_INDEX						+ 25 ,			//	io      指示标志-其他
	EXT_ETSR_GUIDE_SIGN_ZEBRA_CROSSLINE = EXT_ETSR_START_INDEX				+ 26 ,			//	ip      指示标志-人行横道线
	EXT_ETSR_GUIDE_SIGN_VEHICLE_LANE = EXT_ETSR_START_INDEX					+ 27 ,			//	ir1     指示标志-机动车车道
	EXT_ETSR_GUIDE_SIGN_BUS_LANE = EXT_ETSR_START_INDEX						+ 28 ,			//	ir2     指示标志-公交线路专用车道
	EXT_ETSR_GUIDE_SIGN_BIKE_LANE = EXT_ETSR_START_INDEX					+ 29 ,			//	ir3     指示标志-自行车道
	EXT_ETSR_LIGHT_GREEN = EXT_ETSR_START_INDEX								+ 30 ,			//	lg      指示标志-绿色交通灯
	EXT_ETSR_LIGHT_RED = EXT_ETSR_START_INDEX								+ 31 ,			//	lr      指示标志-红色交通灯
	EXT_ETSR_LIGHT_YELLOW = EXT_ETSR_START_INDEX							+ 32 ,			//	ly      指示标志-黄色交通灯
	EXT_ETSR_PROHIBIT_NO_OVERTAKE = EXT_ETSR_START_INDEX					+ 33 ,			//	p1      禁令标志-禁止超车
	EXT_ETSR_PROHIBIT_NO_MOTOR_THROUGH = EXT_ETSR_START_INDEX				+ 34 ,			//	p10     禁令标志-禁止机动车通行
	EXT_ETSR_PROHIBIT_NO_HORN = EXT_ETSR_START_INDEX						+ 35 ,			//	p11     禁令标志-禁止鸣喇叭
	EXT_ETSR_PROHIBIT_NO_MOTOCYCLE = EXT_ETSR_START_INDEX					+ 36 ,			//	p12     禁令标志-禁止二轮摩托车通行
	EXT_ETSR_PROHIBIT_NO_TWO_KINDS = EXT_ETSR_START_INDEX					+ 37 ,			//	p13     禁令标志-禁止某两种车通行
	EXT_ETSR_PROHIBIT_NO_STRAIGHT = EXT_ETSR_START_INDEX					+ 38 ,			//	p14     禁令标志-禁止直行 
	EXT_ETSR_PROHIBIT_NO_RICKSHAW = EXT_ETSR_START_INDEX					+ 39 ,			//	p15     禁令标志-禁止人力车通行 
	EXT_ETSR_PROHIBIT_NO_TRICYCLE_RICKSHAW_CARGO = EXT_ETSR_START_INDEX		+ 40 ,			//	p16     禁令标志-禁止人力货运三轮车通行  
	EXT_ETSR_PROHIBIT_NO_TRICYCLE_RICKSHAW_PEOPLE = EXT_ETSR_START_INDEX	+ 41 ,			//	p17     禁令标志-禁止人力客运三轮车通行
	EXT_ETSR_PROHIBIT_NO_TRACTOR = EXT_ETSR_START_INDEX						+ 42 ,			//	p18     禁令标志-禁止拖拉机通行
	EXT_ETSR_PROHIBIT_NO_TURN_RIGHT = EXT_ETSR_START_INDEX					+ 43 ,			//	p19     禁令标志-禁止向右转弯
	EXT_ETSR_PROHIBIT_NO_ANIMAL_DRIVE = EXT_ETSR_START_INDEX				+ 44 ,			//	p2      禁令标志-禁止畜力车进入   
	EXT_ETSR_PROHIBIT_NO_TURN_LEFT_RIGHT = EXT_ETSR_START_INDEX				+ 45 ,			//	p20     禁令标志-禁止向左向右转弯
	EXT_ETSR_PROHIBIT_NO_STRAIGHT_RIGHT = EXT_ETSR_START_INDEX				+ 46 ,			//	p21     禁令标志-禁止直行和向右转弯
	EXT_ETSR_PROHIBIT_NO_MOTOR_TRICYCLE = EXT_ETSR_START_INDEX				+ 47 ,			//	p22     禁令标志-禁止三轮机动车通行
	EXT_ETSR_PROHIBIT_NO_TURN_LEFT = EXT_ETSR_START_INDEX					+ 48 ,			//	p23     禁令标志-禁止向左转弯
	EXT_ETSR_PROHIBIT_NO_TURN_RIGHT_SPEC = EXT_ETSR_START_INDEX				+ 49 ,			//	p24     禁令标志-禁止向右转弯(某种车)
	EXT_ETSR_PROHIBIT_NO_CAR_ENTRY = EXT_ETSR_START_INDEX					+ 50 ,			//	p25     禁令标志-禁止小型客车驶入标志
	EXT_ETSR_PROHIBIT_NO_CARGO_VEHICLE = EXT_ETSR_START_INDEX				+ 51 ,			//	p26     禁令标志-禁止载货汽车通行
	EXT_ETSR_PROHIBIT_NO_DANGEROUS_GOODS_VEHICLE = EXT_ETSR_START_INDEX		+ 52 ,			//	p27     禁令标志-禁止运输危险物品车辆驶入  
	EXT_ETSR_PROHIBIT_NO_STRAIGHT_LEFT = EXT_ETSR_START_INDEX				+ 53 ,			//	p28     禁令标志-禁止直行和向左转弯
	EXT_ETSR_PROHIBIT_NO_BIKE_AND_PEOPLE = EXT_ETSR_START_INDEX				+ 54 ,			//	p29     禁令标志-禁止自行车和行人通行
	EXT_ETSR_PROHIBIT_NO_LARGE_BUS = EXT_ETSR_START_INDEX					+ 55 ,			//	p3      禁令标志-禁止大型客车驶入
	EXT_ETSR_PROHIBIT_NO_MOTORCYCLE_AND_PEOPLE = EXT_ETSR_START_INDEX		+ 56 ,			//	p30     禁令标志-禁止两轮摩托车和自行车通行
	EXT_ETSR_PROHIBIT_NO_ELEC_TRICYCLE = EXT_ETSR_START_INDEX				+ 57 ,			//	p4      禁令标志-禁止电动三轮车驶入
	EXT_ETSR_PROHIBIT_NO_TURN_AROUND = EXT_ETSR_START_INDEX					+ 58 ,			//	p5      禁令标志-禁止掉头
	EXT_ETSR_PROHIBIT_NO_NON_MOTOR = EXT_ETSR_START_INDEX					+ 59 ,			//	p6      禁令标志-禁止非机动车通行
	EXT_ETSR_PROHIBIT_NO_DRAG_OR_MOUNT_MOTOR = EXT_ETSR_START_INDEX			+ 60 ,			//	p8      禁令标志-禁止汽车拖、挂车通行
	EXT_ETSR_PROHIBIT_NO_PEOPLE = EXT_ETSR_START_INDEX						+ 61 ,			//	p9      禁令标志-禁止行人通行
	EXT_ETSR_PROHIBIT_MAX_LOAD_PER_AXLE_10T = EXT_ETSR_START_INDEX			+ 62 ,			//	pa10    禁令标志-限制轴重10t  
	EXT_ETSR_PROHIBIT_MAX_LOAD_PER_AXLE_12T = EXT_ETSR_START_INDEX			+ 63 ,			//	pa12    禁令标志-限制轴重12t
	EXT_ETSR_PROHIBIT_MAX_LOAD_PER_AXLE_13T = EXT_ETSR_START_INDEX			+ 64 ,			//	pa13    禁令标志-限制轴重13t
	EXT_ETSR_PROHIBIT_MAX_LOAD_PER_AXLE_14T = EXT_ETSR_START_INDEX			+ 65 ,			//	pa14    禁令标志-限制轴重14t
	EXT_ETSR_PROHIBIT_MAX_LOAD_PER_AXLE_8T = EXT_ETSR_START_INDEX			+ 66 ,			//	pa8     禁令标志-限制轴重8t
	EXT_ETSR_PROHIBIT_NO_THROUGH = EXT_ETSR_START_INDEX						+ 67 ,			//	pb      禁令标志-禁止通行 
	EXT_ETSR_PROHIBIT_STOP_TO_CHECK = EXT_ETSR_START_INDEX					+ 68 ,			//	pc      禁令标志-停车检查标志
	EXT_ETSR_PROHIBIT_SLOW_DOWN_GIVE_WAY  = EXT_ETSR_START_INDEX			+ 69 ,			//	pg      禁令标志-减速让行
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_1_5M  = EXT_ETSR_START_INDEX				+ 70 ,			//	ph1.5   禁令标志-限制高度1.5m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_1_6M  = EXT_ETSR_START_INDEX				+ 71 ,			//	ph1.8   禁令标志-限制高度1.6m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_0M  = EXT_ETSR_START_INDEX				+ 72 ,			//	ph2     禁令标志-限制高度2m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_1M  = EXT_ETSR_START_INDEX				+ 73 ,			//	ph2.1   禁令标志-限制高度2.1m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_2M  = EXT_ETSR_START_INDEX				+ 74 ,			//	ph2.2   禁令标志-限制高度2.2m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_4M  = EXT_ETSR_START_INDEX				+ 75 ,			//	ph2.4   禁令标志-限制高度2.4m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_5M  = EXT_ETSR_START_INDEX				+ 76 ,			//	ph2.5   禁令标志-限制高度2.5m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_8M  = EXT_ETSR_START_INDEX				+ 77 ,			//	ph2.8   禁令标志-限制高度2.8m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_2_9M  = EXT_ETSR_START_INDEX				+ 78 ,			//	ph2.9   禁令标志-限制高度2.9m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_3_0M  = EXT_ETSR_START_INDEX				+ 79 ,			//	ph3     禁令标志-限制高度3m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_3_2M  = EXT_ETSR_START_INDEX				+ 80 ,			//	ph3.2   禁令标志-限制高度3.2m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_3_5M  = EXT_ETSR_START_INDEX				+ 81 ,			//	ph3.5   禁令标志-限制高度3.5m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_3_6M  = EXT_ETSR_START_INDEX				+ 82 ,			//	ph3.6   禁令标志-限制高度3.6m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_3_8M  = EXT_ETSR_START_INDEX				+ 83 ,			//	ph3.8   禁令标志-限制高度3.8m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_3_9M  = EXT_ETSR_START_INDEX				+ 84 ,			//	ph3.9   禁令标志-限制高度3.9m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_4_0M  = EXT_ETSR_START_INDEX				+ 85 ,			//	ph4     禁令标志-限制高度4m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_4_2M  = EXT_ETSR_START_INDEX				+ 86 ,			//	ph4.2   禁令标志-限制高度4.2m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_4_3M  = EXT_ETSR_START_INDEX				+ 87 ,			//	ph4.3   禁令标志-限制高度4.3m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_4_5M  = EXT_ETSR_START_INDEX				+ 88 ,			//	ph4.5   禁令标志-限制高度4.5m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_4_7M  = EXT_ETSR_START_INDEX				+ 89 ,			//	ph4.7   禁令标志-限制高度4.7m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_4_8M  = EXT_ETSR_START_INDEX				+ 90 ,			//	ph4.8   禁令标志-限制高度4.8m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_5_0M  = EXT_ETSR_START_INDEX				+ 91 ,			//	ph5     禁令标志-限制高度5m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_5_3M  = EXT_ETSR_START_INDEX				+ 92 ,			//	ph5.3   禁令标志-限制高度5.3m
	EXT_ETSR_PROHIBIT_MAX_HEIGHT_5_5M  = EXT_ETSR_START_INDEX				+ 93 ,			//	ph5.5   禁令标志-限制高度5.5m
	EXT_ETSR_EPROHIBIT_MAX_SPEED_10KM  = EXT_ETSR_START_INDEX				+ 94 ,			//	pl10    禁令标志-限制速度10km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_100KM  = EXT_ETSR_START_INDEX				+ 95 ,			//	pl100   禁令标志-限制速度100km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_110KM  = EXT_ETSR_START_INDEX				+ 96 ,			//	pl110   禁令标志-限制速度110km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_120KM  = EXT_ETSR_START_INDEX				+ 97 ,			//	pl120   禁令标志-限制速度120km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_15KM  = EXT_ETSR_START_INDEX				+ 98 ,			//	pl15    禁令标志-限制速度15km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_20KM  = EXT_ETSR_START_INDEX				+ 99 ,			//	pl20    禁令标志-限制速度20km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_25KM  = EXT_ETSR_START_INDEX				+ 100,			//	pl25    禁令标志-限制速度25km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_30KM  = EXT_ETSR_START_INDEX				+ 101,			//	pl30    禁令标志-限制速度30km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_35KM  = EXT_ETSR_START_INDEX				+ 102,			//	pl35    禁令标志-限制速度35km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_40KM  = EXT_ETSR_START_INDEX				+ 103,			//	pl40    禁令标志-限制速度40km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_5KM  = EXT_ETSR_START_INDEX				+ 104,			//	pl5     禁令标志-限制速度5km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_50KM  = EXT_ETSR_START_INDEX				+ 105,			//	pl50    禁令标志-限制速度50km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_60KM  = EXT_ETSR_START_INDEX				+ 106,			//	pl60    禁令标志-限制速度60km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_70KM  = EXT_ETSR_START_INDEX				+ 107,			//	pl70    禁令标志-限制速度70km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_80KM  = EXT_ETSR_START_INDEX				+ 108,			//	pl80    禁令标志-限制速度80km/h
	EXT_ETSR_EPROHIBIT_MAX_SPEED_90KM  = EXT_ETSR_START_INDEX				+ 109,			//	pl90    禁令标志-限制速度90km/h
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_10T = 	EXT_ETSR_START_INDEX			+ 110,			//	pm10    禁令标志-限制质量10t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_13T = 	EXT_ETSR_START_INDEX			+ 111,			//	pm13    禁令标志-限制质量13t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_15T = 	EXT_ETSR_START_INDEX			+ 112,			//	pm15    禁令标志-限制质量15t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_2T = 	EXT_ETSR_START_INDEX			+ 113,			//	pm2     禁令标志-限制质量2t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_20T = 	EXT_ETSR_START_INDEX			+ 114,			//	pm20    禁令标志-限制质量20t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_25T = 	EXT_ETSR_START_INDEX			+ 115,			//	pm25    禁令标志-限制质量25t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_2_5T = EXT_ETSR_START_INDEX			+ 116,			//	pm2.5   禁令标志-限制质量2.5t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_30T = 	EXT_ETSR_START_INDEX			+ 117,			//	pm30    禁令标志-限制质量30t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_35T = 	EXT_ETSR_START_INDEX			+ 118,			//	pm35    禁令标志-限制质量35t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_40T = 	EXT_ETSR_START_INDEX			+ 119,			//	pm40    禁令标志-限制质量40t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_5T = 	EXT_ETSR_START_INDEX			+ 120,			//	pm5     禁令标志-限制质量5t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_50T = 	EXT_ETSR_START_INDEX			+ 121,			//	pm50    禁令标志-限制质量50t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_55T = 	EXT_ETSR_START_INDEX			+ 122,			//	pm55    禁令标志-限制质量55t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_6T = 	EXT_ETSR_START_INDEX			+ 123,			//	pm6     禁令标志-限制质量6t
	EXT_ETSR_PROHIBIT_MAX_LOAD_TOTAL_8T = 	EXT_ETSR_START_INDEX			+ 124,			//	pm8     禁令标志-限制质量8t
	EXT_ETSR_PROHIBIT_PARK_ANY_TIME = EXT_ETSR_START_INDEX					+ 125,			//	pn      禁令标志-禁止车辆临时或长时停放
	EXT_ETSR_PROHIBIT_ENTRY = EXT_ETSR_START_INDEX							+ 126,			//	pne     禁令标志-禁止驶入
	EXT_ETSR_PROHIBIT_PARK_LONG_TIME = EXT_ETSR_START_INDEX					+ 127,			//	pnl     禁令标志-禁止车辆长时停放
	EXT_ETSR_PROHIBIT_OTHERS = EXT_ETSR_START_INDEX							+ 128,			//	po      禁令标志-其他
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_10KM = EXT_ETSR_START_INDEX			+ 129,			//	pr10    禁令标志-解除限制速度10km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_100KM = EXT_ETSR_START_INDEX			+ 130,			//	pr100   禁令标志-解除限制速度100km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_20KM = EXT_ETSR_START_INDEX			+ 131,			//	pr20    禁令标志-解除限制速度20km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_30KM = EXT_ETSR_START_INDEX			+ 132,			//	pr30    禁令标志-解除限制速度30km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_40KM = EXT_ETSR_START_INDEX			+ 133,			//	pr40    禁令标志-解除限制速度40km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_45KM = EXT_ETSR_START_INDEX			+ 134,			//	pr45    禁令标志-解除限制速度45km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_5KM = EXT_ETSR_START_INDEX				+ 135,			//	pr5     禁令标志-解除限制速度5km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_50KM = EXT_ETSR_START_INDEX			+ 136,			//	pr50    禁令标志-解除限制速度50km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_60KM = EXT_ETSR_START_INDEX			+ 137,			//	pr60    禁令标志-解除限制速度60km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_70KM = EXT_ETSR_START_INDEX			+ 138,			//	pr70    禁令标志-解除限制速度70km/h
	EXT_ETSR_PROHIBIT_NO_LIMIT_SPEED_80KM = EXT_ETSR_START_INDEX			+ 139,			//	pr80    禁令标志-解除限制速度80km/h
	EXT_ETSR_PROHIBIT_STOP_TO_GIVE_WAY = EXT_ETSR_START_INDEX				+ 140,			//	ps      禁令标志-停车让行
	EXT_ETSR_PROHIBIT_SLOW_DOWN = EXT_ETSR_START_INDEX						+ 141,			//	psl     禁令标志-减速慢行
	EXT_ETSR_PROHIBIT_PARK = EXT_ETSR_START_INDEX							+ 142,			//	pss     禁令标志-禁止停车
	EXT_ETSR_PROHIBIT_MAX_WIDTH_2_0M  = EXT_ETSR_START_INDEX				+ 143,			//	pw2     禁令标志-限制宽度2m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_2_5M  = EXT_ETSR_START_INDEX				+ 144,			//	pw2.5   禁令标志-限制宽度2.5m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_2_8M  = EXT_ETSR_START_INDEX				+ 145,			//	pw2.8   禁令标志-限制宽度2.8m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_2_9M  = EXT_ETSR_START_INDEX				+ 146,			//	pw2.9   禁令标志-限制宽度2.9m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_3_0M  = EXT_ETSR_START_INDEX				+ 147,			//	pw3     禁令标志-限制宽度3m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_3_2M  = EXT_ETSR_START_INDEX				+ 148,			//	pw3.2   禁令标志-限制宽度3.2m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_3_5M  = EXT_ETSR_START_INDEX				+ 149,			//	pw3.5   禁令标志-限制宽度3.5m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_4_0M  = EXT_ETSR_START_INDEX				+ 150,			//	pw4     禁令标志-限制宽度4m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_4_2M  = EXT_ETSR_START_INDEX				+ 151,			//	pw4.2   禁令标志-限制宽度4.2m
	EXT_ETSR_PROHIBIT_MAX_WIDTH_4_5M  = EXT_ETSR_START_INDEX				+ 152,			//	pw4.5   禁令标志-限制宽度4.5m
	EXT_ETSR_WARN_SIGN	= EXT_ETSR_START_INDEX								+ 153, 			//	wo      警告标志
	EXT_ETSR_PROHIBIT_NO_OVERTAKE_RELIEVE	= EXT_ETSR_START_INDEX			+ 154 			//	zp1     禁令标志-解除禁止超车
#endif

}MvTsrClassType;

//大类所用枚举类型
typedef enum _MV_TRACKTYPE
{
	MV_TR_PEOPLE,
	MV_TR_CAR,
	MV_TR_OTHER,
	MV_TR_NO_TRACK,		//不需要跟踪的目标
	MV_TR_GRANDMARK,		//路面标志
	MV_OBSTACLE,            //障碍物
	MV_TR_NONMOTORVEHICLE, //非机动车  
	MV_TRAFFIC_SIGN,  //交通标志
	MV_TR_OBSTACLE
}MvTrackType;

enum  MvEventType
{
	NO_EVENT = 0,
	EVT_LDW_LEFT = 1,
	EVT_LDW_RIGHT = 2,
	EVT_FCW = 4,
	EVT_PCW = 8,
	EVT_HMW = 16,
	EVT_UFCW = 32,
	EVT_SLI = 64,
	EVT_IHC = 128,
	EVT_MIDDLELINE_LEFT = 256,
	EVT_MIDDLELINE_RIGHT = 512,
	EVT_FOLLOW_FRONT = 1024,
	EVT_CROSS_WALK = 2048,      //人行横道检测报警
	EVT_BLIND = 4096,
};

//自定义点坐标
#ifndef _MV_POINT_
#define _MV_POINT_
typedef struct _MV_POINT
{
	int x;												//X坐标
	int y;												//Y坐标
}MvPoint; 						
#endif	
//自定义浮点坐标	
#ifndef _MV_FLOAT_POINT_
#define _MV_FLOAT_POINT_					
typedef struct _MV_FLOAT_POINT						
{						
	float x;											//X坐标
	float y;											//Y坐标
}MvFloatPoint;						
#endif						
//自定义矩形框	
#ifndef _MV_RECT_
#define _MV_RECT_
typedef struct _MV_RECT						
{						
	int x;												//左上角点位X坐标
	int y;												//左上角点位Y坐标
	int width;											//矩形宽度
	int height;											//矩形高度
}MvRect;						
#endif						
//自定义浮点矩形框						
typedef struct _MV_FLOAT_RECT						
{						
	float x;											//左上角点位X坐标
	float y;											//左上角点位Y坐标
	float width;										//矩形宽度
	float height;										//矩形高度
}MvFloatRect;	

typedef struct MvEncodePoint
{
	short nStart;
	short nMarky;
	short nLength;
}MvEncodePoint;

//物体的三维空间尺寸
typedef struct MvThreeDimensionSize
{
	float fLength;
	float fWidth;
	float fHeight;
}MvThreeDimensionSize;


//两个2dbox，表示车头，车位
typedef struct Mv3dPoint
{
	float x;											//左上角点位X坐标
	float y;											//左上角点位Y坐标
	float width;										//矩形宽度
	float height;										//矩形高度	
}Mv3dPoint;

typedef struct MvWeightRect
{
	Mv3dPoint boxrectinfo[2];  //0:front box 1:rear box
	MvFloatRect rect;
	float fScore;
}MvWeightRect;

typedef struct MvRectInfo
{
	MvFloatRect rect;
	float fScore;
	int nType;//小类类型
	Mv3dPoint boxrectinfo[2]; //3d目标信息 
}MvRectInfo;

typedef struct _MvFreeSpacePoint
{
	MvPoint pt;											//坐标，相对于1280 720的图像
	MvFloatPoint ptWorld;								//世界坐标系位置
	MvClassType uType;									//属性40：路沿，0人，1车	
}MvFreeSpacePoint;

typedef struct MvFreeSpaceReg_
{
	MvFreeSpacePoint pt[POINTNUMPERFREESPACE];			//坐标，相对于1280 720的图像
	int numbers;										//点数	
}MvFreeSpaceReg;

typedef struct MvFuisonFreeSpaceReg_
{
	MvFreeSpacePoint FusionFSR[POINTNUMPERFREESPACE][8];		
}MvFuisonFreeSpaceReg;


//如果在64位环境,这些假设还成立么？
typedef struct _MV_SETTINGS
{
	//原图
	int nOriginalWidth;
	int nOriginalHeight;

	int nWidth;											//设计的时候，应该注意，该宽度最好是8的倍数
	int nHeight;

	int nGapX;
	int nGapY;

	int nCheckType;										//检测类型
		
	int nDisLen2Tyre;									//镜头和前轮胎之间的距离，单位mm,镜头前方为正向，was nAlertTime
	int nDeviation;										//运行路线和车道线偏移角度，默认45
							
	MvRect rtROI;							
	MvPoint ptVanishing;							
							
	int		m_nCarLen;									//车身长度，单位mm
	int 	m_nCarWidth;								//车身宽度
	int 	m_nFront;									//摄像头到保险杠的水平距离
	int 	m_nCarLeft;									//摄像头到左轮水平距离(mm)
	int 	m_nCarRight;								//摄像头到右轮水平距离(mm)

	int		m_nCarIpmLeft;
	int		m_nCarIpmRight;
	float	m_nRefTop;

	float	fCameraHeight;								//相机高度（mm）
	float	fPitch;										//单位度
	float	fYaw;										//单位度
		
	float	pfCameParam[9];								//内参矩阵	
		
	float	m_cy;										//x方向上的光轴偏移量,单位mm
	float	m_fx;		
	float	m_fy;										//y方向上的物理焦距，单位mm
		
	float	m_fpixX;									//现在被自学习出来的yaw替代，老的作用已经不再需要;x方向上每个单位像素之间的物理距离,单位mm
	float	m_fpixY;									//y方向上每个单位像素之间的物理距离，单位mm
		
	int	nReferencePointFlag;							//目标横向参考点标志 0=摄像头为参考点 1=保险杠中心为参考点
	float fXDistOffset;                   				//横向参考点位移
	int m_nRearAxisDistance;    						//后轴距离(前保险杆到后轴距离) 北汽项目需要
	int nReserve;
	long long pReserve;			
	
#ifndef  MV_USE_SLIM_SETTINGS
	float m_pDis[360];						//不同位置的距离
	short pMapX[360][640];
	short pMapY[360][640];
	short pSrc2IpmX[360][640];
	short pSrc2IpmY[360][640];
#endif

	//保存指针类型
	unsigned char * pMem;

	unsigned char pReserve2[16];
}MvSettings;
typedef struct _MV_SETTINGS_SECOND
{
	//原图
	int nOriginalWidth;
	int nOriginalHeight;

	int nWidth;											//设计的时候，应该注意，该宽度最好是8的倍数
	int nHeight;

	int nGapX;
	int nGapY;

	int nCheckType;										//检测类型
		
	int nDisLen2Tyre;									//镜头和前轮胎之间的距离，单位mm,镜头前方为正向，was nAlertTime
	int nDeviation;										//运行路线和车道线偏移角度，默认45
							
	MvRect rtROI;							
	MvPoint ptVanishing;							
							
	int		m_nCarLen;									//车身长度，单位mm
	int 	m_nCarWidth;								//车身宽度 单位mm
	int 	m_nFront;									//摄像头到保险杠的水平距离 单位mm
	int 	m_nCarLeft;									//摄像头到左轮水平距离(mm)
	int 	m_nCarRight;								//摄像头到右轮水平距离(mm)

	int		m_nCarIpmLeft;
	int		m_nCarIpmRight;
	float	m_nRefTop;

	float	fCameraHeight;								//相机高度（mm）
	float 	fPitch;										//单位 度 俯仰角
	float	fYaw;		                        		//单位 度 偏航角
		
	float 	pfCameParam[9];		
		
	float	m_cy;										//x方向上的光轴偏移量,单位mm
	float	m_fx;		
	float	m_fy;										//y方向上的物理焦距，单位mm
		
	float	m_fpixX;									//自学习航向角固定偏移，解决系统安装误差
	float	m_fpixY;									//y方向上每个单位像素之间的物理距离，单位mm
		
	int	    nReferencePointFlag;						//目标横向参考点标志 0=摄像头为参考点 1=保险杠中心为参考点
	float   fXDistOffset;                   			//横向参考点位移	
    int		m_nRearAxisDistance;    					//后轴距离(前保险杆到后轴距离) 北汽项目需要
	int     nReserve;
	long long pReserve;
	
    float m_pDis[MAX_AD_HEIGHT];						//不同位置的距离


    //外参矩阵
	float fCameraExtrinsicR[9];							//外参R矩阵
	float fCameraExtrinsicT[3];							//外参T矩阵
    float 	fRoll;		       							//单位 度 横滚角,新增
	unsigned char pReserve2[356];						//总共2K:2048Bytes        
    int nCustomerType;									//客户类型 前视前装0（对应前视产品线） 前视后装1（对应后装产品线）
    int nIsExtend;										//是否存在标定文件大小扩展 默认为0 无 1有
    unsigned char uVersion[4];							//版本号
    unsigned char CheckOut[4];							//校验值
}MvSettingsSecond;

typedef struct mv_car_info_input
{
	char cFlagCan;                                //CAN数据是否有效   。               0：无效；1：有效
	char cFlagImu;                                //欧拉角是否是惯性器件计算的。        0：硬件无惯性器件，传递的欧拉角无效；1：单独陀螺仪计算出来的；2：陀螺和加速度计融合出来的；3：9轴融合的；4：从第三方传感器获取的
	char cReserved[2];                          	//内存对齐

	float fYaw;										//偏航角，车前进方向为Y，车左手向为X，车顶向上为Z    单位：度
	float fPitch;				        //俯仰角                                        单位：度
	float fRoll;				        //横滚角                                        单独：度

	float fVelocity;			                //车速                                          单位: km/h
	float fAcceleration;		                //车辆加速度 >0表示加速度   <0表示减速度        单位:m/s2
	int nBrake;					//刹车                                          1，刹车  0，未刹车
	int nLLight;				        //左转向灯                                      1、点亮 0、不亮
	int nRLight;				        //右转向灯                                      1、点亮 0、不亮
	float fAlpha;				        //转向角                                        单位：度
    float fSteeingWheelAngle;                //方向盘转角                                   单位：度
    int nDayNight;							//区分白天夜晚
    float fYawRate;				 			//偏航角速度			                单位：度/秒 °/s
	float fXAcc;							//横向加速度
#ifdef MV_USE_SUPERSONIC
	float fLeftFrontDist;			//超声雷达左前方目标距离
	float fRightFrontDist;			//超声雷达右前方目标距离
   	char cReserved2[124-64];	        //预留部分
#else
	float f_left_front_steel_speed;
	float f_right_front_steel_speed;
	float f_left_rear_steel_speed;
	float f_right_rear_steel_speed;
	char cReserved2[124 - 72];	        //预留部分
#endif

}MvCarInfoInput;

typedef struct mv_thirdnetobjinfo
{
	MvPoint Boxviewpt[8];   //图像8个点坐标
	float fBoxDirection;    //车头朝向，暂时没输出
	MvThreeDimensionSize BoxSize; //物理尺寸
}MvThirdNetObjInfo;


typedef struct _AD_OBJECT_DETECT
{	
	short nType;               							//目标类型:大类目标 0：人/两轮车，1：机动车，2：其他，3：不需要跟踪的目标，4：地面标记
	short nDetectType;			 						//目标类型:小类目标 0：人，1：小汽车/SUV，
														//2：大巴，3：卡车，4：中巴，5：斑马线，6：两轮车，
														//7：三轮车，10：交通标志，13：障碍物
					
	int nId;											//目标ID
	//检测出来的位置					
	short nTop;											//目标框位置左上点纵坐标
	short nLeft;                    					//目标框位置左上点横坐标
	short nRight;                   					//目标框位置右下点横坐标
	short nBottom;                  					//目标框位置右下点纵坐标
	float fWeight;										//如果为负数，表明此结果是推算出来的结果，目标置信度，最大值1，最小值0
	float fDist;										//每个目标的距离	单位m，最大值250，最小值0
	float fVelo;										//相对运动目标的相对速度 m/s最大100			
	float fTTC;											//ttc时间 单位s，最小值0
	short sFcwMark;										//FCW输出标志
	short sAngle;									//输出标志恢复为保留
	float fXDist;                   					//水平方向距离(m)
	float fXVelo;										//水平方向速度(m/s)
	int nIncomingState;									//-255 未初始化，1 是  0否
	float fAcc;											//加速度
	int nObjVaild;   									//1=new;  2 =Measured in this cycle;  3=Not Measured in this cycle;  4 = tracker;
	int nLanePos;										//返回目标车道信息 -255 未初始化 -2：靠左 -1：靠左，左车道，0：当前车道，
														//1靠右,右车道  2 靠右    3 cut-in（在旁边车道）  4-cut out（在当前车道）
	int nIsMainObj;                 					//是否是主目标：1是 0否
	float fObjWidth;									//目标的宽(世界坐标)单位m，最大值10，最小值0
	float fObjHeight;									//目标的高度(世界坐标) 单位m，最大值10，最小值0
	float fXAcc;										//目标横向加速度 单位m/s2，最大值10，最小值0
	int nHisLen;										//跟踪历史长度 连续帧数，最大值255，最小值0
	char chTTCValid;             						//1=FCW/PCW 0=invalid
	char cDeviceNo;										//设备号
	short sRangeTTCScore;								//前车启动标志1
	char nFusionState;									// 0 纯视觉目标， 1视觉雷达融合目标， 2纯雷达目标
	char n_status;										//目标状态，第一位表示是否静止
	char n_map_id;
	char chReserved[1];									//预留字节
	float fObjLength;
}MvOutObject;

typedef struct _AD_OBJ_EXT_INFO
{
	unsigned char f_std_dist;
	unsigned char f_std_xdist;
	unsigned char f_std_velo;
	unsigned char f_std_xvelo;
	char reserverd[8-4];
}MvObjExtInfo;

typedef struct _MV_ALGINPUTOBJECT
{	
	short nType;               							//目标类型:大类目标 0：人/两轮车，1：机动车，2：其他，3：不需要跟踪的目标，4：地面标记
	short nDetectType;			 						//目标类型:小类目标 0：人，1：小汽车/SUV，
														//2：大巴，3：卡车，4：中巴，5：斑马线，6：两轮车，
														//7：三轮车，10：交通标志，13：障碍物
	//检测出来的位置					
	float nTop;											//目标框位置左上点纵坐标
	float nLeft;                    					//目标框位置左上点横坐标
	float nRight;                   					//目标框位置右下点横坐标
	float nBottom;                  					//目标框位置右下点纵坐标
	float fWeight;										//如果为负数，表明此结果是推算出来的结果，目标置信度，最大值1，最小值0
	int nObjVaild;   									//1=new;  2 =Measured in this cycle;  3=Not Measured in this cycle;  4 = tracker;
	

	int nLanePos;										//返回目标车道信息 -255 未初始化 -2：靠左 -1：靠左，左车道，0：当前车道，
														//1靠右,右车道  2 靠右    3 cut-in（在旁边车道）  4-cut out（在当前车道）
	int nIsMainObj;                 					//是否是主目标：1是 0否
	char cDeviceNo;										//设备号
	int nId;											//目标ID
}MvAlgInputObject;

//外部传入跟踪模块参数，暂定10240字节（翻一倍也无妨）
typedef struct _MV_PARAS
{
	unsigned int uIndex;								//帧号
	Mv_int64 ts; //时间戳，暂时算法没有使用

	int nDetectorCount;	//当前帧检测到多少目标

	MvAlgInputObject pDetectors[MAX_COUNT_OBJ_DETECT_PERFRAME];//检测结果
	unsigned char *m_pGrayImage;						//目前暂时存放灰度图
	unsigned char pReserve[8192 - 4104];				//预留字节
}MvParas;



//这个结构用于程序内部数据交互用，相当于数据总线
typedef struct __AD_DATA_BUS
{
	//MvParas *		g_pParas;
	unsigned char *	m_pLocalPara;
	unsigned char *	m_pInnerData;
}MvDataBus;

//分割数据
typedef struct _MitSegDataPointer
{
	unsigned char *pSegLaneData;		//车道线  {For_HUAYU:左虚右实  
	unsigned char *pSegFreeSpaceData;	//可行驶区域
	unsigned char *pSegCurbData;		//路沿
	unsigned char *pSegPeopleData;		//行人
	unsigned char *pSegCarData;			//机动车
	unsigned char *pSegBikeData;		//非动车
	unsigned char *pSegGuideLineData;		//导向线
	unsigned char *pSegWheelData;		//车轮分割图像

	unsigned char *pSegWhiteBrokenData;  //白虚线  For_HUAYU:单虚
	unsigned char *pSegYellowBrokenData; //黄虚线			 单实
	unsigned char *pSegWhiteLineData;    //白实线			 双实				
	unsigned char *pSegYellowLineData;   //黄实线			 左实右虚}

	unsigned char* SmallpSegLaneData;    //小图车道线
	unsigned char* SmallpSegCurbData;    //小图路沿

}MitSegDataPointer;


typedef enum _LANETYPE
{
	MV_WHITE_SOLID_LINE = 10,	//白实线
	MV_YELLOW_SOLID_LINE,		// = 11,//黄实线
	MV_WHITE_DASHED_LINE,		// = 12,//白虚线
	MV_YELLOW_DASHED_LINE,		//13//黄虚线	
	MV_NOTYPE_LINE,				// 14不区分线性及颜色
	MV_DOUBLE_YELLOW_LINE,		//双黄线
	MV_DOUBLE_WHITE_LINE,		//双白线

	MV_SOLID_LINE = 20,			//单实现
	MV_DASHED_LINE,				//单虚线
	MV_DOUBLE_SOLID_LINE,		//双实线
	MV_LEFT_DEFICIENCY_RIGHT_EXCESS,//左虚右实
	MV_LEFT_EXCESS_RIGHT_DEFICIENCY,//左实右虚


	MV_CURB
	
}MV_LANE_TYPE;


typedef enum _POINT_TYPE_SEGMENTATION
{
	MV_POINT_LANE = 0,
	MV_POINT_CURB = 1
	
}MvPointSegmentation;



//自定义车辆轨迹点					
typedef struct _MV_GPSIMU_POSITION					
{						
	MvFloatPoint fpt;
	float fTheta;
}MvGpsImuPosition;	



//原图车道线点显示，saic使用
typedef struct _MV_IMAGELANEPT			
{						
	MvPoint SrcPt[4][720];
	int nNumbers[4];
}MvImageLanePt;

//自定义连通区域点坐标
typedef struct _MV_POINTFLAG
{
	MvPoint pt;
	unsigned char cFlag;
}MvPointFlag; 

//自定义连通区域点坐标
typedef struct _MV_CLUSTERPOINT
{
	int nStart;
	int nEnd;
}MvClusterPoint; 

//自定义连通区域点坐标
typedef struct _MV_VANISHPOINT
{	
	int nFilterPointNumber[256];//每一组个数
	MvFloatPoint FilterPoint[64][256];//每组内边缘点
	MvFloatPoint MiddleFilterPoint[64][256];//每组中间的点
	unsigned char uVpGroupVaild[64];//第一种方法计算灭点是，利用该标志进行筛选，尽量选取主车道进行灭点计算，有利于灭点正确性，利于测距
	unsigned char uBelongtoGroup[64];//记录turegroup是从原来哪个group index来的
	int  Classname[64][256];//每组点的属性
	MvFloatPoint BefDisFilterPoint[64][256];//每组内边缘点
	MvFloatPoint BefDisMiddleFilterPoint[64][256];//每组中间的点
}MvVanishPoint; 

//相机外参数
typedef struct _Calibration_
{
	float 	fCarLen;              //车长，单位mm
	float 	fCarWidth;            //车宽，单位mm
	float 	fRefLeft;             //相机到左侧车轮距离，单位mm
	float 	fRefRight;            //相机到右侧车轮距离，单位mm
	float 	fRefTop;              //相机到前保险杠距离，单位mm
	float	fDisLen2Tyre;		  //镜头和前轮胎之间的距离，单位mm,镜头前方为正向
	float 	fCameraHeight;			//相机距离地面高度，单位m
	int		nReferencePointFlag;	//目标横向参考点标志 0=摄像头为参考点 1=保险杠中心为参考点
	int		res[16 - 8];
}Calibration;

//相机内参数
typedef struct _Camera_
{
	float Intrisic1;					//相机内参（3x3矩阵）
	float Intrisic2;
	float Intrisic3;
	float Intrisic4;
	float Intrisic5;
	float Intrisic6;
	float Intrisic7;
	float Intrisic8;
	float Intrisic9;
	float fCameraFocus;					//相机焦距，单位mm
	float fCameraDx;				//像平面离散化系数，CMOS unit cell size,单位mm
	float res[32 - 11];
}Camera;


typedef struct _MV_TSR_OBJ__
{
	int n_tsr_id;
	short s_type;		//针对张作楠-mapid
	short s_class;
	float f_left;
	float f_top;
	float f_right;
	float f_bottom;
	float f_score;
	int p_reserved[32-7];
}MvTsrObj;

//新的车道线数据格式

//class_name就是点的属性。
//# 'curb', 'laneline', 'solid', 'dashed', 'white', 'yellow', 'fishbone', 'variable', 'waitingline'
//# cls: [attr | type | color] '000', '110'
//# attr : 0 curb, 1 normal, 2 fishbone, 3 variable, 4 waiting
//# type : 0 solid, 1 dashed
//# color : 0 white 1 yellow

#define MAX_POINT_LANE 32                           // 车道线的最大条数
#define LANE_SEG_HEIGHT 104                         // 车道线的最大条数
typedef struct _LANE {
	float cx;                                 // 每一个y坐标对应的x值，如果没有点为-1
	float offset_x;                           // 中间点距离车道线的偏移
	float score;                             //  置信度
	int class_name;                          // 点的属性
}LANE;

typedef struct _LANE_Info {
	LANE lane[LANE_SEG_HEIGHT];         //分割小图的每一列y坐标
}LANEInfo;

typedef struct _LANE_Road {
	int nLane;                               //车道线条数
	int lane_activated[MAX_POINT_LANE];            //标志位
	LANEInfo lanes_list[MAX_POINT_LANE];           //每条车道线的具体信息
}LANE_Road;

#ifndef WIN32
#ifdef __cplusplus
}
#endif
#endif

#endif  //_ALG_COMMON_STRUCT_DEF_H_
