#ifndef BASE_H
#define BASE_H
 
#include <algorithm>
#include <cstdio>
#include "gac_log.h"
#define qDebug(...)  printf(__VA_ARGS__)

#undef GAC_A20
#define GAC_A58
#undef GAC_A29
#undef GAC_A18
#undef VEL_JIHE
#undef VEL_BORUI
#undef VEL_CHANGAN
#undef VEL_DONGFENG_E70
#undef CHERY_EQ
#undef CHERY_T1D



#define PC_SYSTEM           (1)
#define EMBEDDED_SYSTEM     (0)
#define PLATFORM      EMBEDDED_SYSTEM
#define ZU2_PL

#define SIMULATION (0)
#define NO_MIRROR (1)       //无后视镜模式

#define GRID_WIDTH (250)
#define GRID_HEIGHT (150)
#define GRID_NUM (GRID_WIDTH * GRID_HEIGHT)


#define COORDINARTE_TRASFORM

#define WAITING_TIME_BETWEEN_STOP_AND_SECOND_PLAN (1.5)
#define WAITING_TIME_BETWEEN_STOP_AND_PARKING_PROCESS_STOP (5)

#define WAITING_TIME_BETWEEN_STOP_AND_PARKING_PROCESS_STOP_TIMES (150)
#define TIME_CLOCK_INTERVAL (0.02)             //定时器周期

#define SUCCESS ( 0)
#define FAIL    (-1)

// Math ratio
#define PI 								 (3.1415926535897932384626433832795)
#define PI_2 							 (1.5707963267948966192313216916398)
#define M_PI            3.14159265358979323846
#define OBSTACLE_NAG_SIDE_RESOLUTION            (23)    /* 车位对面过道的宽度，x=x=[-1 : 0.5 : 10], total = 23 */



//Slot type
#define PARALLEL (0)
#define OBLIQUE  (1)


//Park type
#define TAIL_PARK_IN  (0)
#define HEAD_PARK_IN  (1)
#define TAIL_PARK_OUT  (2)
#define HEAD_PARK_OUT  (3)
#define REMOTE_IN_OUT  (4)

//Park out Direction
#define RIGHT_PARKING_OUT  (0)
#define LEFT_PARKING_OUT  (1)

//Uss Side Type
#define FRONT_SIDE (0)
#define REAR_SIDE (1)
#define FRONT_REAR_SIDE (2)



//Slot side
#define RIGHTSIDESLOT (1)
#define LEFTSIDESLOT (-1)

#define USS_TYPE    (1)
#define MIX_TYPE    (2)
#define USS_CONFIRM_TYPE    (3)
#define USER_MANUL_TYPE    (4)
//Steering direction
#define STEERING_LEFT   ( 1)
#define STEERING_RIGHT  (-1)

//Running direction
#define FORWARD     ( 1)
#define BACKWARD    (-1)
#define RUN_FORWARD  (true)
#define RUN_BACKWARD (false)

#define OBSTACLE_DIS               (0.1)
#define LIST_NUM                     (3)

#define NO_FREE                      (0x3F)
#define VEL_COLOR                    (122)
#define UNKNOW                      (100)
#define SLOT_ARER                    (32)
#define MV_NOMOTOR_SAVE               (53)

enum _planErrCode{
    NO_ERROR=0,
    E10001, //车位尺寸不够
    E10002, //平行泊车入库点规划失败
    E10003, //垂直车位目标点和车位干涉
    E10004, //可变栅格数量计算错误
    E10005, //搜索失败
    E10006, //超出最大轨迹段数
    E10007, //两次规划轨迹段数差距太大
    E20001=21, //平行泊车重规划全部失败
    E20002, //平行泊车重规划成功，但是车辆较大程度靠里或者靠外
    E20003, //平行泊车库内重规划输入y大于0
    E30001	//泊出空间探测失败
};

//规划模块和控制模块的交互接口
enum ParkingWorkingStatus
{
    freeStauts=0, //0 停车规划
    updated, //1 停车规划完成
    perparing, //2 原地方向盘
    working, //3 车辆正常行驶
    parkingErr, //4 异常停车
    parkingOver, //5 泊车结束
    RunningFreeStauts, //6 垂直最后一段动态规划
    RunningUpdated, //7 垂直最后一段动态规划完成
    Breaking, //8 刹车
    ParkingSucc, //9 泊车成功
    waiting
};

enum AddtionalStatus
{
    NormalS=0, //0
    Pause, //1
    Goon, //2
    StopIt
};

//泊车过程错误代码
enum ParkingErrCode
{
    noErr = 0,
    NoPoints,
    TrackErr,
    ObstacleErr,
    NoMoveErr
};


enum DistanceStatus
{
    SAFE=0,
    DANGEROUS,
    VERYDANG,
    STOPPED
};


#ifdef  GAC_A58

#define AHEAD_CTRL_POINT_NUM (1)
//#define   VisionSlotDelay                               (1)

#define   WIDTH_HALF                                     (0.9435f)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.349f)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.730f)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.956f)
#define   SENSORS_LONGITUDINAL_DIST                      (3.28f)
#define   HALF_VEHICLE_WIDTH                             (WIDTH_HALF)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0269f)
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0269f)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0269f)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0269f)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0269f)

#define FRONT_WHEEL_SPAN  (1.617f)

#define REAR_WHEEL_SPAN         (1.610f)
//1.GAC_A58车车身参数,单位/m
#define VEHICLE_LEN				(4.650f)
#define VEHICLE_WID				(1.887f)
#define FRONT_EDGE2CENTER		(3.674f)
#define REAR_EDGE2CENTER		(0.976f)
#define SIDE_EDGE2CENTER		(VEHICLE_WID / 2)
#define WHEEL_BASE              AXIS_DISTANCE
//jiao dian zxy....
#define CHAMFER_LENGTH (0.4f)
#define FRONT_CHAMFER_L1 (0.)
#define FRONT_CHAMFER_L2 (0.)
#define REAR_CHAMFER_L1 (0.2f)
#define REAR_CHAMFER_L2 (0.2f)
#define CHAMFER_ANGLE (45*PI/180)
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2f)
#define REAR_VIEW_WIDTH  (0.25f)
#define REAR_VIEW_2_FRONT_AXLE  (2.16f)
//2.超声安装参数
#define SIDE_LURF_CENTER_X				(3.259f)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.943f)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(0.569f)            //后侧边超声到车辆后轴中心点的X距离
//#define SENSORS_LONGITUDINAL_DIST		(3.28)			//????????????????????????????
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值


#define LeftCameraX (REAR_VIEW_2_FRONT_AXLE - 0.16f)
#define LeftCameraY (VEHICLE_WID * 0.5 + 0.05)
#define RightCameraX (LeftCameraX)
#define RightCameraY (-LeftCameraY)
#define FrontCameraX (FRONT_EDGE2CENTER)
#define FrontCameraY (0.)
#define RearCameraX (-REAR_EDGE2CENTER)
#define RearCameraY (0.)

/*yrm3,实际车辆的超声安装位置*/
#define SENSOR1_X     (3.564f)
#define SENSOR1_Y     (0.667f)
#define SENSOR1_ANGLE (42 * PI / 180)//132

#define SENSOR2_X     (3.631f)
#define SENSOR2_Y     (0.345f)
#define SENSOR2_ANGLE (7 * PI / 180)//96

#define SENSOR3_X     ( SENSOR2_X )
#define SENSOR3_Y     (-SENSOR2_Y )
#define SENSOR3_ANGLE (-SENSOR2_ANGLE )

#define SENSOR4_X     ( SENSOR1_X )
#define SENSOR4_Y     (-SENSOR1_Y )
#define SENSOR4_ANGLE (-SENSOR1_ANGLE )

#define SENSOR5_X     (-0.902f)
#define SENSOR5_Y     ( 0.603f)
#define SENSOR5_ANGLE (149 * PI / 180)//301

#define SENSOR6_X     (-0.965f)
#define SENSOR6_Y     (0.249f)
#define SENSOR6_ANGLE (173 * PI / 180)

#define SENSOR7_X     (SENSOR6_X )
#define SENSOR7_Y     (-SENSOR6_Y )
#define SENSOR7_ANGLE (-SENSOR6_ANGLE )

#define SENSOR8_X     (SENSOR5_X )
#define SENSOR8_Y     (-SENSOR5_Y )
#define SENSOR8_ANGLE ( -SENSOR5_ANGLE )

#define SENSOR9_X     ( SIDE_LURF_CENTER_X )
#define SENSOR9_Y     ( SIDE_EDGE2CENTER )
#define SENSOR9_ANGLE (82 * PI / 180)

#define SENSOR10_X     ( SENSOR9_X )
#define SENSOR10_Y     (-SENSOR9_Y )
#define SENSOR10_ANGLE (-SENSOR9_ANGLE )

#define SENSOR11_X     (-SIDE_LURR_CENTERX)
#define SENSOR11_Y     ( SIDE_EDGE2CENTER)
#define SENSOR11_ANGLE (92 * PI / 180)// *pi/3 rad, 120 degree
#define COS_SENSOR11_DIS_ANGEL (1.0f)

#define SENSOR12_X     (SENSOR11_X)
#define SENSOR12_Y     (-SENSOR11_Y)
#define SENSOR12_ANGLE (-SENSOR11_ANGLE)
#define COS_SENSOR12_DIS_ANGEL (1.0f)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (4096)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (456)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.2)
#define DANGOUS_VELOCITY (0.3)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值


#define ADDIDEGREE              (0)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)


static float EPSSteeringAngleRatio[11][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 0.182713, 0.186183, 0.193731, 0.195271},
    {200*PI/180, 0.213624, 0.210972, 0.225991, 0.225291},
    {230*PI/180, 0.253340, 0.246417, 0.264447, 0.257712},
    {260*PI/180, 0.281727, 0.283221, 0.295076, 0.292958},
    {290*PI/180, 0.316585, 0.312152, 0.329033, 0.325144},
    {320*PI/180, 0.353413, 0.348024, 0.334337, 0.362713},
    {350*PI/180, 0.368205, 0.380656, 0.402151, 0.392548},
    {380*PI/180, 0.423274, 0.415827, 0.438791, 0.427377},
    {410*PI/180, 0.466571, 0.456298, 0.476865, 0.467039},
    {440*PI/180, 0.504171, 0.483403, 0.517839, 0.498223},
    {470*PI/180, 0.527227, 0.513124, 0.541730, 0.524689}
};

static int EPS_STEERING_ANGLE_RATIO_SEGMENT=11;



//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 45, 45, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     30, 30, 30, 30,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     35, 30, 30, 35,    -1, 25, 30, 30,//backward
    }, //250
    {400,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    35, 35, 35, 35,     -1, 25, 30, 30,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif

#ifdef GAC_A20
#define AHEAD_CTRL_POINT_NUM (1)
//#define   VisionSlotDelay                               (1)

#define   WIDTH_HALF                                     (0.935f)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.399f)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.730f)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.874f)
#define   SENSORS_LONGITUDINAL_DIST                      (3.28f)
#define   HALF_VEHICLE_WIDTH                             (WIDTH_HALF)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0237f)
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0237f)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0237f)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0237f)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0237f)

#define FRONT_WHEEL_SPAN  (1.603f)

#define REAR_WHEEL_SPAN         (1.603f)
//1.GAC_A20车车身参数,单位/m
#define VEHICLE_LEN				(4.410f)
#define VEHICLE_WID				(1.870f)
#define FRONT_EDGE2CENTER		(3.604f)
#define REAR_EDGE2CENTER		(0.806f)
#define SIDE_EDGE2CENTER		(VEHICLE_WID / 2.f)
#define WHEEL_BASE              AXIS_DISTANCE
//jiao dian zxy....
#define CHAMFER_LENGTH (0.4f)
#define FRONT_CHAMFER_L1 (0.)
#define FRONT_CHAMFER_L2 (0.)
#define REAR_CHAMFER_L1 (0.2f)
#define REAR_CHAMFER_L2 (0.2f)
#define CHAMFER_ANGLE (45*PI/180)
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2f)
#define REAR_VIEW_WIDTH  (0.25f)
#define REAR_VIEW_2_FRONT_AXLE  (2.16f)
/*yrm2,超声安装参数*/
#define SIDE_LURF_CENTER_X				(3.187f)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.935f)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(0.499f)             //后侧边超声到车辆后轴中心点的X距离
//#define SENSORS_LONGITUDINAL_DIST		(3.28)			//????????????????????????????
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值

#define LeftCameraX (REAR_VIEW_2_FRONT_AXLE - 0.16f)
#define LeftCameraY (VEHICLE_WID * 0.5 + 0.05)
#define RightCameraX (LeftCameraX)
#define RightCameraY (-LeftCameraY)
#define FrontCameraX (FRONT_EDGE2CENTER)
#define FrontCameraY (0.)
#define RearCameraX (-REAR_EDGE2CENTER)
#define RearCameraY (0.)

/*yrm3,实际车辆的超声安装位置*/
#define SENSOR1_X     (3.405f)
#define SENSOR1_Y     (0.681f)
#define SENSOR1_ANGLE (42 * PI / 180)//132

#define SENSOR2_X     (3.571f)
#define SENSOR2_Y     (0.307f)
#define SENSOR2_ANGLE (7 * PI / 180)//96

#define SENSOR3_X     ( SENSOR2_X )
#define SENSOR3_Y     (-SENSOR2_Y )
#define SENSOR3_ANGLE (-SENSOR2_ANGLE )

#define SENSOR4_X     ( SENSOR1_X )
#define SENSOR4_Y     (-SENSOR1_Y )
#define SENSOR4_ANGLE (-SENSOR1_ANGLE )

#define SENSOR5_X     (-0.679f)
#define SENSOR5_Y     ( 0.639f)
#define SENSOR5_ANGLE (149 * PI / 180)//301

#define SENSOR6_X     (-0.728f)
#define SENSOR6_Y     (0.313f)
#define SENSOR6_ANGLE (173 * PI / 180)

#define SENSOR7_X     (SENSOR6_X )
#define SENSOR7_Y     (-SENSOR6_Y )
#define SENSOR7_ANGLE (-SENSOR6_ANGLE )

#define SENSOR8_X     (SENSOR5_X )
#define SENSOR8_Y     (-SENSOR5_Y )
#define SENSOR8_ANGLE ( -SENSOR5_ANGLE )

#define SENSOR9_X     ( SIDE_LURF_CENTER_X )
#define SENSOR9_Y     ( SIDE_EDGE2CENTER )
#define SENSOR9_ANGLE (82 * PI / 180)

#define SENSOR10_X     ( SENSOR9_X )
#define SENSOR10_Y     (-SENSOR9_Y )
#define SENSOR10_ANGLE (-SENSOR9_ANGLE )

#define SENSOR11_X     (-SIDE_LURR_CENTERX)
#define SENSOR11_Y     ( SIDE_EDGE2CENTER)
#define SENSOR11_ANGLE (92 * PI / 180)// *pi/3 rad, 120 degree
#define COS_SENSOR11_DIS_ANGEL (1.0f)

#define SENSOR12_X     (SENSOR11_X)
#define SENSOR12_Y     (-SENSOR11_Y)
#define SENSOR12_ANGLE (-SENSOR11_ANGLE)
#define COS_SENSOR12_DIS_ANGEL (1.0f)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (4096)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (500)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.2)
#define DANGOUS_VELOCITY (0.3)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值


#define ADDIDEGREE              (0)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)


//8.方向盘转角
static float EPSSteeringAngleRatio[13][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 0.185238, 0.185918, 0.178198, 0.178773},
    {200*PI/180, 0.216776, 0.211710, 0.210206, 0.211081},
    {230*PI/180, 0.248341, 0.250682, 0.241793, 0.244511},
    {260*PI/180, 0.282950, 0.282119, 0.277592, 0.276797},
    {290*PI/180, 0.318129, 0.316244, 0.311963, 0.311320},
    {320*PI/180, 0.351399, 0.347872, 0.350526, 0.343791},
    {350*PI/180, 0.353456, 0.346782, 0.357032, 0.348017},
    {380*PI/180, 0.385332, 0.378588, 0.382510, 0.377838},
    {410*PI/180, 0.419813, 0.408908, 0.416420, 0.408199},
    {440*PI/180, 0.456234, 0.449065, 0.451142, 0.440670},
    {470*PI/180, 0.494509, 0.479953, 0.490604, 0.477333},
    {500*PI/180, 0.537868, 0.519208, 0.529100, 0.512441},
    {530*PI/180, 0.608234, 0.587375, 0.615344, 0.590231}
};

static int EPS_STEERING_ANGLE_RATIO_SEGMENT=13;


//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 45, 45, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     30, 30, 30, 30,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     35, 30, 30, 35,    -1, 25, 30, 30,//backward
    }, //250
    {400,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    35, 35, 35, 35,     -1, 25, 30, 30,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif


#ifdef  GAC_A29

#define AHEAD_CTRL_POINT_NUM (1)
//#define   VisionSlotDelay                               (1)

#define   WIDTH_HALF                                     (0.9375f)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.295f)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.732f)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.956f)
#define   SENSORS_LONGITUDINAL_DIST                      (3.28f)
#define   HALF_VEHICLE_WIDTH                             (WIDTH_HALF)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.024025f)
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.024025f)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.024025f)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.024025f)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.024025f)

#define FRONT_WHEEL_SPAN  (1.603f)

#define REAR_WHEEL_SPAN         (1.603f)
//1.GAC_A29车车身参数,单位/m
#define VEHICLE_LEN				(4.786f)
#define VEHICLE_WID				(1.875f)
#define FRONT_EDGE2CENTER		(3.688f)
#define REAR_EDGE2CENTER		(1.098f)
#define SIDE_EDGE2CENTER		(VEHICLE_WID / 2)
#define WHEEL_BASE              AXIS_DISTANCE
//jiao dian zxy....
#define CHAMFER_LENGTH (0.4f)
#define FRONT_CHAMFER_L1 (0.)
#define FRONT_CHAMFER_L2 (0.)
#define REAR_CHAMFER_L1 (0.2f)
#define REAR_CHAMFER_L2 (0.2f)
#define CHAMFER_ANGLE (45*PI/180)
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2f)
#define REAR_VIEW_WIDTH  (0.25f)
#define REAR_VIEW_2_FRONT_AXLE  (2.16f)
/*yrm2,超声安装参数*/
#define SIDE_LURF_CENTER_X				(3.165f)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.912f)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(0.557f)             //后侧边超声到车辆后轴中心点的X距离
//#define SENSORS_LONGITUDINAL_DIST		(3.28)			//????????????????????????????
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值

#define LeftCameraX (REAR_VIEW_2_FRONT_AXLE - 0.16f)
#define LeftCameraY (VEHICLE_WID * 0.5 + 0.05)
#define RightCameraX (LeftCameraX)
#define RightCameraY (-LeftCameraY)
#define FrontCameraX (FRONT_EDGE2CENTER)
#define FrontCameraY (0.)
#define RearCameraX (-REAR_EDGE2CENTER)
#define RearCameraY (0.)

/*yrm3,实际车辆的超声安装位置*/
#define SENSOR1_X     (3.463f)
#define SENSOR1_Y     (0.683f)
#define SENSOR1_ANGLE (42 * PI / 180)//132

#define SENSOR2_X     (3.664f)
#define SENSOR2_Y     (0.300f)
#define SENSOR2_ANGLE (7 * PI / 180)//96

#define SENSOR3_X     ( SENSOR2_X )
#define SENSOR3_Y     (-SENSOR2_Y )
#define SENSOR3_ANGLE (-SENSOR2_ANGLE )

#define SENSOR4_X     ( SENSOR1_X )
#define SENSOR4_Y     (-SENSOR1_Y )
#define SENSOR4_ANGLE (-SENSOR1_ANGLE )

#define SENSOR5_X     (-0.939f)
#define SENSOR5_Y     ( 0.712f)
#define SENSOR5_ANGLE (149 * PI / 180)//301

#define SENSOR6_X     (-1.081f)
#define SENSOR6_Y     (0.251f)
#define SENSOR6_ANGLE (173 * PI / 180)

#define SENSOR7_X     (SENSOR6_X )
#define SENSOR7_Y     (-SENSOR6_Y )
#define SENSOR7_ANGLE (-SENSOR6_ANGLE )

#define SENSOR8_X     (SENSOR5_X )
#define SENSOR8_Y     (-SENSOR5_Y )
#define SENSOR8_ANGLE ( -SENSOR5_ANGLE )

#define SENSOR9_X     ( SIDE_LURF_CENTER_X )
#define SENSOR9_Y     ( SIDE_EDGE2CENTER )
#define SENSOR9_ANGLE (82 * PI / 180)

#define SENSOR10_X     ( SENSOR9_X )
#define SENSOR10_Y     (-SENSOR9_Y )
#define SENSOR10_ANGLE (-SENSOR9_ANGLE )

#define SENSOR11_X     (-SIDE_LURR_CENTERX)
#define SENSOR11_Y     ( SIDE_EDGE2CENTER)
#define SENSOR11_ANGLE (92 * PI / 180)// *pi/3 rad, 120 degree
#define COS_SENSOR11_DIS_ANGEL (1.0f)

#define SENSOR12_X     (SENSOR11_X)
#define SENSOR12_Y     (-SENSOR11_Y)
#define SENSOR12_ANGLE (-SENSOR11_ANGLE)
#define COS_SENSOR12_DIS_ANGEL (1.0f)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (4096)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (500)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.2)
#define DANGOUS_VELOCITY (0.3)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值


#define ADDIDEGREE              (0)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)


//8.方向盘转角
static float EPSSteeringAngleRatio[13][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 0.167634, 0.166019, 0.167238, 0.165267},
    {200*PI/180, 0.199523, 0.197841, 0.198431, 0.196974},
    {230*PI/180, 0.231289, 0.228570, 0.230350, 0.227709},
    {260*PI/180, 0.264773, 0.259443, 0.265296, 0.260258},
    {290*PI/180, 0.289066, 0.286410, 0.294558, 0.287079},
    {320*PI/180, 0.322947, 0.317532, 0.327242, 0.319231},
    {350*PI/180, 0.353456, 0.346782, 0.357032, 0.348017},
    {380*PI/180, 0.389147, 0.378688, 0.391277, 0.381895},
    {410*PI/180, 0.423581, 0.414356, 0.419975, 0.412760},
    {440*PI/180, 0.452435, 0.444384, 0.458563, 0.445518},
    {470*PI/180, 0.491008, 0.483260, 0.488387, 0.479588},
    {500*PI/180, 0.531320, 0.521447, 0.523395, 0.510926},
    {530*PI/180, 0.563993, 0.551919, 0.563993, 0.545660}
};

static int EPS_STEERING_ANGLE_RATIO_SEGMENT=13;


//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 45, 45, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     30, 30, 30, 30,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     35, 30, 30, 35,    -1, 25, 30, 30,//backward
    }, //250
    {400,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    35, 35, 35, 35,     -1, 25, 30, 30,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif


#ifdef  GAC_A18

#define AHEAD_CTRL_POINT_NUM (0)
//#define   VisionSlotDelay                               (1)

#define   WIDTH_HALF                                     (0.957)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.4185)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.811)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.31)
#define   SENSORS_LONGITUDINAL_DIST                      (3.18)
#define   HALF_VEHICLE_WIDTH                             (WIDTH_HALF)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0268)
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0263)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0263)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0263)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0263)

#define REAR_WHEEL_SPAN         (1.638)
//1.GAC_A18车车身参数,单位/m
#define VEHICLE_LEN				(4.560)
#define VEHICLE_WID				(1.914)
#define FRONT_EDGE2CENTER		(3.697)
#define REAR_EDGE2CENTER		(0.863)
#define SIDE_EDGE2CENTER		(VEHICLE_WID / 2)
#define WHEEL_BASE              AXIS_DISTANCE
//jiao dian zxy....
#define CHAMFER_LENGTH (0.4)
#define FRONT_CHAMFER_L1 (0.4)
#define FRONT_CHAMFER_L2 (0.4)
#define REAR_CHAMFER_L1 (0.)
#define REAR_CHAMFER_L2 (0.)
#define CHAMFER_ANGLE (45*PI/180)
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2)
#define REAR_VIEW_LENGTH_WIDTH  (0.1)
#define REAR_VIEW_2_FRONT_AXLE  (1)
//2.超声安装参数
#define SIDE_LURF_CENTER_X				(3.15)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.9)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(1)             //后侧边超声到车辆后轴中心点的X距离
//#define SENSORS_LONGITUDINAL_DIST		(3.15)			//????????????????????????????
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值

//实际车辆的超声安装位置
#define SENSOR1_X     ( 3.56f )
#define SENSOR1_Y     ( 0.68f )
#define SENSOR1_ANGLE ( 6.283185307179586476925286766559 )

#define SENSOR2_X     ( 3.76f )
#define SENSOR2_Y     ( 0.32f )
#define SENSOR2_ANGLE ( 6.283185307179586476925286766559 )

#define SENSOR3_X     ( 3.76f )
#define SENSOR3_Y     (-0.32f )
#define SENSOR3_ANGLE ( 6.283185307179586476925286766559 )

#define SENSOR4_X     ( 3.56f )
#define SENSOR4_Y     (-0.68f )
#define SENSOR4_ANGLE ( 6.283185307179586476925286766559 )

#define SENSOR5_X     (-1.02f )
#define SENSOR5_Y     ( 0.69f )
#define SENSOR5_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR6_X     (-1.08f )
#define SENSOR6_Y     ( 0.32f )
#define SENSOR6_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR7_X     (-1.08f )
#define SENSOR7_Y     (-0.32f )
#define SENSOR7_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR8_X     (-1.02f )
#define SENSOR8_Y     (-0.69f )
#define SENSOR8_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR9_X     ( 3.15f )
#define SENSOR9_Y     ( 0.90f )
#define SENSOR9_ANGLE (1.5707963267948966192313216916398 )

#define SENSOR10_X     ( 3.15f )
#define SENSOR10_Y     (-0.90f )
#define SENSOR10_ANGLE (4.7123889803846898576939650749193 )

#define SENSOR11_X     (-0.32f)
#define SENSOR11_Y     ( 0.90f)
#define SENSOR11_ANGLE (2.0943951024 )// 2*pi/3 rad, 120 degree
#define COS_SENSOR11_DIS_ANGEL (0.8660254)

#define SENSOR12_X     (-0.32f)
#define SENSOR12_Y     (-0.90f)
#define SENSOR12_ANGLE (-2.09439510243 )
#define COS_SENSOR12_DIS_ANGEL (0.8660254)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (4096)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (500)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.15)
#define DANGOUS_VELOCITY (0.2)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值



#define ADDIDEGREE              (2)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)


/*
//8.方向盘转角
static float EPSSteeringAngleRatio[12][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
	{170*PI/180, 0.177214, 0.177214, 0.177214, 0.177214},
	{200*PI/180, 0.211493, 0.211493, 0.20531, 0.20531},
	{230*PI/180, 0.245125, 0.245125, 0.238752, 0.238752},
	{260*PI/180, 0.272189, 0.272189, 0.267705, 0.267705},
	{290*PI/180, 0.30328, 0.30328, 0.30034, 0.30034},
	{320*PI/180, 0.335041, 0.335041, 0.33099, 0.33099},
	{350*PI/180, 0.365464, 0.365464, 0.365464, 0.365464},
	{380*PI/180, 0.402202, 0.402202, 0.402202, 0.402202},
	{410*PI/180, 0.436992, 0.436992, 0.432843, 0.432843},
	{440*PI/180, 0.472681, 0.472681, 0.468586, 0.468586},
	{470*PI/180, 0.510626, 0.510626, 0.505233, 0.505233},
	{500*PI/180, 0.549477, 0.549477, 0.54641, 0.54641}
};

static int EPS_STEERING_ANGLE_RATIO_SEGMENT=12;
*/
//8.方向盘转角
static float EPSSteeringAngleRatio[9][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
	{90*PI/180, 0.10104, 0.102592, 0.088393, 0.086613},
	{180*PI/180, 0.194989, 0.195847, 0.183703, 0.182488},
	{270*PI/180, 0.289317, 0.288912, 0.284165, 0.277817},
	{360*PI/180, 0.388364, 0.383274, 0.385325, 0.377389},
	{400*PI/180, 0.435512, 0.425776, 0.435575, 0.424082},
	{420*PI/180, 0.458296, 0.447731, 0.461380, 0.447398},
	{440*PI/180, 0.482019, 0.469189, 0.484401, 0.470941},
	{470*PI/180, 0.520243, 0.504592, 0.525121, 0.511412},
	{500*PI/180, 0.558699, 0.538385, 0.565778, 0.549051}
};

static int EPS_STEERING_ANGLE_RATIO_SEGMENT=9;


//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 45, 45, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     30, 30, 30, 30,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     35, 30, 30, 35,    -1, 25, 30, 30,//backward
    }, //250
    {400,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    35, 35, 35, 35,     -1, 25, 30, 30,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif


#ifdef  VEL_BORUI


#define AHEAD_CTRL_POINT_NUM (1)
//#define   VisionSlotDelay                               (1)

#define   WIDTH_HALF                                     (0.931)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.3175)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.87)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.31)
#define   SENSORS_LONGITUDINAL_DIST                      (3.18)
#define   HALF_VEHICLE_WIDTH                             (0.931)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0216)
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0216)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0216)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0216)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0216)

#define REAR_WHEEL_SPAN         (1.605)
//1.博瑞车车身参数,单位/m
#define VEHICLE_LEN				(4.986)
#define VEHICLE_WID				(1.861)
#define FRONT_EDGE2CENTER		(3.886)
#define REAR_EDGE2CENTER		(1.10)
#define SIDE_EDGE2CENTER		(0.9275)
#define WHEEL_BASE              AXIS_DISTANCE
//jiao dian zxy....
#define CHAMFER_LENGTH (0.4)
#define FRONT_CHAMFER_L1 (0.4)
#define FRONT_CHAMFER_L2 (0.4)
#define REAR_CHAMFER_L1 (0.2)
#define REAR_CHAMFER_L2 (0.2)
#define CHAMFER_ANGLE (45*PI/180)
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2)
#define REAR_VIEW_LENGTH_WIDTH  (0.1)
#define REAR_VIEW_2_FRONT_AXLE  (1)
//2.超声安装参数
#define SIDE_LURF_CENTER_X				(3.15)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.9)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(1)             //后侧边超声到车辆后轴中心点的X距离
//#define SENSORS_LONGITUDINAL_DIST		(3.15)			//????????????????????????????
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值

//实际车辆的超声安装位置
#define SENSOR1_X     ( 3.56f )
#define SENSOR1_Y     ( 0.68f )
#define SENSOR1_ANGLE ( 0 )

#define SENSOR2_X     ( 3.76f )
#define SENSOR2_Y     ( 0.32f )
#define SENSOR2_ANGLE ( 0 )

#define SENSOR3_X     ( 3.76f )
#define SENSOR3_Y     (-0.32f )
#define SENSOR3_ANGLE ( 0 )

#define SENSOR4_X     ( 3.56f )
#define SENSOR4_Y     (-0.68f )
#define SENSOR4_ANGLE ( 0 )

#define SENSOR5_X     (-1.02f )
#define SENSOR5_Y     ( 0.69f )
#define SENSOR5_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR6_X     (-1.08f )
#define SENSOR6_Y     ( 0.32f )
#define SENSOR6_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR7_X     (-1.08f )
#define SENSOR7_Y     (-0.32f )
#define SENSOR7_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR8_X     (-1.02f )
#define SENSOR8_Y     (-0.69f )
#define SENSOR8_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR9_X     ( 3.15f )
#define SENSOR9_Y     ( 0.90f )
#define SENSOR9_ANGLE (1.5707963267948966192313216916398 )

#define SENSOR10_X     ( 3.15f )
#define SENSOR10_Y     (-0.90f )
#define SENSOR10_ANGLE (4.7123889803846898576939650749193 )

#define SENSOR11_X     (-0.32f)
#define SENSOR11_Y     ( 0.90f)
#define SENSOR11_ANGLE (2.0943951024 )// 2*pi/3 rad, 120 degree
#define COS_SENSOR11_DIS_ANGEL (0.8660254)

#define SENSOR12_X     (-0.32f)
#define SENSOR12_Y     (-0.90f)
#define SENSOR12_ANGLE (-2.09439510243 )
#define COS_SENSOR12_DIS_ANGEL (0.8660254)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (4096)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (480)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.15)
#define DANGOUS_VELOCITY (0.2)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值



#define ADDIDEGREE              (2)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)



//8.方向盘转角
static float EPSSteeringAngleRatio[12][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 0.1928, 0.1928, 0.1954, 0.1928}, //0
    {200*PI/180, 0.2217, 0.2196, 0.2271, 0.2278}, //1
    {230*PI/180, 0.2558, 0.2575, 0.2614, 0.2607}, //2
    {260*PI/180, 0.2886, 0.2894, 0.2987, 0.2981}, //3
    {290*PI/180, 0.3296, 0.3263, 0.3325, 0.3311}, //4
    {320*PI/180, 0.3624, 0.3630, 0.3682, 0.3655}, //5
    {350*PI/180, 0.4018, 0.4006, 0.4087, 0.4102}, //6
    {380*PI/180, 0.4376, 0.4365, 0.4385, 0.4360}, //7
    {410*PI/180, 0.4764, 0.4687, 0.4722, 0.4722}, //8
    {440*PI/180, 0.5170, 0.5113, 0.5237, 0.5168}, //9
    {470*PI/180, 0.5549, 0.5539, 0.5568, 0.5568}, //10
    {500*PI/180, 0.6020, 0.5957, 0.605,  0.5983}  //11
};
static int EPS_STEERING_ANGLE_RATIO_SEGMENT=12;

//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 45, 45, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     30, 30, 30, 30,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     35, 30, 30, 35,    -1, 25, 30, 30,//backward
    }, //250
    {400,
     45, 45, 45, 45,     -1, -1, -1, -1,     35, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    35, 35, 35, 35,     -1, 25, 30, 30,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif






#ifdef  VEL_JIHE


#define AHEAD_CTRL_POINT_NUM (2)
//#define   VisionSlotDelay                               (1)

#define   WIDTH_HALF                                     (0.902)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.29)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.70)

#define   SENSORS_LONGITUDINAL_DIST                      (3.28)
#define   HALF_VEHICLE_WIDTH                             (0.902)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0419841)
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.041873)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.041873)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0419841)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0419841)

#define REAR_WHEEL_SPAN         (1.569)
//1.几何车车身参数,单位/m
#define VEHICLE_LEN				(4.736)
#define VEHICLE_WID				(1.804)
#define FRONT_EDGE2CENTER		(3.676)
#define REAR_EDGE2CENTER		(1.06)
#define SIDE_EDGE2CENTER		(0.902)
#define WHEEL_BASE              AXIS_DISTANCE

//jiao dian zxy....
#define CHAMFER_LENGTH (0.4)
#define FRONT_CHAMFER_L1 (0.4)
#define FRONT_CHAMFER_L2 (0.4)
#define REAR_CHAMFER_L1 (0.2)
#define REAR_CHAMFER_L2 (0.2)
#define CHAMFER_ANGLE (45*PI/180)
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2)
#define REAR_VIEW_LENGTH_WIDTH  (0.25)
#define REAR_VIEW_2_FRONT_AXLE  (0.75)
//2.超声安装参数
#define SIDE_LURF_CENTER_X				(2.98)			//前侧边超声到车辆后轴中心点的X距离//?
#define SIDE_LUR_CENTERY				(0.902)			//两侧边超声到车辆后轴中心点的Y距离//?
#define SIDE_LURR_CENTERX				(0.35)             //后侧边超声到车辆后轴中心点的X距离//?
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值//?
#define FRONT_USS_TO_FRONT_AXLE_DIST    (FRONT_EDGE2CENTER-AXIS_DISTANCE)
//实际车辆的超声安装位置
#define SENSOR1_X     (FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE-0.14  )
#define SENSOR1_Y     ( 0.68f )
#define SENSOR1_ANGLE ( 0 )

#define SENSOR2_X     ( FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE )
#define SENSOR2_Y     ( 0.32f )
#define SENSOR2_ANGLE ( 0 )

#define SENSOR3_X     ( FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE )
#define SENSOR3_Y     (-0.32f )
#define SENSOR3_ANGLE ( 0 )

#define SENSOR4_X     ( FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE-0.14  )
#define SENSOR4_Y     (-0.68f )
#define SENSOR4_ANGLE ( 0 )

#define SENSOR5_X     (-REAR_EDGE2CENTER+0.05 )
#define SENSOR5_Y     ( 0.65f )
#define SENSOR5_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR6_X     (-REAR_EDGE2CENTER )
#define SENSOR6_Y     ( 0.32f )
#define SENSOR6_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR7_X     (-REAR_EDGE2CENTER )
#define SENSOR7_Y     (-0.32f )
#define SENSOR7_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR8_X     (-REAR_EDGE2CENTER+0.05 )
#define SENSOR8_Y     (-0.65f )
#define SENSOR8_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR9_X     ( SIDE_LURF_CENTER_X )
#define SENSOR9_Y     ( SIDE_EDGE2CENTER )
#define SENSOR9_ANGLE (1.5707963267948966192313216916398 )

#define SENSOR10_X     ( SIDE_LURF_CENTER_X )
#define SENSOR10_Y     (-SIDE_EDGE2CENTER )
#define SENSOR10_ANGLE (SENSOR9_ANGLE+PI )

#define SENSOR11_X     (-SIDE_LURR_CENTERX - 0.17)
#define SENSOR11_Y     ( SIDE_EDGE2CENTER)
#define SENSOR11_ANGLE (SENSOR9_ANGLE + 10*PI/180)// *pi/3 rad, 120 degree
#define COS_SENSOR11_DIS_ANGEL (0.8660254)

#define SENSOR12_X     (-SIDE_LURR_CENTERX - 0.22)
#define SENSOR12_Y     (-SIDE_EDGE2CENTER)
#define SENSOR12_ANGLE (SENSOR10_ANGLE - 10*PI/180)
#define COS_SENSOR12_DIS_ANGEL (0.8660254)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (4096)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (500)
#define MAX_STEERING_ANGLE_SPEED (500)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.15)
#define DANGOUS_VELOCITY (0.3)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值



#define ADDIDEGREE              (0)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)



//8.方向盘转角
static float EPSSteeringAngleRatio[12][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 0.184634, 0.185021, 0.182670, 0.179676},
    {200*PI/180, 0.212748, 0.217784, 0.212386, 0.209240},
    {230*PI/180, 0.250478, 0.244696, 0.246425, 0.247665},
    {260*PI/180, 0.280327, 0.278523, 0.276771, 0.278804},
    {290*PI/180, 0.313974, 0.312112, 0.311415, 0.306215},
    {320*PI/180, 0.350601, 0.346322, 0.349117, 0.342063},
    {350*PI/180, 0.378828, 0.375213, 0.383647, 0.375761},
    {380*PI/180, 0.413450, 0.418469, 0.416581, 0.409129},
    {410*PI/180, 0.454414, 0.451929, 0.454129, 0.443258},
    {440*PI/180, 0.493715, 0.484880, 0.491892, 0.479622},
    {470*PI/180, 0.530489, 0.527471, 0.531915, 0.523556},
    {500*PI/180, 0.575699, 0.568105, 0.571013, 0.563528},
};
static int EPS_STEERING_ANGLE_RATIO_SEGMENT=12;

static float JiHeARadius[12][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 14.457, 14.426, 14.616, 14.865}, //0  LF = 12.53 LR = 12.57  RF = 13.01  RR = 13.06
    {200*PI/180, 12.499, 12.201, 12.521, 12.715}, //1  LF = 10.96 LR = 11.05  RF = 11.09  RR = 11.20
    {230*PI/180, 10.553, 10.813, 10.734, 10.678}, //2  LF = 9.48  LR = 9.48   RF = 9.55   RR = 9.44
    {260*PI/180, 9.378, 9.442, 9.505, 9.432}, //3  LF = 8.24  LR = 8.56   RF = 8.15   RR = 8.54
    {290*PI/180, 8.315, 8.368, 8.388, 8.54}, //4  LF = 7.37  LR = 7.60   RF = 7.46   RR = 7.59
    {320*PI/180, 7.3829, 7.482, 7.417, 7.583}, //5  LF = 6.49  LR = 6.645  RF = 6.49   RR = 6.605
    {350*PI/180, 6.3783, 6.855,  6.689 , 6.844}, //6  LF = 5.8   LR = 6.075  RF = 5.855  RR = 6.00
    {380*PI/180, 6.154, 6.071,  6.102,  6.227}, //7  LF = 5.29  LR = 5.56   RF = 5.325  RR = 5.555
    {410*PI/180, 5.527 , 5.562, 5.531 , 5.687}, //8  LF = 4.815 LR = 5.065  RF = 4.885  RR = 5.09
    {440*PI/180, 5.017 , 5.125, 5.039, 5.191}, //9  LF = 4.41  LR = 4.695  RF = 4.465  RR = 4.675
    {470*PI/180, 4.603, 4.635, 4.588, 4.677}, //10 LF = 4.055 LR = 4.315  RF = 4.075  RR = 4.295
    {500*PI/180, 4.16, 4.23, 4.203,  4.273}  //11 LF = 3.86  LR = 3.99   RF = 3.73   RR = 3.945
};

//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 35, 35, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     35, 35, 35, 35,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     35, 35, 35, 35,     -1, -1, -1, -1,     30, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     35, 35, 35, 35,    -1, 30, -1, -1,//backward
    }, //250
    {400,
     35, 35, 35, 35,     -1, -1, -1, -1,     30, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    35, 35, 35, 35,     -1, 25, -1, -1,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif



#ifdef  CHERY_EQ

#define AHEAD_CTRL_POINT_NUM (3)

#define   WIDTH_HALF                                     (0.835)//yrm
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   MIDDLE_TO_REAR_FREESPACE                       (1.15)// 车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.15)//yrm
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.5)//yrm
#define   SENSORS_LONGITUDINAL_DIST                      (0.27 + AXIS_DISTANCE)//
#define   HALF_VEHICLE_WIDTH                             (0.835)//yrm
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.020058252)//待测量
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.020058252)//待测量
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.018900474)//待测量
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.018900474)//待测量
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE>>1\
                                                       +RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE>>1  )//待测量

#define REAR_WHEEL_SPAN         (1.43)//yrm
//1.奇瑞车车身参数,单位/m
#define VEHICLE_LEN				(3.2)//yrm
#define VEHICLE_WID				(1.67)//yrm
#define REAR_EDGE2CENTER		(0.45)//yrm
#define FRONT_EDGE2CENTER		(VEHICLE_LEN - REAR_EDGE2CENTER)//yrm
#define SIDE_EDGE2CENTER		(WIDTH_HALF)//yrm
#define WHEEL_BASE              AXIS_DISTANCE//yrm
//jiao dian zxy....
#define CHAMFER_LENGTH (0.)//
#define FRONT_CHAMFER_L1 (0.)//
#define FRONT_CHAMFER_L2 (0.)//
#define REAR_CHAMFER_L1 (0.)//
#define REAR_CHAMFER_L2 (0.)//
#define CHAMFER_ANGLE (45*PI/180)//
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2)//
#define REAR_VIEW_LENGTH_WIDTH  (0.1)//
#define REAR_VIEW_2_FRONT_AXLE  (0.7)//
//2.超声安装参数
#define SIDE_LURF_CENTER_X				(0.27 + AXIS_DISTANCE)  //yrm 前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(WIDTH_HALF)			//yrm
#define SIDE_LURR_CENTERX				(0.3)			//yrm 后侧边超声到车辆后轴中心点的X距离
#define FURTHEST_SIDE_DISTANCE			(999)			//yrm 如果探测的距离是0，就赋此值

//实际车辆的超声安装位置
#define SENSOR1_X     ( 2.7f )//yrm
#define SENSOR1_Y     ( 0.75f )//yrm
#define SENSOR1_ANGLE ( 0 )//yrm

#define SENSOR2_X     ( 2.63f )//yrm
#define SENSOR2_Y     ( 0.3f )//yrm
#define SENSOR2_ANGLE ( 0 )//yrm

#define SENSOR3_X     ( 2.63f )//yrm
#define SENSOR3_Y     (-0.28f )//yrm
#define SENSOR3_ANGLE ( 0 )//yrm

#define SENSOR4_X     ( 2.7f )//yrm
#define SENSOR4_Y     (-0.75f )//yrm
#define SENSOR4_ANGLE ( 0 )//yrm

#define SENSOR5_X     (-0.45f )//yrm
#define SENSOR5_Y     ( 0.72f )//yrm
#define SENSOR5_ANGLE ( 3.1415926535897932384626433832795 )//yrm

#define SENSOR6_X     (-0.45f )//yrm
#define SENSOR6_Y     ( 0.3f )//yrm
#define SENSOR6_ANGLE ( 3.1415926535897932384626433832795 )//yrm

#define SENSOR7_X     (-0.45f )//yrm
#define SENSOR7_Y     (-0.3f )//yrm
#define SENSOR7_ANGLE ( 3.1415926535897932384626433832795 )//yrm

#define SENSOR8_X     (-0.45f )//yrm
#define SENSOR8_Y     (-0.72f )//yrm
#define SENSOR8_ANGLE ( 3.1415926535897932384626433832795 )//yrm

#define SENSOR9_X     ( SIDE_LURF_CENTER_X )//yrm
#define SENSOR9_Y     ( SIDE_LUR_CENTERY )//yrm
#define SENSOR9_ANGLE (1.5707963267948966192313216916398 )//yrm

#define SENSOR10_X     ( SIDE_LURF_CENTER_X )//yrm
#define SENSOR10_Y     (-SIDE_LUR_CENTERY )//yrm
#define SENSOR10_ANGLE (4.7123889803846898576939650749193 )//yrm

#define SENSOR11_X     (-SIDE_LURR_CENTERX)//yrm
#define SENSOR11_Y     ( SIDE_LUR_CENTERY)//yrm
#define SENSOR11_ANGLE (1.5707963267948966192313216916398 )//yrm

#define SENSOR12_X     (-SIDE_LURR_CENTERX)//yrm
#define SENSOR12_Y     (-SIDE_LUR_CENTERY)//yrm
#define SENSOR12_ANGLE (4.7123889803846898576939650749193 )//yrm


#define COS_SENSOR11_DIS_ANGEL (0.999)//!!!!

#define COS_SENSOR12_DIS_ANGEL (0.999)

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (255)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (500)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.5)
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.2)
#define DANGOUS_VELOCITY (0.2)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值



#define ADDIDEGREE              (2)


//7.USS state
#define PROXIMITY (16)
#define NORMAL    (0)




//8.方向盘转角
static float EPSSteeringAngleRatio[12][5]=
{
    {170*PI/180, 0.169933, 0.169403, 0.163777, 0.163161},
    {200*PI/180, 0.193708, 0.192169, 0.191493, 0.189657},
    {230*PI/180, 0.223021, 0.223021, 0.221439, 0.223934},
    {260*PI/180, 0.255232, 0.246078, 0.257928, 0.246631},
    {290*PI/180, 0.283846, 0.275691, 0.280600, 0.276036},
    {320*PI/180, 0.319900, 0.312921, 0.319900, 0.314694},
    {350*PI/180, 0.354986, 0.340153, 0.351921, 0.344079},
    {380*PI/180, 0.386035, 0.368980, 0.383740, 0.369283},
    {410*PI/180, 0.419957, 0.401432, 0.414610, 0.399664},
    {440*PI/180, 0.453621, 0.429432, 0.448757, 0.431050},
    {470*PI/180, 0.487522, 0.462256, 0.485489, 0.464113},
    {500*PI/180, 0.508198, 0.494240, 0.522891, 0.498991}

//    //SteeringAngle LeftForward  LeftBack RightForward RightRear
//    {170*PI/180, 0.1928, 0.1928, 0.1954, 0.1928}, //0  LF = 12.53 LR = 12.57  RF = 13.01  RR = 13.06
//    {200*PI/180, 0.2217, 0.2196, 0.2271, 0.2278}, //1  LF = 10.96 LR = 11.05  RF = 11.09  RR = 11.20
//    {230*PI/180, 0.2558, 0.2575, 0.2614, 0.2607}, //2  LF = 9.48  LR = 9.48   RF = 9.55   RR = 9.44
//    {260*PI/180, 0.2886, 0.2894, 0.2987, 0.2981}, //3  LF = 8.24  LR = 8.56   RF = 8.15   RR = 8.54
//    {290*PI/180, 0.3296, 0.3263, 0.3325, 0.3311}, //4  LF = 7.37  LR = 7.60   RF = 7.46   RR = 7.59
//    {320*PI/180, 0.3624, 0.3630, 0.3682, 0.3655}, //5  LF = 6.49  LR = 6.645  RF = 6.49   RR = 6.605
//    {350*PI/180, 0.4018, 0.4006, 0.4087, 0.4102}, //6  LF = 5.8   LR = 6.075  RF = 5.855  RR = 6.00
//    {380*PI/180, 0.4376, 0.4365, 0.4385, 0.4360}, //7  LF = 5.29  LR = 5.56   RF = 5.325  RR = 5.555
//    {410*PI/180, 0.4764, 0.4687, 0.4722, 0.4722}, //8  LF = 4.815 LR = 5.065  RF = 4.885  RR = 5.09
//    {440*PI/180, 0.5170, 0.5113, 0.5237, 0.5168}, //9  LF = 4.41  LR = 4.695  RF = 4.465  RR = 4.675
//    {470*PI/180, 0.5549, 0.5539, 0.5568, 0.5568}, //10 LF = 4.055 LR = 4.315  RF = 4.075  RR = 4.295
//    {500*PI/180, 0.6020, 0.5957, 0.605,  0.5983}  //11 LF = 3.86  LR = 3.99   RF = 3.73   RR = 3.945
};
static int EPS_STEERING_ANGLE_RATIO_SEGMENT=12;

static float CherryEQRadius[12][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 12.53, 12.57, 13.01, 13.06}, //0  LF = 12.53 LR = 12.57  RF = 13.01  RR = 13.06
    {200*PI/180, 10.96, 11.05, 11.09, 11.20}, //1  LF = 10.96 LR = 11.05  RF = 11.09  RR = 11.20
    {230*PI/180, 9.48, 9.48, 9.55, 9.44}, //2  LF = 9.48  LR = 9.48   RF = 9.55   RR = 9.44
    {260*PI/180, 8.24, 8.56, 8.15, 8.54}, //3  LF = 8.24  LR = 8.56   RF = 8.15   RR = 8.54
    {290*PI/180, 7.37, 7.60, 7.46, 7.59}, //4  LF = 7.37  LR = 7.60   RF = 7.46   RR = 7.59
    {320*PI/180, 6.49, 6.645, 6.49, 6.605}, //5  LF = 6.49  LR = 6.645  RF = 6.49   RR = 6.605
    {350*PI/180,  5.8 ,  6.075,  5.855 ,  6.00}, //6  LF = 5.8   LR = 6.075  RF = 5.855  RR = 6.00
    {380*PI/180, 5.29, 5.56,  5.325,  5.555}, //7  LF = 5.29  LR = 5.56   RF = 5.325  RR = 5.555
    {410*PI/180, 4.815 ,  5.065, 4.885 , 5.09}, //8  LF = 4.815 LR = 5.065  RF = 4.885  RR = 5.09
    {440*PI/180,  4.41 , 4.695, 4.465, 4.675}, //9  LF = 4.41  LR = 4.695  RF = 4.465  RR = 4.675
    {470*PI/180, 4.055, 4.315, 4.075, 4.295}, //10 LF = 4.055 LR = 4.315  RF = 4.075  RR = 4.295
    {500*PI/180, 3.86, 3.99, 3.73,  3.945}  //11 LF = 3.86  LR = 3.99   RF = 3.73   RR = 3.945
};

//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     35, 35, 35, 35,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     35, 35, 35, 35,     -1, -1, -1, -1,     25, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     25, 25, 25, 25,    -1, 25, 30, 35,//backward
    }, //250
    {400,
     35, 35, 35, 35,     -1, -1, -1, -1,     25, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    25, 25, 25, 25,     -1, 25, 30, 35,//backward
    }, //400
};


//10.GEAR

#define GEAR_P (3)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)

#endif


#ifdef  VEL_CHANGAN
#define   WIDTH_HALF                                     (0.927)
#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离

#define   AXIS_DISTANCE                                  (2.65)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.31)
#define   SENSORS_LONGITUDINAL_DIST                      (2.85)
#define   HALF_VEHICLE_WIDTH                             (0.927)
//#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.025406181)

#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.02362)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.02362)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.025406181)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.025406181)


#define REAR_WHEEL_SPAN                       (1.594)

#define WHEEL_BASE               AXIS_DISTANCE

//1.changan车车身参数,单位/m
#define VEHICLE_LEN				(4.445)
#define VEHICLE_WID				(1.855)
#define FRONT_EDGE2CENTER		(3.54)
#define REAR_EDGE2CENTER		(0.905)
#define SIDE_EDGE2CENTER		(0.9275)
#define WHEEL_BASE                AXIS_DISTANCE

//2.超声安装参数
#define SIDE_LURF_CENTER_X				(3.15)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.9)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(0.32)			//后侧边超声到车辆后轴中心点的X距离
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (3.6)
#define PLUS_COUNT          (255)

//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE                (430)

//4.环视参数
#define CAMERA_2_MID                (0.58)

#define ADDIDEGREE              (0)

//8.方向盘转角
static float EPSSteeringAngleRatio[11][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {190*PI/180, 0.206, 0.206, 0.2083, 0.2083}, //0
    {220*PI/180, 0.243, 0.243, 0.2401, 0.2401}, //1
    {250*PI/180, 0.273,  0.273, 0.2742, 0.2742}, //2
    {280*PI/180, 0.311, 0.311, 0.3083,  0.3083}, //3
    {310*PI/180, 0.3417, 0.3417, 0.3405, 0.3405}, //4
    {340*PI/180, 0.377, 0.377, 0.3759, 0.3759}, //5
    {370*PI/180, 0.4156, 0.4156, 0.4120, 0.4120}, //6
    {400*PI/180, 0.44925, 0.44925, 0.4478, 0.4478}, //7
    {430*PI/180, 0.4820, 0.4752, 0.4809, 0.4700}, //8
    {460*PI/180, 0.5200, 0.5113, 0.5196, 0.5117}, //9
    {490*PI/180, 0.5669, 0.5669, 0.5655, 0.5655}  //10
};
static int EPS_STEERING_ANGLE_RATIO_SEGMENT=11;

//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     25, 25, 25, 25,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     25, 25, 25, 25,     -1, -1, -1, -1,     40, -1, -1, 20,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,    -1, 40, 20, -1,//backward
    }, //250
    {400,
     25, 25, 25, 25,     -1, -1, -1, -1,     40, -1, -1, 20,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,    -1, 40, 20, -1,//backward1
    }, //400
};
#endif


#ifdef  VEL_DONGFENG_E70
#define   WIDTH_HALF                                     (0.86)
#define   MIDDLE_TO_REAR                                 (0)/*(1.28)*///车辆中心到后轴距离 待测量

#define   MIDDLE_TO_REAR_FREESPACE                       (1.28)//车辆中心到后轴距离
#define   AXIS_DISTANCE                                  (2.70)
#define   FRONT_USS_TO_FRONT_AXLE_DIST                   (0.33)//
#define   SENSORS_LONGITUDINAL_DIST                      (3.03)//
#define   HALF_VEHICLE_WIDTH                             (0.86)
#define   WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0205)//


#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0205)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0205)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0205)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0205)

#define REAR_WHEEL_SPAN                       (1.48)//待测量 NOT USE

//1.E70车车身参数,单位/m
#define VEHICLE_LEN				(4.680)
#define VEHICLE_WID				(1.720)
#define FRONT_EDGE2CENTER		(3.65)//
#define REAR_EDGE2CENTER		(1.03)//
#define SIDE_EDGE2CENTER		(0.86)//
#define WHEEL_BASE               AXIS_DISTANCE

//2.超声安装参数
#define SIDE_LURF_CENTER_X				(3.03)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(0.86)			//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(0.32)			//后侧边超声到车辆后轴中心点的X距离
#define FURTHEST_SIDE_DISTANCE			(999)			//如果探测的距离是0，就赋此值



//实际车辆的超声安装位置
#define SENSOR1_X     ( 3.56f )
#define SENSOR1_Y     ( 0.68f )
#define SENSOR1_ANGLE ( 0 )

#define SENSOR2_X     ( 3.76f )
#define SENSOR2_Y     ( 0.32f )
#define SENSOR2_ANGLE ( 0 )

#define SENSOR3_X     ( 3.76f )
#define SENSOR3_Y     (-0.32f )
#define SENSOR3_ANGLE ( 0 )

#define SENSOR4_X     ( 3.56f )
#define SENSOR4_Y     (-0.68f )
#define SENSOR4_ANGLE ( 0 )

#define SENSOR5_X     (-1.02f )
#define SENSOR5_Y     ( 0.69f )
#define SENSOR5_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR6_X     (-1.08f )
#define SENSOR6_Y     ( 0.32f )
#define SENSOR6_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR7_X     (-1.08f )
#define SENSOR7_Y     (-0.32f )
#define SENSOR7_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR8_X     (-1.02f )
#define SENSOR8_Y     (-0.69f )
#define SENSOR8_ANGLE ( 3.1415926535897932384626433832795 )

#define SENSOR9_X     ( 3.15f )
#define SENSOR9_Y     ( 0.90f )
#define SENSOR9_ANGLE (1.5707963267948966192313216916398 )

#define SENSOR10_X     ( 3.15f )
#define SENSOR10_Y     (-0.90f )
#define SENSOR10_ANGLE (4.7123889803846898576939650749193 )

#define SENSOR11_X     (-0.32f)
#define SENSOR11_Y     ( 0.90f)
#define SENSOR11_ANGLE (1.5707963267948966192313216916398 )

#define SENSOR12_X     (-0.32f)
#define SENSOR12_Y     (-0.90f)
#define SENSOR12_ANGLE (4.7123889803846898576939650749193 )

#define USS_K               (58)
#define USS_K2              (7.752)
#define SPEED_K             (1)
#define PLUS_COUNT          (0x3FF)

//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE                (560)
#define MAX_STEERING_ANGLE_SPEED            (400)                       //最大方向盘角速度
//4.环视参数
#define CAMERA_2_MID                (0.58)

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.15)
#define DANGOUS_VELOCITY (0.2)

//5.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (7)          //目标方向盘转角和实际方向盘转角最大允许差值

#define ADDIDEGREE              (0)

//8.方向盘转角
static float EPSSteeringAngleRatio[11][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {270*PI/180, 0.2619, 0.2634, 0.2556, 0.2614}, //0
    {300*PI/180, 0.2911, 0.2926, 0.2857, 0.2908}, //1
    {330*PI/180, 0.3218, 0.3220, 0.3175, 0.3215}, //2
    {360*PI/180, 0.3526, 0.3552, 0.3489, 0.3515}, //3
    {390*PI/180, 0.3845, 0.3829, 0.3809, 0.3827}, //4
    {420*PI/180, 0.4169, 0.4144, 0.4123, 0.4135}, //5
    {450*PI/180, 0.4491, 0.4449, 0.4453, 0.4439}, //6
    {480*PI/180, 0.4815, 0.4752, 0.4775, 0.4756}, //7
    {510*PI/180, 0.5148, 0.5082, 0.5117, 0.4968}, //8   RR4.98
    {540*PI/180, 0.5481, 0.5423, 0.5335, 0.5288}, //9   LF LR4.48  RF4.54 RR4.62
//    {570*PI/180, 0.5826, 0.5749, 0.5765, 0.5700}, //10
};
static int EPS_STEERING_ANGLE_RATIO_SEGMENT=10;

//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     25, 25, 25, 25,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     25, 25, 25, 25,     -1, -1, -1, -1,     40, -1, -1, 20,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,    -1, 40, 20, -1,//backward
    }, //250
    {400,
     25, 25, 25, 25,     -1, -1, -1, -1,     40, -1, -1, 20,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,    -1, 40, 20, -1,//backward1
    }, //400
};
#endif



#ifdef  CHERY_T1D


#define AHEAD_CTRL_POINT_NUM (1)
//#define   VisionSlotDelay                               (1)

#define   MIDDLE_TO_REAR                                 (0)//(1.3175)//车辆中心到后轴距离//????????????????????????????????????????

#define   MIDDLE_TO_REAR_FREESPACE                       (1.33)//车辆中心到后轴距离//
#define   AXIS_DISTANCE                                  (2.67)
#define   SENSORS_LONGITUDINAL_DIST                      (2.98)//
#define   FL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0234415)
#define   FR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0234415)
#define   RL_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0234846)
#define   RR_WHEEL_SPEED_PULSE_METERS_PER_PULSE             (0.0234846)

//1.t1d车车身参数,单位/m
#define REAR_WHEEL_SPAN         (1.570)
#define VEHICLE_LEN				(4.533)
#define VEHICLE_WID				(1.848)
#define REAR_EDGE2CENTER		(0.93)
#define FRONT_EDGE2CENTER		(VEHICLE_LEN - REAR_EDGE2CENTER)
#define WHEEL_BASE              AXIS_DISTANCE
#define WIDTH_HALF              (VEHICLE_WID / 2.)
#define HALF_VEHICLE_WIDTH      WIDTH_HALF

#define SIDE_EDGE2CENTER		WIDTH_HALF

//jiao dian zxy....
#define CHAMFER_LENGTH (0.4)//????????????????????????????????????????
#define FRONT_CHAMFER_L1 (0.4)//????????????????????????????????????????
#define FRONT_CHAMFER_L2 (0.4)//????????????????????????????????????????
#define REAR_CHAMFER_L1 (0.2)//????????????????????????????????????????
#define REAR_CHAMFER_L2 (0.2)//????????????????????????????????????????
#define CHAMFER_ANGLE (45*PI/180)//????????????????????????????????????????
/*后视镜长度*/
#define REAR_VIEW_LENGTH   (0.2)//????????????????????????????????????????
#define REAR_VIEW_LENGTH_WIDTH  (0.25)//????????????????????????????????????????
#define REAR_VIEW_2_FRONT_AXLE  (0.75)//????????????????????????????????????????
//2.超声安装参数
#define SIDE_LURF_CENTER_X				(3.15)			//前侧边超声到车辆后轴中心点的X距离
#define SIDE_LUR_CENTERY				(VEHICLE_WID / 2)//两侧边超声到车辆后轴中心点的Y距离
#define SIDE_LURR_CENTERX				(0.43)             //后侧边超声到车辆后轴中心点的X距离
#define FRONT_USS_TO_FRONT_AXLE_DIST    (FRONT_EDGE2CENTER-AXIS_DISTANCE)
//实际车辆的超声安装位置
#define SENSOR1_X     (FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE - 0.2 )
#define SENSOR1_Y     ( 0.72f )
#define SENSOR1_ANGLE ( PI / 4 )

#define SENSOR2_X     ( FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE - 0.05 )
#define SENSOR2_Y     ( 0.33f )
#define SENSOR2_ANGLE ( 0 )

#define SENSOR3_X     ( FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE - 0.05 )
#define SENSOR3_Y     (-0.33f )
#define SENSOR3_ANGLE ( 0 )

#define SENSOR4_X     ( FRONT_USS_TO_FRONT_AXLE_DIST+AXIS_DISTANCE - 0.2  )
#define SENSOR4_Y     (-0.72f )
#define SENSOR4_ANGLE ( - PI / 4 )

#define SENSOR5_X     (-REAR_EDGE2CENTER + 0.15 )
#define SENSOR5_Y     ( 0.74f )
#define SENSOR5_ANGLE ( 5 * PI / 6 )

#define SENSOR6_X     (-REAR_EDGE2CENTER + 0.08 )
#define SENSOR6_Y     ( 0.43f )
#define SENSOR6_ANGLE ( PI )

#define SENSOR7_X     (-REAR_EDGE2CENTER + 0.08 )
#define SENSOR7_Y     (-0.43f )
#define SENSOR7_ANGLE ( PI )

#define SENSOR8_X     (-REAR_EDGE2CENTER + 0.15 )
#define SENSOR8_Y     (-0.74f )
#define SENSOR8_ANGLE ( - 5 * PI / 6 )

#define SENSOR9_X     ( SIDE_LURF_CENTER_X )
#define SENSOR9_Y     ( SIDE_EDGE2CENTER )
#define SENSOR9_ANGLE ( PI / 2 )

#define SENSOR10_X     ( SIDE_LURF_CENTER_X )
#define SENSOR10_Y     (-SIDE_EDGE2CENTER )
#define SENSOR10_ANGLE (SENSOR9_ANGLE - PI )

#define SENSOR11_X     (-SIDE_LURR_CENTERX)//????????????????????????????????????????
#define SENSOR11_Y     ( SIDE_EDGE2CENTER)//????????????????????????????????????????
#define SENSOR11_ANGLE (SENSOR9_ANGLE)// *pi/3 rad, 120 degree//????????????????????????????????????????
#define COS_SENSOR11_DIS_ANGEL (0.8660254)//????????????????????????????????????????

#define SENSOR12_X     (-SIDE_LURR_CENTERX)//????????????????????????????????????????
#define SENSOR12_Y     (-SIDE_EDGE2CENTER)//????????????????????????????????????????
#define SENSOR12_ANGLE (SENSOR10_ANGLE)//????????????????????????????????????????
#define COS_SENSOR12_DIS_ANGEL (0.8660254)//????????????????????????????????????????

#define USS_K               (58)//????????????????????????????????????????
#define USS_K2              (7.752)//????????????????????????????????????????
#define SPEED_K             (1)//????????????????????????????????????????
#define PLUS_COUNT          (8192)


//3.规划参数
// 最大规划转向角(°)
#define MAX_STEERING_ANGLE              (460)
#define MAX_STEERING_ANGLE_SPEED (400)                       //最大方向盘角速度
#define MAX_FRONT_AXLE_ANGLE (0.6)//????????????????????????????????????????
//4.环视参数
#define CAMERA_2_MID                (0.58)//????????????????????????????????????????

//5.速度默认
#define VERY_DANGOUS_VELOCITY (0.2)
#define DANGOUS_VELOCITY (0.3)

//6.转向角误差
#define STEERING_WHEEL_ANGLE_ERROR (10)          //目标方向盘转角和实际方向盘转角最大允许差值



#define ADDIDEGREE              (0)


//7.USS state
#define PROXIMITY (16)//????????????????????????????????????????
#define NORMAL    (0)//????????????????????????????????????????



//8.方向盘转角//????????????????????????????????????????
static float EPSSteeringAngleRatio[11][5]=
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 0.182760, 0.182766, 0.188689, 0.185572},
    {200*PI/180, 0.218366, 0.217466, 0.217425, 0.217583},
    {230*PI/180, 0.249532, 0.251491, 0.255900, 0.257084},
    {260*PI/180, 0.279200, 0.279917, 0.291208, 0.288287},
    {290*PI/180, 0.312505, 0.309180, 0.321956, 0.323082},
    {320*PI/180, 0.350383, 0.349538, 0.352095, 0.352095},
    {350*PI/180, 0.385958, 0.383150, 0.390878, 0.384948},
    {380*PI/180, 0.420049, 0.415644, 0.422339, 0.416387},
    {410*PI/180, 0.454713, 0.447165, 0.464071, 0.451769},
    {440*PI/180, 0.494464, 0.487460, 0.500470, 0.493671},
    {470*PI/180, 0.527143, 0.518584, 0.540317, 0.527458}
};
static int EPS_STEERING_ANGLE_RATIO_SEGMENT=11;

static float T1DRadius[11][5]=//????????????????????????????????????????
{
    //SteeringAngle LeftForward  LeftBack RightForward RightRear
    {170*PI/180, 14.4463, 14.4458, 13.9819, 14.2224}, //0
    {200*PI/180, 12.0322, 12.0836, 12.086, 12.0769}, //1
    {230*PI/180, 10.477, 10.3919, 10.205, 10.1559}, //2
    {260*PI/180, 9.31324, 9.28812, 8.90804, 9.00361}, //3
    {290*PI/180, 8.26391, 8.3588, 8.00453, 7.9746}, //4
    {320*PI/180, 7.3058, 7.325, 7.2672, 7.2672}, //5
    {350*PI/180, 6.57089, 6.62416,  6.4793, 6.58997}, //6
    {380*PI/180,  5.97808, 6.04951,  5.94151, 6.03736}, //7
    {410*PI/180, 5.46145 , 5.56756, 5.33435 , 5.50244}, //8
    {440*PI/180, 4.95237 , 5.0365, 4.88195, 4.96179}, //9
    {470*PI/180, 4.58695, 4.6786, 4.45103, 4.58363} //10
};

//9.避障参数
static const short SteeringAngleMinDis[LIST_NUM][25]=
{
    //SteeringAngle sUss1 sUss2 sUss3 sUss4 sUss5 sUss6 sUss7 lUss8 lUss9 lUss10 lUss11

    // Use angle should mulitple 10
    // -1 means disable
    {0,
     25, 25, 25, 25,     -1, -1, -1, -1,     -1, -1, -1, -1,//forward
     -1, -1, -1, -1,     25, 25, 25, 25,     -1, -1, -1, -1,//backward
    }, //0
    {250,
     25, 25, 25, 25,     -1, -1, -1, -1,     25, -1, -1, -1,//forward  默认左转
     -1, -1, -1, -1,     25, 25, 25, 25,    -1, 25, -1, -1,//backward
    }, //250
    {400,
     25, 25, 25, 25,     -1, -1, -1, -1,     25, -1, -1, -1,//forward x,x,x,x,  x,x,x,x,  25, -1, -1, -1,超声安装位置调整之前
     -1, -1, -1, -1,    25, 25, 25, 25,     -1, 25, -1, -1,//backward
    }, //400
};


//10.GEAR
#define GEAR_NONE (0)
#define GEAR_P (1)
#define GEAR_R (2)
#define GEAR_N (3)
#define GEAR_D (4)
#endif







#endif // BASE_H
