#ifndef VEHICLE_H
#define VEHICLE_H
// #include "UltraSlotSearch/point.h"
#include "point.h"
#include "vector_2d.h"
// #include "./UltrasonicLocation/utils/Ultrasonic.h"
#include "base.h"
#include "vector"
#if PLATFORM==PC_SYSTEM
#include <QTime>
#include <QString>
#endif

#include <stdint.h>
#include "AlgCommonStructDef.h" //zzn for MvPoint (float xy )^

#define MAX(a,b) (((a) > (b))  ? (a):(b))
#define MIN(a,b) (((a) < (b))  ? (a):(b))

//#define max(a,b) (((a) > (b))  ? (a):(b))
//#define min(a,b) (((a) < (b))  ? (a):(b))


/*****************************************************************************************/
/*
@brief
filter:SlotToVehDis
超声/视觉一致
*/
#define STANDARD_VETICLE_SLOT_LENGTH    			(2.5)

#define	Vertical_FrontSlotToVehDis_Calib			(6.0)

#define	Parallel_FrontSlotToVehDis_Calib			(6.0)

#define	Oblique_FrontSlotToVehDis_Calib				(6.0)

#define	Vertical_RearSlotToVehDis_Calib				(STANDARD_VETICLE_SLOT_LENGTH*3+1.5)

#define	Parallel_RearSlotToVehDis_Calib				(STANDARD_VETICLE_SLOT_LENGTH*3+1.5)

#define	Oblique_RearSlotToVehDis_Calib				(STANDARD_VETICLE_SLOT_LENGTH*3+1.5)
/*****************************************************************************************/


typedef short Int16;
typedef unsigned short UInt16;
typedef unsigned char byte;
//#define PI (3.1415926)
//#define WHEEL_PLUS_2_SPEED (39.3605)
//#define WHEEL_PLUS_2_METER (0.021654)
//#define EPS_FAILED (0x1)
//#define ESP_FAILED (0x2)
//#define EMS_FAILED (0x4)
//#define EPS2_FAILED (0x8)

//enum ParkingStatus
//{
//    WaitDownEdge,
//    WaitUpEdge,
//    ClauclateParkingState
//};

//enum UltrasonicStatus
//{
//    Normal,
//    BlindZone,
//    OverDetection,
//    rsv
//};

enum WHEEL_SPEED_DIRECTION
{
    StandStill=0,
    Forward,
    Backward,
    Invalid
};


enum SlotSide
{
    SideP0P1 = 0,
    SideP1P2 = 1,
    SideP2P3 = 2,
    SideP3P0 = 3,
    SlotSideNum = 4
};


typedef struct _Line//line of a obstacle:a*x+b*y+c=0;
{
    float a;
    float b;//k=-a/b
    float c;
    float r;//closer to 1, better
    _Line() {
        a=.0;
        b=.0;
        b=.0;
        r=-1.0;
    }
    _Line(float a1, float b1, float c1,float r1):a(a1), b(b1), c(c1),r(r1){}
}Line;

struct LineFunc
{
    PointSimple endpnt0;
    PointSimple endpnt1;

    //linear function: AX + BY + C =0
    float a;
    float b;
    float c;
    float d;  //拟合效果： 1：最好；0：最差；

    LineFunc(PointSimple ep0, PointSimple ep1, float _a, float _b, float _c, float _d) :
        endpnt0(ep0), endpnt1(ep1), a(_a), b(_b), c(_c), d(_d) {}
    LineFunc() :LineFunc(PointSimple(), PointSimple(), 0.0, 0.0, 0.0, 0.0) {}
    LineFunc(const LineFunc&) = default;
    LineFunc& operator= (const LineFunc&) = default;
};

#if 0
typedef enum _slotsideType  //边的属性 SlotLineType[0~4]:分别表示P0P1 P1P2 P2P3 P3P0; 属性 0：真实，1：虚拟，2：路沿，3：墙，4：绿化带，5：uncertain, 6: 开口；
{
    side_uncertain = 0,
    side_virtual = 1,
    side_curb = 2,
    side_wall = 3,
    side_treelawn = 4,
    side_exit = 5,
    side_pillar = 6,
}slotsideType;
#else
typedef enum _slotsideType  //边的属性 SlotLineType[0~4]:分别表示P0P1 P1P2 P2P3 P3P0;
{
    side_real = 0,      //0：真实
    side_virtual = 1,   //1：虚拟
    side_curb = 2,      //2：路沿
    side_wall = 3,      //3：墙
    side_treelawn = 4,  //4：绿化带
    side_entrance = 5,  //5：车位入口
    side_pillar = 6    //6：柱子
}slotsideType;

#endif

typedef enum _ObstByFreespace  //融合车位要过滤的freespace类型
{
    MV_PEOPLE_INSLOT = 0, //行人
    MV_NOMOTOR_INSLOT = 1, //非机动车二轮车
    MV_MOTOR_INSLOT = 2, //机动车  
    MV_PILLAR_INSLOT  = 3, //柱子  
    MV_GROUNDLOCK_INSLOT = 4, //地锁开 
    MV_NOSTOPSIGN_INSLOT = 5, //禁停牌子 
    MV_WARNINGPOLE_INSLOT = 6,//警示柱 
    MV_WARNINGCONE_INSLOT = 7, //警示锥 
    MV_OBSTYPENUMBER_INSLOT = 8
}ObstByFreespace;


struct NXPSTATES
{
    float LonACC;
    float LatACC;
    float yawRate;
    float speed;

    NXPSTATES():LonACC(0),LatACC(0),yawRate(0),speed(0){}
};



struct NXPCTRL
{
    float ACC;
    float torque;
    float speed;
    float distance;
    NXPCTRL():ACC(0),torque(0),speed(0),distance(0){}
};

typedef struct _TargetTrack
{
    Vector2d point;
    float    yaw;
    float    curvature;
    float    speed;
    int      nDetectStates;
}TargetTrack;

struct LocationPoint
{
    float x;
    float y;
    float yaw;
    LocationPoint():x(0),y(0),yaw(0){}
    LocationPoint(float a,float b,float c):x(a),y(b),yaw(c){}
};


typedef struct _CURRPOS
{
    unsigned long long lTimeTick;
    LocationPoint tPos;
    _CURRPOS():lTimeTick(0){}
}CURRPOS;


//VEH outPut参数
typedef struct _VehCtrlCmd
{
    short sEnable;
    short sGear;
    short sTarSteeringAngle;
    unsigned short uAngularSpeed;
    float fTargetDis;
    float fVelocity;
    short sSend;
    short sQuick;

    _VehCtrlCmd(): sEnable(0),sGear(0),sTarSteeringAngle(0),uAngularSpeed(0),
        fTargetDis(10),fVelocity(0),sSend(0){}

}VehCtrlCmd;

struct CtrlPoint {//定义点的数据结构
    float x;
    float y;
    float yaw;
    float kappa;
    float veh_speed;
    int driving_direction;
    CtrlPoint():x(0),y(0),yaw(0),kappa(0),veh_speed(0),driving_direction(0){}
};



//Struct for slot info and VEL start point for calculation
typedef struct _InputParkingIn
{
    float fVehCurX;
    float fVehCurY;
    float fVehCurYaw;
    float fSlotDepth;
    float fSlotWidth;
    float fSlotYaw;
    float fAisleObstacleY;
    float fDeltaYFrontLine;
    float fDeltaYRearLine;
    float fDeltaXRearEdge;//fXDeltaRearEdge
    float fDeltaXFrontEdge;//
    float fXSlotPosInGrid;
    float fYSlotPosInGrid;
    int8_t cSlotPosition;
    int8_t cSlotType;
    int8_t cParkType;
    int8_t cDetectType;
    uint8_t nGroundPinFlag;			//挡轮杆标志位;1-->车位内有挡轮杆(水平车位用)
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

    float m_fYawOffsetOfOpti;
    float m_GroundPinDepth;        //挡轮杆深度 
    float m_slotDepthOfFusion;	   //原m_slotAngle-->融合给的车位深度


    bool m_LastDriving;
    bool m_ParaHybridALastDriving;
    bool m_bStartPathplanningInParaSLot;
    bool m_bFirstPlan;
    bool m_bHitWheelBar;


    int m_nFirstPlanTotalSteps;
    int m_TotalGearSwitchTimes;
    int mParaSlotSteps;
    bool m_bNotNeedParkingOut;
    bool m_bSlotFrontEdgeObstacle;
    bool m_bSlotRearEdgeObstacle;

    LocationPoint m_lastPlanningPoint;//3D
//    LocationPoint m_GoalPoint;//3D

    ParkingWorkingStatus m_ParkingCtrlStatus;

//    LocationPoint m_StartPointOffset;
    LocationPoint m_goalPointOffset;

    bool m_bUSSHasDetectFrontMargin;
    bool m_bUSSHasDetectRearMargin;
//    bool m_bNarrowSlot;
    LocationPoint RotationCoordinate;

	int m_nCurrentStep;
    int m_nTotalStep;
	
	int	InputSendCnt;

    LocationPoint TarVehPoseInSlot;
    LocationPoint tFrontSlotCorner;
    LocationPoint tRearSlotCorner;
    int nAvoidStatus;
    slotsideType SlotLineType[SlotSideNum]; 

	char Reserved[256-184-10*4-4*4];//256-208 //前面是184个字节
	
    //std::vector<std::vector<CtrlPoint> > m_LastPlanningTargetCtrlPointsSet;
    
}InputParkingIn;


//Struct for slot info and VEL start point for calculation
typedef struct _InputParkingIn2
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
//    bool bGroundPoint;
//    float fGroundPointDis;


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
    bool m_bHitWheelBar;


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
    //==============================208======================
    unsigned int uSaveID;
    int nWarningFlag;
    LocationPoint tSaveP0;
    LocationPoint tSaveP1;
    //==============================32======================
    char Reserved[256-208-32];

}InputParkingIn2;


//VEH STATUS参数
typedef struct _ParkingInStates
{

    short sUSSTriLocActive;

    float fDetectedSlotWidth;  //车位宽度
    float fDetectedSlotYaw;  //车位夹角
    float fTargetY;          //车辆目标停止点纵坐标

    LocationPoint tObjCornerC; //车位坐标系下车位右侧障碍物
    LocationPoint tObjCornerD; //车位坐标系下车位左侧障碍物
    LocationPoint tCenter;
    LocationPoint tObjFront;   //车辆前进路径上的障碍物
    float fAisleObstacleY;     //车位坐标系下过道Y值

    int nStatus;    //状态位
    int nMeasureStatus;//发给感知模块，需要测量障碍物的类型
    int nCurrentStep;  //屏幕上显示的当前步数
    int nTotalStep;    //屏幕上显示的总步数
    int nPakingInFinishStatus;  //泊车完成状态：异常退出类型？泊车完成？

    _ParkingInStates(): sUSSTriLocActive(0),fDetectedSlotWidth(0.),fDetectedSlotYaw(0.),
        fTargetY(0.),fAisleObstacleY(0),
        nStatus(0),nMeasureStatus(0),nCurrentStep(0),nTotalStep(0),
        nPakingInFinishStatus(0){}

}ParkingInStates;


//
typedef struct _VehCoordinateRealTime
{
    LocationPoint tVehRearAxleCenterRealTime;
    LocationPoint tVehCornerERealTime;
    LocationPoint tVehCornerFRealTime;
    LocationPoint tVehCornerGRealTime;
    LocationPoint tVehCornerHRealTime;
    LocationPoint p0;
    LocationPoint p1;
    LocationPoint p2;
    LocationPoint p3;

}VehCoordinateRealTime;




struct SlotData
{
    LocationPoint p0;
    LocationPoint p1;
    LocationPoint p2;
    LocationPoint p3;


    LocationPoint CarP0;
    LocationPoint CarP1;
//    LocationPoint CarP2;
//    LocationPoint CarP3;

    LocationPoint leftObs;
    LocationPoint rightObs;

    float rightSide[30];
    float leftSide[30];
    int nSide;
};
//typedef struct point {
//    float x;
//    float y;
//}Point;


typedef struct VehicleInformation
{
    double Speed;
    double Speed_ms;
    double Displacement;
    short TargetSteeringWheelAngle;
    short ActualSteeringWheelAngle;
    unsigned short SteeringWheelAgularVelocity;
    double SteeringWheelTorque;
    double Brake;
    char ECU_status;
    char CommunicationStatus;
    double Yaw;
    double Last_Yaw;
    double X;
    double Y;
    double R;
    VehicleInformation():Displacement(0) {}
}VehInf;

typedef struct WheelSpeed
{
    unsigned char direction;
    unsigned char vaild;
    float data;

}WheelSpeed;

typedef struct SendCmd
{
    float fSteeringAngle;
    unsigned short usStopDis;
    float fSpeedLimit;
    float fAcc;
    unsigned short usTorque;
    unsigned short ucGearReq;
    unsigned short ucEnable;

}SendCmd;

typedef struct VehicleInformationCA
{
    float Speed;
    unsigned long long SpeedPlus;
    float Displacement;
    short TargetSteeringWheelAngle;
    short ActualSteeringWheelAngle;
    float ActualSteeringWheelAngleF;
    float SteeringAngleRateF;
    unsigned short SteeringWheelAgularVelocity;
    float SteeringTorque;
    float ACCBrake;
    unsigned char EPS_FAILED_Status;
    unsigned char ESP_FAILED_Status;
    unsigned char EMS_FAILED_Status;
    unsigned char EPS2_FAILED_Status;
    unsigned char torque_sensor_status;
    unsigned char torque_EMS;
    unsigned char apa_control_feedback;
    WheelSpeed RearLeft;
    WheelSpeed RearRight;
    WheelSpeed FrontLeft;
    WheelSpeed FrontRight;
    unsigned char uGear;
    unsigned char vehicle_speed_valid;
    unsigned short wheel_speed_direction;


    unsigned char FLWheelDirection;
    float FLWheelSpeedKPH;
    float FRWheelSpeedKPH;
    float RLWheelSpeedKPH;
    float RRWheelSpeedKPH;



    unsigned short wheel_speed_rear_right_pulse;
    unsigned short wheel_speed_rear_left_pulse;
    unsigned short wheel_speed_front_right_pulse;
    unsigned short wheel_speed_front_left_pulse;
    unsigned char wheel_speed_count;
    unsigned char steering_angle_valid;
    unsigned char steering_angle_count;
    unsigned char sas_failure_status;

    unsigned char YRS_YawRateSensorState;
    float YRS_YawRate;
    unsigned char YRS_LongitSensorState;
    float YRS_LongitAcce;

    unsigned char lat_acc;
    float long_acc;
    unsigned short yaw_acc;

    unsigned char abort_feedback_status;
    double Yaw;
    double Last_Yaw;
    double X;
    double Y;
    double R;
    bool bIsBreaking;
    bool bIsWheelBar;
    VehicleInformationCA():Displacement(0) {}
}VehInfCA;


typedef struct LIN_STP318_READDATA
{
    unsigned short TOF;
    unsigned short status;
    unsigned char count;
    unsigned short checkSum;
    LIN_STP318_READDATA():TOF(0),status(0),count(0),checkSum(0) {}
}LIN_STP318_ReadData;

typedef struct LIN_STP313_READDATA
{
    unsigned short TOF1;
    unsigned short Level;
    unsigned short Width;
    unsigned short TOF2;
    unsigned short status;
    unsigned char count;
    unsigned short checkSum;
    LIN_STP313_READDATA():TOF1(0),Level(0),Width(0),TOF2(0),status(0),count(0),checkSum(0) {}
}LIN_STP313_ReadData;


typedef struct TIMESTRUCT
{
    unsigned long long SystemTime;
    unsigned long long LastSystemTime;
    unsigned long long LastLin10Time;
    unsigned long long TimeFirst;
    unsigned long long TimeFirstLocal;
    unsigned long long TimeR;
    unsigned long long TimeErr;//时间间隔
    TIMESTRUCT():SystemTime(0),LastSystemTime(0),LastLin10Time(0),TimeErr(0) {}
}TimeStruct;

typedef struct RTK_IMFORMATION
{
    unsigned short Week;
    unsigned int Second;
    float Yaw;
    float Pitch;
    float Roll;
    unsigned int Latitude;
    unsigned int Longitude;
    unsigned int Height;
    float EastVelocity;
    float WestVelocity;
    float SkyVelocity;
    float BaseLineLenght;
    unsigned short AntennaNumber1;
    unsigned short AntennaNumber2;
    unsigned short Status;
}RTK_Imformation;




//Ultrasonic----------------------

typedef struct LURParas
{
    float  m_left_side_uss_front_data;//左前TOF1,9
    float  m_left_side_uss_front_level;//左前level,9

    float  m_right_side_uss_front_data;//右前TOF1,10
    float  m_right_side_uss_front_level;//右前level,10

    float  m_right_side_uss_rear_data;//右后TOF1,12
    float  m_right_side_uss_rear_level;//右后level,12

    float  m_left_side_uss_rear_data;//左后TOF1,11
    float  m_left_side_uss_rear_level;//左后level,11
    LURParas()
    {
        m_left_side_uss_front_data=0.0f;
        m_left_side_uss_front_level=0.0f;
        m_right_side_uss_front_data=0.0f;
        m_right_side_uss_front_level=0.0f;
        m_right_side_uss_rear_data=0.0f;
        m_right_side_uss_rear_level=0.0f;
        m_left_side_uss_rear_data=0.0f;
        m_left_side_uss_rear_level=0.0f;
    }
}LURParas;



// struct UltrasonicAxis
// {
//     double x;
//     double y;
//     float distance;
//     UltrasonicStatus state;
//     UltrasonicAxis():x(0),y(0),distance(100) {}
// };

// struct Distance
// {
//     double distance;
//     UltrasonicStatus state;
// };

// struct ParkingEdgeAxis
// {
//     UltrasonicAxis OutEdge;
//     UltrasonicAxis InnerEdge;
//     Int16 state;
// };

struct ParkingInformation
{
    double Length;
    double Width;
    LocationPoint ParkingPoint;
};

//环视数据--------------------------
typedef struct _MvImagePoint
{
    short x;
    short y;
}MvImagePoint;

typedef struct _MvWorldPoint
{
    short x;
    short y;
}MvWorldPoint;

#if 0
typedef struct _MvPoint
{
    float x;
    float y;
}MvPoint;
#endif
typedef struct _MvParkPoint
{
    MvImagePoint tImagePoint;
    MvWorldPoint tWorldPoint;
}MvParkPoint;

typedef struct _MvParkSlot
{
    unsigned int nIndex;
//    unsigned int nFollowNum;
    MvParkPoint tPoint0;
    MvParkPoint tPoint1;
    MvParkPoint tPoint2;
    MvParkPoint tPoint3;
    int nflag;//车位信息完整标记
    int nConfidence;//置信度
    unsigned uCount;
//    unsigned long long lTimeNow;
    unsigned int aReserved[7];
    _MvParkSlot():nflag(0) {}
}MvParkSlot;

typedef struct _MvParkingInfo
{
    int nParkSlotNum;
    unsigned long long lTimeFirst;
    unsigned uFrameId;
    unsigned long long lTimeNow;
    unsigned short uCount;
    MvParkSlot aSlot[16];
}MvParkingInfo;

typedef struct _MvFreeSpace
{
    MvWorldPoint tAvailablePoint[32];
}MvFreeSpace;

typedef struct _MvLine
{
    MvFloatPoint/*MvPoint*/ tStartP;
    MvFloatPoint/*MvPoint*/ tEndP;
}MvLine;

typedef struct _MvCurbInfo
{
    int nCurbNum;
    MvLine aLine[64];
}MvCurbInfo;

int sgn(double d);

typedef enum _BoundStatus
{
    EmptyMode = 0,
    UssDetectDefaultMode = 200,
    UssVerticalPushLeftRearPointMode = 211,
    UssVerticalPushRightRearPointMode = 212,
    UssVerticalCalLeftRightBoundMode = 214, //change slot, keep vehpose
    UssParallelCalBackBoundMode = 311,      //change slot, keep vehpose
    UssParallelCalFrontBoundMode = 312,      //change slot, keep vehpose
    DetectWheelBar = 666,
    ViewDynamicUpdatePosNearSlot = 2000,
    ViewDynamicUpdatePosInsideSlot = 3000,
    KeepMoving = 9999

}BoundStatus;

#if 1

typedef enum
{
    NotCut = 0,
    HadCut
}SlotSideCutEnum;

typedef struct MvParkSlotData {
    //视觉原始
    PointSimple p0;
    PointSimple p1;
    PointSimple p2;
    PointSimple p3;
    int nConfidence;//置信度
    int nAvailableState;
    int nDirection;
    unsigned int nIndex;
    float fAngle;//fDeg;
    unsigned int nGroundPinFlag;

    //融合增加
    unsigned int tType;
    int nSide;//左右
    float fUssDepth;//车位de超声深度信息
    float fWallScore;//墙的概率
    PointSimple tRearP;
    PointSimple tFrontP;
    float fUssPassageWidth[OBSTACLE_NAG_SIDE_RESOLUTION];//0.5m Per

    float fLeftDistance;
    float fRightDistance;
    float fScore;
    int bFusionIsValid; //1 is :normal（default） to decision
    bool bIsFusdeDepthSuccd;
	int nSlotSrcType;    //0-camera slot, 1-uss slot, 2-fusion slot

    int nMatchedUssSlotId;  //与之匹配的超声车位
    int nMatchedCamSlotId;  //与之匹配的视觉车位
    int bIsComposNoMatchedSlot;  //是复合车位未匹配车位
    int SlotLineType_counter[SlotSideNum]; 
    slotsideType SlotLineType[SlotSideNum]; 
    SlotSideCutEnum bFusionSlotsideCut[SlotSideNum];
	float sideCutOffset[SlotSideNum];
	
    LineFunc BottomMatchLine;   //车位底边超声点云拟合的直线方程
    int bLineExist;              //超声拟合的直线是否存在？
    int bNeedCutBottom;                //需不需要裁剪底边裁剪？
    int bIsMiniSlot;            //极小遥控车位
	int bFusionSlotExistInManage;
	int Obstacle_counter[MV_OBSTYPENUMBER_INSLOT];	 //车位内出现FREESPACE计数
	
    PointSimple UssSlotPnt[SlotSideNum]; //原始超声车位数据(所有的车位类型都会被赋值)

    int bDecisionIsValid; //1 is :normal（default） to show in HMI

	//挡轮杆坐标	
	PointSimple		tGroundWorldPoint0;					//第一个挡轮杆外侧世界坐标顶点
	PointSimple		tGroundWorldPoint1;					//第二个挡轮杆外侧世界坐标顶点
	
    int bParaslotDepthExtendflag;//非超声水平车位深度可扩展标志位;深度小于2.2置1 , 大于2.2置0
    
	int bObstInBottomCounter;   //暂未使用
	float DepthOfObstInBottom;  //水平车位:入口线到底部障碍物距离;未拟合出直线距离给0,超声点云拟合出给的是实际距离。
		
	unsigned char nReserved[480-396-4-24];		//400-->480
}MvParkSlotData;


#else
typedef struct MvParkSlotData {
    //视觉原始
    PointSimple p0;
    PointSimple p1;
    PointSimple p2;
    PointSimple p3;
    int nConfidence;//置信度
    int nAvailableState;
    int nDirection;
    unsigned int nIndex;
    float fAngle;//fDeg;
    unsigned int nGroundPinFlag;

    //融合增加
    unsigned int tType;
    int nSide;//左右
    float fUssDepth;//车位de超声深度信息
    float fWallScore;//墙的概率
    PointSimple tRearP;
    PointSimple tFrontP;
    float fUssPassageWidth[OBSTACLE_NAG_SIDE_RESOLUTION];//0.5m Per

    float fLeftDistance;
    float fRightDistance;
    float fScore;
    int bIsValid;
    bool bIsFusdeDepthSuccd;
	int nSlotSrcType;//0-camera slot, 1-uss slot
	//	  Point tFarLeftP;
	//	  Point tFarRightP;
	
	unsigned char nReserved[256-200];
}MvParkSlotData;
#endif


//车位闭环
typedef struct
{
    /*input*/
    unsigned int index_seq;  /*车位序号*/
    unsigned int slot_type;  /*车位类型 0：水平 1：垂直 2：斜列*/
    int slot_src_type;  /*类型 0：画线车位 1：超声车位 2：融合车位*/
    int slot_side;  /*左右车位 -1：左 1：右*/
    /*原始车位*/
    PointSimple p0;
    PointSimple p1;
    PointSimple p2;
    PointSimple p3;

    /*output*/
    bool fusion_slot_update;  /*融合车位更新标志位 0：不更新 1：更新*/
    LineFunc match_line[SlotSideNum];  /*车位边界超声点云拟合的直线/线段方程*/
    bool line_exist[SlotSideNum];  /*超声拟合的直线/线段是否存在*/
    bool had_cut[SlotSideNum];  /*车位边界是否被裁剪*/
    /*空间车位*/
    PointSimple parking_space_p0;
    PointSimple parking_space_p1;
    PointSimple parking_space_p2;
    PointSimple parking_space_p3;
    /*融合更新车位*/
    PointSimple fusion_slot_p0;
    PointSimple fusion_slot_p1;
    PointSimple fusion_slot_p2;
    PointSimple fusion_slot_p3;

    /*intput and output*/
    SlotSideCutEnum fusion_slotside_cut[SlotSideNum];  /*融合车位边界是否被裁剪*/
    slotsideType slot_line_type[SlotSideNum];  /*原始车位边界属性，空间车位和融合更新车位直接继承*/
    
	unsigned char reserved[320-284];
}MvUpdateSlot;

typedef struct
{
	uint32_t		isUpdated;				 //是否是更新了的数据
	MvUpdateSlot	closeLoopUpdateData;     //车位闭环更新结果
	PointSimple		mixSlotUpdateData[4];	 //车位融合更新结果											
}MvUpdateSlotData;

typedef struct ParkSizeFilter {
    float d1Min = 2.0f;//m
    float d2Min = 2.0f;//m
    float d3Min = 0.5f;//m
    float d3Max = 2.1f;
    float d4Min = 0.5f;
    float d5Min = 2.0f;
    float fPassageWidthMin = 4.0f;
    float fSlotLengthMin = VEHICLE_LEN + 0.8f;
    float fSlotLengthMax = VEHICLE_LEN + 3.0f;
    float fSlotDepthMin = VEHICLE_WID;
    float fSlotDepthMax = 3.0f;
    float fThetaMin = PI/6.0f; //45-->30°
    float fThetaMax = PI * 0.5f + 0.1f;
	float fThetaInSlot = PI / 12.;
    float fussSlotDepthMin = VEHICLE_WID - 0.2f;
	float fFrontSlotToVehDis = Vertical_FrontSlotToVehDis_Calib;
	float fRearSlotToVehDis	 = Vertical_RearSlotToVehDis_Calib;
}ParkSizeFilter;



struct UssPnt
{
	float 		x;
    float 		y;
    float 		z;
    uint32_t 	msg;

    UssPnt(float x_, float y_, float z_, uint32_t msg_): x(x_), y(y_), z(z_), msg(msg_){}
    UssPnt(float x_, float y_): x(x_), y(y_), z(0.f), msg(0U){}
    UssPnt():UssPnt(0.f, 0.f , 0.f, 0U){}
    UssPnt(const UssPnt&) = default;
    UssPnt& operator=(const UssPnt&) = default; 
    const float operator * (const UssPnt& p) const{ return (x*p.y - y*p.x); };   //定义向量叉乘
    const UssPnt operator - (const UssPnt& p) const{ return UssPnt(x-p.x, y-p.y, z, msg); };     //定义向量减法
};

struct UssPnts 
{
	std::vector<UssPnt> rear_pnts;
    std::vector<UssPnt> front_pnts;
    std::vector<UssPnt> right_pnts;
    std::vector<UssPnt> left_pnts;
};


#endif // VEHICLE_H
