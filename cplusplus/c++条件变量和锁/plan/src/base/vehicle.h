#ifndef VEHICLE_H
#define VEHICLE_H
// #include "UltraSlotSearch/point.h"
#include "point.h"
// #include "./UltrasonicLocation/utils/vector_2d.h"
#include "vector_2d.h"
#include "base.h"
#include "vector"
#include "Ultrasonic.h"

#if PLATFORM==PC_SYSTEM
#include <QTime>
#include <QString>
#endif


#define MAX(a,b) (((a) > (b))  ? (a):(b))
#define MIN(a,b) (((a) < (b))  ? (a):(b))

//#define max(a,b) (((a) > (b))  ? (a):(b))
//#define min(a,b) (((a) < (b))  ? (a):(b))

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

enum SlotSide
{
    SideP0P1 = 0,
    SideP1P2 = 1,
    SideP2P3 = 2,
    SideP3P0 = 3,
    SlotSideNum = 4
};

typedef enum _slotsideType  //边的属性 SlotLineType[0~4]:分别表示P0P1 P1P2 P2P3 P3P0; 属性 0：真实，1：虚拟，2：路沿，3：墙，4：绿化带，5：uncertain, 6: 开口；
{
    side_real = 0,
    side_virtual = 1,
    side_coast = 2,
    side_wall = 3,
    side_treelawn = 4,
    side_uncertain = 5,
    side_open = 6,
}slotsideType;

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

    float m_fYawOffsetOfOpti;
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



struct UltrasonicAxis
{
    double x;
    double y;
    float distance;
    UltrasonicStatus state;
    UltrasonicAxis():x(0),y(0),distance(100) {}
};

struct Distance
{
    double distance;
    UltrasonicStatus state;
};

struct ParkingEdgeAxis
{
    UltrasonicAxis OutEdge;
    UltrasonicAxis InnerEdge;
    Int16 state;
};

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

typedef struct _MvPoint
{
    float x;
    float y;
}MvPoint;

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
    MvPoint tStartP;
    MvPoint tEndP;
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
    float fThetaMin = PI * 0.25f;
    float fThetaMax = PI * 0.5f + 0.1f;
	float fThetaInSlot = PI / 12.;
}ParkSizeFilter;


#endif // VEHICLE_H
