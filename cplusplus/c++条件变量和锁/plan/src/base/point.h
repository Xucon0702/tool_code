#ifndef POINT 
#define POINT

//#define NO_SIDE 0
//#define RIGHT_SIDE 1
//#define LEFT_SIDE 2

enum SlotType
{
    UssOnly=0,
    VisionMix
};
typedef struct _pointSimple
{
    float x;
    float y;
    _pointSimple():x(0),y(0){};
}PointSimple;

typedef struct point {
    float x;
    float y;
    float yN;
    float level;
    float y_dist;
    bool isNUll;
    float distance;   //每个超声采样周期中，车辆前进的路程；
    float ego_car_angle;
//    point(float x_=0,float y_=0):x(x_),y(y_){}
    point():x(0),y(0),yN(0),level(0),y_dist(0),isNUll(0),distance(0),ego_car_angle(0){}
}Point;

typedef struct pointA {
    float x;
    float y;
    float data;
    float level;
    int num;
    int side;
    pointA():x(0),y(0),data(0),level(0),num(0),side(0){}
}PointA;

/*
typedef struct MvParkSlotData {
    Point p0;
    Point p1;
    Point p2;
    Point p3;
    float fLeftDistance;
    float fRightDistance;
    float fScore;
    int nConfidence;//置信度
    int nAvailableState;
    unsigned int nIndex;
    //SlotType tType;
    unsigned int tType;
    Point tLeftP;
    Point tRightP;
    Point tFarLeftP;
    Point tFarRightP;
    float fDeg;
    int nSide;
	unsigned int nDirection;										//车位方向 0:左 1:右 2:前 3:后			
}MvParkSlotData;
*/
//-----------------------------------





#endif
