#ifndef MATH_FORMULA_H
#define MATH_FORMULA_H
#include <math.h>
#include <stdlib.h>

#include "base.h"
#include "vehicle.h"

float get_slot_Deg(const PointSimple& tP0, const PointSimple& tP1,
                                    const PointSimple& tP2, const PointSimple& tP3,
                                    int  nSide);
float CalcAngle(const PointSimple &first, const PointSimple &cen, const PointSimple &second);

void TransferRadius2FrontAxleAngle();
//根据车辆方向盘转角/后轴中心点转弯半径/行驶方向，计算车辆四个角点的转弯半径
int CalCornerPointRadius(int nSlotPosition, int nSteeringDirection, float fRearAxleCenterRadius, float &fRightRearPointRadius,
                           float &fRightFrontPointRadius,
                           float &fLeftFrontPointRadius, float &fLeftRearPointRadius);

int CalFourWheelRadius(int nSteeringDirection, float fRearAxleCenterRadius,
                           float &fRightRearPointRadius, float &fRightFrontPointRadius,
                           float &fLeftFrontPointRadius, float &fLeftRearPointRadius);
float CalFrontAxleCenterAngleOrSteeringAngle(int nSteeringDirection, int nRunningDirection, float fRequestAngle, int nRequestType);
//根据车辆方向盘转角/方向盘转向/行驶方向，计算后轴中心点转弯半径
float CalSteeringWheelAngle2Radius(int nSteeringDirection, int nRunningDirection, int fSteeringWheelAngle);

//根据车辆方向盘转角/行驶方向，计算后轴中心点规划最大转角对应的最小转弯半径
float GetMinRearAxleCenterRadius(int nSteeringDirection, int nRunningDirection);

//根据车辆方向盘转角/行驶方向，计算后轴中心点极限最小转弯半径
float GetMinLimitRearAxleCenterRadius(int nSteeringDirection, int nRunningDirection);

//根据后轴中心点转弯半径/方向盘转向/行驶方向，计算车辆方向盘转角
int CalRadius2SteeringWheelAngle(int nSteeringDirection, int nRunningDirection, float fRadius);

//根据当前车辆位置姿态/方向盘转动方向/后轴中心点转弯半径，计算车辆瞬时转动中心坐标
int CalRotationCenterCoordinate(int nSlotPosition, int nSteeringDirection, LocationPoint tVehicleRearAxleCenter,
                                         float fRearAxleCenterRadius, LocationPoint& tRotationCenter);

//根据车辆位置姿态计算12个超声波的探头位置
void CalUSSCoordinate(LocationPoint tVehicleRearAxleCenter, float fXUssPos[], float fYUssPos[]);

//根据车辆位置姿态计算四个角点坐标
void CalCornerCoordinate( LocationPoint tVehicleRearAxleCenter, LocationPoint& tRightRearCornerPoint, LocationPoint& tRightFrontCornerPoint,
                      LocationPoint& tLeftFrontCornerPoint, LocationPoint& tLeftRearCornerPoint);

void CalCornerCoordinateChangeSize(LocationPoint tVehicleRearAxleCenter, LocationPoint& tRightRearCornerPoint, LocationPoint& tRightFrontCornerPoint,
                      LocationPoint& tLeftFrontCornerPoint, LocationPoint& tLeftRearCornerPoint, float SmallSize, float SmallSizeL);

void RadiusCalibrationByFourWheelPulse(int FLCnt, int FRCnt, int RLCnt, int RRCnt, int nSteeringDirection, float &RearAxleCenterRadius,
                                       float &RearAxleCenterRadiusByRearAxle, float &RearAxisWheelSpan, float &FrontAxisWheelSpan);

void RadiusCalibrationByRearWheelPulse(int RLCnt,
                                       int RRCnt,
                                       int nSteeringDirection,
                                       float &RearAxleCenterRadiusByRearAxle);

void RadiusCalibrationByFrontWheelPulse(int FLCnt, int FRCnt, int nSteeringDirection,
                                        float &RearAxleCenterRadius);

void RadiusCalibrationByFrontRearWheelPulse(int FLCnt, int FRCnt, int RLCnt,
                                        int RRCnt, int nSteeringDirection,
                                        float &RearAxleCenterRadius);

void RadiusCalibrationByFrontRearWheelPulse2(int FLCnt, int FRCnt, int RLCnt,
                                        int RRCnt, int nSteeringDirection,
                                        float &RearAxleCenterRadius);

#if 0
bool CalAverageRadius(int FLCnt, int FRCnt, int RLCnt,
                       int RRCnt, int nSteeringDirection,
                       float &RearAxleCenterRadius);
#endif
LocationPoint TransFromWorld2Veh(const LocationPoint& tCurVhPose,
		 									const LocationPoint& tSrcPoint);

void CalCornerCoordinatePlus( LocationPoint tVehicleRearAxleCenter,
                              float fRearEdge2Center,
                              float fVehLen,
                              float fVehWid,
                              LocationPoint& tRightRearCornerPoint,
                              LocationPoint& tRightFrontCornerPoint,
                              LocationPoint& tLeftFrontCornerPoint,
                              LocationPoint& tLeftRearCornerPoint);
//计算一点绕一点旋转一定角度后的坐标
void RotateCoordinateOfPoint(LocationPoint tRotationCenter, LocationPoint tRotationPoint, float fRotationYaw, LocationPoint& tRotationNewPoint);

//返回数字符号的整数
int SgnCus(float fNum);

//计算点和直线距离
float DistanceBetweenLineAndPoint(float fA, float fB, float fC, float fX, float fY);

//计算点和点距离
float DistanceBetweenPointAndPoint(float fX1, float fY1, float fX2, float fY2);

//车辆后轴中心点的实时坐标计算
//    VehCoordinateRealTime CalVehRearAxleCenterRealTime(VehicleMotion *tVeh, float fCurrentDis, int Gear, int nPulseDirection);//0+ ; 1- ; 2STOP
void CalVehRearAxleCenterRealTime(int swa, float TireFL, float TireFR, float TireRL, float TireRR, int nGear, int nPulseDirection,
                                           float &current_angle, float &current_x, float &current_y);
void CalVehRearAxleCenterRealTime(float TireFL,float TireFR,float TireRL,float TireRR,int nGear,int nPulseDirection,
                                  float &current_angle, float &current_x, float &current_y,
                                  float &f_ego_car_motion_line_length);

int CalVehRearAxleCenterRealTimeInParkingProcess(int nSlotPosition, int nActualSteeringAngle, float fActualVelocity,
                                 float TireFL, float TireFR,
                                 float TireRL, float TireRR, int nGear, int nPulseDirection,
                                 float &current_angle, float &current_x, float &current_y);

int CalVehRearAxleCenterRealTimeInParkingProcessAnother(int nSlotPosition, int nActualSteeringAngle, float fActualVelocity,
                                 float TireFL, float TireFR,
                                 float TireRL, float TireRR, int nGear, int nPulseDirection,
                                 float &current_angle, float &current_x, float &current_y);

// 建立车位大地坐标系，并给出车辆在大地坐标系下后轴中心的坐标
void CalVehicleInitPositionBasedOnGroundCoordinateSystem(int nSlotPosition,
                                                         LocationPoint p0,
                                                         LocationPoint p1,
                                                         LocationPoint &VehRearAxleCenterPosition);
void CalUpdatedVehiclePositionBasedOnGroundCoordinateSystem(int nSlotPosition,
                                                         LocationPoint p0,
                                                         LocationPoint p1,
                                                         LocationPoint &CurrentVehRearAxleCenterPosition);

//建立车辆坐标系，并给出车位四个角点p0,p1,p2,p3相对于车辆坐标系的坐标
void CalSlotCoordinateBasedOnVehicleCoordinateSystem(int nSlotPosition,
                                                     float fParkingDepth, float fFrontEdge, float fRearEdge,
                                                     LocationPoint tVeh,
                                                     LocationPoint &p0,
                                                     LocationPoint &p1,
                                                     LocationPoint &p2,
                                                     LocationPoint &p3);
int CalVehRearAxleCenterRealTimeSimulation(int nActualSteeringAngle, float fActualVelocity,
                                           int nRunningDirectionRealTime,
                                           float &current_angle, float &current_x, float &current_y);

int round_double(double number);

int CalPointsOfLine(std::pair<int, int> tP0, std::pair<int, int> tP1, std::vector<std::pair<int, int>>& vLine);

float mod2pi(float x);
float mod2twopi(float x);


bool pointInPolygon(float x, float y, int polySides, float* polyX, float* polyY);//点是否在多边形内

bool intersection(float l1x1, float l1y1, float l1x2, float l1y2,
                  float l2x1, float l2y1, float l2x2, float l2y2);//判断2个线段相交

bool lineCrossPolygon(float lx1, float ly1,float lx2, float ly2, int polySides, float* polyX, float* polyY);//线段和多边形是否相交

Line GetLineThetaOfManyPoints(LocationPoint *p);

void CalFrontAxleCenterCoordinate( LocationPoint tVehicleRearAxleCenter, LocationPoint& tFrontAxleCenter);


/**
 * @brief 世界坐标转车身坐标
 */
void CoordinateTransformationFromWorldToBodyPos(const LocationPoint &body, LocationPoint &object);

/**
 * @brief 车身坐标转世界坐标
 */
void CoordinateTransformationFromBodyToWorldPos(const LocationPoint &body, LocationPoint &object);

/**
 * @brief 车身坐标转世界坐标PointSimple
 */
PointSimple CoordTransFromBodyToWorld(const LocationPoint &body, PointSimple &object);

LocationPoint TransFromVeh2World(const LocationPoint &body, LocationPoint &object);

#endif // MATH_FORMULA_H
