/*
 * ultrasonic.cpp
 *
 *  Created on: January 8 2018
 *      Author: Guohua Zhu
 */
/*****************************************************************************/
/* FILE NAME: ultrasonic.cpp                      COPYRIGHT (c) Motovis 2018 */
/*                                                       All Rights Reserved */
/* DESCRIPTION: this module process the ultrasonic data				         */
/*****************************************************************************/
/* REV      AUTHOR        DATE              DESCRIPTION OF CHANGE            */
/* ---   -----------    ----------------    ---------------------            */
/* 1.0	 Guohua Zhu     January 8 2019      Initial Version                  */
/*****************************************************************************/

#include "Ultrasonic.h"

Ultrasonic::Ultrasonic() {
	Init();

	ScheduleTimeCnt.setContainer(this);
	ScheduleTimeCnt.getter(&Ultrasonic::getScheduleTimeCnt);
	ScheduleTimeCnt.setter(&Ultrasonic::setScheduleTimeCnt);

	ReadStage.setContainer(this);
	ReadStage.getter(&Ultrasonic::getReadStage);
	ReadStage.setter(&Ultrasonic::setReadStage);

	SystemTime.setContainer(this);
	SystemTime.getter(&Ultrasonic::getSystemTime);
	SystemTime.setter(&Ultrasonic::setSystemTime);

//	UltrasonicDatas.setContainer(this);
//	UltrasonicDatas.getter(&Ultrasonic::getUltrasonicDatas);

//	UltrasonicLocationDatas.setContainer(this);
//	UltrasonicLocationDatas.getter(&Ultrasonic::getUltrasonicLocationDatas);

	UltrasonicPacket.setContainer(this);
	UltrasonicPacket.getter(&Ultrasonic::getUltrasonicPacket);

	UltrasonicLocationPacket.setContainer(this);
	UltrasonicLocationPacket.getter(&Ultrasonic::getUltrasonicLocationPacket);

	AbstacleBodyPositionDirect.setContainer(this);
	AbstacleBodyPositionDirect.getter(&Ultrasonic::getAbstacleBodyPositionDirect);

	AbstacleBodyPositionTriangle.setContainer(this);
	AbstacleBodyPositionTriangle.getter(&Ultrasonic::getAbstacleBodyPositionTriangle);

	AbstacleGroundPositionTriangle.setContainer(this);
	AbstacleGroundPositionTriangle.getter(&Ultrasonic::getAbstacleGroundPositionTriangle);
}

Ultrasonic::~Ultrasonic() {

}

void Ultrasonic::Init(void)
{
	_system_time = 0;
	_schedule_time_cnt = 0;
	_read_stage = 0;
}

uint8_t Ultrasonic::getScheduleTimeCnt()             {return _schedule_time_cnt ;}
void    Ultrasonic::setScheduleTimeCnt(uint8_t value){_schedule_time_cnt = value;}

uint8_t Ultrasonic::getReadStage()             {return _read_stage ;}
void    Ultrasonic::setReadStage(uint8_t value){_read_stage = value;}

uint32_t Ultrasonic::getSystemTime()              {return _system_time ;}
void     Ultrasonic::setSystemTime(uint32_t value){_system_time = value;}

//LIN_RAM* Ultrasonic::getUltrasonicDatas(){return _ultrasonic_datas;}

//LIN_RAM* Ultrasonic::getUltrasonicLocationDatas(){return _ultrasonic_location_datas;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicPacket(){return _ultrasonic_packet;}

Ultrasonic_Packet* Ultrasonic::getUltrasonicLocationPacket(){return _ultrasonic_location_packet;}

ObstacleLocationPacket* Ultrasonic::getAbstacleBodyPositionDirect(){return _abstacle_body_position_direct;}

ObstacleLocationPacket* Ultrasonic::getAbstacleBodyPositionTriangle(){return _abstacle_body_position_triangle;}

ObstacleLocationPacket* Ultrasonic::getAbstacleGroundPositionTriangle(){return _abstacle_ground_position_triangle;}

void Ultrasonic::setUltrasonicPacket(uint8_t n,Ultrasonic_Packet p)
{
	_ultrasonic_packet[n] = p;
}

void Ultrasonic::setUltrasonicLocationPacket(uint8_t n,Ultrasonic_Packet p)
{
	_ultrasonic_location_packet[n] = p;
}

void Ultrasonic::setAbstacleGroundPositionTriangle(uint8_t n,ObstacleLocationPacket p)
{
	_abstacle_ground_position_triangle[n] = p;
}

/*
 * 三角定位地面坐标系的转换
 * vehicle:车辆状态信息
 * body   :障碍物相对于载体坐标系的坐标
 * ground :障碍物相对于地面坐标系的坐标
 * */
void Ultrasonic::GroundTriangleCalculate(Vector2d p,float yaw,ObstacleLocationPacket body,ObstacleLocationPacket *ground)
{
    ground->Position = p + body.Position.rotate(yaw);
	ground->Status   = body.Status;
}
