/*
 * PnCInteract.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */
#include <stdio.h>
#include <unistd.h>
#include "PnCInteract.h"
#include "pps_send_mcu.h"
#include "pps_recv_mcu.h"
#include "gac_log.h"
#include "pps_apa_fusion.h"


static float pi2pi(float angle)
{
    while(angle > PI)
    {
        angle = angle - 2*PI;
    }
    while(angle < -PI)
    {
        angle = angle + 2*PI;
    }
    return angle;
}

PnCInteract::PnCInteract() {
    Init();
}

PnCInteract::~PnCInteract() {

}

void PnCInteract::Init()
{
    DDS::_PathAttribute.Path_Gear = DDS::None;
    DDS::_PathAttribute.Path_ID = 0;
    DDS::_PathAttribute.PathSize = 0;
#ifdef GAC_PORT
    DDS::_PathAttribute.PlanComplete = 1; //解控
#else
    DDS::_PathAttribute.PlanComplete = 0; //解控
#endif
    DDS::_PathAttribute.PlanError = DDS::NoError;
    DDS::_PathAttribute.PathLen = 0;
    DDS::_PathAttribute.RmPathLen = 0;

    DDS::_APAfusion_PosReset = 0;

    DDS::_WarningType = 0;
}


int PnCInteract::SendInfo(ParkingWorkingStatus& tParkingStatus, MvTMVehPont *pVehPont, bool bInit)
{
    VehPos.x = pVehPont->tMvVehPont.fx;
    VehPos.y = pVehPont->tMvVehPont.fy;
    VehPos.yaw = pVehPont->tMvVehPont.fyaw;
    DDS::_PathAttribute.PlanError = DDS::NoError;

	
	DrResetSuccessFlag tDrResetSuccessFlag = {0};

	GAC_LOG_DEBUG("SendInfo11111111111111\n");

    double dist = 0;
    float yaw = 0.;
    switch (tParkingStatus)
    {
    case freeStauts:
        break;

    case RunningFreeStauts:
        break;

    case updated:
#ifdef GAC_PORT
        DDS::_PathAttribute.PlanComplete = 0;
#else
        DDS::_PathAttribute.PlanComplete = 1;
#endif
        m_nCurrentPos = 0;
		GAC_LOG_DEBUG("xc:CtrlEndOK = %d;CtrlPathID %d\n",DDS::_CtrlOut.CtrlEndOK,DDS::_CtrlOut.CtrlPathID);
        if (DDS::_CtrlOut.CtrlEndOK == 1)
        {
            if (aTracks.front().speed < 0)
                yaw = pi2pi(aTracks.front().yaw + PI);
            else
                yaw = aTracks.front().yaw;

            if (bInit)
			{
				GAC_LOG_DEBUG("reset dr 2222222222222\n");
				bLastInit = bInit;
			}

            if (bLastInit && !PosClose(VehPos, aTracks.front().point.getX(), aTracks.front().point.getY(), yaw))
            {	
            	//MvPpsSendDrResetSuccessFlag(0);
            	tDrResetSuccessFlag.FirstStepDrResetFlag = 0;
				tDrResetSuccessFlag.SencondStepDrResetFlag = 0;				
				MvPpsSendDrResetSuccessFlag(&tDrResetSuccessFlag);
					
                DDS::_APAfusion_PosReset = 1;
                DDS::_APAfusion_PosValue.vehPos_X = aTracks.front().point.getX();
                DDS::_APAfusion_PosValue.vehPos_Y = aTracks.front().point.getY();
                DDS::_APAfusion_PosValue.vehPos_Yaw = yaw;
                MvPpsSendApafusionPosReset(DDS::_APAfusion_PosReset);
                MvPpsSendApafusionPosValue(&DDS::_APAfusion_PosValue);
                //cheak here
				GAC_LOG_INFO("plan reset dr !!!!!!!!!!!!!!!\n");

                //TransformTrajectory(VehPos); //转换坐标
            }
            else
            {
            	//MvPpsSendDrResetSuccessFlag(1);
            	tDrResetSuccessFlag.FirstStepDrResetFlag = 0;
				tDrResetSuccessFlag.SencondStepDrResetFlag = 1;					
				tDrResetSuccessFlag.tDrResetValue.vehPos_X = DDS::_APAfusion_PosValue.vehPos_X;	
				tDrResetSuccessFlag.tDrResetValue.vehPos_Y = DDS::_APAfusion_PosValue.vehPos_Y;	
				tDrResetSuccessFlag.tDrResetValue.vehPos_Yaw = DDS::_APAfusion_PosValue.vehPos_Yaw;	
				
				MvPpsSendDrResetSuccessFlag(&tDrResetSuccessFlag);
				
				GAC_LOG_INFO("xc:UpdatePath !!!!!!!!!!!!!!!\n");//每段路径只执行一次(静态),UpdatePath可能多次
                bLastInit = false;
                DDS::_APAfusion_PosReset = 0;
                DDS::_PathAttribute.Path_ID = DDS::_CtrlOut.CtrlPathID + 1;
				UpdatePath(VehPos, aTracks);
                MvPpsSendApafusionPosReset(DDS::_APAfusion_PosReset);
                //tParkingStatus = perparing; //已经传出数据
            }
        }
		//MvDdsSendApaPathAttribute(&DDS::_PathAttribute);
        break;

    case RunningUpdated: // 动态update 更新路径
        UpdatePath(VehPos, aTracks);
        //DDS::app_ctrol_U.tPathInfo.uPath_ID = DDS::CtrlOut.CtrlPathID + 1;
        break;

    case working:
        dist = (Vector2d(DDS::_PathPoints.point[DDS::_PathAttribute.PathSize-1].X,
                         DDS::_PathPoints.point[DDS::_PathAttribute.PathSize-1].Y)
                - Vector2d(VehPos.x, VehPos.y)).Length();
        if (dist < 1.0 && DDS::_PathAttribute.RmPathLen > 0)
        {
            UpdatePath(VehPos, aTracks); //小于50cm更新路径
        }
        break;

    case Breaking:
        DDS::_PathAttribute.PlanError = DDS::DynamicFailed;
		DDS_SEND_PATH_ATTRIBUTE;
        break;

    case ParkingSucc:
        tParkingStatus = parkingOver;
#ifdef GAC_PORT
        DDS::_PathAttribute.PlanComplete = 1; //解控
#else
        DDS::_PathAttribute.PlanComplete = 0; //解控
#endif
        DDS_SEND_PATH_ATTRIBUTE;
        break;
	case parkingOver:
    default:
        break;
    }

	//DDS_SEND_PATH_ATTRIBUTE;

    return m_nCurrentPos;
}

void PnCInteract::HandleFeedback(ParkingWorkingStatus& tParkingStatus)
{
    uint8_t uDDSdata = 0;
    MvPpsGetCtrlOut(&DDS::_CtrlOut);
    MvPpsGetCtrllState(&uDDSdata);
    DDS::_Control_State = uDDSdata & 0x0f;
    DDS::_Brk_Flag = (uDDSdata >> 4) & 0x0f;
	static uint32_t feedBackCnt = 0;
	GAC_LOG_INFO("ctrl_out.CtrlEndOK=%d\n", DDS::_CtrlOut.CtrlEndOK);

    if (DDS::_Control_State != DDS::NoErr)
    {
        tParkingStatus = parkingErr;
    }
    else if (tParkingStatus == ParkingSucc || tParkingStatus == parkingOver)
    {
    }
    else if (DDS::_Control_State == DDS::NoErr)
    {
        if (DDS::_CtrlOut.CtrlEndOK == 1 && tParkingStatus != updated)
        {
        	#if 0 //xc-21-11-11
			if(feedBackCnt>50)
			{
        		tParkingStatus = freeStauts;
				GAC_LOG_DEBUG("swh:feedBackCnt = %d\n",feedBackCnt);
				feedBackCnt = 0;
			}
			#else
            tParkingStatus = freeStauts;
			#endif			
			feedBackCnt++;
        }
        else if (DDS::_CtrlOut.CtrlEndOK == 0 && tParkingStatus != Breaking)
        {
            tParkingStatus = working;
        }
    }

    // GAC_LOG_DEBUG("tParkingStatus=%d\n", tParkingStatus);


    // tParkingErr = noErr;
    // if (DDS::_Control_State == DDS::ObstacleErr)
    // {
    //     tParkingErr = ObstacleErr;
    // }
    // else if (DDS::_Control_State == DDS::TrackErr)
    // {
    //     tParkingErr = TrackErr;
    // }
}

void PnCInteract::SetTrack(vector<TargetTrack>& aTracksPlan, ParkingWorkingStatus& tParkingStatus, MvTMVehPont *pVehPont, bool bInit)
{
    aTracks = aTracksPlan;
    SendInfo(tParkingStatus, pVehPont, bInit);
}

void PnCInteract::SendWarningType(int nFreespaceSts, int nType)
{
    uint8_t uDDSdata = nFreespaceSts | (nType<<4);
    if (uDDSdata != DDS::_WarningType)
    {
        DDS::_WarningType = uDDSdata;
        MvPpsSendWarningType(DDS::_WarningType);
    }
}

bool PnCInteract::PosClose(const LocationPoint& tVel, float x, float y, float yaw)
{
    return (fabs(tVel.x - x) < 1e-6) && (fabs(tVel.y - y) < 1e-6) && (fabs(tVel.yaw - yaw) < 1e-6);
}

int PnCInteract::CalculateNearestPointByPosition(const double x, const double y, vector<TargetTrack>& aTracks)
{
    Vector2d current_point;
    double current_value, _min_value;
    int _target_point_index;

    current_point = Vector2d(x, y);
    _min_value = (aTracks.at(0).point - current_point).Length();
    _target_point_index = 0;
    for(int i = 1; i < aTracks.size(); i++)
    {
        current_value = (aTracks.at(i).point - current_point).Length();
        if(current_value < _min_value)
        {
            _target_point_index = i;
            _min_value = current_value;
        }
    }

    return _target_point_index;
}

void PnCInteract::UpdatePath(const LocationPoint& tVel, vector<TargetTrack>& aTracks)
{
	//GAC_LOG_DEBUG("xc:UpdatePath*****\n");
    Vector2d target_point;
    m_nCurrentPos = CalculateNearestPointByPosition(tVel.x, tVel.y, aTracks);

#ifdef GAC_PORT
    for (int i=0; i < 40 && (m_nCurrentPos+i) < (int)aTracks.size(); i++)
    {
        target_point = aTracks.at(m_nCurrentPos + i).point;
        if (aTracks.at(m_nCurrentPos + i).speed < 0)
        {
            DDS::_PathPoints.point[i].Curv = -aTracks.at(m_nCurrentPos + i).curvature;
            DDS::_PathPoints.point[i].Yaw = pi2pi(aTracks.at(m_nCurrentPos + i).yaw + PI);
        }
        else
        {
            DDS::_PathPoints.point[i].Curv = aTracks.at(m_nCurrentPos + i).curvature;
            DDS::_PathPoints.point[i].Yaw = float(aTracks.at(m_nCurrentPos + i).yaw);
        }
        DDS::_PathPoints.point[i].Vmax = fabs(aTracks.at(m_nCurrentPos + i).speed)*3.6;
        DDS::_PathPoints.point[i].X = target_point.getX();
        DDS::_PathPoints.point[i].Y = target_point.getY();
        
    }
    DDS::_PathAttribute.Path_Gear = aTracks.at(m_nCurrentPos + 0).speed >  1.0e-6f ? DDS::Drive :
                                    aTracks.at(m_nCurrentPos + 0).speed < -1.0e-6f ? DDS::Reverse : DDS::Neutral;
    DDS::_PathAttribute.PathSize = min(40, (int)aTracks.size()-m_nCurrentPos);
    DDS::_PathAttribute.PlanError = DDS::NoError;
    DDS::_PathAttribute.PathLen = hypot(aTracks.back().point.getX() - aTracks.front().point.getX(),
                                        aTracks.back().point.getY() - aTracks.front().point.getY());
    DDS::_PathAttribute.RmPathLen = hypot(DDS::_PathPoints.point[DDS::_PathAttribute.PathSize-1].X - aTracks.back().point.getX(),
                                          DDS::_PathPoints.point[DDS::_PathAttribute.PathSize-1].Y - aTracks.back().point.getY());
    GAC_LOG_DEBUG("DDS::_PathAttribute.PathLen %f %f atracks back %f %f front %f %f\n", 
                    DDS::_PathAttribute.PathLen, DDS::_PathAttribute.RmPathLen,
                    aTracks.back().point.getX(), aTracks.back().point.getY(),
                    aTracks.front().point.getX(), aTracks.front().point.getY());
#else
    for (int i=0; i < 40 && (m_nCurrentPos+i) < (int)aTracks.size(); i++)
    {
        target_point = aTracks.at(m_nCurrentPos + i).point;
        DDS::_PathPoints.point[i].Curv = aTracks.at(m_nCurrentPos + i).curvature;
        DDS::_PathPoints.point[i].Vmax = aTracks.at(m_nCurrentPos + i).speed;
        DDS::_PathPoints.point[i].X = target_point.getX();
        DDS::_PathPoints.point[i].Y = target_point.getY();
        DDS::_PathPoints.point[i].Yaw = float(aTracks.at(m_nCurrentPos + i).yaw);
    }
    DDS::_PathAttribute.Path_Gear = aTracks.at(m_nCurrentPos + 0).speed >  1.0e-6f ? DDS::Drive :
                                    aTracks.at(m_nCurrentPos + 0).speed < -1.0e-6f ? DDS::Reverse : DDS::Neutral;
    DDS::_PathAttribute.PathSize = min(40, (int)aTracks.size()-m_nCurrentPos);
    DDS::_PathAttribute.PlanError = DDS::NoError;
    DDS::_PathAttribute.PathLen = hypot(aTracks.back().point.getX() - aTracks.front().point.getX(),
                                        aTracks.back().point.getY() - aTracks.front().point.getY());
    DDS::_PathAttribute.RmPathLen = hypot(DDS::_PathPoints.point[DDS::_PathAttribute.PathSize-1].X - aTracks.back().point.getX(),
                                          DDS::_PathPoints.point[DDS::_PathAttribute.PathSize-1].Y - aTracks.back().point.getY());
#endif

    DDS_SEND_PATH_POINTS;
}

void PnCInteract::ResetPlan()
{
    MvPpsGetCtrlOut(&DDS::_CtrlOut);

    DDS::_PathAttribute.PlanError = DDS::NoError;
    DDS::_PathAttribute.PlanComplete = 0;
    DDS::_PathAttribute.Path_ID = 0; //DDS::_CtrlOut.CtrlPathID;
    DDS::_PathAttribute.PathSize = 0;
	DDS::_PathAttribute.Path_Gear = 0;
    //DDS_SEND_PATH_ATTRIBUTE;
    DDS_SEND_PATH_RESET_ATTRIBUTE;
}

void PnCInteract::SendPlanError(const DDS::PlanError ePlanErr)
{
    DDS::_PathAttribute.PlanError = ePlanErr;
    //DDS::_PathAttribute.PlanComplete = 1;
    DDS_SEND_PATH_ATTRIBUTE;
}

#ifdef GAC_A58
void PnCInteract::A58PortSendPathAttribute()
{
    DDS::_ApaPathData.PathAttribute.Path_Gear = DDS::_PathAttribute.Path_Gear;
	DDS::_ApaPathData.PathAttribute.Path_ID = DDS::_PathAttribute.Path_ID;
	DDS::_ApaPathData.PathAttribute.Path_type = 0;
	DDS::_ApaPathData.PathAttribute.PathSize = DDS::_PathAttribute.PathSize;
	DDS::_ApaPathData.PathAttribute.PlanComplete = DDS::_PathAttribute.PlanComplete;
	DDS::_ApaPathData.PathAttribute.PlanError = DDS::_PathAttribute.PlanError;
	DDS::_ApaPathData.PathAttribute.PathLen = DDS::_PathAttribute.PathLen;
	DDS::_ApaPathData.PathAttribute.RmPathLen = DDS::_PathAttribute.RmPathLen;

	GAC_LOG_DEBUG("DDS::_ApaPathData.PathAttribute.Path_Gear = %d\n", DDS::_ApaPathData.PathAttribute.Path_Gear);
	GAC_LOG_DEBUG("DDS::_ApaPathData.PathAttribute.Path_ID = %d\n", DDS::_ApaPathData.PathAttribute.Path_ID);
	GAC_LOG_DEBUG("DDS::_ApaPathData.PathAttribute.PathSize = %d\n", DDS::_ApaPathData.PathAttribute.PathSize);
	GAC_LOG_DEBUG("DDS::_ApaPathData.PathAttribute.PlanComplete = %d\n", DDS::_ApaPathData.PathAttribute.PlanComplete);
	GAC_LOG_DEBUG("DDS::_ApaPathData.PathAttribute.PlanError = %d\n", DDS::_ApaPathData.PathAttribute.PlanError);


    MvPpsSendApaPathData(&DDS::_ApaPathData);
}

void PnCInteract::A58PortSendPathPoints()
{
    DDS::_ApaPathData.PathAttribute.Path_Gear = DDS::_PathAttribute.Path_Gear;
	DDS::_ApaPathData.PathAttribute.Path_ID = DDS::_PathAttribute.Path_ID;
	DDS::_ApaPathData.PathAttribute.Path_type = 0;
	DDS::_ApaPathData.PathAttribute.PathSize = DDS::_PathAttribute.PathSize;
	DDS::_ApaPathData.PathAttribute.PlanComplete = DDS::_PathAttribute.PlanComplete;
	DDS::_ApaPathData.PathAttribute.PlanError = DDS::_PathAttribute.PlanError;
	DDS::_ApaPathData.PathAttribute.PathLen = DDS::_PathAttribute.PathLen;
	DDS::_ApaPathData.PathAttribute.RmPathLen = DDS::_PathAttribute.RmPathLen;

    for (int i=0; i < 40; i++)
    {
        DDS::_ApaPathData.PathPoint[i].Curv = DDS::_PathPoints.point[i].Curv;
        DDS::_ApaPathData.PathPoint[i].Yaw = DDS::_PathPoints.point[i].Yaw;
        DDS::_ApaPathData.PathPoint[i].Vmax = DDS::_PathPoints.point[i].Vmax;
        DDS::_ApaPathData.PathPoint[i].X = DDS::_PathPoints.point[i].X;
        DDS::_ApaPathData.PathPoint[i].Y = DDS::_PathPoints.point[i].Y;
        DDS::_ApaPathData.PathPoint[i].Lamp = 0;
    }

		GAC_LOG_DEBUG("DDS::_ApaPathData.PathPoint[0].X = %f\n", DDS::_ApaPathData.PathPoint[0].X);
		GAC_LOG_DEBUG("DDS::_ApaPathData.PathPoint[0].Y = %f\n", DDS::_ApaPathData.PathPoint[0].Y);
		GAC_LOG_DEBUG("DDS::_ApaPathData.PathPoint[0].Yaw = %f\n", DDS::_ApaPathData.PathPoint[0].Yaw);
		GAC_LOG_DEBUG("DDS::_ApaPathData.PathPoint[0].Curv = %f\n", DDS::_ApaPathData.PathPoint[0].Curv);
		GAC_LOG_DEBUG("DDS::_ApaPathData.PathPoint[0].Vmax = %f\n", DDS::_ApaPathData.PathPoint[0].Vmax);
		GAC_LOG_DEBUG("DDS::_ApaPathData.PathPoint[0].Lamp = %d\n", DDS::_ApaPathData.PathPoint[0].Lamp);	
    MvPpsSendApaPathData(&DDS::_ApaPathData);
}

void PnCInteract::A58PortSendPathResetAttribute()
{	
		GAC_LOG_INFO("A58PortSendPathResetAttribute\n");
		DDS::_ApaPathData.PathAttribute.Path_Gear = 0;
		DDS::_ApaPathData.PathAttribute.Path_ID = 0;
		DDS::_ApaPathData.PathAttribute.Path_type = 0;
		DDS::_ApaPathData.PathAttribute.PathSize = 0;
		DDS::_ApaPathData.PathAttribute.PlanComplete = 0;
		DDS::_ApaPathData.PathAttribute.PlanError = DDS::NoError;
		DDS::_ApaPathData.PathAttribute.PathLen = 0;
		DDS::_ApaPathData.PathAttribute.RmPathLen = 0;
	
		for (int i=0; i < 40; i++)
		{
			DDS::_ApaPathData.PathPoint[i].Curv = 0.0;
			DDS::_ApaPathData.PathPoint[i].Yaw = 0.0;
			DDS::_ApaPathData.PathPoint[i].Vmax = 0.0;
			DDS::_ApaPathData.PathPoint[i].X = 0.0;
			DDS::_ApaPathData.PathPoint[i].Y = 0.0;
			DDS::_ApaPathData.PathPoint[i].Lamp = 0;
		}
	
			GAC_LOG_DEBUG("resetplan:DDS::_ApaPathData.PathPoint[0].X = %f\n", DDS::_ApaPathData.PathPoint[0].X);
			GAC_LOG_DEBUG("resetplan:DDS::_ApaPathData.PathPoint[0].Y = %f\n", DDS::_ApaPathData.PathPoint[0].Y);
			GAC_LOG_DEBUG("resetplan:DDS::_ApaPathData.PathPoint[0].Yaw = %f\n", DDS::_ApaPathData.PathPoint[0].Yaw);
			GAC_LOG_DEBUG("resetplan:DDS::_ApaPathData.PathPoint[0].Curv = %f\n", DDS::_ApaPathData.PathPoint[0].Curv);
			GAC_LOG_DEBUG("resetplan:DDS::_ApaPathData.PathPoint[0].Vmax = %f\n", DDS::_ApaPathData.PathPoint[0].Vmax);
			GAC_LOG_DEBUG("resetplan:DDS::_ApaPathData.PathPoint[0].Lamp = %d\n", DDS::_ApaPathData.PathPoint[0].Lamp);	
		MvPpsSendApaPathData(&DDS::_ApaPathData);


}

#else

void PnCInteract::A20PortSendPathResetAttribute()
{	
	GAC_LOG_INFO("A20PortSendPathResetAttribute\n");
	MvPpsSendApaPathAttribute(&DDS::_PathAttribute);	
}



void PnCInteract::A18PortSendPathPoints()
{
    MvPpsSendApaPathPoints(&DDS::_PathPoints);
    MvPpsSendApaPathAttribute(&DDS::_PathAttribute);    
}

#endif

