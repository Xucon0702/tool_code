/////////////////////////////////////////////////////////////////////////
// Copyright (C) 2021-2026, by Motovis All Rights Reserved.
//
// 本代码仅用于魔视智能与广州汽车集团股份有限公司汽车工程研究院合作的
// X3V项目（以下简称本项目），魔视智能对本代码及基于本代码开发产生的
// 所有内容拥有全部知识产权，任何人不得侵害或破坏，未经魔视智能授权许可
// 或其他法律认可的方式，任何企业或个人不得随意复制、分发、下载和使用，
// 以及用于非本项目的其他商业用途。
// 
// 本代码仅供指定接收人（包括但不限于      ）在魔视智能授权范围内使用，
// 指定接收人必须征得魔视智能授权，才可在软件库中加入本代码。
//
// 本代码是受法律保护的保密信息，如您不是指定接收人，请立即将本代码删除，
// 法律禁止任何非法的披露、或以任何方式使用本代码。指定接收人应对本代码
// 保密信息负有保密义务，未经允许，不得超出本项目约定的披露、复制、传播
// 或允许第三方披露、复制、传播本代码部分或全部信息。
//
// 
/////////////////////////////////////////////////////////////////////////


#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <cstdio>
#include <unistd.h>
#include "hybrid_a_star.h"
#include <fstream>

#include <sys/neutrino.h>

#include "pps_ipc_init.h"

#include "PnCInteract.h"

#include "planState.h"
#include "gac_log.h"
#define PRT_PLAN  50

#define GAC_PLAN_VER		"\n*** GAC PLAN Software Ver : 1.0.70 ***\n"

#define SAVE_PLAN_TXT  1

extern INT32 cv_MvPpsGetPpsPlanData(PpsPlanData *pPpsPlanData);

ParkingWorkingStatus gParkingStatus;
HybridAStarPart::HybridAStar gHybridAStar;
//std::vector<TargetTrack> aTracksPlan;

InputParkingIn gObliqueInput;
unsigned char gAPASpaceSmallSave[250*150];

static PnCInteract pnc;
static PlanningStatus gPlanSuccess = PlanningSucc;
static UINT32 plan_first_in = 0;
static UINT32 gPlanCnt = 0;
static UINT32 uCurrentStep = 0;
static UINT32 uTotalStep = 0;
static UINT8 bDrRsetFlag = 1;
static UINT32 gLastPlanEnable = 0;
static UINT8 PlanEnable = 0;

static PpsPlanData gPpsPlanData={0};
static UINT32 gSendPathPlanRetDataNum = 0;


int convertAtracks(std::vector<TargetTrack>& aTracks,PlanATracks* aTracksPlan)
{
	GAC_LOG_INFO("aTracks.size():%d\n",aTracks.size());
	aTracksPlan->num = aTracks.size();
	if(aTracks.size()>400)
	{
		aTracksPlan->num = 400;
	}

	for(int i = 0; i < aTracksPlan->num;i++)
	{
		aTracksPlan->aTracks[i].point.x = aTracks[i].point.getX();
		aTracksPlan->aTracks[i].point.y = aTracks[i].point.getY();
		aTracksPlan->aTracks[i].yaw = aTracks[i].yaw;
		aTracksPlan->aTracks[i].curvature = aTracks[i].curvature;
		aTracksPlan->aTracks[i].speed = aTracks[i].speed;
		aTracksPlan->aTracks[i].nDetectStates = aTracks[i].nDetectStates;
	}
		
	return 0;
}

PlanningStatus MvPlanInterface(PpsPlanData *tPpsPlanData,
							   ParkingWorkingStatus &tParkingStatus,
                               PnCInteract *tPnC,
                               unsigned char &bResetFlag,
                               MvTMVehPont *pVehPont,
							   unsigned char bExpectPlanning)
{
    PlanningStatus planSuccess = PlanningSucc;
    static UINT8 bHasPlanned = 0;
    std::vector<TargetTrack> aTracksPlan;

	ParkingWorkingStatus ParkingStatusForFeedBack = freeStauts;

	_planErrCode PlanErrCode  = 0;

	#if 1
	tPnC->HandleFeedback(ParkingStatusForFeedBack);
	#endif
    
	if(bExpectPlanning == 1)
	{		
		if (((tParkingStatus == freeStauts)||(tParkingStatus == RunningFreeStauts)))
		{
			//保存地图和Input数据:
			#if SAVE_PLAN_TXT
			MvSaveFile("./plan/plan_process/map.txt", (char*)tPpsPlanData->apaGridMap.gridmap,250*150);
			MvSaveFile("./plan/plan_process/planInit.txt", (char*)(&tPpsPlanData->inputPk), sizeof(PpsInputParkingIn));
			#endif

			
			#if  1
			GAC_LOG_DEBUG("xc_input:ppsData:num %d;fVehCurX %f;fVehCurY %f;fVehCurYaw %f;fSlotDepth %f;fSlotWidth %f;fSlotYaw %f;m_nTotalStep %d;m_nCurrentStep %d;m_ParkingCtrlStatus %d\n",
				tPpsPlanData->num,tPpsPlanData->inputPk.fVehCurX,tPpsPlanData->inputPk.fVehCurY,tPpsPlanData->inputPk.fVehCurYaw,tPpsPlanData->inputPk.fSlotDepth,tPpsPlanData->inputPk.fSlotWidth,tPpsPlanData->inputPk.fSlotYaw,tPpsPlanData->inputPk.m_nTotalStep,tPpsPlanData->inputPk.m_nCurrentStep,tPpsPlanData->inputPk.m_ParkingCtrlStatus);
			GAC_LOG_DEBUG("bResetFlag %d\n", bResetFlag);
			#endif
			
			tPpsPlanData->inputPk.m_ParkingCtrlStatus = (ParkingWorkingStatus_pps)tParkingStatus;
			bool update = gHybridAStar.InitMapInforAndVehicleInfor((unsigned char*)tPpsPlanData->apaGridMap.gridmap,(InputParkingIn &)tPpsPlanData->inputPk);
			
			UINT8 tUpdate = (UINT8)update;
			if(plan_first_in == 1)
			{
				MvPpsSendUpdateValue(&tUpdate);
			}
			if(update)
			{
				planSuccess = gHybridAStar.PathPlanningRealTimeState((InputParkingIn &)tPpsPlanData->inputPk, aTracksPlan);

				GAC_LOG_INFO("gPlanSuccess %d\n", planSuccess);
			}
			else
			{
				GAC_LOG_ERROR("InitMapInforAndVehicleInfor err\n");
				planSuccess =PlanningFail;
			}

			//规划失败,返回失败原因给决策-->mvplayer做显示
			if(planSuccess == PlanningFail)
			{
				PlanErrCode = gHybridAStar.GetPlanErrCode();
				GAC_LOG_ERROR("PlanningFail:PlanErrCode %d\n",PlanErrCode);
				if(PlanErrCode == E10006)
				{
					tPnC->SendPlanError(DDS::PlanError::OverParkStep);
				}
				else
				{
					tPnC->SendPlanError(DDS::PlanError::StaticFailed);
				}
			}

			//计算CurrentStep+TotalStep
			uCurrentStep = gHybridAStar.getCurrentStep();
			uTotalStep = gHybridAStar.getTotalStep();

			if (planSuccess == planningOver)
			{
				tParkingStatus = ParkingSucc;
			}
			else if (planSuccess == PlanningFail)
			{
				if (tParkingStatus == freeStauts)
				{
					tParkingStatus = parkingOver;
					// tPnC->SendPlanError(DDS::PlanError::StaticFailed);
				}
				else if (tParkingStatus == RunningFreeStauts)
				{
					tParkingStatus = Breaking;
				}
				else
				{
					tParkingStatus = Breaking;//never here
				}
			}
			else
			{
				if (tParkingStatus == freeStauts)
					tParkingStatus = updated;
				else if (tParkingStatus == RunningFreeStauts)
					tParkingStatus = RunningUpdated;
				else
				{
					tParkingStatus = Breaking;//never here
				}

				if(tParkingStatus == updated || tParkingStatus == RunningUpdated)
				{
					tPnC->SetTrack(aTracksPlan, tParkingStatus, pVehPont, bResetFlag);
					bResetFlag = 0;//发完路径把dr重置的清0,只有第一次进规划需要做重置。
					
					if(tParkingStatus == RunningUpdated)
					{
						tParkingStatus == working;
					}
				}
			}
			tPnC->SendInfo(tParkingStatus, pVehPont, 0);

			// 发送 必要状态 过程信息: 规划状态(gPlanSuccess) 当前步数 全部步数 tParkingStatus;
			#if 1
	
			gSendPathPlanRetDataNum++;
			
			PathPlanRet tPathPlanRet = {0};
			tPathPlanRet.ParkingStatus = tParkingStatus;
			tPathPlanRet.PlanSuccess = planSuccess;
			tPathPlanRet.uCurrentStep = uCurrentStep;
			tPathPlanRet.uTotalStep = uTotalStep;
			tPathPlanRet.num = gSendPathPlanRetDataNum;
			tPathPlanRet.PlanErrCodeE = (ppsPlanErrCodeEnum)PlanErrCode;

			
			memcpy(&(tPathPlanRet.parkInToApa),&(tPpsPlanData->inputPk),sizeof(PpsInputParkingIn));

			convertAtracks(aTracksPlan,&(tPathPlanRet.aTracksPlan));

			GAC_LOG_DEBUG("swh:ParkingStatus:%d;PlanSuccess %d;sizeof(PathPlanRet) =%d;atracks num =%d;PlanErrCodeE %d,gSendPathPlanRetDataNum %d\n",tPathPlanRet.ParkingStatus,tPathPlanRet.PlanSuccess,sizeof(PathPlanRet),tPathPlanRet.aTracksPlan.num,tPathPlanRet.PlanErrCodeE,tPathPlanRet.num);

			for(int i=0;i<3;i++)
			{
				MvPpsSendPathPlanRet(&tPathPlanRet);
				usleep(50);
			}
			#endif

			for(int i = 0; i< aTracksPlan.size(); i++)
			{
				GAC_LOG_DEBUG("%d aTracksPlan x %f y %f yaw %f\n", i, aTracksPlan[i].point.getX(), aTracksPlan[i].point.getY(), aTracksPlan[i].yaw);
			}
		}
		else
		{
			GAC_LOG_WARN("tParkingStatus err %d\n", tParkingStatus);
		}
	}
	else
	{
		//tPnC->HandleFeedback(tParkingStatus);
		tPnC->SendInfo(tParkingStatus, pVehPont, 0);
	}
    return planSuccess;
}

extern uint32_t g_plan_enable;
extern std::mutex mtx_plan_enable;
extern std::condition_variable cv_plan_enable;
extern void cv_clearPlanData();

//return: 0:normal	1:planOut
void cv_waitPlanEnable()
{
    bool plan_enable = 0;
    {
        std::lock_guard<std::mutex> lk(mtx_plan_enable);
        plan_enable = g_plan_enable;
        std::cout << "plan enable: " << g_plan_enable << std::endl;
    }

    if (plan_enable == 0) {
       gSendPathPlanRetDataNum = 0;

       pnc.ResetPlan();
       plan_first_in = 0;
       bDrRsetFlag = true;
       cv_clearPlanData();

       PathPlanRet tPathPlanRet;
       memset(&tPathPlanRet, 0, sizeof(PathPlanRet));
       MvPpsSendPathPlanRet(&tPathPlanRet);

       std::unique_lock<std::mutex> lk(mtx_plan_enable);
       cv_plan_enable.wait(lk, []{return g_plan_enable;});
       std::cout << "after wait: plan enable: " << g_plan_enable << std::endl;
    }
}

extern std::condition_variable cv_plan_num;
extern std::mutex mtx_plan_num;
extern uint32_t g_plan_num;
extern PpsPlanData cv_g_PpsPlanData;

//return: 0:normal   1:continue
UINT32 cv_ApaPlanHandler(UINT8 recvInputFlag, ParkingWorkingStatus tParkingStatusInput)
{
    UINT32 ret = 0;
    switch (plan_first_in) {
    case PLAN_INIT:
        {
            std::unique_lock<std::mutex> lk(mtx_plan_num);
            cv_plan_num.wait(lk, []{return cv_g_PpsPlanData.num != 0;});

            plan_first_in = 1;
            pnc.ResetPlan();
            gHybridAStar.Init();
            gParkingStatus = freeStauts;
        }
        break;

    case PLAN_ALG:
        if (1 == recvInputFlag) {
            gParkingStatus = tParkingStatusInput;
        } else {
            if (1 == Get_recvParkStatusFlag()) {
                GAC_LOG_DEBUG("Get_recvParkStatusFlag \n");
                MvPpsGetParkingSta((UINT8*) &gParkingStatus);
                Set_recvParkStatusFlag_lock(0);
            }
        }
        break;

    default:
        GAC_LOG_WARN("@@@ unknown plan_first_in  = %d\n", plan_first_in);
        break;
    }

    return ret;
}

extern void clearPlanDataUpdate();
extern uint8_t getPlanDataUpdate();

int main(int argc, char *argv[])
{
  
	unsigned char bPlanOnOff = 0;
    PlanningStatus tPlanSuccess;	
    UINT8 bExpectPlanning = 0;//是否需要做规划
	
	PpsObsGridMap tObsGridMap={0};
	PpsInputParkingIn tPpsInputParkingIn={0};
	//PpsPlanData tPpsPlanData={0};

	//gaclog settings
	gaclog_set_backup_file_count(5);
	gaclog_set_file_size(200);
	gaclog_set_store_path((char*) "motovis/");
	gaclog_disable_console_output();
	gaclog_set_log_level(LOG_LEVEL_DEBUG);
  
	MvPpsApaVehPos tVehPos ={0};
	MvTMVehPont pVehPont = {0};
	uint32_t ret = 0;
	//UINT8 PlanEnable = 0;	
    uint8_t BrkFlag = 0;
	uint8_t ppsBrk = 0;
		
	MvPpsIpcInit();

	unsigned int runmask = 0x01;
	ThreadCtl(_NTO_TCTL_RUNMASK, (void*)runmask);

	pthread_setschedprio(pthread_self(), 40);//线程优先级设置

	plan_process_init();

	memset(&gPpsPlanData,0,sizeof(PpsPlanData));

	while(!bPlanOnOff)
	{		

		#if 1
		if(gPlanCnt%100 == 0)
			GAC_LOG_DEBUG(GAC_PLAN_VER);
		#endif
		
		gPlanCnt++;

		// 等待使能
		cv_waitPlanEnable();
				
        uint8_t is_plan_need = getPlanDataUpdate();
		ret = cv_ApaPlanHandler(is_plan_need, (ParkingWorkingStatus)gPpsPlanData.inputPk.m_ParkingCtrlStatus);

		//接收DR:pVehPont
		MvPpsGetApaVehPos(&tVehPos);
		pVehPont.tMvVehPont.fx = tVehPos.vehPos_X;
		pVehPont.tMvVehPont.fy = tVehPos.vehPos_Y;
		pVehPont.tMvVehPont.fyaw = tVehPos.vehPos_Yaw;

		//获取map+parkInput数据
		cv_MvPpsGetPpsPlanData(&gPpsPlanData);
		
		//规划过程:
		tPlanSuccess = MvPlanInterface(	&gPpsPlanData,
										gParkingStatus,
										&pnc,
										bDrRsetFlag,
										&pVehPont,
                                        is_plan_need
										);
		
		if(gPlanCnt%10 == 0) //100
		{
			MvPpsGetCtrllState(&ppsBrk);
			BrkFlag = (ppsBrk >> 0) & 0x0f;
			GAC_LOG_DEBUG("gParkingStatus %d ;BrkFlag %d ;pVehPont.x %f pVehPont.y %f pVehPont.yaw %f bExpectPlanning %d\n", 

				gParkingStatus,BrkFlag, pVehPont.tMvVehPont.fx, pVehPont.tMvVehPont.fy, pVehPont.tMvVehPont.fyaw, bExpectPlanning);

		}

        if (is_plan_need) {
            clearPlanDataUpdate();
        }

		usleep(80*1000);//150
	}

	return 0;
}
