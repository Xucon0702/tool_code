
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

#include "pps_ipc_init.h"
#include "pps_instance.h"
#include "ppscontrol.h"
#include "pps_apa_fusion.h"
#include "topic_apa_fusion.h"
#include "motovis_pps_protocal.h"
#include "motovis_topic_ipcadapter.h"
#include "topic_ipcadapter.h"
#include "pps_recv_mcu.h"

#include <mutex>

std::mutex mtx_plan_enable;
std::condition_variable cv_plan_enable;
uint32_t g_plan_enable;

std::mutex mtx_plan_num;
std::condition_variable cv_plan_num;
PpsPlanData cv_g_PpsPlanData;
uint8_t cv_plandata_update;

void cv_clearPlanData()
{
    std::lock_guard<std::mutex> lk(mtx_plan_num);
    return memset(&cv_g_PpsPlanData, 0, sizeof(PpsPlanData));
}

INT32 cv_MvPpsGetPpsPlanData(PpsPlanData *pPpsPlanData)
{
    std::lock_guard<std::mutex> lk(mtx_plan_num);
    return memcpy(pPpsPlanData, &cv_g_PpsPlanData, sizeof(PpsPlanData));
}

void clearPlanDataUpdate()
{
    std::lock_guard<std::mutex> lk(mtx_plan_num);
    cv_plandata_update = 0;
}

uint8_t getPlanDataUpdate()
{
    std::lock_guard<std::mutex> lk(mtx_plan_num);
    return cv_plandata_update;
}

void cv_PpsPlanDataCallback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData)
{
    PpsPlanData *pPpsPlanData = (PpsPlanData *)pData;
    if (nTopicId == TOPICID_MOTOVIS_APA_PLAN_INPUT_DATA && nCmdid == CMDID_APA_PLAN_DATA) {
        std::lock_guard<std::mutex> lk(mtx_plan_num);

        memcpy(&cv_g_PpsPlanData, pPpsPlanData, sizeof(PpsPlanData));
        cv_plandata_update = 1;

        if (cv_g_PpsPlanData.num != 0) {
            cv_plan_num.notify_one();
        }
    }
}

int cv_MvPpsPlanDataRecvInit()
{
	DESY::PPS_CFG_STRU tPpscfg = TopicApaPlanData::cfg(1, 0);  
	return MvPpsRegisterInstance(tPpscfg, cv_PpsPlanDataCallback);
}

void cv_PpsApaStateMachineCallback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData)
{
    if (nTopicId == TOPICID_MOTOVIS_APPAPASTATEMACHINE_SWC && nCmdid == CMDID_APPAPASTATEMACHINE_SWC_STATUS) {

        AlgInterface_appapastatemachine_swc_status *pMsg = NULL;
        pMsg = (AlgInterface_appapastatemachine_swc_status *)pData;

        {
            std::lock_guard<std::mutex> lk(mtx_plan_enable);
            g_plan_enable = pMsg->APA_PlanEnable;
        }

        if (g_plan_enable == 1) {
            cv_plan_enable.notify_one();
        }
    }
}

INT32 cv_MvPpsRecvApaStateInit(void)
{
    return MvPpsRegisterInstance(topicid_motovis_appapastatemachine_swc::cfg_subscribe, cv_PpsApaStateMachineCallback);
}


INT32 MvPpsIpcInit(void)
{

    //2022-02-16
    #if 1
    cv_MvPpsPlanDataRecvInit();
    MvPpsRecvPlanToA72Init();
    #else
    MvPpsPlanInputParkingInRecvInit();
    #endif

    MvPpsRecvMcuInit();
    cv_MvPpsRecvApaStateInit();

    MvPpsSendMcuInit();

    MvPpsSendPlanUpdateValueInit();
    MvPpsSendPathPlanRetInit();

    MvPpsDrResetSuccessFlagSendInit();

    MvPpsInstanceInit();

    return 0;
}


