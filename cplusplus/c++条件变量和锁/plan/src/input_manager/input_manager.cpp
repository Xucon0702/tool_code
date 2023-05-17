#include "input_manager.h"

#include <iostream>

namespace plan
{

bool InputManager::need_plan = false;
std::mutex InputManager::mtx_;
std::condition_variable InputManager::cv_;

// input cache
std::deque<uint8_t> InputManager::park_sts_cache_;
std::deque<PpsPlanData*> InputManager::plan_data_cache_;
std::deque<AlgInterface_appapastatemachine_swc_status*> InputManager::apa_fsm_cache_;

const int32_t InputManager::park_sts_cache_size_;
const int32_t InputManager::plan_data_cache_size_;
const int32_t InputManager::apa_fsm_cache_size_;

InputManager::InputManager()
{
    InitPpsReader();
    MvPpsInstanceInit();
}

InputManager::~InputManager()
{

}

void InputManager::InitPpsReader()
{
    // grid map, slots
    DESY::PPS_CFG_STRU tPpscfg = TopicApaPlanData::cfg(1, 0);
    MvPpsRegisterInstance(tPpscfg, PpsPlanDataCallback);

    // apa plan to a72
    MvPpsRegisterInstance(topicid_motovis_apa_plan_to_a72::cfg_subscribe, PpsPlanToA72Callback);

    // state machine
    MvPpsRegisterInstance(topicid_motovis_appapastatemachine_swc::cfg_subscribe, PpsApaStateMachineCallback);
}

void InputManager::PpsPlanDataCallback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData)
{
    if (plan_data_cache_.size() >= BUFCNT_DEFAULT) {
        plan_data_cache_.pop_front();
    }

    plan_data_cache_.push_back(reinterpret_cast<PpsPlanData*>(pData));
}

void InputManager::PpsPlanToA72Callback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData)
{
    if (park_sts_cache_.size() >= park_sts_cache_size_) {
        park_sts_cache_.pop_front();
    }

    park_sts_cache_.push_back(*reinterpret_cast<uint8_t*>(pData));
}

void InputManager::PpsApaStateMachineCallback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData)
{
    if (nTopicId == TOPICID_MOTOVIS_APPAPASTATEMACHINE_SWC && nCmdid == CMDID_APPAPASTATEMACHINE_SWC_STATUS) {
        if (apa_fsm_cache_.size() >= apa_fsm_cache_size_) {
            apa_fsm_cache_.pop_front();
        }

        apa_fsm_cache_.push_back(reinterpret_cast<AlgInterface_appapastatemachine_swc_status*>(pData));

        if (apaState == 1)
        {
            PlanEnable = 0;
        }
        else if(apaState == 3)
        {
            PlanEnable = 1;
        }

        if (PlanEnable == 0)
        {
           if (gLastPlanEnable == 1)
           {
               pnc.ResetPlan();
               plan_first_in = 0;
               bDrRsetFlag = true;
               Set_recvPlanDataFlag_lock(0);
               gLastPlanEnable = 0;
           }

           return 1;
        }
        else if(PlanEnable == 1)
        {
            if(gLastPlanEnable == 0)
            {
                gLastPlanEnable = 1;
            }
            return 0;
        }
        else
        {
            GAC_LOG_WARN("wrong PlanEnable data\n");
            return 1;
        }

        if (apa_fsm_cache_.back()->APA_State == 3) {
            std::lock_guard<std::mutex> lck(mtx_);
            need_plan = true;
            cv_.notify_one();
        } else if (apa_fsm_cache_.back()->APA_State == 1) {
            std::lock_guard<std::mutex> lck(mtx_);
            need_plan = false;
        }
    }
}

void InputManager::Wait()
{
    std::unique_lock<std::mutex> lck(mtx_);
    cv_.wait(lck, [this]{ return need_plan; });
}

void InputManager::GetInput(MvTMVehPont& pVehPont, PpsPlanData& tPpsPlanData)
{

}

} //namespace plan
