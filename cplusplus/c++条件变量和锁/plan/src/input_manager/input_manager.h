#ifndef __H_INPUT_MANAGER_H__
#define __H_INPUT_MANAGER_H__

#include "pps_ipc_init.h"
#include "MvApaCommonStruct.h"

#include "pps_instance.h"
#include "ppscontrol.h"
#include "pps_apa_fusion.h"
#include "topic_apa_fusion.h"
#include "motovis_pps_protocal.h"
#include "motovis_topic_ipcadapter.h"
#include "topic_ipcadapter.h"
#include "typedef_ipcadapter.h"
#include "motovis_typedef_ipcadapter.h"

#include <condition_variable>
#include <mutex>
#include <deque>

namespace plan
{

class InputManager {
public:
    InputManager();
    ~InputManager();

    void Wait();
    void GetInput(MvTMVehPont& pVehPont, PpsPlanData& tPpsPlanData);

private:
    void InitPpsReader();
    static void PpsPlanDataCallback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData);
    static void PpsPlanToA72Callback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData);
    static void PpsApaStateMachineCallback(INT32 nTopicId, INT32 nCmdid, INT32 nSize, char *pData);

private:
    static bool need_plan;
    static std::mutex mtx_;
    static std::condition_variable cv_;

    // input cache
    static std::deque<uint8_t> park_sts_cache_;
    static std::deque<PpsPlanData*> plan_data_cache_;
    static std::deque<AlgInterface_appapastatemachine_swc_status*> apa_fsm_cache_;

    const static int32_t park_sts_cache_size_ = 2;
    const static int32_t plan_data_cache_size_ = 3;
    const static int32_t apa_fsm_cache_size_ = 16;
};

}
// namespace plan

#endif // __H_INPUT_MANAGER_H__
