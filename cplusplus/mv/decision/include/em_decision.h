#ifndef EM_DECISION_H
#define EM_DECISION_H

#include<stdint.h>
#include<string.h>


#include "MvApaCommon.h"
#include "vehicle.h"
#include "pps_apa_decision.h"


#include "mvLog.h"

#include "base.h"

#include "MvPlatformCommonStruct.h"
#include "MvApaCommonStruct.h"

typedef int32_t  INT32;

INT32 updateTargetSlotOnHdmi(MvTMVehPont *pVehPont, ApaToHdmiInfo *pApaTHdmiInfo, HdmiToApaInfo *pHdmiTApaInfo,MvUpdateSlotData*	pMvUpdateSlotData);


#endif