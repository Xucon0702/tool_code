/*
 * PnCInfo.cpp
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */
#include "PnCInfo.h"

namespace DDS {
#ifdef GAC_A58
MvPpsApaPathData  _ApaPathData;
#endif

//发送
MvPpsPathPointArr40   _PathPoints;
MvPpsApaPathAttribute _PathAttribute;

UINT8                _APAfusion_PosReset;
MvApafusionPosValue  _APAfusion_PosValue;

UINT8         _WarningType;

//接收
UINT8         _Control_State;
UINT8         _Brk_Flag;
MvPpsCtrlOut  _CtrlOut;

// MvDdsApaVehPos  _APA_VehPos;
}

void getControlFeedback(DDS::ControlState &Control_State)
{
    Control_State = (DDS::ControlState)DDS::_Control_State;
}
