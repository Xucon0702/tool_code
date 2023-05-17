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

#ifndef __PLANSTATE_H__
#define __PLANTATE_H__

#ifdef __cplusplus
extern "C"{
#endif

#include <stdlib.h>
#include <stdint.h>
#include "gac_log.h"

//#include "typedefs.h"

//plan定义
typedef enum 
{
	PLAN_INIT = 0,				//进入第一阶段:INIT
	PLAN_ALG,					//进入第二阶:ALG				
}APA_plan_STATE_E;

typedef struct _planCfg
{	
	int32_t  dynamicProgramming;//是否动态规划
	int32_t  reserved[31];
}planCfg;



	
int MvSaveFile(const char *filename, char *buf, int length);
void MvSetPlanCfg(planCfg *pPlanCfg);
void MvGetPlanCfg(planCfg *pPlanCfg);
int plan_process_init();



#ifdef __cplusplus
}
#endif

#endif


