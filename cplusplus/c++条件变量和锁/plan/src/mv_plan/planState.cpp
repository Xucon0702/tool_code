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
#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "planState.h"


static 	pthread_mutex_t 	gPlanCfgMutex = PTHREAD_MUTEX_INITIALIZER;
static  planCfg 			g_planCfg = {0};


/*
* 函数名称: MvSaveFile 
* 函数功能: 保存文件
* 输入参数: 文件名，内容，长度
* 输出参数: 无
* 返回值:  0成功，1失败
*/ 
int MvSaveFile(const char *filename, char *buf, int length)
{
	FILE *fp = NULL;
	int nfilelen = 0;
	int ret = 0;	

	GAC_LOG_INFO("begin save file %s len %d!\n",filename,length);	
	
	nfilelen = length;		
	if ((fp = fopen(filename, "w+")) == NULL)
	{	
		GAC_LOG_ERROR("create file %s error!\n",filename);
		return 1;
	}

	while(nfilelen)
	{	
		
		if(nfilelen >= 1024*64)
		{
			if(fwrite(buf,1024*64,1,fp)==1)
			{
				buf += 1024*64;
				nfilelen -= 1024*64;	
			}		
			else
			{
				GAC_LOG_ERROR("write len %d error!\n",1024*64);	
			}
		}	
		else
		{
			if(fwrite(buf,nfilelen,1,fp)==1)
			{				
				buf += nfilelen;
				nfilelen -= nfilelen;	
			}
			else
			{
				GAC_LOG_ERROR("write len %d error!\n",nfilelen);		
			}		
		}			
		GAC_LOG_DEBUG("nfilelen len %d!\n",nfilelen);
	}	

	fclose(fp);	

	return 0;
}


//read app_plan.cfg
int loadPlanCfg(const char *filename,planCfg* param)
{
     FILE *ifp = fopen(filename,"r");
     if(ifp == NULL){
          fprintf(stderr,"can not open planCfg file: %s\n",filename);
          return -1;
      }
     fscanf(ifp,"dynamicProgramming:%d\n",&param->dynamicProgramming);     
     fclose(ifp);
     return 0;
}


void MvSetPlanCfg(planCfg *pPlanCfg)
{
	pthread_mutex_lock(&gPlanCfgMutex);
	memcpy(&g_planCfg, pPlanCfg, sizeof(planCfg));
	pthread_mutex_unlock(&gPlanCfgMutex);
}

void MvGetPlanCfg(planCfg *pPlanCfg)
{
	pthread_mutex_lock(&gPlanCfgMutex);
	memcpy(pPlanCfg, &g_planCfg, sizeof(planCfg));
	pthread_mutex_unlock(&gPlanCfgMutex);
}


void* plan_process_handler(void *arg)
{	
	int ret = 0;
	planCfg tPlanCfg = {0};
	char fileName[100] = "./app_plan.cfg";
	
	while(1)
	{
		ret = loadPlanCfg(fileName,&tPlanCfg);
		if(0 == ret)
		{
			MvSetPlanCfg(&tPlanCfg);
			GAC_LOG_DEBUG("read app_plan.cfg success:dynamicProgramming %d\n",g_planCfg.dynamicProgramming);
		}
		else
		{
			GAC_LOG_ERROR("read app_plan.cfg failed\n");
		}
		
		usleep(10*1000*1000);
	}

	pthread_exit(0);
}


int plan_process_init()
{
	int ret = 0;
	pthread_t nplanThreadID;
	
	g_planCfg.dynamicProgramming = 0;
	
	ret = pthread_create(&nplanThreadID, NULL, plan_process_handler, NULL);
	if (ret != 0)
	{
		GAC_LOG_ERROR("can't create plan_process_handler:%s\n", strerror(ret));
		return -1;
	}
	pthread_detach(nplanThreadID);

	
	return 0;

}


