
#include "em_decision.h"
#include "math_formula.h"

/**************GLOBAL****/
MvTMVehPont gVehPont;
ApaToHdmiInfo gApaTHdmiInfo;
HdmiToApaInfo gHdmiToApaInfo;
MvUpdateSlotData gMvUpdateSlotData;


// for updateTargetSlot
ParkingWorkingStatus gCurrParkingStatus = freeStauts;
UINT32 gSlotTypeLockFlag = 0;
INT32 gTargetPointHasReachGoalFlag;
InputParkingIn tObliqueInput;

UINT32			gSlotId = 0;//车位ID
MvSlotOutput    gSlotOutput = {0};
UINT32   gIsNotPerdicular = 0;
UINT32 glockType =0;

//2023-05-17
static void ShrinkWidth(LocationPoint tp0,LocationPoint tp1, float fWidth, LocationPoint &tNp0,LocationPoint &tNp1)
{
    float flen = hypot(tp0.x-tp1.x , tp0.y-tp1.y);
    LocationPoint tpMid;

    if (flen >= fWidth)
    {
        tpMid.x = (tp0.x+tp1.x)*0.5;
        tpMid.y = (tp0.y+tp1.y)*0.5;

        tNp0.x = tpMid.x - (tpMid.x-tp0.x)*fWidth/flen;
        tNp0.y = tpMid.y - (tpMid.y-tp0.y)*fWidth/flen;

        tNp1.x = tpMid.x + (tp1.x-tpMid.x)*fWidth/flen;
        tNp1.y = tpMid.y + (tp1.y-tpMid.y)*fWidth/flen;
    }
    else
    {
        tNp0.x = tp0.x;
        tNp0.y = tp0.y;

        tNp1.x = tp1.x;
        tNp1.y = tp1.y;
    }

}

static void ShrinkWidthOblique(LocationPoint tp0,LocationPoint tp1, float fWidth, LocationPoint &tNp0,LocationPoint &tNp1)
{
    float flen = hypot(tp0.x-tp1.x , tp0.y-tp1.y);
    LocationPoint tpMid;

    tpMid.x = (tp0.x+tp1.x)*0.5;
    tpMid.y = (tp0.y+tp1.y)*0.5;

    //tNp0.x = tpMid.x - (tpMid.x-tp0.x)*fWidth/flen;
    //tNp0.y = tpMid.y - (tpMid.y-tp0.y)*fWidth/flen;
	tNp0.x = tp0.x;
	tNp0.y = tp0.y;

    tNp1.x = tp0.x + (tp1.x-tpMid.x)*fWidth/(flen/2.0f);
    tNp1.y = tp0.y + (tp1.y-tpMid.y)*fWidth/(flen/2.0f);

}




static void Park2Oblique(int nSlotPosition, const LocationPoint& tVeh, const LocationPoint& tRotation, float fSlotWidth, float fSlotDepth, float fSlotAngle, 
                LocationPoint &p0, LocationPoint &p1, LocationPoint &p2, LocationPoint &p3, bool bMode)
{
    LocationPoint tP0;
    LocationPoint tP1;
    LocationPoint tP2;
    LocationPoint tP3;
    LocationPoint tRotationCenter(0,0,0);
    LocationPoint tNewVeh;
    float fCut = fabs(fSlotWidth*cosf(fSlotAngle))/2.f;

    if (!bMode)
    {
        /* code */
        fCut = 0.f;
    }
    
    if(nSlotPosition== RIGHTSIDESLOT)
    {
        tP0.x = fSlotWidth + tRotation.x;
        tP0.y = 0;

        tP1.x = 0 + tRotation.x;
        tP1.y = 0;

        tP2.x = tP1.x - fSlotDepth*cosf(fSlotAngle);
        tP2.y = tP1.y - fSlotDepth*sinf(fSlotAngle);

        tP3.x = tP0.x - fSlotDepth*cosf(fSlotAngle);
        tP3.y = tP0.y - fSlotDepth*sinf(fSlotAngle);

        if(fSlotAngle < M_PI/2.f)
        {
            // tP0.x = -(fSlotDepth - fCut)*cosf(-M_PI + fSlotAngle) + tP3.x;
            // tP0.y = -(fSlotDepth - fCut)*sinf(-M_PI + fSlotAngle) + tP3.y;

            tP0.x += 2 * fCut*cosf(-M_PI + fSlotAngle);
            tP0.y += 2 * fCut*sinf(-M_PI + fSlotAngle);

            tP2.x -= 2 * fCut*cosf(-M_PI + fSlotAngle);
            tP2.y -= 2 * fCut*sinf(-M_PI + fSlotAngle);

            //tP1.x -= fCut*cosf(-M_PI + fSlotAngle);
            //tP1.y -= fCut*sinf(-M_PI + fSlotAngle);

            //tP2.x -= fCut*cosf(-M_PI + fSlotAngle);
            //tP2.y -= fCut*sinf(-M_PI + fSlotAngle);
        }
        else
        {
            // tP1.x = -(fSlotDepth - fCut)*cosf(-M_PI + fSlotAngle) + tP2.x;
            // tP1.y = -(fSlotDepth - fCut)*sinf(-M_PI + fSlotAngle) + tP2.y;

            tP1.x += 2 * fCut*cosf(-M_PI + fSlotAngle);
            tP1.y += 2 * fCut*sinf(-M_PI + fSlotAngle);  

            tP3.x -= 2 * fCut*cosf(-M_PI + fSlotAngle);
            tP3.y -= 2 * fCut*sinf(-M_PI + fSlotAngle);     

            //tP0.x -= fCut*cosf(-M_PI + fSlotAngle);
            //tP0.y -= fCut*sinf(-M_PI + fSlotAngle);

            //tP3.x -= fCut*cosf(-M_PI + fSlotAngle);
            //tP3.y -= fCut*sinf(-M_PI + fSlotAngle);
        }

    }
    else
    {
        tP0.x = 0 + tRotation.x;
        tP0.y = 0;

        tP1.x = fSlotWidth + tRotation.x;
        tP1.y = 0;

        
        tP2.x = tP1.x + fSlotDepth*cosf(M_PI - fSlotAngle);
        tP2.y = tP1.y + fSlotDepth*sinf(M_PI - fSlotAngle);

        tP3.x = tP0.x + fSlotDepth*cosf(M_PI - fSlotAngle);
        tP3.y = tP0.y + fSlotDepth*sinf(M_PI - fSlotAngle);

        if(fSlotAngle < M_PI/2.f)
        {
            // tP1.x = -(fSlotDepth - fCut)*cosf(M_PI - fSlotAngle) + tP2.x;
            // tP1.y = -(fSlotDepth - fCut)*sinf(M_PI - fSlotAngle) + tP2.y;

            tP1.x += 2 * fCut*cosf(M_PI - fSlotAngle);
            tP1.y += 2 * fCut*sinf(M_PI - fSlotAngle);  

            tP3.x -= 2 * fCut*cosf(M_PI - fSlotAngle);
            tP3.y -= 2 * fCut*sinf(M_PI - fSlotAngle);  

            //tP0.x -= fCut*cosf(M_PI - fSlotAngle);
            //tP0.y -= fCut*sinf(M_PI - fSlotAngle);

            //tP3.x -= fCut*cosf(M_PI - fSlotAngle);
            //tP3.y -= fCut*sinf(M_PI - fSlotAngle);

        }
        else
        {
            // tP0.x = -(fSlotDepth - fCut)*cosf(M_PI - fSlotAngle) + tP3.x;
            // tP0.y = -(fSlotDepth - fCut)*sinf(M_PI - fSlotAngle) + tP3.y;

            tP0.x += 2 * fCut*cosf(M_PI - fSlotAngle);
            tP0.y += 2 * fCut*sinf(M_PI - fSlotAngle);

            tP2.x -= 2 * fCut*cosf(M_PI - fSlotAngle);
            tP2.y -= 2 * fCut*sinf(M_PI - fSlotAngle);

            //tP1.x -= fCut*cosf(M_PI - fSlotAngle);
            //tP1.y -= fCut*sinf(M_PI - fSlotAngle);

            //tP2.x -= fCut*cosf(M_PI - fSlotAngle);
            //tP2.y -= fCut*sinf(M_PI - fSlotAngle);

        }   
    }


    RotateCoordinateOfPoint(tRotationCenter, tP0, -tVeh.yaw, tP0);
    RotateCoordinateOfPoint(tRotationCenter, tP1, -tVeh.yaw, tP1);
    RotateCoordinateOfPoint(tRotationCenter, tP2, -tVeh.yaw, tP2);
    RotateCoordinateOfPoint(tRotationCenter, tP3, -tVeh.yaw, tP3);
    RotateCoordinateOfPoint(tRotationCenter, tVeh, -tVeh.yaw, tNewVeh);

    p0.x = tP0.x - tNewVeh.x;
    p0.y = tP0.y - tNewVeh.y;

    p1.x = tP1.x - tNewVeh.x;
    p1.y = tP1.y - tNewVeh.y;

    p2.x = tP2.x - tNewVeh.x;
    p2.y = tP2.y - tNewVeh.y;

    p3.x = tP3.x - tNewVeh.x;
    p3.y = tP3.y - tNewVeh.y;

	return ;
}

/*
@brief:斜列车位转换为矩形做显示。
@param:nMode 1-->不做转换
*/
void ObliqueSlot2Display(int nSlotPosition, LocationPoint &p0, LocationPoint &p1, LocationPoint &p2, LocationPoint &p3, bool nMode)
{
    LocationPoint tP0;
    LocationPoint tP1;
    LocationPoint tP2;
    LocationPoint tP3;

    tP0 = p0;
    tP1 = p1;
    tP2 = p2;
    tP3 = p3;

	float fP0P1Angle = atan((p0.y - p1.y)/(p0.x-p1.x));	

    if(nSlotPosition== RIGHTSIDESLOT)
    {
        float fSlotAngle = acos(((p1.x - p0.x)*(p2.x - p1.x) + (p1.y - p0.y)*(p2.y - p1.y))/hypot(p1.x - p0.x, p1.y - p0.y)/hypot(p2.x - p1.x, p2.y - p1.y));
        float fSlotWidth = hypot(p0.x - p1.x, p0.y - p1.y);
        float fCut = fabs(fSlotWidth*cosf(fSlotAngle));// /2.f
        if (nMode)
        {
            /* code */
            fCut = 0.f;
        }
        if(fSlotAngle < M_PI/2.f)//right fish
        {
            // tP0.x = -(fSlotDepth - fCut)*cosf(-M_PI + fSlotAngle) + tP3.x;
            // tP0.y = -(fSlotDepth - fCut)*sinf(-M_PI + fSlotAngle) + tP3.y;

            tP0.x += fCut*cosf(-M_PI + fSlotAngle + fP0P1Angle);
            tP0.y += fCut*sinf(-M_PI + fSlotAngle + fP0P1Angle);
			tP2.x -= fCut*cosf(-M_PI + fSlotAngle + fP0P1Angle);
            tP2.y -= fCut*sinf(-M_PI + fSlotAngle + fP0P1Angle);


#if 0
            tP3.x += fCut*cosf(-M_PI + fSlotAngle);
            tP3.y += fCut*sinf(-M_PI + fSlotAngle);

            tP1.x -= fCut*cosf(-M_PI + fSlotAngle);
            tP1.y -= fCut*sinf(-M_PI + fSlotAngle);
            tP2.x -= fCut*cosf(-M_PI + fSlotAngle);
            tP2.y -= fCut*sinf(-M_PI + fSlotAngle);
			
#endif
        }
        else//right antifish
        {
            // tP1.x = -(fSlotDepth - fCut)*cosf(-M_PI + fSlotAngle) + tP2.x;
            // tP1.y = -(fSlotDepth - fCut)*sinf(-M_PI + fSlotAngle) + tP2.y;

            tP1.x += fCut*cosf(-M_PI + fSlotAngle + fP0P1Angle);
            tP1.y += fCut*sinf(-M_PI + fSlotAngle + fP0P1Angle);  
            tP3.x -= fCut*cosf(-M_PI + fSlotAngle + fP0P1Angle);
            tP3.y -= fCut*sinf(-M_PI + fSlotAngle + fP0P1Angle);

#if 0
            tP2.x += fCut*cosf(-M_PI + fSlotAngle);
            tP2.y += fCut*sinf(-M_PI + fSlotAngle);     

            tP0.x -= fCut*cosf(-M_PI + fSlotAngle);
            tP0.y -= fCut*sinf(-M_PI + fSlotAngle);
            tP3.x -= fCut*cosf(-M_PI + fSlotAngle);
            tP3.y -= fCut*sinf(-M_PI + fSlotAngle);
#endif
        }
    }
    else
    {
        float fSlotAngle = acos(((p0.x - p1.x)*(p3.x - p0.x) + (p0.y - p1.y)*(p3.y - p0.y))/hypot(p0.x - p1.x, p0.y - p1.y)/hypot(p3.x - p0.x, p3.y - p0.y));
        float fSlotWidth = hypot(p0.x - p1.x, p0.y - p1.y);
        float fCut = fabs(fSlotWidth*cosf(fSlotAngle));// /2.f
        if (nMode)
        {
            /* code */
            fCut = 0.f;
        }
        if(fSlotAngle < M_PI/2.f)//left fish
        {
            // tP1.x = -(fSlotDepth - fCut)*cosf(M_PI - fSlotAngle) + tP2.x;
            // tP1.y = -(fSlotDepth - fCut)*sinf(M_PI - fSlotAngle) + tP2.y;

            tP1.x += fCut*cosf(M_PI - fSlotAngle + fP0P1Angle);
            tP1.y += fCut*sinf(M_PI - fSlotAngle + fP0P1Angle);  
            tP3.x -= fCut*cosf(M_PI - fSlotAngle + fP0P1Angle);
            tP3.y -= fCut*sinf(M_PI - fSlotAngle + fP0P1Angle);  

#if 0
            tP2.x += fCut*cosf(M_PI - fSlotAngle);
            tP2.y += fCut*sinf(M_PI - fSlotAngle);  

            tP0.x -= fCut*cosf(M_PI - fSlotAngle);
            tP0.y -= fCut*sinf(M_PI - fSlotAngle);
            tP3.x -= fCut*cosf(M_PI - fSlotAngle);
            tP3.y -= fCut*sinf(M_PI - fSlotAngle);
#endif
        }
        else//left antifish
        {
            // tP0.x = -(fSlotDepth - fCut)*cosf(M_PI - fSlotAngle) + tP3.x;
            // tP0.y = -(fSlotDepth - fCut)*sinf(M_PI - fSlotAngle) + tP3.y;

            tP0.x += fCut*cosf(M_PI - fSlotAngle + fP0P1Angle);
            tP0.y += fCut*sinf(M_PI - fSlotAngle + fP0P1Angle);
            tP2.x -= fCut*cosf(M_PI - fSlotAngle + fP0P1Angle);
            tP2.y -= fCut*sinf(M_PI - fSlotAngle + fP0P1Angle);

#if 0
            tP3.x += fCut*cosf(M_PI - fSlotAngle);
            tP3.y += fCut*sinf(M_PI - fSlotAngle);

            tP1.x -= fCut*cosf(M_PI - fSlotAngle);
            tP1.y -= fCut*sinf(M_PI - fSlotAngle);
            tP2.x -= fCut*cosf(M_PI - fSlotAngle);
            tP2.y -= fCut*sinf(M_PI - fSlotAngle);
#endif
        }   
    }

    p0 = tP0;
    p1 = tP1;
    p2 = tP2;
    p3 = tP3;
}


INT32 updateTargetSlotOnHdmi(MvTMVehPont *pVehPont, ApaToHdmiInfo *pApaTHdmiInfo, HdmiToApaInfo *pHdmiTApaInfo,MvUpdateSlotData*	pMvUpdateSlotData)
{
	INT32 nRet					= -1;
	if (gCurrParkingStatus == parkingErr)
	{
		pApaTHdmiInfo->tPlanToHdmi.uCmd = 1;
		pApaTHdmiInfo->tPlanToHdmi.uCurrentStep = 0;
		pApaTHdmiInfo->tPlanToHdmi.uTotalStep = 0;
		pApaTHdmiInfo->tPlanToHdmi.uDisStep = 0;
		gSlotTypeLockFlag = 2;
		gTargetPointHasReachGoalFlag = 3;
		nRet = -2;
		return nRet;
	}
	else if(gCurrParkingStatus == parkingOver)
	{
		pApaTHdmiInfo->tPlanToHdmi.uCmd = 1;
		pApaTHdmiInfo->tPlanToHdmi.uCurrentStep = 0;
		pApaTHdmiInfo->tPlanToHdmi.uTotalStep = 0;
		pApaTHdmiInfo->tPlanToHdmi.uDisStep = 0;
		gSlotTypeLockFlag = 2;
		gTargetPointHasReachGoalFlag = 3;
		nRet = -3;
		return nRet;
	}

	LocationPoint tCurrentPosion;
    tCurrentPosion.x = pVehPont->tMvVehPont.fx;
    tCurrentPosion.y = pVehPont->tMvVehPont.fy;
    tCurrentPosion.yaw = pVehPont->tMvVehPont.fyaw; // DDS DR

	LocationPoint p0,p1,p2,p3;
	
	Park2Oblique(tObliqueInput.cSlotPosition, 
	   			tCurrentPosion,
				tObliqueInput.RotationCoordinate,//rotation coordinate
				tObliqueInput.fSlotWidth,
				tObliqueInput.fSlotDepth,
				tObliqueInput.fSlotYaw,
				p0, p1, p2, p3, (tObliqueInput.cDetectType == USER_MANUL_TYPE));
	
	pApaTHdmiInfo->tSoltNum = 1;
	pApaTHdmiInfo->tSolt[0].nSlotId = gSlotId;
	
#if	1 //USE_MANAGESLOT_UPDATE_TARGETSLOT_ON_HMI  //暂时不可以开启，管理车位在踩刹车是会错乱
	
	#if PARIN_UPDATE_SLOT_FUSION_DATA
	if((tObliqueInput.cDetectType == MIX_TYPE) && (pMvUpdateSlotData->isUpdated == 1) && (gSlotOutput.nParkSlotNum != 0))/*vision*/
	{
			pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.x = pMvUpdateSlotData->mixSlotUpdateData[0].x;
			pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.y = pMvUpdateSlotData->mixSlotUpdateData[0].y;
			pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.x = pMvUpdateSlotData->mixSlotUpdateData[1].x;
			pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.y = pMvUpdateSlotData->mixSlotUpdateData[1].y;
			pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.x = pMvUpdateSlotData->mixSlotUpdateData[2].x;
			pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.y = pMvUpdateSlotData->mixSlotUpdateData[2].y;
			pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.x = pMvUpdateSlotData->mixSlotUpdateData[3].x;
			pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.y = pMvUpdateSlotData->mixSlotUpdateData[3].y;
			
			GAC_LOG_DEBUG("xc_update_hmi_slot:based on mixUpdateSlot (%f,%f)(%f,%f)(%f,%f)(%f,%f)\n",
			pMvUpdateSlotData->mixSlotUpdateData[0].x,pMvUpdateSlotData->mixSlotUpdateData[0].y,
			pMvUpdateSlotData->mixSlotUpdateData[1].x,pMvUpdateSlotData->mixSlotUpdateData[1].y,
			pMvUpdateSlotData->mixSlotUpdateData[2].x,pMvUpdateSlotData->mixSlotUpdateData[2].y,
			pMvUpdateSlotData->mixSlotUpdateData[3].x,pMvUpdateSlotData->mixSlotUpdateData[3].y
			);
	
	}
	#else		
	if(tObliqueInput.cDetectType == MIX_TYPE && gSlotOutput.nParkSlotNum != 0)/*vision*/
	{
		GAC_LOG_DEBUG("Jira1255 gSlotOutput: %f %f %f %f %f %f %f %f\n", 
				gSlotOutput.tParkSlot[0].tPoint0.tWorldPoint.x,
				gSlotOutput.tParkSlot[0].tPoint0.tWorldPoint.y,
				gSlotOutput.tParkSlot[0].tPoint1.tWorldPoint.x,
				gSlotOutput.tParkSlot[0].tPoint1.tWorldPoint.y,
				gSlotOutput.tParkSlot[0].tPoint2.tWorldPoint.x,
				gSlotOutput.tParkSlot[0].tPoint2.tWorldPoint.y,
				gSlotOutput.tParkSlot[0].tPoint3.tWorldPoint.x,
				gSlotOutput.tParkSlot[0].tPoint3.tWorldPoint.y
				);
		GAC_LOG_DEBUG("Jira1751 fTheta fBestScore %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d\n", 
				gSlotOutput.tParkSlot[0].fSlotTheta, 
				gSlotOutput.tParkSlot[0].fBestSlotScore, 
				gSlotOutput.tParkSlot[0].fAngle, 
				tCurrentPosion.x, tCurrentPosion.y, tCurrentPosion.yaw,
				p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, gSlotOutput.tParkSlot[0].nSlotId);

		pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.x = gSlotOutput.tParkSlot[0].tPoint0.tWorldPoint.x;
		pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.y = gSlotOutput.tParkSlot[0].tPoint0.tWorldPoint.y;
		pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.x = gSlotOutput.tParkSlot[0].tPoint1.tWorldPoint.x;
		pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.y = gSlotOutput.tParkSlot[0].tPoint1.tWorldPoint.y;
		pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.x = gSlotOutput.tParkSlot[0].tPoint2.tWorldPoint.x;
		pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.y = gSlotOutput.tParkSlot[0].tPoint2.tWorldPoint.y;
		pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.x = gSlotOutput.tParkSlot[0].tPoint3.tWorldPoint.x;
		pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.y = gSlotOutput.tParkSlot[0].tPoint3.tWorldPoint.y;
	}
	#endif
#else
	if(0)
	{
		
	}
#endif
	else
    {
		if(tObliqueInput.cDetectType == USER_MANUL_TYPE)
		{
			Park2Oblique(tObliqueInput.cSlotPosition, 
	   					tCurrentPosion,
						tObliqueInput.RotationCoordinate,//rotation coordinate
						tObliqueInput.fSlotWidth,
						tObliqueInput.fSlotDepth,
						tObliqueInput.fSlotYaw,
						p0, p1, p2, p3, (tObliqueInput.cDetectType == USER_MANUL_TYPE));

			GAC_LOG_INFO("jinweiqi1124 here 1 %f %f %f %f %f %f %f %f\n", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
			if(tObliqueInput.cSlotType == OBLIQUE || (tObliqueInput.cDetectType == USER_MANUL_TYPE && gIsNotPerdicular == 1))
			{
				//斜列车位
				ShrinkWidth(p0, p1, VEHICLE_WID, p0, p1);
				ShrinkWidth(p2, p3, VEHICLE_WID, p2, p3);

				if(fabsf(fabsf(tObliqueInput.fSlotYaw) - PI_2) > PI/15)
				{
					//解决斜列车位显示问题
					GAC_LOG_INFO("jinweiqi1124 here 2 %f %f %f %f %f %f %f %f\n", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
					ShrinkWidthOblique(p0, p3, VEHICLE_LEN, p0, p3);
					ShrinkWidthOblique(p1, p2, VEHICLE_LEN, p1, p2);
				}
				else
				{
					GAC_LOG_INFO("jinweiqi1124 here 3 %f %f %f %f %f %f %f %f\n", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
					ShrinkWidth(p0, p3, VEHICLE_LEN, p0, p3);
					ShrinkWidth(p1, p2, VEHICLE_LEN, p1, p2);
				}
			}
			else
			{
				GAC_LOG_INFO("jinweiqi1124 here 4 %f %f %f %f %f %f %f %f\n", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
				ShrinkWidth(p0, p1, VEHICLE_LEN, p0, p1);
				ShrinkWidth(p2, p3, VEHICLE_LEN, p2, p3);
				ShrinkWidth(p0, p3, VEHICLE_WID, p0, p3);
				ShrinkWidth(p1, p2, VEHICLE_WID, p1, p2);
			}
			GAC_LOG_INFO("jinweiqi1124 here 5 %f %f %f %f %f %f %f %f\n", p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);

		}
		pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.x = p0.x;
		pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.y = p0.y;
		pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.x = p1.x;
		pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.y = p1.y;
		pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.x = p2.x;
		pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.y = p2.y;
		pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.x = p3.x;
		pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.y = p3.y;
    }

	// lock oblique slot for display
	GAC_LOG_DEBUG("jira1328-2 gSlotTypeLockFlag %d   glockType %d\n", gSlotTypeLockFlag, glockType);
	if(gSlotTypeLockFlag == 1 && glockType == OrgPerceptionObliqueSlot)//lock first parking slot type
	{
		p0.x = pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.x;
		p0.y = pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.y;
		p1.x = pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.x;
		p1.y = pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.y;
		p2.x = pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.x;
		p2.y = pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.y;
		p3.x = pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.x;
		p3.y = pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.y;

		GAC_LOG_DEBUG("jira1328-2 lock and cut before_(%f,%f)(%f,%f)(%f,%f)(%f,%f)\n",p0.x,p0.y,p1.x,p1.y,p2.x,p2.y,p3.x,p3.y);
			
        int cSlotPosition = (p0.x > p1.x) ? RIGHTSIDESLOT:LEFTSIDESLOT;
			
		ObliqueSlot2Display(cSlotPosition,p0,p1,p2,p3,0);

		pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.x = p0.x;
		pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.y = p0.y;
		pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.x = p1.x;
		pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.y = p1.y;
		pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.x = p2.x;
		pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.y = p2.y;
		pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.x = p3.x;
		pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.y = p3.y;
			
		GAC_LOG_DEBUG("jira1328-2 lock and cut after_(%f,%f)(%f,%f)(%f,%f)(%f,%f)\n",p0.x,p0.y,p1.x,p1.y,p2.x,p2.y,p3.x,p3.y);
	}

	GAC_LOG_INFO("SlotId=%d, SoltType:%d, DetectType:%d, ParkSlotNum=%d\n",gSlotId,pHdmiTApaInfo->uSoltType,tObliqueInput.cDetectType,gSlotOutput.nParkSlotNum);
	GAC_LOG_DEBUG("jira523-jira1225 DrYaw:%f Drx:%f Dry:%f\n", pVehPont->tMvVehPont.fyaw, pVehPont->tMvVehPont.fx, pVehPont->tMvVehPont.fy);
	GAC_LOG_DEBUG("TargetSolt (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
					pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.x,pApaTHdmiInfo->tSolt[0].tPoint0.tWorldPoint.y,
					pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.x,pApaTHdmiInfo->tSolt[0].tPoint1.tWorldPoint.y,
					pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.x,pApaTHdmiInfo->tSolt[0].tPoint2.tWorldPoint.y,
					pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.x,pApaTHdmiInfo->tSolt[0].tPoint3.tWorldPoint.y);
	GAC_LOG_DEBUG("SlotYaw:%f x:%f y:%f Yaw:%f Depth:%f Width:%f SlotType:%d\n",tObliqueInput.fSlotYaw,tObliqueInput.fVehCurX,tObliqueInput.fVehCurY,
					tObliqueInput.fVehCurYaw,tObliqueInput.fSlotDepth,tObliqueInput.fSlotWidth,tObliqueInput.cSlotType);
		
		
	
	//添加自选车位
	
	if(pHdmiTApaInfo->uSoltType == 1)
	{
		pApaTHdmiInfo->tSolt[0].nAvailableState = 1;
	}		
	nRet = 0;
	return nRet; 
}
