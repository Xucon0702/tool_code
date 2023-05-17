/*
 * pnc_interact.h
 *
 *  Created on: 2019年4月15日
 *      Author: zhuguohua
 */

#ifndef PNCINTERACT_PNC_INTERACT_H_
#define PNCINTERACT_PNC_INTERACT_H_

#include <vector>
#include <assert.h>
#include "math.h"
#include "MvApaCommonStruct.h"
#include "PnCInfo.h"
#include "Track.h"
#include "pps_send_mcu.h"
#include "pps_apa_plan.h"

#ifdef GAC_A58

#define DDS_SEND_PATH_ATTRIBUTE    A58PortSendPathAttribute()
#define DDS_SEND_PATH_POINTS       A58PortSendPathPoints()
#define DDS_SEND_PATH_RESET_ATTRIBUTE A58PortSendPathResetAttribute()

#else

#define DDS_SEND_PATH_ATTRIBUTE    MvPpsSendApaPathAttribute(&DDS::_PathAttribute)
#define DDS_SEND_PATH_POINTS       A18PortSendPathPoints()
#define DDS_SEND_PATH_RESET_ATTRIBUTE A20PortSendPathResetAttribute()

#endif


using namespace std;

class PnCInteract
{
public:
	PnCInteract();
	virtual ~PnCInteract();

    void Init();

    int SendInfo(ParkingWorkingStatus& tParkingStatus, MvTMVehPont *pVehPont, bool bInit);

    void HandleFeedback(ParkingWorkingStatus& tParkingStatus);

    void SetTrack(vector<TargetTrack>& aTracksPlan, ParkingWorkingStatus& tParkingStatus, MvTMVehPont *pVehPont, bool bInit);

    void SendWarningType(int nFreespaceSts, int nType);

    void ResetPlan(void);

    void SendPlanError(const DDS::PlanError ePlanErr);

private:
    bool PosClose(const LocationPoint& tVel, float x, float y, float yaw);

    void UpdatePath(const LocationPoint& tVel, vector<TargetTrack>& aTracks);

    int CalculateNearestPointByPosition(const double x, const double y, vector<TargetTrack>& aTracks);

#ifdef GAC_A58
    void A58PortSendPathAttribute();
    void A58PortSendPathPoints();
	void A58PortSendPathResetAttribute();
#else
    void A18PortSendPathPoints();
	void A20PortSendPathResetAttribute();

#endif

    int m_nCurrentPos = 0;

    vector<TargetTrack> aTracks;

    LocationPoint VehPos;

    MvVehPont TransParams;

    bool bLastInit = false;
};

#endif /* PNCINTERACT_PNC_INTERACT_H_ */
