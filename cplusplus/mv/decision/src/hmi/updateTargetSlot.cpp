#include "em_decision.h"

extern MvTMVehPont gVehPont;
extern ApaToHdmiInfo gApaTHdmiInfo;
extern HdmiToApaInfo gHdmiToApaInfo;
extern MvUpdateSlotData gMvUpdateSlotData;

extern InputParkingIn tObliqueInput;
extern ParkingWorkingStatus gCurrParkingStatus;
extern UINT32			gSlotId;//车位ID
extern MvSlotOutput    gSlotOutput;
extern UINT32 gSlotTypeLockFlag;
extern UINT32 glockType;
extern UINT32   gIsNotPerdicular;

int inputInit()
{
    printf("inputInit\n");
    memset(&gVehPont,0,sizeof(MvTMVehPont));
    memset(&gApaTHdmiInfo,0,sizeof(ApaToHdmiInfo));
    memset(&gHdmiToApaInfo,0,sizeof(HdmiToApaInfo));
    memset(&gMvUpdateSlotData,0,sizeof(MvUpdateSlotData));
    return 0;
}

int setTObliqueInput(float fSlotYaw,float fVehCurX,float fVehCurY,float fVehCurYaw,float fSlotDepth,float fSlotWidth,int cSlotType)
{
    tObliqueInput.fSlotYaw = fSlotYaw;
    tObliqueInput.fVehCurX = fVehCurX;
    tObliqueInput.fVehCurY = fVehCurY;
    tObliqueInput.fVehCurYaw = fVehCurYaw;
    tObliqueInput.fSlotDepth = fSlotDepth;
    tObliqueInput.fSlotWidth = fSlotWidth;    
    tObliqueInput.cSlotType = cSlotType;  //PARALLEL/OBLIQUE
    return 0;
}

int setVehDr(float yaw,float x,float y)
{
    gVehPont.tMvVehPont.fx = x;
    gVehPont.tMvVehPont.fy = y;
    gVehPont.tMvVehPont.fyaw = yaw;
    return 0;
}


int setViewSlot(MvSlotOutput &tSlotOutput,float p0x,float p0y,float p1x,float p1y,float p2x,float p2y,float p3x,float p3y)
{
    tSlotOutput.tParkSlot[0].tPoint0.tWorldPoint.x = p0x;
    tSlotOutput.tParkSlot[0].tPoint0.tWorldPoint.y = p0y;
    tSlotOutput.tParkSlot[0].tPoint1.tWorldPoint.x = p1x;
    tSlotOutput.tParkSlot[0].tPoint1.tWorldPoint.y = p1y;
    tSlotOutput.tParkSlot[0].tPoint2.tWorldPoint.x = p2x;
    tSlotOutput.tParkSlot[0].tPoint2.tWorldPoint.y = p2y;
    tSlotOutput.tParkSlot[0].tPoint3.tWorldPoint.x = p3x;
    tSlotOutput.tParkSlot[0].tPoint3.tWorldPoint.y = p3y;
    return 0;
}

int setInput()
{
    printf("setInput\n");
    
    gCurrParkingStatus = working; //泊入中

    //tObliqueInput属性
    tObliqueInput.cSlotPosition = 1; //1-->右侧;-1:左侧
    memset(&tObliqueInput.RotationCoordinate,0,sizeof(LocationPoint));
    // tObliqueInput.RotationCoordinate.x = 1;
    // tObliqueInput.RotationCoordinate.y = 1;
    // tObliqueInput.RotationCoordinate.yaw = 1;
    tObliqueInput.cDetectType = MIX_TYPE;
    
    //log SlotYaw: [updateTargetSlotOnHdmi:5769] SlotYaw:1.569769 x:0.882524 y:1.895499 Yaw:-0.043193 Depth:5.500000 Width:2.500000 SlotType:1
    setTObliqueInput(1.569769,0.882524,1.895499,-0.043193,5.500000,2.500000,1);

    //dr
    // setVehDr(0,0,0);

    //一些planInit中的设置
    gIsNotPerdicular = 1; //主要是自选车位使用，水平车位为0,自选车位正前正后也置为1
    gSlotTypeLockFlag = 1; //车位类型锁住标志
    glockType = OrgPerceptionVerticalSlot; //车位类型
    
    //hdmiTApaInfo
    
    gHdmiToApaInfo.uSoltType = 0;//0:视觉 1：自选 2：超声

    //视觉车位
    gSlotId = 27;

    gSlotOutput.nParkSlotNum = 1;

    if(gSlotOutput.nParkSlotNum)//有视觉结果
    {
        //set dr:log jira-523-jira-1225
        setVehDr(-0.043979,0.936258,1.893157);

        //set view slot data
        // gSlotOutput.nParkSlotNum = 1;
        setViewSlot(gSlotOutput,1.637359,-1.830462,-0.607668,-1.896060,-0.457590,-7.032343,1.786528,-6.935615);
        // setTObliqueInput(1.569769,0.882524,1.895499,-0.043193,5.500000,2.500000,1);
    }
    else//dr 跟踪结果
    {
        // gSlotOutput.nParkSlotNum = 0;
        gVehPont.tMvVehPont.fx = 0.882524;
        gVehPont.tMvVehPont.fy = 1.895499;
        gVehPont.tMvVehPont.fyaw = -0.043193;
        // setTObliqueInput(1.569769,0.882524,1.895499,-0.043193,5.500000,2.500000,1);
        setVehDr(-0.044561,0.989994,1.890773);
    }

    //man user
    #if 0
    tObliqueInput.cDetectType = USER_MANUL_TYPE;    
    #endif

    
    gApaTHdmiInfo.tSolt[0].nAvailableState = 1;


    return 0;
}

/*
    仿真输入输出核心思想:
    1、输入输出整合为一个整体存储,或在板端把所有需要的输入打印出来做保存
    2、自动截取log设置数据.脚本处理
    3、连通板子传输数据实时跑
*/
int setInputData_simulation()
{
    return 0;
}


int runTest()
{
    printf("runTest\n");
    updateTargetSlotOnHdmi(&gVehPont,&gApaTHdmiInfo,&gHdmiToApaInfo,&gMvUpdateSlotData);

    //可视化显示

    return 0;
}

int updateTargetSlotTest()
{
    inputInit();

    setInput();
    // setInputData_simulation();

    runTest();

    return 0;
}