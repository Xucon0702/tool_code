#include<cstdio>
#include<iostream>
#include<unistd.h>

#include "updateTargetSlot.h"
#include "em_decision.h"

int main()
{

    bool runFlag = true;

    updateTargetSlotTest();

    GAC_LOG_DEBUG("over\n");
    // printf("over 222\n");
    // std::cout<<"hello"<<"\n";

    while(runFlag)
    {
        GAC_LOG_DEBUG("running\n");
        sleep(1);
    }


    return 0;
}

