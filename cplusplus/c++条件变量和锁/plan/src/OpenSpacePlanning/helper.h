/**
* @file constants.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 为规划模块提供公共调用函数
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/


#ifndef HELPER
#define HELPER
#include "math.h"




#include <cmath>
#include <algorithm>

#include "constants.h"

namespace Helper
{
/*!
    \brief The namespace that wraps helper.h
    \namespace Helper
*/


/*!
   \fn static inline float normalizeHeading
   \brief Normalizes a heading given in degrees to (0,2PI]
   \param t heading in degrees
*/
static float normalizeHeadingRad_0_2pi(float x)
{
    float v = fmod(x, M_PI*2);
    if (v < 0)
    {
        v += M_PI*2;
    }
    else
    {
        ;
    }
    return v;
}

/*!
   \fn static inline float normalizeHeading
   \brief Normalizes a heading given in degrees to (-PI,PI]
   \param t heading in degrees
*/
static float normalizeHeadingRad_Npi_Ppi(float x)
{
    float v = fmod(x, M_PI*2);
    if (v < -M_PI)
    {
        v += M_PI*2;
    }
    else if (v > M_PI)
    {
        v -= M_PI*2;
    }
    return v;
}


/*!
   \fn float clamp(float n, float lower, float upper)
   \brief Clamps a number between a lower and an upper bound
   \param t heading in rad
*/
static float clamp(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}

static float getAcos(float cosTheta){
    float Theta;
    float error=1e-8;
    if((cosTheta - 1)>error)
    {
        Theta = 0;}
    else if((cosTheta + 1)<error)
    {
        Theta = M_PI;}
    else
    {
        Theta = acos(clamp(cosTheta, - 1+error,1-error));
    }
    if(fabs(Theta)<1e-3)
    {
        Theta=0.0;
    }
    return Theta;
}


//排斥实验
static bool IsRectCross(const LocationPoint &p1,const LocationPoint &p2,const LocationPoint &q1,const LocationPoint &q2)
{
    bool ret = MIN(p1.x,p2.x) <= MAX(q1.x,q2.x) &&
            MIN(q1.x,q2.x) <= MAX(p1.x,p2.x) &&
            MIN(p1.y,p2.y) <= MAX(q1.y,q2.y) &&
            MIN(q1.y,q2.y) <= MAX(p1.y,p2.y);
    return ret;
}
//跨立判断
static bool IsLineSegmentCross(const LocationPoint &P1,const LocationPoint &P2,const LocationPoint &Q1,const LocationPoint &Q2)
{
    if(
            ((Q1.x-P1.x)*(Q1.y-Q2.y)-(Q1.y-P1.y)*( Q1.x-Q2.x)) * ((Q1.x-P2.x)*(Q1.y-Q2.y)-(Q1.y-P2.y)*(Q1.x-Q2.x)) < 0 ||
            ((P1.x-Q1.x)*(P1.y-P2.y)-(P1.y-Q1.y)*(P1.x-P2.x)) * ((P1.x-Q2.x)*(P1.y-P2.y)-(P1.y-Q2.y)*( P1.x-P2.x)) < 0
            )
        return true;
    else
        return false;
}

/**
求线段P1P2与Q1Q2的交点。
先进行快速排斥实验和跨立实验确定有交点再进行计算。
交点（x,y）使用引用返回。
没有验证过
**/
static bool GetCrossPoint(const LocationPoint &p1,const LocationPoint &p2,const LocationPoint &q1,const LocationPoint &q2, LocationPoint &interPoint)
{
    float tmpLeft,tmpRight;
    if(IsRectCross(p1,p2,q1,q2))
    {
        if (IsLineSegmentCross(p1,p2,q1,q2))
        {
            //求交点
            tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
            tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);
            interPoint.x = tmpRight/tmpLeft;

            tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
            tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x- p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
            interPoint.y = tmpRight/tmpLeft;
            return true;
        }
    }
    return false;
}

}


#endif // HELPER

