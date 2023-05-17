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
/**
* @file publicfunpathplan.cpp
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 规划算法的公共函数
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/

#include"publicfunpathplan.h"

namespace HybridAStarPart
{

float normalizeHeadingRad_0_2pi(float x)
{
    float v = fmod(x, PI*2);
    if (v < 0)
    {
        v += PI*2;
    }
    else
    {
        ;
    }
    return v;
}

float normalizeHeadingRad_Npi_Ppi(float x)
{
    float v = fmod(x, PI*2);
    if (v < -PI)
    {
        v += PI*2;
    }
    else if (v > PI)
    {
        v -= PI*2;
    }
    return v;
}

double clamp(double n, double lower, double upper)
{
    return std::max(lower, std::min(n, upper));
}

double getAcos(double cosTheta)
{
    double Theta;
    double error=1e-10f;
    if((cosTheta - 1)>error)
    {
        Theta = 0;}
    else if((cosTheta + 1)<error)
    {
        Theta = PI;}
    else
    {
        Theta = acos(clamp(cosTheta, - 1+error,1-error));
    }

    if(fabs(Theta) < 1e-4)
    {
        Theta = 0.0;
    }
    return Theta;
}


/**
 * @brief 排斥判断: p1p2为斜边的矩形以及q1q2为斜边的矩形是否重叠
 * @return 1-重叠 0-不重叠
 */
bool IsRectCross(const LocationPoint &p1,const LocationPoint &p2,const LocationPoint &q1,const LocationPoint &q2)
{
    bool ret = MIN(p1.x,p2.x) <= MAX(q1.x,q2.x) &&
               MIN(q1.x,q2.x) <= MAX(p1.x,p2.x) &&
               MIN(p1.y,p2.y) <= MAX(q1.y,q2.y) &&
               MIN(q1.y,q2.y) <= MAX(p1.y,p2.y);
    return ret;
}


/**
 * @brief  跨立判断: 判断一个线段是否相交
 * @return 1-相交 0-不相交
 */
bool IsLineSegmentCross(const LocationPoint &P1,
                        const LocationPoint &P2,
                        const LocationPoint &Q1,
                        const LocationPoint &Q2)
{
    float fVal1 = ((Q1.x - P1.x) * (Q1.y - Q2.y) - (Q1.y - P1.y) * (Q1.x - Q2.x)) * ((Q1.x - P2.x) * (Q1.y - Q2.y) - (Q1.y - P2.y) * (Q1.x - Q2.x));
    float fVal2 = ((P1.x - Q1.x) * (P1.y - P2.y) - (P1.y - Q1.y) * (P1.x - P2.x)) * ((P1.x - Q2.x) * (P1.y - P2.y) - (P1.y - Q2.y) * (P1.x - P2.x));

    /*矢量法判断一个线段是否跨越另一条线段*/
    if(fVal1 < 0 && fVal2 < 0)
        return true;
    else
        return false;
}

/**
 * @brief 求线段P1P2与Q1Q2的交点。
 * @param interPoint 交点
 * @return 1-相交 0-不相交
 */
bool GetCrossPoint(const LocationPoint &p1,const LocationPoint &p2,const LocationPoint &q1,const LocationPoint &q2, LocationPoint &interPoint)
{
    float tmpLeft,tmpRight;
    /*先进行快速排斥实验和跨立实验确定有交点再进行计算。*/
    if(IsRectCross(p1,p2,q1,q2))
    {
        if (IsLineSegmentCross(p1,p2,q1,q2))
        {
            //求交点
            tmpLeft = (q2.x - q1.x) * (p1.y - p2.y) - (p2.x - p1.x) * (q1.y - q2.y);
            tmpRight = (p1.y - q1.y) * (p2.x - p1.x) * (q2.x - q1.x) + q1.x * (q2.y - q1.y) * (p2.x - p1.x) - p1.x * (p2.y - p1.y) * (q2.x - q1.x);
            interPoint.x = tmpRight / tmpLeft;

            tmpLeft = (p1.x - p2.x) * (q2.y - q1.y) - (p2.y - p1.y) * (q1.x - q2.x);
            tmpRight = p2.y * (p1.x - p2.x) * (q2.y - q1.y) + (q2.x- p2.x) * (q2.y - q1.y) * (p1.y - p2.y) - q2.y * (q1.x - q2.x) * (p2.y - p1.y);
            interPoint.y = tmpRight / tmpLeft;
            return true;
        }
    }
    return false;
}


/**
 * @brief 圆弧插值
 * @param nDrivingDirection 行驶方向
 * @param nSteeringDirection 转动方向
 * @param kappa 曲率
 * @param fCurveLength 曲线长度
 * @param tPoint 起点
 * @param targetSeg 输出结果
 */
void CircleInterpolate(int8_t nDrivingDirection,
                       int8_t nSteeringDirection,
                       float kappa,
                       float fCurveLength,
                       LocationPoint tPoint,
                       std::vector<CtrlPoint> &targetSeg)
{
    CtrlPoint CtrlPoint_;
    float fStepSize = Constants::step_size;
    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;
    float fLastx = 0.0;
    float fLasty = 0.0;
    float fLastyaw = 0.0;
    int nSegNum = 0;

    targetSeg.clear();

    kappa = nSteeringDirection * fabsf(kappa);
    nSegNum = round(fCurveLength / fStepSize);

    fLastx = tPoint.x;
    fLasty = tPoint.y;
    fLastyaw = normalizeHeadingRad_Npi_Ppi(tPoint.yaw);

    CtrlPoint_.x = fLastx;
    CtrlPoint_.y = fLasty;
    if(nDrivingDirection == FORWARD)
    {
        CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(fLastyaw);
    }
    else
    {
        CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(fLastyaw + PI);
        fStepSize = -1 * fStepSize;
    }
    CtrlPoint_.kappa = kappa * nDrivingDirection;
    CtrlPoint_.veh_speed = 0;
    CtrlPoint_.driving_direction = nDrivingDirection;

    targetSeg.push_back(CtrlPoint_);

    for(int i = 1; i <= nSegNum; i++)
    {
        x = fLastx + fStepSize * cosf(fLastyaw + fStepSize * kappa / 2);
        y = fLasty + fStepSize * sinf(fLastyaw + fStepSize * kappa / 2);
        yaw = fLastyaw + fStepSize * kappa;

        CtrlPoint_.x = x;
        CtrlPoint_.y = y;
        if(nDrivingDirection == FORWARD)
        {
            CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(yaw);
        }
        else
        {
            CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(yaw + PI);
        }
        CtrlPoint_.kappa = kappa * nDrivingDirection;

        CtrlPoint_.veh_speed = 0;
        CtrlPoint_.driving_direction = nDrivingDirection;

        targetSeg.push_back(CtrlPoint_);

        fLastx = x;
        fLasty = y;
        fLastyaw = yaw;
    }
}

/**
 * @brief 速度规划
 * @param target_curvature_sets 目标控制点集合
 */
void VelocityPlanning(std::vector<CtrlPoint>& targetCtrlPointSets)
{
    float s= 0.0;
    float s0 = 0.0;
    float fAcc = 0.0;
    float x = 0.0;
    float y = 0.0;
    float fLastx = 0.0;
    float fLasty = 0.0;
    float vmax = 0.0;
    float fVehSpeed = 0.0;
    float fMaxCtrlSpeed = 0.0;
    float fStepSize = 0.0;
    float fKappa = 0.0;
    float fLastKappa = 0.0;
    int nDetaSteeingAngle = 0.0;
    int nLastSteeingAngle = 0.0;
    int nSteeingAngle = 0.0;

    int8_t nDrivingDirection = 0;
    int8_t nSteeringDirection = 0;
    bool bFindV = false;
    if(targetCtrlPointSets.size() <= 1)
    {

    }
    else
    {
        float discretS[targetCtrlPointSets.size()];
        nDrivingDirection = targetCtrlPointSets[0].driving_direction;
//        if(nDrivingDirection == FORWARD)
//        {
//            fAcc = FORWARD_ACC;
//        }
//        else
//        {
//            fAcc = BACKWARD_ACC;
//        }
        fLastx = targetCtrlPointSets[0].x;
        fLasty = targetCtrlPointSets[0].y;
        fStepSize = hypotf(targetCtrlPointSets[1].x - fLastx, targetCtrlPointSets[1].y - fLasty);
        /*计算距离*/
        for(size_t i = 0; i < targetCtrlPointSets.size(); ++i)
        {
            x = targetCtrlPointSets[i].x;
            y = targetCtrlPointSets[i].y;
            s += hypotf(x - fLastx, y - fLasty);
            fLastx = x;
            fLasty = y;
            discretS[i] = s;
        }

        if(s > 1.5f)
        {
            fAcc = FORWARD_ACC;
        }
        else
        {
            fAcc = BACKWARD_ACC;
        }

        /*计算最大速度*/
        if(s > LOW_SPEED_DIS_LIMIT)
        {
            for(float v = MAX_PLANNING_VELOCITY; v >= MIN_PLANNING_VELOCITY - CAL_ERROR; v -= 0.05)
            {

                if( v * v / 2 /fAcc < s /*- MIN_DIS_VPLAN*/)
                {
                    s0 = s - v * v / 2 / fAcc;
                    vmax = v;
                    bFindV = true;
                    break;
                }
                else
                {
                    s0 = 0;
                    vmax = v;
                }
            }
        }
        else
        {
            vmax = MIN_PLANNING_VELOCITY;
        }

        /*速度规划*/
        for(size_t i = 0; i < targetCtrlPointSets.size(); ++i)
        {
            /*计算按照行驶距离的规划速度*/
            if(bFindV)
            {
                if(discretS[i] < s0)
                {
                    fVehSpeed = vmax;
                }
                else
                {
                    fVehSpeed = sqrt((s - discretS[i]) * 2 * fAcc);
                }
            }
            else
            {
                fVehSpeed = vmax;
            }

            /*前进速度调高点*/
//            if(nDrivingDirection == FORWARD)
//            {
//                fVehSpeed = MAX(fVehSpeed, MIN_PLANNING_VELOCITY+0.05);
//            }
//            else
            {
                fVehSpeed = MAX(fVehSpeed, MIN_PLANNING_VELOCITY);
            }

            /*计算当前点和上一点的行驶曲率*/
            if(i == 0)
            {
                fLastKappa = fabsf(targetCtrlPointSets[i].kappa);
            }
            else
            {
                fLastKappa = fabsf(targetCtrlPointSets[i - 1].kappa);
            }
            fKappa = fabsf(targetCtrlPointSets[i].kappa);

            /*计算当前点和上一点的对应的方向盘转角*/
            nSteeringDirection = SgnCus(fKappa) * nDrivingDirection;
            if(fabsf(fKappa) < 0.001f)
            {
                nSteeingAngle = 0;
            }
            else
            {
                nSteeingAngle = nSteeringDirection * CalRadius2SteeringWheelAngle(nSteeringDirection, nDrivingDirection, 1 / fKappa);
            }

            nSteeringDirection = SgnCus(fLastKappa) * nDrivingDirection;
            if(fabsf(fLastKappa) < 0.001f)
            {
                nLastSteeingAngle = 0;
            }
            else
            {
                nLastSteeingAngle = nSteeringDirection * CalRadius2SteeringWheelAngle(nSteeringDirection, nDrivingDirection, 1 / fLastKappa);
            }
            nDetaSteeingAngle = abs(nSteeingAngle - nLastSteeingAngle);

            /*根据当前点和上一点的转角差和方向盘的最大角速度、两点的行驶距离计算最大控制车速*/
            if(nDetaSteeingAngle != 0)
            {
                fMaxCtrlSpeed = MAX(MIN(fStepSize * PLAN_MAX_STEERING_ANGLE_SPEED / nDetaSteeingAngle, MAX_PLANNING_VELOCITY), MIN_PLANNING_VELOCITY);
            }
            else
            {
                fMaxCtrlSpeed = MAX_PLANNING_VELOCITY;
            }

            if(fMaxCtrlSpeed < fVehSpeed + CAL_ERROR)
            {
                size_t nStart = 0;
                if(i >= CHANGE_SPEED_NUM)
                {
                    nStart = i - CHANGE_SPEED_NUM;
                }
                else
                {
                    nStart = 0;
                }
                for(size_t nCurrIndex = nStart; nCurrIndex <= i; ++nCurrIndex)
                {
                    targetCtrlPointSets[nCurrIndex].veh_speed = fMaxCtrlSpeed * nDrivingDirection;
                }
            }
            else
            {
                fVehSpeed = MIN(fMaxCtrlSpeed, fVehSpeed);
                targetCtrlPointSets[i].veh_speed = fVehSpeed * nDrivingDirection;
            }
        }

#ifdef PrintKappa
        for(size_t i=0; i< targetCtrlPointSets.size(); i++)
        {
            GAC_LOG_DEBUG("k: %f, v: %f\n", targetCtrlPointSets[i].kappa, targetCtrlPointSets[i].veh_speed);
        }
#endif

    }
}


/**
 * @brief 计算离散曲率和速度规划
 * @param targetCtrlPointSets 目标控制点集合,最少三个点
 */
void CalDiscretePointCurvature(std::vector<CtrlPoint>& targetCtrlPointSets)
{
    std::vector<CtrlPoint> PartitionedSegment = targetCtrlPointSets;
    std::vector<Vector2D> x;
    CtrlPoint CtrlPoint_;
    Vector2D detaXi(0, 0);
    Vector2D detaXip1(0, 0);
    Vector2D xi(0, 0);

    int nRotaDir = 0;
    double fYawi = 0;
    double fYawip1 = 0;
    double fNormdeltaXi = 0;
    double fNormdeltaXip1 = 0;
    double detaFi;
    double k = 0;

    x.clear();
    targetCtrlPointSets.clear();

    GAC_LOG_INFO("start print kappa!!!!!!!!\n");

    if(PartitionedSegment.size() == 0)
    {

    }
    else
    {
        for (size_t i = 0; i < PartitionedSegment.size(); ++i)
        {
            xi.setXY(PartitionedSegment[i].x, PartitionedSegment[i].y);
            x.push_back(xi);
        }

        for(size_t i = 0; i < PartitionedSegment.size(); ++i)
        {
            if(i == 0)
            {
                CtrlPoint_.x = PartitionedSegment[i].x;
                CtrlPoint_.y = PartitionedSegment[i].y;
                detaXip1 = (x[i + 1] - x[i]);
                fYawip1 = atan2f(detaXip1.getY(),detaXip1.getX());
                CtrlPoint_.yaw = fYawip1;
                CtrlPoint_.kappa = 0;
            }
            else if(i == PartitionedSegment.size()-1)
            {
                /*最后一个点的曲率和倒数第二个相同*/
                CtrlPoint_.x = PartitionedSegment[i].x;
                CtrlPoint_.y = PartitionedSegment[i].y;
                detaXi = (x[i] - x[i-1]);
                fYawi = atan2f(detaXi.getY(),detaXi.getX());
                CtrlPoint_.yaw = fYawi;
                CtrlPoint_.kappa = targetCtrlPointSets[i - 1].kappa;
            }
            else
            {
                CtrlPoint_.x = PartitionedSegment[i].x;
                CtrlPoint_.y = PartitionedSegment[i].y;
                detaXi = x[i] - x[i - 1];
                detaXip1 = (x[i + 1] - x[i]);
//                fYawi = atan2(detaXi.getY(),detaXi.getX());
                fYawip1 = atan2f(detaXip1.getY(),detaXip1.getX());
                fNormdeltaXi = detaXi.length();
                fNormdeltaXip1 = detaXip1.length();
//                detaXi = detaXip1;
                detaFi = getAcos(detaXi.dot(detaXip1) / (fNormdeltaXi * fNormdeltaXip1));
                k = detaFi / fNormdeltaXi;
                if(fabsf(k) < 1e-2f)
                {
                    k = 0.0f;
                }
                nRotaDir = SgnCus(detaXi.cross(detaXip1));
                k =  nRotaDir * k;
                CtrlPoint_.kappa = k;
                CtrlPoint_.yaw = fYawip1;
                /*第一个点的曲率和第二个相同*/
                if(i == 1)
                {
                    targetCtrlPointSets[0].kappa = k;
                }
            }
            CtrlPoint_.driving_direction = PartitionedSegment[i].driving_direction;
            targetCtrlPointSets.push_back(CtrlPoint_);
        }
#ifdef PrintKappa
        for( int i=0; i < targetCtrlPointSets.size(); i++)
        {
            GAC_LOG_DEBUG("k: %f",targetCtrlPointSets[i].kappa);
        }
#endif
        GAC_LOG_INFO("stop print kappa!!!!!!!!!!!\n");
    }

}

bool JudgeOneVectorIsInMiddleOfOtherTwoVectors(Vector2D vec1, Vector2D vec2, Vector2D vec, int nClockDir)
{
    bool bRet = false;
    if(nClockDir == 1)
    {
        if(vec1.cross(vec2) >= 0)
        {
            if((vec1.cross(vec) > 0) && (vec.cross(vec2) > 0)) { bRet = true;}
        }
        else
        {
            if((vec1.cross(vec) > 0) || (vec.cross(vec2) > 0)) { bRet = true;}
        }
    }
    else
    {
        if(vec1.cross(vec2) <= 0)
        {
            if((vec1.cross(vec) < 0) && (vec.cross(vec2) < 0)) { bRet = true;}
        }
        else
        {
            if((vec1.cross(vec) < 0) || (vec.cross(vec2) < 0)) { bRet = true;}
        }
    }
    return bRet;
}

/**
 * @brief 计算含偏航角点的直线方程点的直线方程
 */
void CalLineEquation(LocationPoint tp, LineEq  &LineEq_)
{
    float x1, x2, y1, y2 = 0.0;

    x1 = tp.x;
    y1 = tp.y;
    x2 = tp.x + 1 * cosf(tp.yaw);
    y2 = tp.y + 1 * sinf(tp.yaw);

    LineEq_.A = y2 - y1;
    LineEq_.B = x1 - x2;
    LineEq_.C = y1 * (x2 - x1) - (y2 - y1) * x1;
}

float CalPointAndLineDis(const cpoint &tp, const LineEq  &LineEq_)
{
    float dis = 0.0f;
    dis = fabsf(LineEq_.A * tp.x + LineEq_.B * tp.y + LineEq_.C) / sqrtf(LineEq_.A * LineEq_.A + LineEq_.B * LineEq_.B);
    return dis;
}

bool LeftOfLine(LocationPoint tp1,LocationPoint tp2, LocationPoint tCheckPoint)
{
    bool bLeft = false;
    float fTemp = (tp1.y - tp2.y) * tCheckPoint.x
                  + (tp2.x - tp1.x) * tCheckPoint.y + tp1.x * tp2.y - tp2.x * tp1.y;
    if(fTemp > 0)
    {
        bLeft = true;
    }
    return bLeft;
}

/**
 * @brief 直线交点
 */
bool CalLineCrossPoint(LineEq  LineEq_1, LineEq  LineEq_2, LocationPoint &tp)
{
    float D = LineEq_1.A * LineEq_2.B - LineEq_2.A * LineEq_1.B;
    if(fabsf(D) < CAL_ERROR)
    {
        /*两个直线平行*/
        return false;
    }
    else
    {
        tp.x = (LineEq_1.B * LineEq_2.C - LineEq_2.B * LineEq_1.C) / D;
        tp.y = (LineEq_2.A * LineEq_1.C - LineEq_1.A * LineEq_2.C) / D;
    }
    return true;
}

/**
 * @brief 圆和直线的交点Ax + By + C = 0与（x - X）^2 + (y - Y）^2 = R^2交点1和交点2
 */
bool InterPointBetweenCircleAndLine(Circle circle,
                                    LineEq line,
                                    std::vector<LocationPoint> &Points)
{
    float X, Y, R, A, B, C = 0.0;
    float a = 0.0;
    float b = 0.0;
    float c = 0.0;
    float fTemp = 0.0;
    bool bRet = true;
    LocationPoint p1, p2;

    X = circle.x;
    Y = circle.y;
    R = circle.R;
    A = line.A;
    B = line.B;
    C = line.C;
    Points.clear();

    if(fabs(A) <= CAL_ERROR)
    {
        fTemp = pow(R, 2) - pow(-C / B - Y, 2);
        if(fTemp >= 0)
        {
            if(fabs(B) > CAL_ERROR)
            {
                p1.x = X + sqrt(fTemp);
                p1.y = -C / B;
                p2.x = X - sqrt(fTemp);
                p2.y = -C / B;
                Points.push_back(p1);
                Points.push_back(p2);
            }
            else
            {
                bRet = false;
            }
        }
        else
        {
            bRet = false;
        }
    }
    else
    {
        a = pow(B / A, 2) + 1;
        b = 2 * (C / A + X) * B / A - 2 * Y;
        c = pow(C / A + X, 2) + pow(Y, 2) - pow(R, 2);

        fTemp = b * b - 4 * a * c;
        if(fTemp > CAL_ERROR)
        {
            p1.y = (-b + sqrt(fTemp)) / (2.0 * a);
            p1.x = (-C - B * p1.y) / A;
            p2.y = (-b - sqrt(fTemp)) / (2.0 * a);
            p2.x = (-C - B * p2.y) / A;
            Points.push_back(p1);
            Points.push_back(p2);
        }
        else if(fTemp > -CAL_ERROR)
        {
            p1.y = (-b) / (2.0 * a);
            p1.x = (-C - B * p1.y) / A;
            Points.push_back(p1);
        }
        else
        {
            bRet = false;
        }
    }
    return bRet;
}


/**
 * @brief 计算车辆左右后视镜位置点坐标
 * @param tVehPos 后轴坐标
 * @param tMirror 后视镜外侧点坐标
 */
void CalMirrorPointOfVehicle(const LocationPoint &tVehPos, LocationPoint tMirror[2])
{
    LocationPoint tPoint[2];
    LocationPoint tOriginPoint;
    float fSinYaw = sinf(tVehPos.yaw);
    float fCosYaw = cosf(tVehPos.yaw);

    tOriginPoint.x = 0;
    tOriginPoint.y = 0;
    tPoint[0].x = Constants::RearViewMirrorToCenter - Constants::RearViewWidth;
    tPoint[0].y = Constants::width / 2 + Constants::RearViewLength;
    tPoint[1].x = tPoint[0].x;
    tPoint[1].y = -tPoint[0].y;

    for(int i = 0; i < 2; ++i)
    {
        GetNewPointOfRotatation(tOriginPoint,
                                tPoint[i],
                                fSinYaw,
                                fCosYaw,
                                tMirror[i]);
        tMirror[i].x += tVehPos.x;
        tMirror[i].y += tVehPos.y;
    }
}

/**
 * @brief 计算后视镜等效直线
 * @param tVehPos 后轴坐标
 * @param mirrorLine 后视镜等效直线
 */
void CalMirrorLineOfVehicle(const LocationPoint &tVehPos, LineSeg mirrorLine[2])
{
    LocationPoint tMirror[2];
    float fSinYaw = sinf(tVehPos.yaw);
    float fCosYaw = cosf(tVehPos.yaw);

    CalMirrorPointOfVehicle(tVehPos, tMirror);

    mirrorLine[0].p0.x = tMirror[0].x + Constants::RearViewLength * fSinYaw;
    mirrorLine[0].p0.y = tMirror[0].y - Constants::RearViewLength * fCosYaw;
    mirrorLine[0].p1.x = tMirror[0].x;
    mirrorLine[0].p1.y = tMirror[0].y;

    mirrorLine[1].p0.x = tMirror[1].x - Constants::RearViewLength * fSinYaw;
    mirrorLine[1].p0.y = tMirror[1].y + Constants::RearViewLength * fCosYaw;
    mirrorLine[1].p1.x = tMirror[1].x;
    mirrorLine[1].p1.y = tMirror[1].y;

}
/**
 * @brief 点到点距离
 */
float CalPointDis(LocationPoint p1, LocationPoint p2)
{
    return (hypotf(p1.x - p2.x, p1.y - p2.y));
}

/**
 * @brief 判断点在车辆轮廓内
 */
bool pointInVehicle(const LocationPoint &vehPos, const cpoint &p)
{
    LocationPoint tCorner[4];
    CalVehicleCornerPoint(vehPos, tCorner);

    int polySides = 4;
    float polyX[4] = {tCorner[0].x, tCorner[1].x, tCorner[2].x, tCorner[3].x};
    float polyY[4] = {tCorner[0].y, tCorner[1].y, tCorner[2].y, tCorner[3].y};

    return(pointInPolygon(p.x, p.y, polySides, polyX, polyY));
}

/**
 * @brief 判断点在多边形内
 */
bool pointInPolygon(float x, float y, int polySides, float* polyX, float* polyY)
{

  int   i,j=polySides-1;
  bool  oddNodes=false;
  for (i=0;i<polySides; i++)
  {
    if(((polyY[i]< y && polyY[j]>=y)||(polyY[j]<y && polyY[i]>=y))&& (polyX[i]<=x || polyX[j]<=x))
    {
      oddNodes^=(polyX[i]+(y-polyY[i])/(polyY[j]-polyY[i])*(polyX[j]-polyX[i])<x);
    }
    j=i;
  }
  return oddNodes;
}

/**
 * @brief CalSafeDis 计算安全距离
 * @param tR 车位后角点
 * @param tF 车位前角点
 * @param slotCenterLineEq 车位中线方程
 * @return fSafeDis 安全距离
 */
void CalSafeDis(const cpoint &tR, const cpoint &tF, const LineEq &slotCenterLineEq,
                 float &fVehContourSafeDis,float &fMirrorSafeDis)
{
    float fWidth = 0.0f;

    /*取两侧角点对中线的较小距离作为车位综合可用宽度*/
    fWidth = 2 * MIN(CalPointAndLineDis(tF, slotCenterLineEq), CalPointAndLineDis(tR, slotCenterLineEq));

#ifdef BYD_HAN
    if(fWidth > Constants::width + 0.5f)
    {
        fVehContourSafeDis = MIN(MAX(((fWidth - Constants::width) / 2) * 0.4f, 0.15f), 0.25);
        fMirrorSafeDis = MAX(MIN(((fWidth - Constants::width) / 2 - Constants::RearViewLength) * 0.5f, 0.2f), 0.01);
    }
    else
    {
        fVehContourSafeDis = MAX((fWidth - Constants::width) / 2 - 0.15f, 0.05f);
        fMirrorSafeDis = 0.0f;
    }

#else
    if(fWidth > Constants::width + 0.5f)
    {
        fVehContourSafeDis = MIN(MAX(((fWidth - Constants::width) / 2) * 0.4f, 0.15f), 0.25);
        fMirrorSafeDis = MAX(MIN(((fWidth - Constants::width) / 2 - Constants::RearViewLength) * 0.5f, 0.2f), 0.01);
    }
    else
    {
        fVehContourSafeDis = MAX((fWidth - Constants::width) / 2 - 0.1f, 0.1f);
        fMirrorSafeDis = 0.0f;
    }
#endif

    return;

}

/**
 * @brief 车位角点和车辆轮廓是否碰撞
 * @param tVehPos 车辆坐标
 * @param tR 车位后角点
 * @param tF 车位前角点
 * @param fSafeDis 安全距离
 * @return 1-碰 0-安全
 */
bool CollisionCheckByVehKeyPointAndSlotCorner(const LocationPoint &tVehPos,
                                              const cpoint &tR,
                                              const cpoint &tR2,
                                              const cpoint &tF,
                                              const cpoint &tF2,
                                              float fSafeDisOfVehContour,
                                              float fSafeDisOfMirror,
                                              bool bHasSlotBottomLine)
{
    float fDis = 0.0;
    bool bColl = false;
    bool bVehContourColl = false;
    bool bVehMirrorColl = false;

    bool bBreak = false;
    int nIndex = 0;
    LocationPoint tCorner[4];
    LineSeg mirrorLine[2];
    LineSeg vehContourLineSeg[4];
    LineSeg slotLineSeg[3];
    LineSeg LineSeg_[2];


//    QTime T1;
//    T1.restart();
//    /*计算车辆轮廓线段*/
//    for(int i = 0; i < 100e4; ++i)
//    {
//        CalCornerCoordinate(tVehPos, tCorner[0], tCorner[1], tCorner[2], tCorner[3]);
//    }
//    GAC_LOG_DEBUG("time is %d", T1.restart());

    CalVehicleCornerPoint(tVehPos, tCorner);
    /*计算车辆轮廓线段*/
    for(int i = 0; i < 4; ++i)
    {
        vehContourLineSeg[i].p0.x = tCorner[i].x;
        vehContourLineSeg[i].p0.y = tCorner[i].y;
        if(i != 3)
        {
            nIndex = i + 1;
        }
        else
        {
            nIndex = 0;
        }
        vehContourLineSeg[i].p1.x = tCorner[nIndex].x;
        vehContourLineSeg[i].p1.y = tCorner[nIndex].y;
    }

    /*计算车位线段*/
    slotLineSeg[0].p0.x = tR.x;
    slotLineSeg[0].p0.y = tR.y;
    slotLineSeg[0].p1.x = tR2.x;
    slotLineSeg[0].p1.y = tR2.y;
    slotLineSeg[1].p0.x = tF.x;
    slotLineSeg[1].p0.y = tF.y;
    slotLineSeg[1].p1.x = tF2.x;
    slotLineSeg[1].p1.y = tF2.y;

    slotLineSeg[2].p0.x = tR2.x;
    slotLineSeg[2].p0.y = tR2.y;
    slotLineSeg[2].p1.x = tF2.x;
    slotLineSeg[2].p1.y = tF2.y;


    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 4; ++j)
        {
            if((!bHasSlotBottomLine) && (i == 2))
            {
                continue;
            }
            LineSeg_[0] = slotLineSeg[i];
            LineSeg_[1] = vehContourLineSeg[j];
            fDis = CalDisBetweenLineSegments(LineSeg_);
            if(fDis < fSafeDisOfVehContour)
            {
                bVehContourColl = true;
                bBreak = true;
                break;
            }
        }
        if(bBreak)
        {
            break;
        }
    }

    if(!bVehContourColl && (fabsf(fSafeDisOfMirror) > 1e-3f))
    {
        CalMirrorLineOfVehicle(tVehPos, mirrorLine);
        for(int i = 0; i < 2; ++i)
        {
            for(int j = 0; j < 2; ++j)
            {
                LineSeg_[0] = slotLineSeg[i];
                LineSeg_[1] = mirrorLine[j];
                fDis = CalDisBetweenLineSegments(LineSeg_);
                if(fDis < fSafeDisOfMirror)
                {
                    bVehMirrorColl = true;
                    bBreak = true;
                    break;
                }
            }
            if(bBreak)
            {
                break;
            }
        }
    }
    bColl = bVehContourColl || bVehMirrorColl;
    return bColl;
}

/**
 * @brief 矢量单位化
 */
void normalizeVec(Vector2D &vec)
{
    float fInverselength = 1 / vec.length();
    vec.setXY(vec.getX() * fInverselength, vec.getY() * fInverselength);
    return;
}

/**
 * @brief 判断点投影在线段上
 * @param LineSeg_ 线段
 * @param point 点
 * @return 1-投影点在线段上， 0-投影点在线段外
 */
bool CheckPointProjectionInLineSegment(const LineSeg &LineSeg_, cpoint point)
{
    float fVal = 0.0f;
    bool bProjInLine = false;
    Vector2D BA(point.x - LineSeg_.p0.x, point.y - LineSeg_.p0.y);
    Vector2D BC(LineSeg_.p1.x - LineSeg_.p0.x, LineSeg_.p1.y - LineSeg_.p0.y);
    Vector2D CA(point.x - LineSeg_.p1.x, point.y - LineSeg_.p1.y);
    Vector2D CB;

    CB = -BC;
    fVal = BA.dot(BC) * (CA.dot(CB));
    if(fVal > 0)
    {
        bProjInLine = true;
    }
    else
    {
        bProjInLine = false;
    }
    return bProjInLine;
}

/**
 * @brief 点和线段的最近距离
 */
float CalDisBetweenLineSegmentAndPoint(const LineSeg &LineSeg_, const cpoint &point)
{
    float fDis = 0.0f;
    Vector2D BA(point.x - LineSeg_.p0.x, point.y - LineSeg_.p0.y);
    Vector2D BC(LineSeg_.p1.x - LineSeg_.p0.x, LineSeg_.p1.y - LineSeg_.p0.y);
    Vector2D CA(point.x - LineSeg_.p1.x, point.y - LineSeg_.p1.y);
    Vector2D BD;
    Vector2D AD;
    Vector2D CB;
    CB = -BC;
    if(BA.dot(BC) * (CA.dot(CB)) > 0)
    {
        /*点的投影点在线段中, 最近距离是点到直线距离*/
        BD = BC * (BA.dot(BC) / (BC.sqlength()));
        AD = BD - BA;
        fDis = AD.length();
    }
    else
    {
        /*点的投影点在线段外, 最近距离是点到两个端点最小距离*/
        fDis = MIN(BA.length(), CA.length());
    }
    return fDis;
}

/**
 * @brief 线段之间的最小距离
 */
float CalDisBetweenLineSegments(const LineSeg LineSeg_[2])
{

    float fDis = 0.0f;
    float fMinDis = 1e5f;
    LocationPoint p1, p0, q1, q0;

    p0.x = LineSeg_[0].p0.x;
    p0.y = LineSeg_[0].p0.y;
    p1.x = LineSeg_[0].p1.x;
    p1.y = LineSeg_[0].p1.y;

    q0.x = LineSeg_[1].p0.x;
    q0.y = LineSeg_[1].p0.y;
    q1.x = LineSeg_[1].p1.x;
    q1.y = LineSeg_[1].p1.y;

    if(IsRectCross(p0, p1, q0, q1) &&  (IsLineSegmentCross(p0, p1, q0, q1)))
    {
        /*线段相交*/
        fMinDis = 0.0f;
    }
    else
    {
        for(int i = 0; i < 2; ++i)
        {
            fDis = MIN(CalDisBetweenLineSegmentAndPoint(LineSeg_[i], LineSeg_[1 - i].p0),
                    CalDisBetweenLineSegmentAndPoint(LineSeg_[i], LineSeg_[1 - i].p1));
            if(fDis < fMinDis)
            {
                fMinDis = fDis;
            }
        }
    }
    return fMinDis;
}

/**
 * @brief 凸多边形在过零点的投影轴上的相对于零点的投影区间
 * @param polygon 凸多边形
 * @param axes 投影轴
 * @return 投影区间
 */
 void GetPolyProjectionWithAxis(const Polygon &polygon, const Vector2D &axes, Vector2D &vproj)
{
    float fmin = 0.0f;
    float fmax = 0.0f;
    float fproj = 0.0f;
    Vector2D vertice;

    if(polygon.vertices.size() == polygon.edgsNum && (!polygon.vertices.empty()))
    {
        vertice.setXY(polygon.vertices[0].x, polygon.vertices[0].y);
        fproj = vertice.dot(axes);
        fmin = fproj;
        fmax = fmin;

        for(uint i = 1; i < polygon.edgsNum; ++i)
        {
            vertice.setXY(polygon.vertices[i].x, polygon.vertices[i].y);
            fproj = vertice.dot(axes);
            if(fproj < fmin)
            {
                fmin = fproj;
            }

            if(fproj > fmax)
            {
                fmax = fproj;
            }
        }
        vproj.setXY(fmin, fmax);
    }
    else
    {
        GAC_LOG_INFO("多边形输入错误!!!\n");
    }
    return;
}

/**
 * @brief 圆在过零点的投影轴上的相对于零点的投影区间
 * @param circle 圆
 * @param axes 投影轴
 * @return 投影区间
 */
void GetCircleProjectionWithAxis(const Circle &circle, const Vector2D &axes, Vector2D &vproj)
{
    float fmin = 0.0f;
    float fmax = 0.0f;
    float fproj = 0.0f;
    Vector2D p1, p2;

    p1.setXY(circle.x + circle.R * axes.getX(), circle.y + circle.R * axes.getY());
    p2.setXY(circle.x - circle.R * axes.getX(), circle.y - circle.R * axes.getY());
    fmin = p1.dot(axes);
    fmax = fmin;

    fproj = p2.dot(axes);
    if(fproj < fmin)
    {
        fmin = fproj;
    }

    if(fproj > fmax)
    {
        fmax = fproj;
    }
    vproj.setXY(fmin, fmax);
    return;
}

/**
 * @brief 投影重叠判断
 * @param vproj1 投影1
 * @param vproj2 投影2
 * @return 1-重叠 0-不重叠
 */
bool ProjectionOverlaps(const Vector2D &vproj1, const Vector2D &vproj2)
{
    float fmin1 = vproj1.getX();
    float fmax1 = vproj1.getY();
    float fmin2 = vproj2.getX();
    float fmax2 = vproj2.getY();

    if(fmin1 > fmax1)
    {
        fmin1 = vproj1.getY();
        fmax1 = vproj1.getX();
    }

    if(fmin2 > fmax2)
    {
        fmin2 = vproj2.getY();
        fmax2 = vproj2.getX();
    }

    return (!((fmin2 > fmax1) || (fmax2 < fmin1)));
}

/**
 * @brief 创建多边形
 */
Polygon CreatePolygon(ushort edgsNum, std::vector<cpoint> vertices)
{
    Polygon polygon;
    LineSegment edge;
    std::vector<LineSegment> edges;

    if(vertices.size() == edgsNum && (!vertices.empty()))
    {
        polygon.edgsNum = edgsNum;
        polygon.vertices = vertices;

        for(uint i = 0; i < edgsNum; i++)
        {
            if(i + 1 < edgsNum)
            {
                edge.s = vertices[i];
                edge.e = vertices[i + 1];
            }
            else
            {
                edge.s = vertices[i];
                edge.e = vertices[0];
            }
            edge.dir.setXY(edge.e.x - edge.s.x, edge.e.y - edge.s.y);
            normalizeVec(edge.dir);
            edges.push_back(edge);
        }
        polygon.edgs = edges;
    }
    else
    {
        GAC_LOG_INFO("多边形输入错误!!!\n");
    }
    return polygon;
}


/**
 * @brief 分离轴(SAT)投影法:凸多边形和圆的相交判断
 * @param polygon 凸多边形
 * @param circle 圆
 * @return 1-相交 0-不相交
 */
bool CrossDetectorPolyVsCircle(const Polygon &polygon, const Circle &circle)
{
    bool bOverlap = false;
    Vector2D axes;
    Vector2D vproj1;
    Vector2D vproj2;

    if((polygon.vertices.size() == polygon.edgsNum) && (!polygon.vertices.empty()))
    {
        /*建立多边形的边的法向量作为投影轴*/
        for(uint i = 0; i < polygon.edgsNum; i++)
        {
            /*这里axes已经单位化了*/
            axes.setXY(polygon.edgs[i].dir.getY(), -polygon.edgs[i].dir.getX());
            GetPolyProjectionWithAxis(polygon, axes, vproj1);
            GetCircleProjectionWithAxis(circle, axes, vproj2);
            bOverlap = ProjectionOverlaps(vproj1, vproj2);
            if(!bOverlap)
            {
                return false;
            }
        }

        /*建立圆心到多边形顶点的连线作为投影轴*/
        for(uint i = 0; i < polygon.edgsNum; i++)
        {
            axes.setXY(polygon.vertices[i].x - circle.x, polygon.vertices[i].y - circle.y);
            /*axes单位化*/
            normalizeVec(axes);
            GetPolyProjectionWithAxis(polygon, axes, vproj1);
            GetCircleProjectionWithAxis(circle, axes, vproj2);
            bOverlap = ProjectionOverlaps(vproj1, vproj2);
            if(!bOverlap)
            {
                return false;
            }
        }
    }
    else
    {
        GAC_LOG_INFO("多边形输入错误!!!\n");
    }

    return true;
}


/**
 * @brief 圆和多边形碰撞debug代码
 */
bool CrossDetectorPolyVsCircleDEBUG(const std::vector<cpoint> &PolyVertices,
                                    const Circle &circle,
                                    std::vector<cpoint>  &circlePoints)
{
    float x = 0.0f;
    float y = 0.0f;
    cpoint point;
    circlePoints.clear();
    for(int i = 0; i < 360; ++i)
    {
       point.x = circle.x + circle.R * cos(i * PI / 180);
       point.y = circle.y + circle.R * sin(i * PI / 180);
       circlePoints.push_back(point);
    }
    return(CrossDetectorPolyVsCircle(CreatePolygon(PolyVertices.size(), PolyVertices),
                                   circle));

}

/**
 * @brief 创建车位区域多边形
 * @param slotPosInfor_
 * @param polygon
 */
void CreatSlotAreaPolygon(const slotPosInfor &slotPosInfor_, Polygon &polygon)
{
    cpoint verticeTopF, verticeTopR, verticeBottomF, verticeBottomR;
    std::vector<cpoint> vertices;
    float fDis = 0.4f;
    verticeTopF.x = slotPosInfor_.fXFrontSlotCorner + slotPosInfor_.fXSlotPosInMap + fDis;
    verticeTopF.y = slotPosInfor_.fYFrontSlotCorner + slotPosInfor_.fYSlotPosInMap + slotPosInfor_.nSlotSide*fDis;
    vertices.push_back(verticeTopF);

    verticeTopR.x = slotPosInfor_.fXRearSlotCorner + slotPosInfor_.fXSlotPosInMap - fDis;
    verticeTopR.y = slotPosInfor_.fYRearSlotCorner + slotPosInfor_.fYSlotPosInMap + slotPosInfor_.nSlotSide * fDis;
    vertices.push_back(verticeTopR);


    verticeBottomR.x = -(slotPosInfor_.fSlotWidth / fabsf(sinf(slotPosInfor_.fSlotAngle)))
            * cosf(slotPosInfor_.fSlotAngle) + slotPosInfor_.fXSlotPosInMap;
    verticeBottomR.y = -slotPosInfor_.fSlotWidth / fabsf(sinf(slotPosInfor_.fSlotAngle))
            * sinf(slotPosInfor_.fSlotAngle) + slotPosInfor_.fYSlotPosInMap;
    vertices.push_back(verticeBottomR);

    verticeBottomF.x = slotPosInfor_.fSlotLength - slotPosInfor_.fSlotWidth / fabsf(sinf(slotPosInfor_.fSlotAngle))
            * cosf(slotPosInfor_.fSlotAngle) + slotPosInfor_.fXSlotPosInMap;
    verticeBottomF.y = -slotPosInfor_.fSlotWidth / fabsf(sinf(slotPosInfor_.fSlotAngle))
            * sinf(slotPosInfor_.fSlotAngle) + slotPosInfor_.fYSlotPosInMap;
    vertices.push_back(verticeBottomF);

    polygon = CreatePolygon(4, vertices);

    return;
}


/**
 * @brief 根据车辆位置姿态计算四个角点坐标
 * @param tVehicleRearAxleCenter 后轴中心点位置姿态
 * @param tCorner 角点坐标
 */
void CalVehicleCornerPoint(const LocationPoint &tVehicleRearAxleCenter, LocationPoint tCorner[4])
{
    LocationPoint tOriginPoint;
    LocationPoint tPoint[4];
    float fSinYaw = sinf(tVehicleRearAxleCenter.yaw);
    float fCosYaw = cosf(tVehicleRearAxleCenter.yaw);
    tOriginPoint.x = 0;
    tOriginPoint.y = 0;
    tPoint[0].x = -Constants::RearEdgeToCenter;
    tPoint[0].y = -Constants::width / 2;
    tPoint[1].x = Constants::FrontEdgeToCenter;
    tPoint[1].y = tPoint[0].y;
    tPoint[2].x = tPoint[1].x;
    tPoint[2].y = Constants::width / 2;
    tPoint[3].x = tPoint[0].x;
    tPoint[3].y = tPoint[2].y;

    for(int i = 0; i < 4; ++i)
    {
        GetNewPointOfRotatation(tOriginPoint,
                                tPoint[i],
                                fSinYaw,
                                fCosYaw,
                                tCorner[i]);
        tCorner[i].x += tVehicleRearAxleCenter.x;
        tCorner[i].y += tVehicleRearAxleCenter.y;
    }
}


/**
 * @brief 计算一点绕一点旋转一定角度后的新坐标
 * @param tCenter 中心点
 * @param tRotationPoint 旋转点
 * @param fSinYaw sin旋转角度
 * @param fCosYaw cos旋转角度
 * @param tRotationNewPoint 新坐标
 */
void GetNewPointOfRotatation(const LocationPoint &tCenter,
                             const LocationPoint &tRotationPoint,
                             float fSinYaw,
                             float fCosYaw,
                             LocationPoint& tRotationNewPoint)
{
    tRotationNewPoint.x = tCenter.x + (tRotationPoint.x - tCenter.x) * fCosYaw - (tRotationPoint.y - tCenter.y) * fSinYaw;
    tRotationNewPoint.y = tCenter.y + (tRotationPoint.x - tCenter.x) * fSinYaw + (tRotationPoint.y - tCenter.y) * fCosYaw;
}



}
