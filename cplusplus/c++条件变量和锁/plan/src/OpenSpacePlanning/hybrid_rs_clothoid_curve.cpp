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
* @file hybrid_rs_clothoid_curve.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief ReedSheep曲线衍生曲线函数，为hybrid A*搜索算法提供连接曲线
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该曲线包括CCS,SCS，CS三种曲线形式；三种曲线为ReedsShepp曲线所不具备的，但是却是泊车中重要的连接曲线形式；
* 该曲线的数学推导形式主要参考reedsSheep曲线，其中最重要的是四种曲线变换:timeflip，reflect,timeflip+reflect,backforwards;
* 依据该理论衍生本算法,算法的详细推导参考文档；
* 引用文献 :
* 1 Reeds, J.A. & Shepp, L.A.. (1990). Optimal paths for a car that goes both forwards and backwards. Pacific Journal of Mathematics. 145. 367-393.
* 2 Dubins, L.E.. (1957). On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents. American Journal of Mathematics. 79. 10.2307/2372560.
*/

#include "hybrid_rs_clothoid_curve.h"

namespace HybridAStarPart {


/**
 * @brief mod2_Npi_Ppi 角度归一
 * @param x 偏航角度
 * @return v 归一角度
 */
float mod2_Npi_Ppi(float x)
{
    float v = fmod(x, PI * 2.0f);
    if (v < -PI)
    {
        v += PI * 2;
    }
    else if (v > PI)
    {
        v -= PI * 2;
    }
    return v;
}

float mod2_0_2pi(float x)
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

/**
 * @brief LpRnSn 左转向前+右转后退+直线后退
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param t 第1段曲线长度
 * @param u 第2段曲线长度
 * @param v 第3段曲线长度
 * @return 1-曲线有解;0-曲线无解
 */
bool LpRnSn(float x, float y, float phi, float &t, float &u, float &v)
{
    float fCosPhi = cosf(phi);
    float fSinPhi = sinf(phi);
    float fAcosu1= 0.0;
    float xi = x + fSinPhi, eta = y - 1.0f - fCosPhi, u1 = 0.0f;
    float ts1, us1, vs1 = 0.0f;
    float ts2, us2, vs2 = 0.0f;
    bool validity = false;

    u1 = (xi * fSinPhi - eta * fCosPhi) / 2.0f;

    phi = mod2_Npi_Ppi(phi);
    if (u1 >= -1 && u1 <= 1)
    {
        fAcosu1= acosf(u1);
        ts1 = mod2_Npi_Ppi(fAcosu1 + phi);
        us1 = mod2_Npi_Ppi(ts1 - phi);
        if (fabsf(fabsf(phi) - PI/2) > PI/4)
        {
            vs1 = (xi - 2 * sinf(ts1)) / fCosPhi;
        }
        else
        {
            vs1 = (eta + 2 * cosf(ts1)) / fSinPhi;
        }
#if 0
        float e1 = (2 * sinf(ts1) + vs1 * fCosPhi - x - fSinPhi);
        float e2 = (-2 * cosf(ts1) + vs1 * fSinPhi - eta);

        if(fabsf(e1) > RS_ERR || fabsf(e2) > RS_ERR)
        {
            GAC_LOG_ERROR("LpRnSn error!!");
        }
#endif
        ts2 = mod2_Npi_Ppi(-fAcosu1 + phi);
        us2 = mod2_Npi_Ppi(ts2 - phi);

        if(fabsf(fabsf(phi) - PI / 2) > PI / 4)
        {
            vs2 = (xi - 2 * sinf(ts2)) / fCosPhi;
        }
        else
        {
            vs2 = (eta + 2 * cosf(ts2)) / fSinPhi;
        }
#if 0
        e1 = 2 * sinf(ts2) + vs2 * fCosPhi- x - fSinPhi;
        e2 = -2 * cosf(ts2) + vs2 * fSinPhi - eta;

        if(fabsf(e1) > RS_ERR || fabsf(e2) > RS_ERR)
        {
            GAC_LOG_ERROR("LpRnSn error!!");
        }
#endif
        if(ts1 >= MIN_ARC_LEN && us1 <= -ZERO && vs1 <= -MIN_LINE_LEN)
        {
            t = ts1;
            u = us1;
            v = vs1;
            validity = true;
        }
        else if(ts2 >= MIN_ARC_LEN && us2 <= -ZERO && vs2 <= -MIN_LINE_LEN)
        {
            t = ts2;
            u = us2;
            v = vs2;
            validity = true;
        }
        else
        {
            ;
        }
    }
    return validity;
}

/**
 * @brief LnRnSn 左转后退+右转后退+直线后退
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param t 第1段曲线长度
 * @param u 第2段曲线长度
 * @param v 第3段曲线长度
 * @return 1-曲线有解;0-曲线无解
 */
bool LnRnSn(float x, float y, float phi, float &t, float &u, float &v)
{
    float fCosPhi = cosf(phi);
    float fSinPhi = sinf(phi);

    float xi = x + fSinPhi, eta = y - 1.0f - fCosPhi, u1 = 0.0f;
    u1 = (xi * fSinPhi - eta * fCosPhi)/2;
    float fAcosu1 = 0.0;

    float ts1, us1, vs1 = 0.0;
    float ts2, us2, vs2 = 0.0;
    bool validity = false;

    phi = mod2_Npi_Ppi(phi);
    if(u1 >= -1 && u1 <= 1)
    {
        fAcosu1= acosf(u1);
        ts1 = mod2_Npi_Ppi(fAcosu1 + phi);
        us1 = mod2_Npi_Ppi(ts1 - phi);
        if(fabsf(fabsf(phi) - PI / 2) > PI / 4)
        {
            vs1 = (xi - 2 * sinf(ts1)) / fCosPhi;

        }
        else
        {
            vs1 = (eta + 2 * cosf(ts1)) / fSinPhi;
        }
#if 0
        float e1, e2, e3, e4 = 0.0f;
        e1 = 2 * sinf(ts1) + vs1 * fCosPhi - xi;
        e2 = -2 * cosf(ts1) + vs1 * fSinPhi - eta;

        if(fabsf(e1) > 1e-4 || fabsf(e2) > 1e-4)
        {
            GAC_LOG_ERROR("LnRnSn error!!");
        }
#endif
        ts2 = mod2_Npi_Ppi(-fAcosu1 + phi);
        us2 = mod2_Npi_Ppi(ts2 - phi);
        if(fabsf(fabsf(phi) - PI / 2) > PI / 4)
        {
            vs2 = (xi-2 * sinf(ts2)) / fCosPhi;

        }
        else
        {
            vs2 = (eta + 2 * cosf(ts2)) / fSinPhi;
        }

 #if 0
        e3 = 2 * sinf(ts2) + vs2 * fCosPhi - xi;
        e4 = -2 * cosf(ts2) + vs2 * fSinPhi - eta;
        if(fabsf(e3) > RS_ERR || fabsf(e4) > RS_ERR)
        {
            GAC_LOG_ERROR("LnRnSn error!!");
        }
#endif
        if(ts1 < -ZERO && us1<=-ZERO && vs1 <= -MIN_LINE_LEN)
        {
            t = ts1;
            u = us1;
            v = vs1;
            validity = true;
        }
        else if(ts2 < -ZERO && us2 <= -ZERO && vs2 <= -MIN_LINE_LEN)
        {
            t = ts2;
            u = us2;
            v = vs2;
            validity = true;
        }
        else
        {
            ;
        }
    }
    return validity;
}

/**
 * @brief SnLnSn 直线后退+左转后退+直线后退
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param t 第1段曲线长度
 * @param u 第2段曲线长度
 * @param v 第3段曲线长度
 * @return 1-曲线有解;0-曲线无解
 */
bool SnLnSn(float x, float y, float phi, float &t, float &u, float &v)
{
    phi = mod2_Npi_Ppi(phi);
    u = mod2_Npi_Ppi(phi);
    float fCosu = cosf(u);
    float fSinu = sinf(u);
    if(fabsf(u) > CAL_ERROR)
    {
        v = (y - 1 + fCosu) / fSinu;
        t = x - fSinu - v * fCosu;
#if 0
        float e1 = (t + fSinu + v * fCosu - x);
        float e2 = (1 - fCosu + v * fSinu - y);

        if(fabsf(e1) > RS_ERR || fabsf(e2) > RS_ERR)
        {
            GAC_LOG_ERROR("SnLnSn error!!");
        }
#endif
        return (t <= -MIN_LINE_LEN && u <= -ZERO && v <= -MIN_LINE_LEN);
    }
    return false;
}

/**
 * @brief SnCnSn 不可换向的直线-曲线-直线,用在垂直泊车的入库曲线
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param path 曲线路径
 * @return 1-曲线有解;0-曲线无解
 */
bool SnCnSn(float x, float y, float phi, ReedsSheppPath &path)
{
    float Lmin = path.totalLength_;
    float t = 0;
    float u = 0;
    float v = 0;
    float L = 0;

    bool HaveSolution=false;

    if (SnLnSn(x, y, phi, t, u, v))
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[4], t, u, v);
            Lmin = L;
        }
    }
    if (SnLnSn(x, -y, -phi, t, u, v)) // reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[5], t, u, v);
            Lmin = L;
        }
    }
    if (SnLnSn(-x, y, -phi, t, u, v)) // timeflip
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[4], -t, -u, -v);
            Lmin = L;
        }
    }
    if (SnLnSn(-x, -y, phi, t, u, v)) // timeflip+reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[5], -t, -u, -v);
            Lmin = L;
        }
    }
    return HaveSolution;
}

/**
 * @brief CpCnSn 可换向的曲线-曲线-直线,用在垂直泊车的入库曲线
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param path 曲线路径
 * @return 1-曲线有解;0-曲线无解
 */
bool CpCnSn(float x, float y, float phi, ReedsSheppPath &path)
{
    float Lmin = path.totalLength_;
    float t = 0;
    float u = 0;
    float v = 0;
    float L = 0;
    bool HaveSolution=false;

    if (LpRnSn(x, y, phi, t, u, v))
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[0], t, u, v);
            Lmin = L;
        }
    }
    if (LpRnSn(x, -y, -phi, t, u, v)) // reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[1], t, u, v);
            Lmin = L;
        }
    }

    if (LpRnSn(-x, y, -phi, t, u, v)) // timeflip
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[0], -t, -u, -v);
            Lmin = L;
        }
    }

    if (LpRnSn(-x, -y, phi, t, u, v)) // timeflip+reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[1], -t, -u, -v);
            Lmin = L;
        }
    }

    return HaveSolution;
}

/**
 * @brief CnCnSn 不可换向曲线-曲线-直线,用在垂直泊车的入库曲线
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param path 曲线路径
 * @return 1-曲线有解;0-曲线无解
 */
bool CnCnSn(float x, float y, float phi, ReedsSheppPath &path)
{
    float Lmin = path.totalLength_;
    float t = 0;
    float u = 0;
    float v = 0;
    float L = 0;
    bool HaveSolution=false;

    if (LnRnSn(x, y, phi, t, u, v))
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[0], t, u, v);
            Lmin = L;
        }
    }
    if (LnRnSn(x, -y, -phi, t, u, v)) // reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[1], t, u, v);
            Lmin = L;
        }
    }
    if (LnRnSn(-x, y, -phi, t, u, v)) //timeflip
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[0], -t, -u, -v);
            Lmin = L;
        }
    }

    if (LnRnSn(-x, -y, phi, t, u, v)) //timeflip+reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[1], -t, -u, -v);
            Lmin = L;
        }
    }

    return HaveSolution;
}


/**
 * @brief SnCnCn 不可换向直线-曲线-曲线,用在平行泊车的入库曲线
 * @param x 归一x坐标
 * @param y 归一y坐标
 * @param phi 归一偏航角度
 * @param path 曲线路径
 * @return 1-曲线有解;0-曲线无解
 */
bool SnCnCn(float x, float y, float phi, ReedsSheppPath &path)
{
    float Lmin = path.totalLength_;
    float t = 0;
    float u = 0;
    float v = 0;
    float L = 0;
    bool HaveSolution=false;
    // backwards
    float xb = x * cosf(phi) + y * sinf(phi), yb = x * sinf(phi) - y * cosf(phi);
    if (LnRnSn(xb, yb, phi, t, u, v))
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[3], v, u, t);
            Lmin = L;
        }
    }
    if (LnRnSn(xb, -yb, -phi, t, u, v)) // reflect
    {
        L = fabsf(t) + fabsf(u) + fabsf(v);
        if(Lmin > L)
        {
            HaveSolution=true;
            SetReedsSheppPath(path, reedsSheppPathType[2], v, u, t);
            Lmin = L;
        }
    }
    return HaveSolution;
}

/**
 * @brief SetReedsSheppPath 设置路径类型和路径长度
 * @param path 路径
 * @param type 路径类型
 * @param t
 * @param u
 * @param v
 * @param w
 * @param x
 */
void SetReedsSheppPath(ReedsSheppPath &path, const ReedsSheppPathSegmentType* type,
                       float t, float u, float v, float w, float x)
{
    path.type_ = type;
    path.length_[0] = t;
    path.length_[1] = u;
    path.length_[2] = v;
    path.length_[3] = w;
    path.length_[4] = x;
    path.totalLength_ = fabsf(t) + fabsf(u) + fabsf(v) + fabsf(w) + fabsf(x);
    path.rho_ = 1;
}

/**
 * @brief reedsShepp RS曲线
 * @param reverse 0-可以换向的RS曲线; 1-不可以换向的RS曲线
 * @param q0 起点
 * @param q1 终点
 * @param rho_ 比例系数半径）
 * @param path 路径
 */
void reedsShepp(bool reverse, LocationPoint q0, LocationPoint q1, float rho_, ReedsSheppPath& path)
{

    bool HasSolution1=false;
    bool HasSolution2=false;
    bool HasSolution3=false;

    float dx = 0.0;
    float dy = 0.0;
    float dth = 0.0;
    float x = 0.0;
    float y = 0.0;

    dx = q1.x - q0.x;
    dy = q1.y - q0.y;
    dth = q1.yaw - q0.yaw;
    x = (cosf(q0.yaw) * dx + sinf(q0.yaw) * dy) / rho_;
    y = (-sinf(q0.yaw) * dx + cosf(q0.yaw) * dy) / rho_;

    if(reverse)
    {
        HasSolution1 = CpCnSn(x, y, dth, path);
        HasSolution2 = SnCnSn(x, y, dth, path);
        HasSolution3 = CnCnSn(x, y, dth, path);
        path.validity = HasSolution1 || HasSolution2 || HasSolution3;
    }
    else
    {
        HasSolution1 = CnCnSn(x, y, dth, path);
        path.validity = HasSolution1;
    }
    path.rho_ = rho_;

    if(!path.validity)
    {
        SetReedsSheppPath(path, reedsSheppPathType[0]);
    }
}

/**
 * @brief reedsSheppParallel 平行泊车曲线连接函数
 * @param q0 起点
 * @param q1 终点
 * @param rho_ 比例系数(半径）
 * @param path 路径
 */
void reedsSheppParallel(LocationPoint q0, LocationPoint q1, float rho_, ReedsSheppPath& path)
{
    bool HasSolution1 = false;
    bool HasSolution2 = false;
    float dx = 0.0;
    float dy = 0.0;
    float dth = 0.0;
    float x = 0.0;
    float y = 0.0;

    dx = q1.x - q0.x;
    dy = q1.y - q0.y;
    dth = q1.yaw - q0.yaw;
    x = (cosf(q0.yaw) * dx + sinf(q0.yaw) * dy) / rho_;
    y = (-sinf(q0.yaw) * dx + cosf(q0.yaw) * dy) / rho_;

    HasSolution1 = SnCnCn(x, y, dth, path);
    HasSolution2 = SnCnSn(x, y, dth, path);

    path.rho_ = rho_;
    path.validity = HasSolution1 || HasSolution2;
}

/**
 * @brief CheckRSCurveIsSameDirection 判断是不是同向的RS曲线
 * @param path 路径
 * @return 1-同向 0-换向
 */
bool CheckRSCurveIsSameDirection(ReedsSheppPath path)
{
    int LastDirection = 0;
    int Direction = 0;
    bool IsSameDirection = false;
    if(path.length_[0] > 0)
    {
        LastDirection = FORWARD;
    }
    else
    {
        LastDirection = BACKWARD;
    }

    for(int i=1;i<5;i++)
    {
        if(path.type_[i] == RS_NOP)
        {
            break;
        }
        if(path.length_[i] > 0)
        {
            Direction = FORWARD;
        }
        else
        {
            Direction = BACKWARD;
        }
        if(Direction != LastDirection)
        {
            IsSameDirection = false;
            break;
        }
        else
        {
            IsSameDirection = true;
        }
        LastDirection = Direction;
    }
    return IsSameDirection;

}

/**
 * @brief AddLineSegInCpCnSnPath 在CpCnSn增加一段缓和直线
 * @param path 路径
 * @return 1-增加成功，0-增加失败
 */
bool AddLineSegInCpCnSnPath(ReedsSheppPath &path)
{
    float fInsertS= Constants::ReedsSheepsStepSize / path.rho_;
    int nDirection = 0;
    bool bRet = true;
    float R = 0.0;

    if(path.validity && (!CheckRSCurveIsSameDirection(path)))
    {
        if(path.type_[0] == RS_LEFT
           && path.type_[1] == RS_RIGHT
           && path.type_[2] == RS_STRAIGHT)
        {
            R = path.rho_;
            if(path.length_[0] > 0)
            {
                nDirection = FORWARD;
            }
            else
            {
                nDirection = BACKWARD;
            }
            SetReedsSheppPath(path,
                              reedsSheppPathType[6],
                              path.length_[0],
                              nDirection * fInsertS,
                              -nDirection * fInsertS,
                              path.length_[1],
                              path.length_[2]);
            path.rho_ = R;
        }
        else if(path.type_[0] == RS_RIGHT
                && path.type_[1] == RS_LEFT
                && path.type_[2] == RS_STRAIGHT)
        {
            R = path.rho_;
            if(path.length_[0] > 0)
            {
                nDirection = FORWARD;
            }
            else
            {
                nDirection = BACKWARD;
            }
            SetReedsSheppPath(path,
                              reedsSheppPathType[7],
                              path.length_[0],
                              nDirection * fInsertS,
                              -nDirection * fInsertS,
                              path.length_[1],
                              path.length_[2]);
            path.rho_ = R;
        }

    }
    else
    {
        bRet = false;
    }
    return bRet;
}

/**
 * @brief GetSteeringDirAndDrivingDir 获取Rs路径第i段的转动方向和行驶方向
 * @param path Rs路径
 * @param i 第i段
 * @param s 转动方向
 * @param d 行驶方向
 * @return
 */
int GetSteeringDirAndDrivingDir(const ReedsSheppPath &path, int i, int8_t &s, int8_t &d)
{
    int ret = -1;
    if(path.type_)
    {
        if(path.type_[i] == RS_LEFT)
        {
            s = STEERING_LEFT;
        }
        else if(path.type_[i] == RS_RIGHT)
        {
            s = STEERING_RIGHT;
        }
        else
        {
            s = 0;
        }

        if(path.length_[i] < 0)
        {
            d = BACKWARD;
        }
        else
        {
            d = FORWARD;
        }
        ret = 0;
    }
    else
    {

    }
    return ret;
}

/**
 * @brief SetVehPos 给点定义属性
 * @param q0 坐标
 * @param nSteeringDirection 转动方向
 * @param nDrivingDirection 行驶方向
 * @param fMaxSteerAngle 转角
 * @param VehPos_ 点
 * @return
 */
void SetVehPos(LocationPoint q0,
               int8_t nSteeringDirection,
               int8_t nDrivingDirection,
               float fMaxSteerAngle,
               Node3D &VehPos_)
{
    VehPos_.x = q0.x;
    VehPos_.y = q0.y;
    VehPos_.t = mod2_0_2pi(q0.yaw);
    VehPos_.nDirection = nDrivingDirection;
    VehPos_.fSteering = nSteeringDirection * fMaxSteerAngle;
    VehPos_.isStart = false;
    VehPos_.isRs = true;
}

/**
 * @brief ReedsShepp_interpolate
 * @param q0 起点
 * @param path 路径
 * @param fUnitArcLength 单位长度
 * @param rho_ 比例系数(半径）
 * @param s 在单位长度位置的插值坐标
 */
void ReedsShepp_interpolate(LocationPoint q0, float fUnitArcLength, float rho_, ReedsSheppPath &path, LocationPoint &s)
{

    float fUnitSegLength = fUnitArcLength;
    float phi = 0.0;
    float v = 0.0;

    fUnitSegLength = MIN(MAX(fUnitSegLength, 0), path.totalLength_);
    s.x = 0.;
    s.y = 0.;
    s.yaw = q0.yaw;

    for ( int i = 0; i < 5 && fUnitSegLength > 1e-3; ++i)
    {
        if (path.length_[i] < 0)
        {
            v = std::max(-fUnitSegLength, path.length_[i]);
            fUnitSegLength += v;
        }
        else
        {
            v = std::min(fUnitSegLength, path.length_[i]);
            fUnitSegLength -= v;
        }
        phi = s.yaw;

        switch(path.type_[i])
        {
        case RS_LEFT:
            s.x += ( sinf(phi + v) - sinf(phi));
            s.y += (-cosf(phi + v) + cosf(phi));
            s.yaw = phi + v;
            break;

        case RS_RIGHT:
            s.x += (-sinf(phi - v) + sinf(phi));
            s.y += ( cosf(phi - v) - cosf(phi));
            s.yaw = phi - v;
            break;

        case RS_STRAIGHT:
            s.x += (v * cosf(phi));
            s.y += (v * sinf(phi));
            break;

        case RS_NOP:
            break;
        }
    }

    s.x = s.x * rho_ + q0.x;
    s.y = s.y * rho_ + q0.y;
}

/**
 * @brief Generate_ReedsShepp_path 生成RS连接路径
 * @param slotType 车位类型
 * @param reverse 0-不可换挡; 1-可以换挡
 * @param q0 起点
 * @param q1 终点
 * @param rho_ 比例系数(半径）
 * @param step_size 插值点步长
 * @param points 插值点
 * @param path2 输出的RS路径
 * @return 1-曲线有解;0-曲线无解
 */
bool Generate_ReedsShepp_path(int slotType,
                              bool reverse,
                              LocationPoint q0,
                              LocationPoint q1,
                              float rho_,
                              float step_size,
                              std::vector<Node3D> &points,
                              ReedsSheppPath& path2)
{
    bool HaveSolution = true;
    int RSSegmentNUm = 0;
    int8_t nDrivingDirection = 0;
    int8_t NextDrivingDirection = 0;
    int8_t nSteeringDirection = 0;
    LocationPoint qnew;
    float s = 0.0;
    float length[5];
    float detas = step_size;
    float max_steering_angle = 0.0;
    float seg = 0.0;
    Node3D VehPos_;
    ReedsSheppPath path;

    MIN_LINE_LEN = Constants::TravelDis / rho_ ;
    MIN_ARC_LEN = Constants::TravelDis / rho_ * 2;
    if(slotType==OBLIQUE)
    {
        reedsShepp(reverse, q0, q1, rho_, path);
//        AddLineSegInCpCnSnPath(path);
    }
    else if(slotType==PARALLEL)
    {
        reedsSheppParallel(q0, q1, rho_, path);
    }
    else
    {

    }
    path2 = path;

    points.clear();
    if(!path.validity)
    {
        HaveSolution=false;
    }
    else
    {
        max_steering_angle = atanf(Constants::WheelBase / path.rho_);
        for(int i = 0;i < 5; i++)
        {
            length[i]= path.length_[i] * path.rho_;
            if(path.type_[i] != RS_NOP)
            {
                RSSegmentNUm++;
            }
        }

        GetSteeringDirAndDrivingDir(path, 0, nSteeringDirection, nDrivingDirection);

        SetVehPos(q0, nSteeringDirection, nDrivingDirection, max_steering_angle, VehPos_);

        points.push_back(VehPos_);

        for(int segNum=0; segNum < RSSegmentNUm; segNum++)
        {
            GetSteeringDirAndDrivingDir(path, segNum, nSteeringDirection, nDrivingDirection);
            //check next segment driving direction
            if(segNum < RSSegmentNUm - 1)
            {
                if(path.length_[segNum + 1] < 0)
                {
                    NextDrivingDirection = BACKWARD;
                }
                else
                {
                    NextDrivingDirection = FORWARD;
                }
            }
            else
            {
                NextDrivingDirection = 0;
            }

            for (seg = s + detas; seg < s + fabsf(length[segNum]); seg += step_size)
            {
                ReedsShepp_interpolate(q0, seg/path.rho_, path.rho_, path, qnew);
                SetVehPos(qnew, nSteeringDirection, nDrivingDirection, max_steering_angle, VehPos_);
                points.push_back(VehPos_);
            }

            //cal the detaS in start point in every segment
            if(nDrivingDirection != NextDrivingDirection)
            {
                s += fabsf(length[segNum]);
                ReedsShepp_interpolate(q0, s / path.rho_, path.rho_, path, qnew);
                SetVehPos(qnew, nSteeringDirection, nDrivingDirection, max_steering_angle, VehPos_);
                points.push_back(VehPos_);
                detas = step_size;
            }
            else
            {
                s += fabsf(length[segNum]);
                detas = seg - s;
            }
        }
    }
    return HaveSolution;
}
}
