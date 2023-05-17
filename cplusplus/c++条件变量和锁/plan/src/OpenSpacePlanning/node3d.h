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
* @file node3d.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 提供3D节点类,(x, y, theta)
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/

#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>
#include "constants.h"
//#include "helper.h"

namespace HybridAStarPart {

struct VarialeCellBoundary
{
    int nxs;
    int nxe;
    int nys;
    int nye;
    bool bLargeHeadings;
    bool bLargeCell;
    VarialeCellBoundary():nxs(0), nxe(0), nys(0), nye(0), bLargeHeadings(false), bLargeCell(false){}
};

class Node3D
{
 public:

    /*默认构造函数*/
    Node3D(): x(0),y(0),t(0),TrajCost(0),HeuristicCost(0),fSteering(0.0), nDirection(0), nSameDirNodeNum(0), isRs(false),
        isStart(false), idx(-1), o(false), c(false), pred(nullptr){}

    /*通过给定参数构造节点*/
    Node3D(float x, float y, float t, float TrajCost, float HeuristicCost,
           Node3D* pred, int8_t nDirection = 0,
           bool isStart = false, float fSteering = 0.0,
           int nSameDirNodeNum = 0)
    {
        this->x = x;
        this->y = y;
        this->t = t;
        this->TrajCost = TrajCost;
        this->HeuristicCost = HeuristicCost;
        this->pred = pred;
        this->o = false;
        this->c = false;
        this->idx = -1;
        this->isRs = false;
        this->fSteering = fSteering;
        this->nDirection = nDirection;
        this->isStart= isStart;
        this->nSameDirNodeNum = nSameDirNodeNum;
    }

    /*获初始化成员变量*/
    void Init();

    /*获取总代价值*/
    float GetCost() const { return TrajCost + HeuristicCost; }

    /*获取节点的索引位置*/
    int getIdx() const { return idx; }

    /*判断是否属于open set*/
    bool isOpen() const { return o; }

    /*判断是否属于close set*/
    bool isClosed() const { return c; }

    /*获取父类节点*/
    Node3D* getPred() const { return pred; }

    /*设置节点的索引位置*/
    int setIdx(const VarialeCellBoundary &bound);

    /*把节点设置成open状态*/
    void open() { o = true; c = false;}

    /*把节点设置成close状态*/
    void close() { c = true; o = false; }

    /*设置pred成节点父类节点*/
    void setPred(Node3D* pred) { this->pred = pred; }

    /*连接调用RS连接曲线时是否在距离范围内*/
    bool isInRange(const Node3D& goal) const;//检测是否可以分析方法找到解

    /*节点是否超出3D网格表示范围*/
    bool isOnGrid() const;//检查该点是否在3D array范围内


    float x;/*栅格地图中的x坐标*/

    float y;/*栅格地图中的y坐标*/

    float t;/*栅格地图中的车辆偏航角*/

    float TrajCost;/*轨迹已行驶代价值*/

    float HeuristicCost;/*启发式代价值*/

    float fSteering;/*前轴转角*/

    int8_t nDirection;/*行驶方向*/

    int nSameDirNodeNum;

    bool isRs;/*1-是RS连接曲线上的点,0-不是RS连接曲线上的点*/

    bool isStart;/*1-是规划的起点,0-不是规划的起点*/

private:

    int idx;/*节点的索引位置*/
    bool o;/*1-属于open set,0-不属于open set*/
    bool c;/*1-属于close set,0-不属于close set*/
    Node3D* pred;/*父类节点*/

};
}
#endif // NODE3D_H
