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
* @file node3d.cpp
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 提供3D节点类
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note
*/
#include "node3d.h"
namespace HybridAStarPart {

void Node3D::Init()
{
    this->x = 0;
    this->y = 0;
    this->t = 0;
    this->TrajCost = 0;
    this->HeuristicCost = 0;
    this->pred = nullptr;
    this->o = false;
    this->c = false;
    this->idx = -1;
    this->isRs = false;
    this->fSteering = 0.0;
    this->nDirection = 0;
    this->isStart= false;
    this->nSameDirNodeNum = 0;
}

/*设置节点的索引位置*/
int Node3D::setIdx(const VarialeCellBoundary &bound)
{
#ifdef VARIABLE_CELL

    float cellSize = Constants::cellSize;
    int width = Constants::Node3dWidth;
    int height = Constants::Node3dHeight;
    int headings = Constants::headings;
    int variableCellheadings = 0;
    float freverseVariableCellDeltaHeadingRad = 0.0;
    int nt = MIN((int)(t * Constants::reverseDeltaHeadingRad), headings - 1);
    int nx = MIN((int)(x * Constants::reverseCellSize), width - 1);
    int ny = MIN((int)(y * Constants::reverseCellSize),  height - 1);
    int nxs = bound.nxs;
    int nxe = bound.nxe;
    int nys = bound.nys;
    int nye = bound.nye;
    int nt2 = 0;
    int nx2 = 0;
    int ny2 = 0;
    int N1 = width * bound.nys * headings;
    int N2 = 0;

    if(bound.bLargeHeadings)
    {
        variableCellheadings = Constants::variableCellheadings2;
        freverseVariableCellDeltaHeadingRad = Constants::reverseVariableCellDeltaHeadingRad2;
    }
    else
    {
        variableCellheadings = Constants::variableCellheadings1;
        freverseVariableCellDeltaHeadingRad = Constants::reverseVariableCellDeltaHeadingRad1;
    }

    if(bound.bLargeCell)
    {
        nt2 = MIN(t * freverseVariableCellDeltaHeadingRad, variableCellheadings - 1);
        N2 = bound.nxs * headings + (bound.nxe - bound.nxs + 1) * variableCellheadings
                    + (width - 1 - bound.nxe) * headings;
        if(ny >= 0 && (ny < nys))
        {
            idx = (nx + ny * width) * headings + nt;
        }
        else if(ny >= nys && (ny <= nye))
        {
            if(nx >= 0 && (nx < nxs))
            {
                idx = N1 + (ny - nys) * N2 + nx * headings + nt;
            }
            else if(nx >= nxs && (nx <= nxe))
            {
                idx = N1 + (ny - nys) * N2 + nxs * headings + (nx - nxs) * variableCellheadings
                        + nt2;
            }
            else
            {
                idx = N1 + (ny - nys) * N2 + nxs * headings + (nxe - nxs + 1) * variableCellheadings
                        + (nx - nxe - 1) * headings + nt;
            }
        }
        else
        {
            idx = N1 + (nye - nys + 1) * N2 + (nx + (ny - nye - 1) * width) * headings + nt;
        }
    }
    else
    {
        nt2 = MIN(t * freverseVariableCellDeltaHeadingRad, variableCellheadings - 1);
        nx2 = MIN((x - nx * cellSize) * Constants::reverseCellSize2, 1);
        ny2 = MIN((y - ny * cellSize) * Constants::reverseCellSize2, 1);

        N2 = bound.nxs * headings + (bound.nxe - bound.nxs + 1) * 4 * variableCellheadings
                    + (width - 1 - bound.nxe) * headings;
        if(ny >= 0 && (ny < nys))
        {
            idx = (nx + ny * width) * headings + nt;
        }
        else if(ny >= nys && (ny <= nye))
        {
            if(nx >= 0 && (nx < nxs))
            {
                idx = N1 + (ny - nys) * N2 + nx * headings + nt;
            }
            else if(nx >= nxs && (nx <= nxe))
            {
                idx = N1 + (ny - nys) * N2 + nxs * headings + (nx - nxs) * 4 * variableCellheadings
                        + (nx2 + ny2 * 2) * variableCellheadings + nt2;
            }
            else
            {
                idx = N1 + (ny - nys) * N2 + nxs * headings + (nxe - nxs + 1) * 4 * variableCellheadings
                        + (nx - nxe - 1) * headings + nt;
            }
        }
        else
        {
            idx = N1 + (nye - nys + 1) * N2 + (nx + (ny - nye - 1) * width) * headings + nt;
        }
    }
#else
    float cellSize = Constants::cellSize;
    int width = Constants::Node3dWidth;
    int height = Constants::Node3dHeight;
    int headings = Constants::headings;
    int nt = MIN((int)(t * Constants::reverseDeltaHeadingRad), headings - 1);
    int nx = MIN((int)(x * Constants::reverseCellSize), width - 1);
    int ny = MIN((int)(y * Constants::reverseCellSize),  height - 1);
    idx = (nx + ny * width) * headings + nt;
#endif
    return idx;
}

/*判断是否在3D网格上*/
bool Node3D::isOnGrid() const
{
    int nx = (int)(x / Constants::cellSize);
    int ny = (int)(y / Constants::cellSize);

    return (nx >= 0 && nx < Constants::Node3dWidth) && (ny >= 0 && ny < Constants::Node3dHeight)
            && (t > -1e-3f && t < 2 * PI + 1e-3f);
}


/*连接调用RS连接曲线时是否在距离范围内*/
bool Node3D::isInRange(const Node3D &goal) const
{
  float dx = fabsf(x - goal.x);
  float dy = fabsf(y - goal.y);
  bool val =  hypotf(dx, dy) < Constants::ReedsSheepsShotDistance;
  return val;
}

}
