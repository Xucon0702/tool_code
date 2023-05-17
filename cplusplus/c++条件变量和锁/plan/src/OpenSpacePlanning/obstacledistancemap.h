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
* @file obstacledistancemap.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 实现栅格地图快速欧式距离地图转换
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该算法参考由Shih提出的双向扫描的快速欧式距离地图转换
* * 引用文献 :
* 1 Shih, Frank & Wu, Yi-Ta. (2004). Note Fast Euclidean distance transformation in two scans using a 3*3 neighborhood. Computer Vision and Image Understanding.
*/

#ifndef OBSTACLEDISTANCEMAP_H
#define OBSTACLEDISTANCEMAP_H

#include <cmath>
#include "constants.h"
#if PLATFORM==PC_SYSTEM
#include "QDebug"
#endif
namespace HybridAStarPart {

/*点相对障碍物的偏移量绝对值x y*/
typedef struct OBS_OFFSET
{
    unsigned int x;
    unsigned int y;
    OBS_OFFSET(): x(0), y(0){}
} obs_offset;

class ObstacleDistanceMap
{
public:
    ObstacleDistanceMap():minDis(0){}
    void Init();

    /**
     * @brief 初始化data数组（存储SED值）, 和R数组（存储该点相对障碍物的偏移量绝对值）
     * @param _gridMap 二值地图
     * @note  * O----------------->x
     *          |
     *          |
     *          |
     *        y |
     *          v
     *
     */
    void initializeMap(const bool *_gridMap);

    /**
     * @brief 二值地图转欧氏距离地图
     * @param _gridMap 二值地图
     * @note   8邻域组成 q2  q3  q4
     *                  q1   p  q5
     *                  q8  q7  q6
     */
    void EuclideanDistanceTransform(const bool *_gridMap);

    /**
     * @brief 计算文中p点减去q点相对于最近障碍物的欧式距离平方差
     * @param Rxq q点最近障碍物在x方向的偏移量绝对值Rx
     * @param Ryq q点最近障碍物在y方向的偏移量绝对值Ry
     * @param casenum q编号
     * @return p点减去q点相对于最近障碍物的欧式距离平方差
     */
    int calH(int Rxq, int Ryq, int casenum);

    /**
     * @brief 更新p点相对于最近障碍物在x和y方向的偏移量绝对值Rx，Ry
     * @param casenum q编号
     * @param x p的x坐标
     * @param y p的y坐标
     */
    void updateR(int casenum, int x, int y);

    /**
     * @brief 获取不同SED值的最小值和最小SED值对应的q点编号
     * @param fp, fq1, fq2, fq3, fq4 根据四个q点计算得到的p点不同SED值
     * @return 最小SED值对应的q点编号
     */
    int getMin(int fp, int fq1, int fq2, int fq3, int fq4);

    /**
     * @brief 将SED值转换成ED值
     * @param x 栅格x
     * @param y 栅格y
     * @return fNearObsDis 欧式距离
     */
    float getNearObsDis(int x, int y);

private:
    int data[Constants::ob_grid_width][Constants::ob_grid_height];

    /*R数组: 存储该点相对障碍物的偏移量绝对值*/
    obs_offset R[Constants::ob_grid_width][Constants::ob_grid_height];

    int minDis;
};

} // namespace

#endif // OBSTACLEDISTANCEMAP_H
