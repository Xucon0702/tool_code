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
* @file obstacledistancemap.cpp
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 实现栅格地图快速欧式距离地图转换
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该算法参考由Shih提出的8邻域双向扫描的快速欧式距离地图转换
* * 引用文献 :
* 1 Shih, Frank & Wu, Yi-Ta. (2004). Note Fast Euclidean distance transformation in two scans using a 3*3 neighborhood. Computer Vision and Image Understanding.
*/

#include "obstacledistancemap.h"
namespace HybridAStarPart {

void ObstacleDistanceMap::Init()
{
    minDis = 0;
}
/**
 * @brief 初始化data数组:存储SED(Square euclidean distance)值, 和R数组: 存储该点相对障碍物的偏移量绝对值.
 * @param _gridMap 二值地图
 * @note  * O----------------->x
 *          |
 *          |
 *          |
 *        y |
 *          v
 *
 */
void ObstacleDistanceMap::initializeMap(const bool *_gridMap)
{
    minDis = 0;
    for(int x = 0; x < Constants::ob_grid_width; x++)
    {
        for(int y = 0; y < Constants::ob_grid_height; y++)
        {
            R[x][y].x = 0;
            R[x][y].y = 0;

            if((x == 0) || (x == Constants::ob_grid_width - 1) || (y == 0) || (y == Constants::ob_grid_height - 1))
            {
                data[x][y] = 0.; // map四边都认为是障碍物
            }
            else
            {
                if(0 != _gridMap[y * Constants::ob_grid_width + x])
                {
                    data[x][Constants::ob_grid_height - 1 - y] = 0.; // 障碍物
                }
                else
                {
                    data[x][Constants::ob_grid_height - 1 - y] = 3e4; // 如果是freespace则初始化欧拉距离值为无穷大
                }
            }
        }
    }
}


/**
 * @brief 二值地图转欧氏距离地图
 * @param _gridMap 二值地图
 * @note   8邻域组成 q2  q3  q4
 *                  q1   p  q5
 *                  q8  q7  q6
 */
void ObstacleDistanceMap::EuclideanDistanceTransform(const bool *_gridMap)
{
    int fp = 0;
    int fq1 = 0;
    int fq2 = 0;
    int fq3 = 0;
    int fq4 = 0;
    int fq5 = 0;
    int fq6 = 0;
    int fq7 = 0;
    int fq8 = 0;
    int minIndex = 0;

    initializeMap(_gridMap);

    // forward scan
    for(int y = 1; y < Constants::ob_grid_height - 1; y++)
    {
        for(int x = 1; x < Constants::ob_grid_width - 1; x++)
        {
            fp = data[x][y];
            if(fp > 0)
            {
                fq1 = data[x-1][y] + calH(R[x-1][y].x, R[x-1][y].y, 1);
                fq2 = data[x-1][y-1] + calH(R[x-1][y-1].x, R[x-1][y-1].y, 2);
                fq3 = data[x][y-1] + calH(R[x][y-1].x, R[x][y-1].y, 3);
                fq4 = data[x+1][y-1] + calH(R[x+1][y-1].x, R[x+1][y-1].y, 4);
                minIndex = getMin(fp, fq1, fq2, fq3, fq4);
                if (minIndex != 0)
                {
                  data[x][y] = minDis;
                  updateR(minIndex, x, y);
                }
            }
        }
    }

    // backward scan
    for(int y = Constants::ob_grid_height - 2; y >= 1; y--)
    {
        for(int x = Constants::ob_grid_width - 2; x >= 1; x--)
        {
            fp = data[x][y];
            if(fp > 0){
                fq5 = data[x+1][y] + calH(R[x+1][y].x, R[x+1][y].y, 5);
                fq6 = data[x+1][y+1] + calH(R[x+1][y+1].x, R[x+1][y+1].y, 6);
                fq7 = data[x][y+1] + calH(R[x][y+1].x, R[x][y+1].y, 7);
                fq8 = data[x-1][y+1] + calH(R[x-1][y+1].x, R[x-1][y+1].y, 8);
                minIndex = getMin(fp, fq5, fq6, fq7, fq8);
                if (minIndex != 0){
                  data[x][y] = minDis;
                  updateR(minIndex + 4, x, y);
                }
            }
        }
    }
}

/**
 * @brief 计算文中p点减去q点相对于最近障碍物的SED差
 * @param Rxq q点最近障碍物在x方向的偏移量绝对值Rx
 * @param Ryq q点最近障碍物在y方向的偏移量绝对值Ry
 * @param casenum q编号
 * @return p点减去q点相对于最近障碍物的SED差
 */
int ObstacleDistanceMap::calH(int Rxq, int Ryq, int casenum)
{
    int h = 0;

    switch(casenum){
    case 1:
    case 5:
        h = 2 * Rxq + 1;
        break;

    case 3:
    case 7:
        h = 2 * Ryq + 1;
        break;

    case 2:
    case 4:
    case 6:
    case 8:
        h = 2 * (Rxq + Ryq + 1);
        break;

    default:
        break;
    }

    return h;
}


/**
 * @brief 更新p点相对于最近障碍物在x和y方向的偏移量绝对值Rx，Ry
 * @param casenum q编号
 * @param x p的x坐标
 * @param y p的y坐标
 */
void ObstacleDistanceMap::updateR(int casenum, int x, int y)
{
    obs_offset Rp;
    int Gx = 0, Gy = 0;
    switch (casenum){
    case 1:
        Gx = 1;
        Rp = R[x-1][y];
        break;

    case 2:
        Gx = 1;
        Gy = 1;
        Rp = R[x-1][y-1];
        break;

    case 3:
        Gy = 1;
        Rp = R[x][y-1];
        break;

    case 4:
        Gx = 1;
        Gy = 1;
        Rp = R[x+1][y-1];
        break;

    case 5:
        Gx = 1;
        Rp = R[x+1][y];
        break;

    case 6:
        Gx = 1;
        Gy = 1;
        Rp = R[x+1][y+1];
        break;

    case 7:
        Gy = 1;
        Rp = R[x][y+1];
        break;

    case 8:
        Gx = 1;
        Gy = 1;
        Rp = R[x-1][y+1];
        break;

    default:
        break;
    }

    R[x][y].x = Rp.x + Gx;
    R[x][y].y = Rp.y + Gy;
}


/**
 * @brief 获取不同SED值的最小值和最小SED值对应的q点编号
 * @param fp, fq1, fq2, fq3, fq4 根据四个q点计算得到的p点不同SED值
 * @return 最小SED值对应的q点编号
 */
int ObstacleDistanceMap::getMin(int fp, int fq1, int fq2, int fq3, int fq4)
{
    int temp[] = {fp, fq1, fq2, fq3, fq4};
    int minIndex = 0;

    minDis = temp[0];
    for(int i = 1; i < 5; i++){
        if(temp[i] < minDis){
            minDis = temp[i];
            minIndex = i;
        }
    }

    return minIndex;
}

/**
 * @brief 将SED值转换成ED值
 * @param x 栅格x
 * @param y 栅格y
 * @return fNearObsDis 欧式距离
 */
float ObstacleDistanceMap::getNearObsDis(int x, int y)
{
    float fNearObsDis = 0.;

    if((x > 0) && (x < Constants::ob_grid_width) && (y > 0) && (y < Constants::ob_grid_height))
    {
        fNearObsDis = sqrt(data[x][Constants::ob_grid_height - 1 - y]);
    }

    return fNearObsDis;
}


}
