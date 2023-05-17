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
* @file collisiondetection.h
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 实现车辆在栅格地图中某坐标和姿态下的精确碰撞检测
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该算法参考由Ziegler提出的快速碰撞检测算法
* * 引用文献 :
* Ziegler, Julius & Stiller, Christoph. (2010). Fast Collision Checking for Intelligent Vehicle Motion Planning.
*/

#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <vector>
#include "base.h"
#include "constants.h"
#include "node3d.h"
#include"publicfunpathplan.h"
#include"obstacledistancemap.h"
namespace HybridAStarPart
{

/*CollisionDetection用于检测给定车辆的一个配置q是否与环境发生碰撞*/
class CollisionDetection
{
 public:
  CollisionDetection(); //构造函数
~CollisionDetection(); //析构函数

  void Init();
  /**
   * @brief 可通行性检验
   * @param[in] node 3D节点
   * @return true-为自由网格；false-车辆和障碍物碰撞
   */
  bool isTraversable(const Node3D* node, bool bHasSlotBottomLine = false);

  /**
   * @brief 设置车位关键角点和安全距离
   * @param tR 外侧后角点
   * @param tR2 内侧后角点
   * @param tF 外侧前角点
   * @param tF2 内侧前角点
   * @param fSafeDis 安全距离
   */
  void setSlotConer(cpoint tR, cpoint tR2, cpoint tF, cpoint tF2, float fSafeDisOfVehContour, float fSafeDisOfMirror);

  /**
   * @brief 设置车位外和车位内的不同安全距离
   * @param polygon 车位区域多边形
   * @param fOutSlotSafeDis 车位外圆碰撞检测安全距离
   * @param fInSlotSafeDis 车位内圆碰撞检测安全距离
   */
  void setAreaSafeDis(const Polygon &polygon,
                      float fOutSlotSafeDis,
                      float fInSlotSafeDis);

  /**
   * @brief 判断是否是自由栅格
   * @param grid 栅格类型
   * @return 1-自由栅格 0-不是
   */
  bool gridIsFree(unsigned char grid);
  /**
   * @brief 把char类型栅格地图转换距离地图
   * @param[in] map char类型栅格地图
   */
  void updateGrid(unsigned char *map);

  float GetMinObstacleDisOfVehicle(float x,
                                   float y,
                                   float t);
  /**
   * @brief 获取char类型栅格地图
   * @return OccupyGrid* 栅格地图
   */
  bool* getGrid(void) {return binMap;}

  /**
   * @brief 计算车辆角点
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   * @param[in] p0 车的左下角位置
   * @param[in] p1 车的左上角位置
   * @param[in] p2 车的右上角位置
   * @param[in] p3 车的右下角位置
   */
  void CalVehicleCornerPoint(float x, float y, float t,
                             cpoint &p0, cpoint &p1, cpoint &p2, cpoint &p3,
                             float fLenBloating = 0.0,
                             float fWidBloating = 0.0);

  /**
   * @brief 计算车辆包围盒
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   * @param[in] X1bounding 包围盒较小的x坐标
   * @param[in] X2bounding 包围盒较大的x坐标
   * @param[in] Y1bounding 包围盒较小的y坐标
   * @param[in] Y2bounding 包围盒较大的y坐标
   */
  void CalBoundingBox(float x,
                      float y,
                      float t,
                      float fLenBloating,
                      float fWidBloating,
                      float &X1bounding,
                      float &X2bounding,
                      float &Y1bounding,
                      float &Y2bounding);

  /**
   * @brief 更新当前车辆所覆盖的栅格地图值为0
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   */
  void UpdateCurrentVehiclePositionOccupyMap(float x, float y, float t, float bloating);

  /**
   * @brief 更新当前车辆所覆盖的栅格地图值为255
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   */
  void deleteCurrentVehiclePositionOccupyMap(float x, float y, float t);

  /**
   * @brief 点在多边形内
   * @param[in] x 点x坐标
   * @param[in] y 点y坐标
   * @param[in] polySides 多边形边数
   * @param[in] polyX 多边形点x坐标
   * @param[in] polyY 多边形点y坐标
   * @return true-点在多边形内;false-点不在多边形内;
   */
  bool pointInPolygon(float x, float y, int polySides, float* polyX, float* polyY);

  /**
   * @brief 车辆粗糙外圆和障碍物碰撞检测
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   * @return true-外圆和障碍物碰撞;false-外圆不和障碍物碰撞;
   */
  bool CoarseOuterDiskCollisionCheck(float x, float y, float t);

  /**
   * @brief 计算12个小圆和4个大圆的精确圆碰撞检测
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   * @return true-外圆和障碍物碰撞;false-外圆不和障碍物碰撞;
   */
  bool AccurateOuterDiskCollisionCheck(float x, float y, float t, bool &bEnterSlotArea);

  /**
   * @brief 车辆矩形轮廓和车位角点的碰撞检测
   * @param[in] x 3D网格中的x坐标
   * @param[in] y 3D网格中的y坐标
   * @param[in] t 3D网格中的偏航角
   * @return true-碰撞;false-不碰撞;
   */
  bool SlotCornerCollisionCheck(float x, float y, float t);
 private:

  /*距离地图*/
  ObstacleDistanceMap DistanceMap;

  /*bool类型二值地图*/
  bool binMap[Constants::ob_grid_num];

  /*0-不考虑后视镜 1-考虑后视镜*/
  bool bHasSetCorner;
  bool bHasSetSlotArea;
  float m_fSlotAreaSafeDis;
  float m_fSafeDisOfVehContour;
  float m_fMirrorSafeDis;
  float m_fOutSlotAreaSafeDis;
  cpoint m_tF;
  cpoint m_tR;
  cpoint m_tF2;
  cpoint m_tR2;
  Polygon m_polygon;
  bool m_bHasSlotBottomLine;
};
}
#endif // COLLISIONDETECTION_H
