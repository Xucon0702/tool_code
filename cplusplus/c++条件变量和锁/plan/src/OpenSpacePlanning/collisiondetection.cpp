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
* @file collisiondetection.cpp
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 实现车辆在栅格地图中某坐标和姿态下的精确碰撞检测
* @version V2.0
* @author  xingyu zhang
* @date 2020年7月2日
* @note 该算法参考由Ziegler提出的快速碰撞检测算法
* * 引用文献 :
* Ziegler, Julius & Stiller, Christoph. (2010). Fast Collision Checking for Intelligent Vehicle Motion Planning.
*/

#include "collisiondetection.h"

namespace HybridAStarPart {

CollisionDetection::CollisionDetection()
{
    Init();
}

CollisionDetection::~CollisionDetection(){}

void CollisionDetection::Init()
{
    m_tF.x = 1.0e5;
    m_tF.y = 1.0e5;
    m_tR.x = -1.0e5;
    m_tR.y = -1.0e5;
    m_fSafeDisOfVehContour = 0.15;
    bHasSetCorner = false;
    bHasSetSlotArea = false;
    m_fSlotAreaSafeDis = 0.0f;
    m_fOutSlotAreaSafeDis = 0.0f;
    m_bHasSlotBottomLine = false;
    DistanceMap.Init();
}

/**
 * @brief 设置车位关键角点和安全距离
 * @param tR 外侧后角点
 * @param tR2 内侧后角点
 * @param tF 外侧前角点
 * @param tF2 内侧前角点
 * @param fSafeDis 安全距离
 */
void CollisionDetection::setSlotConer(cpoint tR,cpoint tR2,
                                      cpoint tF, cpoint tF2,
                                      float fSafeDisOfVehContour,
                                      float fSafeDisOfMirror)
{
    m_tR = tR;
    m_tF = tF;
    m_tR2 = tR2;
    m_tF2 = tF2;
    bHasSetCorner = true;
    m_fSafeDisOfVehContour = fSafeDisOfVehContour;
    m_fMirrorSafeDis = fSafeDisOfMirror;
}

/**
 * @brief 设置车位外和车位内的不同安全距离
 * @param polygon 车位区域多边形
 * @param fOutSlotSafeDis 车位外圆碰撞检测安全距离
 * @param fInSlotSafeDis 车位内圆碰撞检测安全距离
 */
void CollisionDetection::setAreaSafeDis(const Polygon &polygon,
                                        float fOutSlotSafeDis,
                                        float fInSlotSafeDis)
{
    m_polygon = polygon;
    bHasSetSlotArea = true;
    m_fSlotAreaSafeDis = fInSlotSafeDis;
    m_fOutSlotAreaSafeDis = fOutSlotSafeDis;
}

/**
 * @brief 判断是否是自由栅格
 * @param grid 栅格类型
 * @return 1-自由栅格 0-不是
 */
bool CollisionDetection::gridIsFree(unsigned char grid)
{
    if(grid == VEL_COLOR
            || grid == 0
            || grid == SLOT_ARER
            ||grid == UNKNOW
            /*||grid == 11*//*MV_OBSTACLE*/
            ||grid == 16/*MV_GROUNDLOCK*/)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 * @brief 把char类型栅格地图转换距离地图
 * @param[in] map char类型栅格地图
 */
void CollisionDetection::updateGrid(unsigned char *map)
{
    for (int i = 0; i < Constants::ob_grid_num; i++)
    {
       binMap[i] = gridIsFree(map[i]) ? false : true;
    }//转化为二值地图

    DistanceMap.EuclideanDistanceTransform(binMap);
}

bool CollisionDetection::isTraversable(const Node3D* node, bool bHasSlotBottomLine)
{
  bool bIsTraversable = false;
  bool bEnterSlotArea = false;
  float x = 0;
  float y = 0;
  float t = 0;

  m_bHasSlotBottomLine = bHasSlotBottomLine;

  if(node != nullptr)
  {
      x = node->x;
      y = node->y;
      t = node->t;
      if(node->isStart)
      {
          return true;
      }
  }

  if(!CoarseOuterDiskCollisionCheck(x, y, t))
  {
      bIsTraversable = true;
  }
  else if(!AccurateOuterDiskCollisionCheck(x, y, t, bEnterSlotArea))
  {
      if(!bHasSetCorner)
      {
          bIsTraversable = true;
      }
      else
      {
          if(!bEnterSlotArea)
          {
              /*如果是起点, 不在车位区域,不对车位角点进行碰撞检测,避免第一个点距离车位角点很近时搜索异常!!!*/
              bIsTraversable = true;
          }
          else
          {
              if(!SlotCornerCollisionCheck(x, y, t))
              {
                  bIsTraversable = true;
              }
              else
              {
                  bIsTraversable = false;
              }
          }
      }

  }
  else
  {
      bIsTraversable = false;
  }

  return bIsTraversable;
}

float CollisionDetection::GetMinObstacleDisOfVehicle(float x,
                                                     float y,
                                                     float t)
{
    int nObsx = 0;
    int nObsy = 0;
    int nSmallCircleNum = Constants::SmallCircleNum;
    int nCircleNum = Constants::CircleNum;
    int BigCircleNum = Constants::BigCircleNum;

    float fObsDis = 0.0;
    float fMinObsDis = 1.0e5;
    float fCircleRadius = 0.0;
    float fSmallCicleRadius = Constants::SmallCicleRadius;
    float fBigCicleRadius = Constants::BigCicleRadius;
    float fRearViewCicleRadius = Constants::RearViewCicleRadius;
    float fObSize = Constants::obSize;
    float fInverseObSize = Constants::inverseObSize;
    cpoint temp;
    cpoint circleCenter;

    /*1 检查车轮廓圆的碰撞*/
    for(int nCircleIndex = 0; nCircleIndex < nCircleNum; nCircleIndex++)
    {
        if(nCircleIndex < nSmallCircleNum)
        {
            fCircleRadius = fSmallCicleRadius;
        }
        else if(nCircleIndex < nSmallCircleNum + BigCircleNum)
        {
            fCircleRadius = fBigCicleRadius;
        }
        else
        {
//            continue;
            fCircleRadius = fRearViewCicleRadius;
        }

        circleCenter.x = Constants::CircleCenter[nCircleIndex][0];
        circleCenter.y = Constants::CircleCenter[nCircleIndex][1];

        temp.x = circleCenter.x;
        temp.y = circleCenter.y;

        circleCenter.x = temp.x * cos(t) - temp.y * sin(t) + x;
        circleCenter.y = temp.x * sin(t) + temp.y * cos(t) + y;

         nObsx = int(circleCenter.x * fInverseObSize);
         nObsy = int(circleCenter.y * fInverseObSize);
         fObsDis = DistanceMap.getNearObsDis(nObsx, nObsy) * fObSize;
         fObsDis = fObsDis - fCircleRadius;

         if(fObsDis  < fMinObsDis)
         {
             fMinObsDis = fObsDis;
         }
    }
    return fMinObsDis;

}


/**
 * @brief 车辆粗糙外圆和障碍物碰撞检测
 * @param[in] x 3D网格中的x坐标
 * @param[in] y 3D网格中的y坐标
 * @param[in] t 3D网格中的偏航角
 * @return true-外圆和障碍物碰撞;false-外圆不和障碍物碰撞;
 */
bool CollisionDetection::CoarseOuterDiskCollisionCheck(float x, float y, float t)
{
    bool bFlag = false;
    bool bCollision = false;
    int nObsx = 0;
    int nObsy = 0;
    int nLNum =  Constants::OuterDiskLNum;
    float fObsDis = 0.0;
    float fRearViewMirrorToCenter = Constants::RearViewMirrorToCenter - Constants::RearViewWidth * 0.5;
    float fRearViewLength = Constants::RearViewLength;
    float fWidth = Constants::width;
    float OuterDiskRadius = Constants::OuterDiskRadius;
    float fRearViewCicleRadius = Constants::RearViewCicleRadius;
    float fCosT = cosf(t);
    float fSinT = sinf(t);
    float fInverseObSize = Constants::inverseObSize;

    cpoint temp;
    cpoint circleCenter;
    cpoint LMirrorCenter;
    cpoint RMirrorCenter;

    /*1 检查车轮廓圆的碰撞*/

    for(int i = 0; i < nLNum; ++i)
    {
        circleCenter.x = Constants::BigCircleCenter[i][0];
        circleCenter.y = Constants::BigCircleCenter[i][1];

        temp.x = circleCenter.x;
        temp.y = circleCenter.y;

        circleCenter.x = temp.x * fCosT - temp.y * fSinT + x;
        circleCenter.y = temp.x * fSinT + temp.y * fCosT + y;

        nObsx = int(circleCenter.x * fInverseObSize);
        nObsy = int(circleCenter.y * fInverseObSize);
        fObsDis = DistanceMap.getNearObsDis(nObsx, nObsy) * Constants::obSize;

        if(fObsDis < OuterDiskRadius + 0.2f)
        {
            bCollision = true;
            bFlag = true;
            break;
        }
    }

    /*2 检查两个后视镜的碰撞*/
    if(!bCollision)
    {
        LMirrorCenter.x = fRearViewMirrorToCenter;
        LMirrorCenter.y = fWidth * 0.5 + fRearViewLength * 0.5;

        temp.x = LMirrorCenter.x;
        temp.y = LMirrorCenter.y;

        LMirrorCenter.x = temp.x * fCosT - temp.y * fSinT + x;
        LMirrorCenter.y = temp.x * fSinT + temp.y * fCosT + y;

        nObsx = int(LMirrorCenter.x * fInverseObSize);
        nObsy = int(LMirrorCenter.y * fInverseObSize);
        fObsDis = DistanceMap.getNearObsDis(nObsx, nObsy)*Constants::obSize;

        /*当后视镜中心到障碍物距离小于后视镜外接圆半径则碰撞*/
        if(fObsDis < fRearViewCicleRadius + m_fOutSlotAreaSafeDis)
        {
            bCollision = true;
        }
        else
        {
            RMirrorCenter.x = fRearViewMirrorToCenter;
            RMirrorCenter.y = -(fWidth * 0.5 + fRearViewLength * 0.5);

            temp.x = RMirrorCenter.x;
            temp.y = RMirrorCenter.y;

            RMirrorCenter.x = temp.x * fCosT - temp.y * fSinT + x;
            RMirrorCenter.y = temp.x * fSinT + temp.y * fCosT + y;

            nObsx = int(RMirrorCenter.x * fInverseObSize);
            nObsy = int(RMirrorCenter.y * fInverseObSize);
            fObsDis = DistanceMap.getNearObsDis(nObsx, nObsy) * Constants::obSize;

            if(fObsDis < fRearViewCicleRadius + m_fOutSlotAreaSafeDis)
            {
                bCollision = true;
            }
        }
    }
    return bCollision;
}

/**
 * @brief 计算12个小圆和4个大圆的精确圆碰撞检测
 * @param[in] x 3D网格中的x坐标
 * @param[in] y 3D网格中的y坐标
 * @param[in] t 3D网格中的偏航角
 * @return true-外圆和障碍物碰撞;false-外圆不和障碍物碰撞;
 */
bool CollisionDetection::AccurateOuterDiskCollisionCheck(float x, float y, float t, bool &bEnterSlotArea)
{

    bool bFlag = false;
    bool bCollision = false;
    int nObsx = 0;
    int nObsy = 0;
    int nSmallCircleNum = Constants::SmallCircleNum;
    int BigCircleNum = Constants::BigCircleNum;
    int nCircleNum = Constants::CircleNum;

    float fObsDis = 0.0;
    float fCircleRadius = 0.0;
    float fSmallCicleRadius = Constants::SmallCicleRadius;
    float fBigCicleRadius = Constants::BigCicleRadius;
    float fRearViewCicleRadius = Constants::RearViewCicleRadius;
    float fObSize = Constants::obSize;
    float fInverseObSize = Constants::inverseObSize;
    float fBloating = 0.0;
    float fCosT = cosf(t);
    float fSinT = sinf(t);
    cpoint temp;
    cpoint circleCenter;
    Circle circle;

    bEnterSlotArea = false;
    /*1 检查车轮廓圆的碰撞*/
    for(int nCircleIndex = 0; nCircleIndex < nCircleNum; ++nCircleIndex)
    {
        if(nCircleIndex < nSmallCircleNum)
        {
            fCircleRadius = fSmallCicleRadius;
        }
        else if(nCircleIndex < nSmallCircleNum + BigCircleNum)
        {
            fCircleRadius = fBigCicleRadius;
        }
        else
        {
//            continue;
            fCircleRadius = fRearViewCicleRadius;
        }

        circleCenter.x = Constants::CircleCenter[nCircleIndex][0];
        circleCenter.y = Constants::CircleCenter[nCircleIndex][1];

        temp.x = circleCenter.x;
        temp.y = circleCenter.y;

        circleCenter.x = temp.x * fCosT - temp.y * fSinT + x;
        circleCenter.y = temp.x * fSinT + temp.y * fCosT + y;

         nObsx = int(circleCenter.x * fInverseObSize);
         nObsy = int(circleCenter.y * fInverseObSize);
         fObsDis = DistanceMap.getNearObsDis(nObsx, nObsy) * fObSize;

         circle.x = circleCenter.x;
         circle.y = circleCenter.y;
         circle.R = fCircleRadius;

         if(bHasSetSlotArea)
         {
             if(CrossDetectorPolyVsCircle(m_polygon, circle))
             {
                 if(nCircleIndex < nSmallCircleNum + BigCircleNum)
                 {
                     fBloating = m_fSlotAreaSafeDis;
                     /*车辆包络圆和车位四边形区域重叠*/
                     bEnterSlotArea = true;
                 }
                 else
                 {
                     continue;
                 }
             }
             else
             {
                 fBloating = m_fOutSlotAreaSafeDis;
             }
         }
         else
         {
             fBloating = 0.0f;
         }

         if(fObsDis < fCircleRadius + fBloating)
         {
             bCollision = true;
             bFlag = true;
             break;
         }else{}
    }
    return bCollision;
}

/**
 * @brief 车辆矩形轮廓和车位角点的碰撞检测
 * @param[in] x 3D网格中的x坐标
 * @param[in] y 3D网格中的y坐标
 * @param[in] t 3D网格中的偏航角
 * @return true-碰撞;false-不碰撞;
 */
bool CollisionDetection::SlotCornerCollisionCheck(float x, float y, float t)
{
    bool bCollision = false;
    LocationPoint tVeh(x, y, t);
    /* 角点碰撞检测，对freespace栅格地图带来的精度误差0-20cm做补充,
     * 同时对栅格地图，原先在车位角点存在障碍物和后续freespace更新时车位角点障碍物又出现较大改变做保护*/
     bCollision = CollisionCheckByVehKeyPointAndSlotCorner(tVeh,
                                                           m_tR,
                                                           m_tR2,
                                                           m_tF,
                                                           m_tF2,
                                                           m_fSafeDisOfVehContour,
                                                           m_fMirrorSafeDis,
                                                           m_bHasSlotBottomLine);

    return bCollision;
}

/**
 * @brief 点在多边形内
 * @param[in] x 点x坐标
 * @param[in] y 点y坐标
 * @param[in] polySides 多边形边数
 * @param[in] polyX 多边形点x坐标
 * @param[in] polyY 多边形点y坐标
 * @return true-点在多边形内;false-点不在多边形内;
 */
bool CollisionDetection:: pointInPolygon(float x, float y, int polySides, float* polyX, float* polyY)
{
  int   i,j=polySides-1;
  bool  oddNodes=false;
  for (i=0;i<polySides; i++)
  {
    if((polyY[i]< y && polyY[j]>=y||polyY[j]<y && polyY[i]>=y)&& (polyX[i]<=x || polyX[j]<=x))
    {
      oddNodes^=(polyX[i]+(y-polyY[i])/(polyY[j]-polyY[i])*(polyX[j]-polyX[i])<x);
    }
    j=i;
  }
  return oddNodes;
}


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
void CollisionDetection::CalVehicleCornerPoint(float x, float y, float t,
                                               cpoint& p0, cpoint& p1, cpoint& p2, cpoint& p3,
                                               float fLenBloating, float fWidBloating)
{
    cpoint temp;//临时值
    // points of the rectangle
    cpoint p[4];//四个点的坐标
    cpoint nP[4];
    float fCosT = cos(t);
    float fSinT = sin(t);

    p[0].x = x - Constants::RearEdgeToCenter - fLenBloating; //车的左下角位置
    p[0].y = y - Constants::width * 0.5 - fWidBloating;

    p[1].x = x - Constants::RearEdgeToCenter - fLenBloating;//车的左上角位置
    p[1].y = y + Constants::width * 0.5  + fWidBloating;

    p[2].x = x + (Constants::FrontEdgeToCenter) + fLenBloating;//车的 右上角位置
    p[2].y = y + Constants::width * 0.5  + fWidBloating;

    p[3].x = x + (Constants::FrontEdgeToCenter) + fLenBloating;//车的右下角位置
    p[3].y = y - Constants::width * 0.5 - fWidBloating;
    for (int j = 0; j < 4; ++j)
    {
        temp.x = p[j].x - x;//以车的中心为坐标原点
        temp.y = p[j].y - y;
        // rotate and shift back:将车旋转theta角度
        nP[j].x = temp.x * fCosT - temp.y * fSinT + x;
        nP[j].y = temp.x * fSinT + temp.y * fCosT + y;
    }
    p0 = nP[0];
    p1 = nP[1];
    p2 = nP[2];
    p3 = nP[3];
}


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
void CollisionDetection::CalBoundingBox(float x,
                                        float y,
                                        float t,
                                        float fLenBloating,
                                        float fWidBloating,
                                        float& X1bounding,
                                        float& X2bounding,
                                        float& Y1bounding,
                                        float& Y2bounding)
{
    // points of the rectangle
    cpoint nP[4];
    std::vector<float> px;
    std::vector<float> py;

    CalVehicleCornerPoint(x, y, t, nP[0], nP[1], nP[2], nP[3], fLenBloating, fWidBloating);

    for (int j = 0; j < 4; ++j)
    {
        px.push_back(nP[j].x);
        py.push_back(nP[j].y);
    }
    std::sort(px.begin(), px.end());
    std::sort(py.begin(), py.end());
    X1bounding=px[0];
    X2bounding=px[3];
    Y1bounding=py[0];
    Y2bounding=py[3];
}

/**
 * @brief 更新当前车辆所覆盖的栅格地图值为0
 * @param[in] x 3D网格中的x坐标
 * @param[in] y 3D网格中的y坐标
 * @param[in] t 3D网格中的偏航角
 */
void CollisionDetection::UpdateCurrentVehiclePositionOccupyMap(float x, float y, float t, float bloating)
{
    float X1bounding;
    float X2bounding;
    float Y1bounding;
    float Y2bounding;
    float obx;
    float oby;
    float fInverseObSize = Constants::inverseObSize;
    // points of the rectangle
    cpoint nP[4];

    CalBoundingBox(x, y, t,  bloating, bloating, X1bounding, X2bounding, Y1bounding, Y2bounding);
    CalVehicleCornerPoint(x, y, t, nP[0], nP[1], nP[2], nP[3], bloating, bloating);

    float pPolyX[4]={nP[0].x, nP[1].x, nP[2].x, nP[3].x};
    float pPolyY[4]={nP[0].y,nP[1].y,nP[2].y,nP[3].y};

    for (int i = int(X1bounding * fInverseObSize); i < int(X2bounding * fInverseObSize); ++i)
    {
        for (int j = int(Y1bounding * fInverseObSize); j < int(Y2bounding * fInverseObSize); ++j)
        {
            obx=(i + 0.5f) * Constants::obSize;
            oby=(j + 0.5f) * Constants::obSize;

            if(pointInPolygon(obx, oby, 4, pPolyX, pPolyY))
            {
                binMap[j * Constants::ob_grid_width + i]=0;
            }
        }
    }

    DistanceMap.EuclideanDistanceTransform(binMap);

}

/**
 * @brief 更新当前车辆所覆盖的栅格地图值为255
 * @param[in] x 3D网格中的x坐标
 * @param[in] y 3D网格中的y坐标
 * @param[in] t 3D网格中的偏航角
 */
void CollisionDetection::deleteCurrentVehiclePositionOccupyMap(float x, float y, float t)
{
    float X1bounding = 0.0;
    float X2bounding = 0.0;
    float Y1bounding = 0.0;
    float Y2bounding = 0.0;
    float obx = 0.0;
    float oby = 0.0;
    float fInverseObSize = Constants::inverseObSize;
    // points of the rectangle
    cpoint nP[4];

    CalBoundingBox(x, y, t, 0.0, 0.0, X1bounding, X2bounding, Y1bounding, Y2bounding);
    CalVehicleCornerPoint(x, y, t, nP[0], nP[1], nP[2], nP[3], 0.1, 0.1);
    float pPolyX[4]={nP[0].x, nP[1].x, nP[2].x, nP[3].x};
    float pPolyY[4]={nP[0].y,nP[1].y,nP[2].y,nP[3].y};

    for (int i = int(X1bounding * fInverseObSize); i < int(X2bounding * fInverseObSize); ++i)
    {
        for (int j = int(Y1bounding * fInverseObSize); j < int(Y2bounding * fInverseObSize); ++j)
        {
            obx = (i + 0.5f) * Constants::obSize;
            oby = (j + 0.5f) * Constants::obSize;

            if(pointInPolygon(obx, oby, 4, pPolyX, pPolyY))
            {
                binMap[j * Constants::ob_grid_width + i]= 1;
            }
        }
    }
}
}
