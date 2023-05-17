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
* @file hybrid_a_star.cpp
* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
* @brief 路径规划的主函数,主要实现垂直泊车，斜列泊车，平行泊车
* hybrid A*搜索路径、轨迹分割处理、共轭梯度轨迹优化算法、控制点发布等主要功能
* @version V2.0
* @author  xingyu zhang
* @date 2021年4月14日
* @note 该算法参考由Dmitri Dolgov在2007 DARPA Urban Challenge提出的hybrid A*算法主架构，
* 针对狭窄空间的泊车特殊情况,不同之处在于reedsSheep的曲线连接方式，路径的优化目标函数，以及惩罚函数的设计
* 引用文献 :
* 1 Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2010). Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments.
* 2 Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2008). Practical Search Techniques in Path Planning for Autonomous Driving.
* 3 Dolgov, Dmitri & Thrun, Sebastian. (2009). Autonomous driving in semi-structured environments: Mapping and planning.
* 4 KARL KURZER. (2016) . Path Planning in Unstructured Environments : A Real-time Hybrid A' Implementation for Fast and Deterministic Path Generation for the KTH Research Concept Vehicle.
* 5 Mizuno, Naoki & Ohno, Kazunori & Hamada, Ryunosuke & Kojima, Hiroyoshi & Fujita, Jun & Amano, Hisanori & Westfechtel, Thomas & Suzuki, Takahiro & Tadokoro, Satoshi. (2019). Enhanced path smoothing based on conjugate gradient descent for firefighting robots in petrochemical complexes.
* 6 Tianyu Gu , Jarrod Snider , John M. Dolan. (2013) .Focused Trajectory Planning for Autonomous On-Road Driving.
* 7 Ziegler, Julius & Stiller, Christoph. (2010). Fast Collision Checking for Intelligent Vehicle Motion Planning.
* 8 Shih, Frank & Wu, Yi-Ta. (2004). Note Fast Euclidean distance transformation in two scans using a 3 3 neighborhood. Computer Vision and Image Understanding.
*/

#include "hybrid_a_star.h"


namespace HybridAStarPart
{
static Node3D gNodes3D[NODE3D_NUM];

void HybridAStar::Init(void)
{
    m_OriginSlotYaw = 0.0;
    m_ParaHybridALastDriving = false;
    m_bStartPathplanningInParaSLot =false;
    m_TotalGearSwitchTimes = 0;
    m_nDrivingDirection = 0;
    m_bFirstPlan = false;
    m_nFirstPlanTotalSteps = 0;
    mParaSlotSteps = 0;
    m_bUSSHasDetectFrontMargin = false;
    m_bUSSHasDetectRearMargin = false;
    m_ParkingCtrlStatus = freeStauts;
    m_RotationCoordinate.x = 0;
    m_RotationCoordinate.y = 0;
    m_RotationCoordinate.yaw = 0;
    m_bLastDriving = false;
    m_HybridAResults.clear();
    m_PartitionedTargetCurvatureSets.clear();
    m_nCurrentStep = 0;
    m_nTotalStep = 0;
    m_fResidualDis = 0.0;
    m_fMovingDisF = 0.0;
    m_fMovingDisR = 0.0;
    m_deltaRearY = 0.0;
    m_deltaRearX = 0.0;
    m_deltaFrontY = 0.0;
    m_deltaFrontX = 0.0;
    m_bBestSafeObliParkingInPath = false;
    m_pReedsSheepsNodes = nullptr;
    m_configurationSpace.Init();
    lmfOpter.Init();
    m_bFindPath = false;
    m_nAdditionalSearchTimes = 0;
    m_nCollisonCheckTimes = 0;
    m_fYawOffsetOfOpti = 0;
    m_fHeadInRsRadius = 0;
    m_bHitWheelBar = false;
    planErrCode=NO_ERROR;
}


/**
 * @brief hybridAStar路径规划算法主函数
 * @param start 起点
 * @param goal 终点
 * @param nodes3D 3D节点
 * @param width 3D网格宽度
 * @param height 3D网格高度
 * @param configurationSpace 用于碰撞检测的配置空间
 * @return 终点的3D节点
 */
bool HybridAStar::hybridAStar(Node3D& start,
                              Node3D& goal,
                              Node3D* nodes3D,
                              CollisionDetection& configurationSpace,
                              vector<PARTITION_PATH> &reservedPath)
{
  GAC_LOG_INFO("node 3d is %d, colsapce is %d", sizeof(gNodes3D), sizeof(m_configurationSpace));
  GAC_LOG_INFO("one step is %f, one step is %f",
       Constants::TravelDis / (Constants::WheelBase / Constants::fTanSteering[3]) * 57.3,
        Constants::TravelDis / (Constants::WheelBase / Constants::fTanSteering[4]) * 57.3);
  bool bFinishShooting = false;
  bool bGoalValid = false;
  int iPred = 0;
  int iSucc = 0;
  int nStartIndexs = 0; /*开始的搜索树索引号*/
  int nEndIndexs = Constants::next_node_num_; /*结束的搜索树索引号*/
  int iterations = 0;//迭代计数
  int explored_node_num = 0;
  float newG = 0.0;

  Node3D *NextNode = nullptr;
  Node3D *CurrentNode  = nullptr;

#ifdef DEGUG_TIME
  float reedsshepp_time=0.0;
  float sort_time_=0.0;
  float valid_check_time=0.0;
  float emplace_time=0.0;
  float heauristic_time=0.0;
  float CalCost_time = 0.0;
  struct timeval tpstartRS,tpendRS;
  float timeuseRS;

  struct timeval tpstartValidCheck,tpendValidCheck;
  float timeuseValidCheck;

  struct timeval tpstartheuristic,tpendheuristic;
  float timeuse_heuristic;

  struct timeval tpstart_sort_time,tpend_sort_time;
  float timeuse_sort_time;

  struct timeval tpstart_generate_next_node,tpend_generate_next_node;
  float timeuse_generate_next_node;
  float generate_next_node_time=0.0;


  struct timeval tpstart_emplace,tpend_emplace;
  float timeuse_emplace=0.0;

  struct timeval tpstartCalCost,tpendCalCost;
  float timeuse_CalCost=0.0;

  struct timeval tpstartCalID,tpendCalID;
  float timeuse_CalID=0.0;
  float CalID_time = 0.0;
  float delete_time = 0.0;

  struct timeval tpstartdelete,tpendCaldelete;
  float timeUsedelete = 0.0;
#endif
  open_pq = decltype(open_pq)();
  close_pq = decltype(close_pq)();
  reservedPath.clear();
  if(m_slotPosInfor.nSlotType == PARALLEL)
  {
      configurationSpace.UpdateCurrentVehiclePositionOccupyMap(goal.x,
                                                               goal.y, goal.t,
                                                               Constants::paraVehBloating);
  }
  if(configurationSpace.isTraversable(&start))
  {
      GAC_LOG_INFO("start is free!!!");
  }

  /*判断终点是否无碰撞*/
  if(!configurationSpace.isTraversable(&goal))
  {
      /*E10003垂直车位目标点和车位干涉*/
      planErrCode=E10003;
      GAC_LOG_INFO("Planing Fail, goal point hit the slot!!!\n");
      NextNode = nullptr;
      bGoalValid = false;
#if 0
      goal.x -= VEH_MOVE_DIS;
      if(!configurationSpace.isTraversable(&goal))
      {
          goal.x += 2 * VEH_MOVE_DIS;
          if(!configurationSpace.isTraversable(&goal))
          {
              GAC_LOG_DEBUG("Planing Fail, goal point hit the slot!!!\n");
              NextNode = nullptr;
              bGoalValid = false;
          }
          else
          {
              bGoalValid = true;
              GAC_LOG_DEBUG("goal move +0.1m in the slot!!!\n");
          }
      }
      else
      {
          bGoalValid = true;
          GAC_LOG_DEBUG("goal move -0.1m in the slot!!!\n");
      }
#endif
  }
  else
  {
      bGoalValid = true;
  }

  if(bGoalValid)
  {
      configurationSpace.UpdateCurrentVehiclePositionOccupyMap(start.x,
                                                               start.y,
                                                               start.t,
                                                               Constants::startVehBloating);
      /*将点标记为open*/
      start.open();

      /*计算到目标的启发值*/
      updateH(start, goal);

      /*计算索引位置*/
      iPred = start.setIdx(m_varialeCellBound);

      /*把start节点装入3D数组内*/
      nodes3D[iPred] = start;
      /*将start加入open队列*/
      open_pq.emplace(&nodes3D[iPred]);

      CurrentNode = open_pq.top();

      /*直到队列元素为空*/
      while(!open_pq.empty())
      {
          /*记录迭代次数*/
          iterations++;
          if(iterations == 1019)
          {
              float a = 0;
          }
          /*循环部分：从队列中取出一个最低代价的点*/
#ifdef DEGUG_TIME
          gettimeofday(&tpstart_sort_time, NULL);
#endif
          CurrentNode = open_pq.top();
#ifdef DEGUG_TIME
          gettimeofday(&tpend_sort_time,NULL);
          timeuse_sort_time=(1000000*(tpend_sort_time.tv_sec-tpstart_sort_time.tv_sec) + tpend_sort_time.tv_usec-tpstart_sort_time.tv_usec)/1000.0;
          sort_time_ += timeuse_sort_time;
#endif
          /*获取该点在nodes3D的索引*/
          iPred = CurrentNode->getIdx();
          /*检查该点是否closed状态*/
          if (nodes3D[iPred].isClosed())
          {
              /*如果为closed，说明该点已经处理过，将它从open队列中移除*/
              open_pq.pop();
              continue;
          }
          else if(nodes3D[iPred].isOpen())
          {
              close_pq.emplace(&nodes3D[iPred]);
              /*如果该点是在open状态，即正在扩张的点,将它的状态标记为closed*/
              nodes3D[iPred].close();
              /*将它从open队列中移除*/
              open_pq.pop();
          }
          else
          {
              GAC_LOG_WARN("search error !!!");
              break;
          }
          if (iterations > Constants::iterations)
          {
              /*超过最大迭代次数退出*/
              NextNode = nullptr;
              break;
          }
          else
          {
              if ((CurrentNode->isInRange(goal) || (CurrentNode->isStart)) && (m_slotPosInfor.nParkingType != HEAD_PARK_OUT))
              {
                  /*如果当前点在连接RS曲线的有效距离范围内,进行RS曲线的解析扩展*/
#ifdef DEGUG_TIME
                  gettimeofday(&tpstartRS,NULL);
#endif
                  bFinishShooting = ReedsSheepsShot(*CurrentNode, goal, configurationSpace, reservedPath);
#ifdef DEGUG_TIME
                  gettimeofday(&tpendRS,NULL);
                  timeuseRS=(1000000*(tpendRS.tv_sec-tpstartRS.tv_sec) + tpendRS.tv_usec-tpstartRS.tv_usec)/1000.0;
                  reedsshepp_time+=timeuseRS;
#endif
                  /*如果当前状态是动态规划，那么直接返回*/
                  if (m_ParkingCtrlStatus == RunningFreeStauts)
                  {
                      break;
                  }
                  else
                  {
                      if (bFinishShooting)
                      {
                          /*如果RS曲线能够无碰撞连接,返回终点节点*/
                          break;
                      }
                      else
                      {
                          NextNode = nullptr;
                      }
                  }
              }else if(m_slotPosInfor.nParkingType == HEAD_PARK_OUT && CheckParkoutSearch(CurrentNode)){
                    // reservedPath
                    TraceSearchPath(*CurrentNode, reservedPath);
                    break;

              }

              /*确定搜索树的扩展类型*/
              nStartIndexs = 0;
              nEndIndexs = Constants::next_node_num_;

              /*扩展节点*/
              for (int nNextNodeIndex = nStartIndexs; nNextNodeIndex < nEndIndexs; nNextNodeIndex++)
              {
                  /*由当前节点扩展节点*/
#ifdef DEGUG_TIME
                  gettimeofday(&tpstart_generate_next_node,NULL);
#endif
                  NextNode = NextNodeGenerator(CurrentNode, nNextNodeIndex);
//                  if(!NextNodeIsValid(NextNode, m_slotPosInfor))
//                  {
//                      delete NextNode;
//                      continue;
//                  }
                  explored_node_num++;
#ifdef DEGUG_TIME
                  gettimeofday(&tpend_generate_next_node,NULL);
                  timeuse_generate_next_node = (1000000*(tpend_generate_next_node.tv_sec-tpstart_generate_next_node.tv_sec) + tpend_generate_next_node.tv_usec-tpstart_generate_next_node.tv_usec)/1000.0;
                  generate_next_node_time += timeuse_generate_next_node;
#endif
                  /*设置索引值*/
#ifdef DEGUG_TIME
                  gettimeofday(&tpstartCalID,NULL);
#endif
                  iSucc = NextNode->setIdx(m_varialeCellBound);//索引值
                  if(iSucc == CurrentNode->getIdx())
                  {
                      GAC_LOG_INFO("invalid expansion");
                  }
#ifdef DEGUG_TIME
                  gettimeofday(&tpendCalID,NULL);
                  timeuse_CalID = (1000000*(tpendCalID.tv_sec-tpstartCalID.tv_sec) + tpendCalID.tv_usec-tpstartCalID.tv_usec)/1000.0;
                  CalID_time += timeuse_CalID;
#endif
                  /*判断扩展的下一个节点是否在地图有效范围内和是否碰撞障碍物*/
#ifdef DEGUG_TIME
                  gettimeofday(&tpstartValidCheck,NULL);
#endif
                  m_nCollisonCheckTimes++;
                  if (NextNode->isOnGrid() && configurationSpace.isTraversable(NextNode, true))
                  {
#ifdef DEGUG_TIME
                      gettimeofday(&tpendValidCheck,NULL);
                      timeuseValidCheck=(1000000*(tpendValidCheck.tv_sec-tpstartValidCheck.tv_sec) + tpendValidCheck.tv_usec-tpstartValidCheck.tv_usec)/1000.0;
                      valid_check_time+=timeuseValidCheck;
#endif
                      if (!nodes3D[iSucc].isClosed())
                      {
                          /*计算已经行驶代价*/
#ifdef DEGUG_TIME
                          gettimeofday(&tpstartCalCost,NULL);
#endif
                          NextNode->TrajCost = CurrentNode->TrajCost + TrajCost(CurrentNode, NextNode);
                          newG = NextNode->TrajCost;
#ifdef DEGUG_TIME
                          gettimeofday(&tpendCalCost,NULL);
                          timeuse_CalCost=(1000000*(tpendCalCost.tv_sec-tpstartCalCost.tv_sec) + tpendCalCost.tv_usec-tpstartCalCost.tv_usec)/1000.0;

                          CalCost_time += timeuse_CalCost;
#endif
                          /*如果扩展节点不是close状态也不是open状态（说明该节点索引号未探查)*/
                          /*或者扩展节点是open状态，但是搜到到达该节点更短的路径*/
                          if ((!nodes3D[iSucc].isOpen())
                                  || (nodes3D[iSucc].isOpen() && (newG < nodes3D[iSucc].TrajCost)))
                          {
                              /*计算将要行驶代价*/
#ifdef DEGUG_TIME
                              gettimeofday(&tpstartheuristic,NULL);
#endif
                              updateH(*NextNode, goal);
#ifdef DEGUG_TIME
                              gettimeofday(&tpendheuristic,NULL);
                              timeuse_heuristic=(1000000*(tpendheuristic.tv_sec-tpstartheuristic.tv_sec) + tpendheuristic.tv_usec-tpstartheuristic.tv_usec)/1000.0;
                              heauristic_time+=timeuse_heuristic;
#endif
                              /*扩展节点设置open状态*/
                              NextNode->open();

                              /*把扩展节点信息更新给3D网格索引号对应的节点*/
                              nodes3D[iSucc] = *NextNode;

                              /*将节点加入open队列*/
#ifdef DEGUG_TIME
                              gettimeofday(&tpstart_emplace,NULL);
#endif
                              open_pq.emplace(&nodes3D[iSucc]);
#ifdef DEGUG_TIME
                              gettimeofday(&tpend_emplace, NULL);
                              timeuse_emplace = (1000000*(tpend_emplace.tv_sec-tpstart_emplace.tv_sec) + tpend_emplace.tv_usec-tpstart_emplace.tv_usec)/1000.0;
                              emplace_time += timeuse_emplace;
#endif
                          }
                      }
                  }
                  else
                  {
#ifdef DEGUG_TIME
                      gettimeofday(&tpendValidCheck,NULL);
                      timeuseValidCheck = (1000000 * (tpendValidCheck.tv_sec-tpstartValidCheck.tv_sec) + tpendValidCheck.tv_usec-tpstartValidCheck.tv_usec)/1000.0;
                      valid_check_time += timeuseValidCheck;
#endif
                  }
                  /*释放扩展节点*/
                  delete NextNode;
                  NextNode = nullptr;
              }
          }
      }
  }
  GAC_LOG_INFO("Total iterations is %d, explored_node_num is %d\n",iterations, explored_node_num);
  GAC_LOG_INFO("A* expansion collison check times is %d \n", m_nCollisonCheckTimes);

#ifdef DEGUG_TIME
  GAC_LOG_DEBUG("reedsshepp time is %f, valid_check_time is %f\n", reedsshepp_time, valid_check_time);
  GAC_LOG_DEBUG("sort time is %f reedsshepp time is %f, valid_check_time is %f heauristic_time is %f emplace_time is %f, generate_next_node_time is %f\n", sort_time_, reedsshepp_time, valid_check_time,heauristic_time, emplace_time, generate_next_node_time);
  GAC_LOG_DEBUG("reedsshepp time is %f, valid_check_time is %f\n", reedsshepp_time, valid_check_time);
  GAC_LOG_DEBUG("A* expansion collison check times is %d, valid_check_time is %f, time cost of one point colliosion is %f\n", m_nCollisonCheckTimes, valid_check_time, valid_check_time/m_nCollisonCheckTimes);
  GAC_LOG_DEBUG("CalCost_time is %f, CalID_time is %f, delete_time is %f\n", CalCost_time, CalID_time, delete_time);
#endif
  if(reservedPath.empty())
  {
      return false;
  }
  else
  {
      return true;
  }
}


/**
 * @brief 计算到目标的行驶代价
 * @param start 起点
 * @param goal 终点
 */
void HybridAStar:: updateH(Node3D& start, const Node3D& goal)
{
    float fCenterLineDisCost = 0.0;
    float fCenterLineDis = 0.0;
    float fMaxDis= 4.0;
    if(m_slotPosInfor.nSlotType == OBLIQUE)
    {
        if(m_slotPosInfor.nParkingType == HEAD_PARK_IN)
        {
            fMaxDis = 7.0f;
        }
        fCenterLineDis = fabsf(m_slotPosInfor.LineEq_Goal.A * start.x + m_slotPosInfor.LineEq_Goal.B * start.y + m_slotPosInfor.LineEq_Goal.C)
                / sqrt(m_slotPosInfor.LineEq_Goal.A * m_slotPosInfor.LineEq_Goal.A + m_slotPosInfor.LineEq_Goal.B * m_slotPosInfor.LineEq_Goal.B);
        if(fCenterLineDis <= fMaxDis)
        {
            fCenterLineDisCost = 0;
        }
        else
        {
            fCenterLineDisCost = fCenterLineDis - fMaxDis;
        }
    }
    else
    {
        fCenterLineDisCost = 0.0;
    }
//    start.HeuristicCost = 2 * fCenterLineDisCost;
    float fEuclideanDisCost = fabs(normalizeHeadingRad_Npi_Ppi(start.t - goal.t)) * Constants::r + hypot(start.x - goal.x, start.y - goal.y);
    start.HeuristicCost = 2 * fCenterLineDisCost + fEuclideanDisCost;

    if(m_slotPosInfor.nParkingType == HEAD_PARK_OUT)
    {
        start.HeuristicCost = fabs(normalizeHeadingRad_Npi_Ppi(start.t - goal.t)) * Constants::r + hypot(start.x - goal.x, start.y - goal.y);
    }
}


/**
 * @brief 根据扩展树索引号，由当前节点生成下一个节点
 * @param CurrentNode 当前节点
 * @param nNextNodeIndex 生成下一个节点扩展树索引号
 * @return 节点
 */
Node3D* HybridAStar::NextNodeGenerator(Node3D* CurrentNode, int nNextNodeIndex)
{
    float fLastX = 0.0;
    float fLastY = 0.0;
    float fLastPhi = 0.0;
    float fNextX = 0.0;
    float fNextY = 0.0;
    float fNextPhi = 0.0;
    float fTan = 0.0;
    float L = Constants::WheelBase;
    float fSteering=0.0;
    float fTraveledDistance= 0.0;
    float fStepSize = Constants::TravelDis;
    float fCurvature = 0.0;
    int8_t nDirection = 0;
    int nNextNodeNum = Constants::next_node_num_;
    int nSameDirNodeNum = 0;

    if(CurrentNode != nullptr)
    {
        if(nNextNodeIndex < nNextNodeNum / 2)
        {
            fTraveledDistance = fStepSize;
            nDirection = FORWARD;
        }
        else
        {
            fTraveledDistance = -fStepSize;
            nDirection = BACKWARD;
        }
        fTan = Constants::fTanSteering[nNextNodeIndex];
        fCurvature =  fTan / L; /*左转为正 右转为负*/
        fSteering = Constants::fSteering[nNextNodeIndex];
        fLastX = CurrentNode->x;
        fLastY = CurrentNode->y;
        fLastPhi = CurrentNode->t;
        fNextPhi = normalizeHeadingRad_0_2pi(fLastPhi + fTraveledDistance * fCurvature);
        fNextX = fLastX + fTraveledDistance * cos(fLastPhi + fTraveledDistance * fCurvature / 2);
        fNextY = fLastY + fTraveledDistance * sin(fLastPhi + fTraveledDistance * fCurvature / 2);

        if(CurrentNode->isStart || (CurrentNode->nDirection != nDirection))
        {
            nSameDirNodeNum = 1;
        }
        else
        {
            nSameDirNodeNum = CurrentNode->nSameDirNodeNum + 1;
        }

    }else{}

   return new Node3D(fNextX, fNextY, fNextPhi, 0, 0,
                     CurrentNode, nDirection, false,
                     fSteering, nSameDirNodeNum);
}


bool HybridAStar::NextNodeIsValid(Node3D* NextNode, slotPosInfor &slotPosInfor_)
{
    cpoint tHead;
    cpoint tTail;
    float fMaxDis = 6.3f;

    tHead.x = NextNode->x + Constants::FrontEdgeToCenter * cosf(NextNode->t);
    tHead.y = NextNode->y + Constants::FrontEdgeToCenter * sinf(NextNode->t);

    tTail.x = NextNode->x - Constants::RearEdgeToCenter * cosf(NextNode->t);
    tTail.y = NextNode->y - Constants::RearEdgeToCenter * sinf(NextNode->t);
    if(CalPointAndLineDis(tHead, slotPosInfor_.LineEq_Goal) < fMaxDis
            || CalPointAndLineDis(tTail, slotPosInfor_.LineEq_Goal) < fMaxDis)
    {
        return true;
    }
    else
    {
        return false;
    }

}

/**
* @brief 不合理的转向判断
* @param NextNode 下一个节点
* @return 1-合理 0-不合理
*/
bool HybridAStar::IllogicalSteeringCheck(Node3D *NextNode)
{
   bool bIllogicalSteering = false;
   float fError = 1e-3;
   float fCrossValue = m_slotPosInfor.fCrossValueStartAndGoalPoint;
   float fGoalYaw3d = m_slotPosInfor.goalPointInMap.yaw;
   float fStartYaw3d = m_slotPosInfor.startPointInMap.yaw;

   if(NextNode != nullptr)
   {
       if(fabsf(normalizeHeadingRad_Npi_Ppi(fGoalYaw3d - fStartYaw3d)) < 10 * PI / 180)
       {
           bIllogicalSteering = false;
       }
       else
       {
           /*用叉乘表达起点向量和终点向量是顺时针还是逆时针*/
           if(fCrossValue > 0)
           {
               /*起点向量和终点向量为逆时针*/
               if(((NextNode->nDirection == BACKWARD) && (NextNode->fSteering > fError))
                       ||((NextNode->nDirection == FORWARD) && (NextNode->fSteering < -fError)))
               {
                   bIllogicalSteering = true;
               }
               else
               {
                   bIllogicalSteering = false;
               }
           }
           else
           {
               /*起点向量和终点向量为顺时针*/
               if(((NextNode->nDirection == BACKWARD) && (NextNode->fSteering < -fError))
                       ||((NextNode->nDirection == FORWARD) && (NextNode->fSteering > fError)))
               {
                   bIllogicalSteering = true;
               }
               else
               {
                   bIllogicalSteering = false;
               }
           }
       }
   }else{}
   return bIllogicalSteering;
}

/**
* @brief 计算转向惩罚
* @param 下一个节点
* @return 惩罚值
*/
float HybridAStar::CalTrajSteeringPenalty(Node3D *NextNode)
{
   int nDirection = 0;
   float fTrajSteeringPenalty = 0.0;
   float fTrajSteeringdiffer = 0.0;
   float fSteering = 0;
   float  fMaxSteeringAngle = Constants::planning_steering_angle;
   float crossValue = 0.0;
   float fStartYaw3d = m_slotPosInfor.startPointInMap.yaw;
   float fGoalYaw3d = m_slotPosInfor.goalPointInMap.yaw;
   Vector2D StartPoint(0, 0);
   Vector2D GoalPoint(0, 0);

   if(NextNode != nullptr)
   {
       fSteering = NextNode->fSteering;
       nDirection = NextNode->nDirection;

       /*设置起点和终点的单位向量*/
       StartPoint.setXY(cos(fStartYaw3d), sin(fStartYaw3d));
       GoalPoint.setXY(cos(fGoalYaw3d), sin(fGoalYaw3d));

       if(fabsf(normalizeHeadingRad_Npi_Ppi(fGoalYaw3d - fStartYaw3d)) < 10 * PI / 180)
       {
           fTrajSteeringdiffer = fabsf(NextNode->fSteering);
       }
       else
       {
           /*用叉乘表达起点向量和终点向量是顺时针还是逆时针*/
           crossValue = StartPoint.cross(GoalPoint);
           if(crossValue > 0)
           {
               /*起点向量和终点向量为逆时针*/
               fTrajSteeringdiffer = fabsf(fSteering - nDirection * fMaxSteeringAngle);
           }
           else
           {
               /*起点向量和终点向量为顺时针*/
               fTrajSteeringdiffer = fabsf(fSteering + nDirection * fMaxSteeringAngle);
           }
       }

       fTrajSteeringPenalty = fTrajSteeringdiffer * Constants::traj_steer_penalty_obli;
   }else{}

   return fTrajSteeringPenalty;
}

float HybridAStar::CalObstacleDisCost(float fDis)
{
   float fObstacleDisCost = 0.0;
   float fMaxObstacleDis = 0.5;

   fObstacleDisCost = MAX(0, 1 / ( fMaxObstacleDis) * (fMaxObstacleDis - fDis));
   return fObstacleDisCost;
}
/**
* @brief 总代价：行驶 + 转向 + 转向偏离 + 换挡 + 不合理转向惩罚
* @param CurrentNode 当前节点
* @param NextNode 下一个节点
* @return 总代价
*/
float HybridAStar::TrajCost(Node3D *CurrentNode, Node3D *NextNode)
{
   float fPiecewiseCost = 0.0;
   float fStepSize = Constants::TravelDis;

   if((CurrentNode != nullptr) && (NextNode != nullptr))
   {
       /*1 行驶惩罚*/
       if(NextNode->nDirection == FORWARD)
       {
           /*前向行驶惩罚*/
           fPiecewiseCost += fStepSize*Constants::traj_forward_penalty;
       }
       else
       {
           /*后退行驶惩罚*/
           fPiecewiseCost += fStepSize*Constants::traj_back_penalty;
       }

       /*2 换挡惩罚*/
       if(CurrentNode->isStart)
       {
           /*当前点是起点时，起点的行驶方向和下一个节点的行驶方向不同,不需要加换挡惩罚*/
           if(NextNode->nDirection == CurrentNode->nDirection)
           {
                if(m_slotPosInfor.isAvoidance)
                {
                    fPiecewiseCost += 1e4;
                }
                else
                {
                    fPiecewiseCost += Constants::traj_same_direc_penalty;
                }
           }
           else{}
       }
       else
       {
           /*当前点不是起点时，当前点的行驶方向和下一个节点的行驶方向不同,需要加换挡惩罚*/
           if(NextNode->nDirection != CurrentNode->nDirection)
           {
               fPiecewiseCost += Constants::traj_gear_switch_penalty;
           }else{}
       }

       /*3 转向惩罚*/
#if 0
       if(m_slotPosInfor.nSlotType == OBLIQUE)
       {
           fPiecewiseCost += CalTrajSteeringPenalty(NextNode);
       }
       else
       {
           fPiecewiseCost += fabsf(NextNode->fSteering) * Constants::traj_steer_penalty_para;
       }
#endif
       /*4 转向改变惩罚*/
       if(!CurrentNode->isStart && (CurrentNode->nDirection == NextNode->nDirection))
       {
           fPiecewiseCost += Constants::traj_steer_change_penalty * fabsf(NextNode->fSteering - CurrentNode->fSteering);
       }
       /*5 不合理转向改变惩罚*/
       if(IllogicalSteeringCheck(NextNode))
       {
           fPiecewiseCost += Constants::traj_illogical_steer_penalty * fabsf(NextNode->fSteering);
       }
   }else{}

   return fPiecewiseCost;
}

/**
 * @brief RS曲线碰撞检测
 * @param configurationSpace 碰撞检测配置空间
 * @param points 输入点
 * @return 1-可行驶; 0-不可行驶
 */
bool HybridAStar:: ReedsSheepsCollisionCheck(CollisionDetection &configurationSpace,
                                             const std::vector<Node3D> &points)
{
    size_t nNum = points.size();
    size_t nIndex = 0;
    int nLastDir = 0;
    int nCurDir = 0;
    bool bIsTraversable = false;
    bool bHasRsChangeDirPoint = false;
    Node3D CheckPoint;
    Node3D ChangeDirPoint;

    if(!points.empty() && nNum > 1)
    {
        nLastDir = points.front().nDirection;
    }
    else
    {
        return false;
    }
    for(size_t i = 1; i < points.size(); i++)
    {
        nCurDir = points[i].nDirection;
        if(nCurDir != nLastDir)
        {
            ChangeDirPoint.x = points[i - 1].x;
            ChangeDirPoint.y = points[i - 1].y;
            ChangeDirPoint.t = normalizeHeadingRad_0_2pi(points[i - 1].t);
            bHasRsChangeDirPoint = true;
            break;
        }
    }

    if ((bHasRsChangeDirPoint && configurationSpace.isTraversable(&ChangeDirPoint)) || (!bHasRsChangeDirPoint))
    {
        while (nIndex <  nNum)
        {
            CheckPoint.isStart = points[nIndex].isStart;
            CheckPoint.x = points[nIndex].x;
            CheckPoint.y = points[nIndex].y;
            CheckPoint.t = normalizeHeadingRad_0_2pi(points[nIndex].t);
            CheckPoint.nDirection = points[nIndex].nDirection;
            CheckPoint.fSteering = points[nIndex].fSteering;
            CheckPoint.isRs = true;
            CheckPoint.setIdx(m_varialeCellBound);

            m_nCollisonCheckTimes++;
            if (configurationSpace.isTraversable(&CheckPoint))
            {
                bIsTraversable = true;
                ++nIndex;
            }
            else
            {
                bIsTraversable = false;
                break;
            }
        }
    }
    else
    {
        bIsTraversable = false;
    }

    return bIsTraversable;
}

/**
 * @brief 计算车头泊入需要的转弯半径
 * @param tR 后角点
 * @param tF 前角点
 * @param slotCenterLineEq 中线方程
 * @param fSafeDis 安全距离
 * @return fHeadInR 转弯半径
 */
float HybridAStar::CalHeadInParkingRadius(const cpoint &tR,
                                          const cpoint &tF,
                                          const LineEq &slotCenterLineEq,
                                          float fSafeDis)
{
    float R, Rg, theta, thetaG, deltaX, L = 0.0;
    float fMaxDeltaX = 0.25;
    float fHeadInR = Constants::MaxROfHeadIn;
    float fWidth = 0.0f;
    bool bFind = false;

    /*取两侧角点对中线的较小距离作为车位综合可用宽度*/
    fWidth = 2 * MIN(CalPointAndLineDis(tF, slotCenterLineEq), CalPointAndLineDis(tR, slotCenterLineEq));

    fMaxDeltaX = (fWidth - Constants::width) / 2 - fSafeDis;
    fMaxDeltaX = MAX(fMaxDeltaX, 0.15f);
    for(int i = 0; i < 100; ++i)
    {
        R = Constants::r + i * 0.5;
        theta = acos(1 - 0.2f / 2 / R);
        Rg = sqrt(pow((R + Constants::width / 2), 2) + pow(Constants::FrontEdgeToCenter, 2));
        thetaG = acos(Constants::FrontEdgeToCenter / Rg);
        deltaX = Rg * (sin(thetaG + theta) - sin(thetaG));
        L = 2 * R * sin(theta);
        if(deltaX < fMaxDeltaX)
        {
            bFind = true;
            break;
        }
    }
    if(bFind)
    {
        fHeadInR = R;
    }
    fHeadInR = MIN(fHeadInR, Constants::MaxROfHeadIn);
    GAC_LOG_INFO("fHeadInR is %f, deltaX is %f, fMaxDeltaX is %f, steering angle is %d",
           fHeadInR, deltaX, fMaxDeltaX, CalRadius2SteeringWheelAngle(1,1,fHeadInR));

    return fHeadInR;
}
/**
 * @brief 变半径RS曲线
 * @param ShottingPoint 起点
 * @param goal 终点
 * @param configurationSpace 碰撞检测配置空间
 * @param slotPosInfor_ 车位
 * @param bReverse 1-可换向 0-不可换向
 * @param nVarRadiusNum 可变半径单方向扩展数目
 * @param nVarRadiusDir 可变半径方向数目
 * @param points 输出点
 * @return 1-可行驶; 0-不可行驶
 */
bool HybridAStar::ConnectByVariableRSRadius(const Node3D& ShottingPoint,
                                            const Node3D& goal,
                                            CollisionDetection& configurationSpace,
                                            slotPosInfor &slotPosInfor_,
                                            bool bReverse,
                                            int nVarRadiusNum,
                                            int nVarRadiusDir,
                                            PARTITION_PATH &TotalSegs)
{
    float fVariableRadius = 0;
    float fOriginR = Constants::r;
    float fAmp = 0.0;
    bool bGenerRSValid = false;
    bool bRSShotValid = false;
    bool bShotValid = false;
    std::vector<Node3D> points;
    LocationPoint q0(ShottingPoint.x, ShottingPoint.y, ShottingPoint.t);
    LocationPoint q1(goal.x, goal.y, goal.t);
    ReedsSheppPath path;
    points.clear();

    if(slotPosInfor_.nSlotType == OBLIQUE && slotPosInfor_.nParkingType == HEAD_PARK_IN)
    {
        fOriginR = m_fHeadInRsRadius;
    }
    for(int j = 0; j < nVarRadiusNum; j++)
    {
        fAmp = j * Constants::VarRadiusStepSize;
        for(int i = 0; i < nVarRadiusDir; i++)
        {
            if(j == 0 && i == 1)
            {
                continue;
            }
            if(i == 0)
            {
                fVariableRadius = fOriginR + fAmp;
            }
            else
            {
                fVariableRadius = MAX(fOriginR - fAmp, Constants::min_steering_radius);
            }

            points.clear();
            bGenerRSValid = Generate_ReedsShepp_path(slotPosInfor_.nSlotType,
                                                     bReverse,
                                                     q0,
                                                     q1,
                                                     fVariableRadius,
                                                     Constants::TravelDis,
                                                     points,
                                                     path);

            if(ShottingPoint.isStart && (!points.empty()))
            {
                points.front().isStart = true;
            }
            bRSShotValid = bGenerRSValid && ReedsSheepsCollisionCheck(configurationSpace, points);

            if(bRSShotValid)
            {
                /*根据Rs曲线连接结果追溯整条hybrid A*轨迹并进行分段*/
                TracePathByRSAndShotPoint(ShottingPoint, points, TotalSegs);

                if(JudgeValidityOfHyBridAStarPath(slotPosInfor_, TotalSegs, fVariableRadius))
                {
                    bShotValid = true;
                    break;
                }
            }
        }
        if(bShotValid)
        {
            break;
        }
    }
//    if(bShotValid)
//    {
//        if((path.type_[0] == RS_STRAIGHT) && (path.type_[2] == RS_STRAIGHT))
//        {

//        /*ReedsShepp曲线成功连接目标位置，打印出曲线信息*/
//#ifdef DEBUG_PRINT
//        GAC_LOG_DEBUG("曲线的总长是%f\n", path.totalLength_ * path.rho_);
//        ReedsSheppPathSegmentType PathSegment[5];
//        for(int i = 0;i < 5; i++)
//        {
//            PathSegment[i] = path.type_[i];
//            switch (PathSegment[i])
//            {
//            case 0: GAC_LOG_DEBUG("RS_NOP\n"); break;
//            case 1: GAC_LOG_DEBUG("RS_LEFT: %fm \n ",path.length_[i] * path.rho_); break;
//            case 2: GAC_LOG_DEBUG("RS_STRAIGHT:%fm \n",path.length_[i] * path.rho_); break;
//            case 3: GAC_LOG_DEBUG("RS_RIGHT:%fm \n",path.length_[i] * path.rho_); break;
//            default:GAC_LOG_DEBUG("NONE\n");
//            }
//        }
//        if (m_ParkingCtrlStatus == RunningFreeStauts)
//        {
//            GAC_LOG_DEBUG("曲线的radius是%f\n", path.rho_);
//            GAC_LOG_DEBUG("停车中点x坐标%f, 车位长度是%f\n", slotPosInfor_.fSlotLength/2, slotPosInfor_.fSlotLength);
//        }else{GAC_LOG_DEBUG("曲线的radius是%f\n", path.rho_);}
//#endif
//        }
//    }

    return bShotValid;
}

/**
  * @brief RS曲线连接的有效性和是否可行驶
  * @param ShottingPoint 起点
  * @param goal 终点
  * @param configurationSpace 碰撞检测配置空间
  * @return 1-连接成功; 0-连接失败
  */
bool HybridAStar::ReedsSheepsShot(const Node3D& ShottingPoint,
                                  const Node3D& goal,
                                  CollisionDetection& configurationSpace,
                                  vector<PARTITION_PATH> &reservedPath)
 {
     bool bReverse = true;
     bool bShotValid = false;
     bool bFinishShooting = false;
     int fVarRadiusNum = 1;
     int fVarRadiusDir = 1;
     PARTITION_PATH TotalSegs;

     if(m_ParkingCtrlStatus == RunningFreeStauts)
     {
         /*垂直最后一段入库运动实时规划，采用变半径RS曲线,提高连接成功可能性*/
         configurationSpace.UpdateCurrentVehiclePositionOccupyMap(ShottingPoint.x, ShottingPoint.y, ShottingPoint.t, Constants::startVehBloating);
         bReverse = false;
         fVarRadiusNum = Constants::VarRadiusNum;
         fVarRadiusDir = Constants::VarRadiusDir;
     }
     else
     {
         bReverse = true;
         if(ShottingPoint.isStart)
         {
             /*当前点是起点时，采用变半径RS曲线,提高连接成功可能性*/
             fVarRadiusNum = Constants::VarRadiusNum;
             fVarRadiusDir = Constants::VarRadiusDir;
         }
         else
         {
             /*当前点是搜索中间点时，采用定半径RS曲线,降低运算*/
             fVarRadiusNum = 1;
             fVarRadiusDir = 1;
         }
     }

     bShotValid = ConnectByVariableRSRadius(ShottingPoint,
                                            goal,
                                            configurationSpace,
                                            m_slotPosInfor,
                                            bReverse,
                                            fVarRadiusNum,
                                            fVarRadiusDir,
                                            TotalSegs);
     if(bShotValid)
     {
         m_bFindPath = true;
         reservedPath.push_back(TotalSegs);
     }else{}

     if(m_bFindPath)
     {
         ++m_nAdditionalSearchTimes;
     }else{}

     if(m_nAdditionalSearchTimes > Constants::MaxAdditionalSearchTimes)
     {
         bFinishShooting = true;
     }
     else
     {
         bFinishShooting = false;
     }
     return bFinishShooting;
 }

 /**
  * @brief 判断HyBridASta曲线是否满足要求
  * @param configurationSpace 碰撞检测配置空间
  * @param slotPosInfor_ 车位信息
  * @param TotalSegs  追溯的整条分段轨迹
  * @return bShotValid 1-连接成功; 0-连接失败
  */
 bool HybridAStar::JudgeValidityOfHyBridAStarPath(const slotPosInfor &slotPosInfor_,
                                                  const PARTITION_PATH &TotalSegs,
                                                  float R)
 {
     bool bValidity = false;

     if(TotalSegs.empty())
     {
         return false;
     }

     for(size_t i = 0; i < TotalSegs.size(); ++i)
     {
         if((TotalSegs[i].size() == 0 )|| (TotalSegs[i].size() == 1))
         {
             return false;
         }
     }
     if(slotPosInfor_.nSlotType == PARALLEL)
     {
         bValidity = true;
     }
     else
     {
         bValidity = JudgeObliqueSlotValidityOfHyBridAStarPath(slotPosInfor_,
                                                               TotalSegs,
                                                               R);
     }
     return bValidity;
 }

 /**
  * @brief 判断垂直车位HyBridASta曲线是否满足要求
  * @param configurationSpace 碰撞检测配置空间
  * @param slotPosInfor_ 车位信息
  * @param TotalSegs  追溯的整条分段轨迹
  * @return 1-连接成功; 0-连接失败
  */
 bool HybridAStar::JudgeObliqueSlotValidityOfHyBridAStarPath(const slotPosInfor &slotPosInfor_,
                                                             const PARTITION_PATH &TotalSegs,
                                                             float R)
  {
      bool bShotValid = true;

//      if(ShootingFailForUnexpectedLineTraj(TotalSegs))
//      {
//          /*1 由于轨迹中存在直行的小段轨迹被截断*/
//          bShotValid = false;
//      }
//      else
      if(ShootingFailForUnexpectedRSradius(TotalSegs, R))
      {
          /*2 由于不合理的连接转弯半径截断*/
          bShotValid = false;
      }
//      else if(ShootingFailForUnexpectedShortLengthPath(ShottingPoint, points))
//      {
//          /*3 因为rs曲线上一段轨迹为短轨迹，截断连接*/
//          bShotValid = false;
//      }
//      else if(ShootingFailForUnexpectedSmallTrajChangedAngle(TotalSegs))
//      {
//          /*4 因为最后三段曲线车辆偏航角改变太少，截断连接*/
//          bShotValid = false;
//      }
//      else if(ShootingFailForBadSafetyofRearMirror(slotPosInfor_, TotalSegs))
//      {
//          /*5 因为后视镜入库时位置不安全，截断连接*/
//          bShotValid = false;
//      }else{}

      return bShotValid;
  }

 /**
  * @brief 由于轨迹中存在直行的小段轨迹被截断
  * @param TotalSegs 追溯的整条分段轨迹
  * @return 0-连接成功; 1-连接失败
  */
 bool HybridAStar::ShootingFailForUnexpectedLineTraj(const PARTITION_PATH &TotalSegs)
 {
     bool bShotFail = false;

     for(size_t i = 0; i < TotalSegs.size(); ++i)
     {
         if((TotalSegs[i].size() == 2) && (fabs(TotalSegs[i].back().fSteering) < 1e-3f))
         {
             GAC_LOG_INFO("由于轨迹中存在直行的小段轨迹被截断！！！");
             bShotFail = true;
             break;
         }
         else
         {
             bShotFail = false;
         }
     }
     return bShotFail;

 }

 /**
  * @brief 由于不合理的连接转弯半径截断
  * @param TotalSegs 追溯的整条分段轨迹
  * @param R Rs连接转弯半径
  * @return 0-连接成功; 1-连接失败
  */
 bool HybridAStar::ShootingFailForUnexpectedRSradius(const PARTITION_PATH &TotalSegs, float R)
 {
     bool bShotFail = false;
     float fMinR = Constants::RNotInLastCurve;

     if(TotalSegs.empty())
     {
         return true;
     }
     if(TotalSegs.size() == 1)
     {
         /*是最后一段，Rs连接转弯半径不留余量*/
         bShotFail = false;
     }
     else
     {
         /*不是最后一段，Rs连接转弯半径留余量*/
         if((R - fMinR < 1e-3f))
         {
             bShotFail = true;
             GAC_LOG_INFO("轨迹不是最后一段，连接转弯半径为%f m,截断！！！", R);
         }
         else
         {
             bShotFail = false;
         }
     }
     return bShotFail;

 }
 /**
  * @brief 因为rs曲线上一段轨迹为短轨迹，截断连接
  * @param start RS起点
  * @param points  reedsSheep曲线离散点
  * @return 1-连接失败; 0-连接成功
  */
 bool HybridAStar::ShootingFailForUnexpectedShortLengthPath(const Node3D &start,
                                                            const std::vector<Node3D> &points)
 {
     bool bShotFail = false;

     if(points.empty())
     {
         return true;
     }

     if((start.nDirection != points.front().nDirection) && (start.nSameDirNodeNum == 1))
     {
         bShotFail = true;
     }

     return bShotFail;
 }

 /**
  * @brief 因为最后三段曲线车辆偏航角改变太少，截断连接
  * @param TotalSegs 追溯的整条分段轨迹
  * @return 1-连接失败; 0-连接成功
  */
 bool HybridAStar::ShootingFailForUnexpectedSmallTrajChangedAngle(const PARTITION_PATH &TotalSegs)
 {
     bool bShotFail = false;
     float fSearchStartYaw =  0.0;
     float fLastCurveStartYaw = 0.0;
     float fYawChange = 0.0f;

     if(TotalSegs.empty())
     {
         return true;
     }

     if(TotalSegs.size() == 3)
     {
         fSearchStartYaw = TotalSegs.front().front().t;
         fLastCurveStartYaw = TotalSegs.back().front().t;
         fYawChange = fabsf(normalizeHeadingRad_Npi_Ppi(fLastCurveStartYaw - fSearchStartYaw));
         if (fYawChange < 10 * PI / 180)
         {
             bShotFail = true;
             GAC_LOG_INFO("kill Last Three Curve with small yaw change!!!\n");
         }
     }
     return bShotFail;
 }

 /**
  * @brief 因为后视镜入库时位置不安全，截断连接
  * @param slotPosInfor_ 车位信息
  * @param TotalSegs 追溯的整条分段轨迹
  * @return 1-连接失败,0-连接成功
  */
  bool HybridAStar::ShootingFailForBadSafetyofRearMirror(const slotPosInfor &slotPosInfor_,
                                                         const PARTITION_PATH &TotalSegs)
  {
      bool bShotFail = false;
      bool bLastCurve = false;
      bool bValidPath = false;
      size_t nIndex = 0;
      int nSteeringTimes = 1;
      float fLastSteering = 0.0;
      vector<Node3D> last_Curve;
      LocationPoint tVehPos;
      LocationPoint KeyPoint[2];

      if(TotalSegs.empty())
      {
          return true;
      }
      last_Curve = TotalSegs.back();
      if(TotalSegs.size() == 1)
      {
          bLastCurve = true;
      }

      if(((last_Curve.front().nDirection == FORWARD) && (slotPosInfor_.nParkingType == TAIL_PARK_IN))
              || ((last_Curve.front().nDirection == BACKWARD) && (slotPosInfor_.nParkingType == HEAD_PARK_IN)))
      {
          bValidPath = false;
      }
      else
      {
          bValidPath = true;
      }

      if(!bValidPath)
      {
          return true;
      }

#if 0
      LocationPoint SafePoint = slotPosInfor_.safePointParkingIn;
      if(m_bBestSafeObliParkingInPath)
      {
          if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
          {
              if(!last_Curve.empty())
              {
                  /**找到倒数第一个转向的点**/
                  for(size_t i = last_Curve.size() ; i > 0 ; --i)
                  {
                      if (fabsf(last_Curve[i - 1].fSteering) > CAL_ERROR)
                      {
                          nIndex = i - 1;
                          break;
                      }
                  }

                  /**如果是最后一段降低阈值**/
                  if(bLastCurve)
                  {
                      if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
                      {
                          SafePoint.x = slotPosInfor_.safePointParkingIn.x - LAST_CURVE_SAFE_POINT_ERROR * cos(slotPosInfor_.goalPointInSlot.yaw);
                          SafePoint.y = slotPosInfor_.safePointParkingIn.y - LAST_CURVE_SAFE_POINT_ERROR * sin(slotPosInfor_.goalPointInSlot.yaw);
                      }
                      else if(slotPosInfor_.nParkingType == HEAD_PARK_IN)
                      {
                          SafePoint.x = slotPosInfor_.safePointParkingIn.x + LAST_CURVE_SAFE_POINT_ERROR * cos(slotPosInfor_.goalPointInSlot.yaw);
                          SafePoint.y = slotPosInfor_.safePointParkingIn.y + LAST_CURVE_SAFE_POINT_ERROR * sin(slotPosInfor_.goalPointInSlot.yaw);
                      }
                  }

                  if ((last_Curve[nIndex].y  - SafePoint.y) * slotPosInfor_.nSlotSide < 0)
                  {
                      bShotFail = true;
                      GAC_LOG_INFO("方向盘回正点延迟 %fm\n", fabsf(last_Curve[nIndex].y  - slotPosInfor_.safePointParkingIn.y));
                  }
                  else
                  {
                      GAC_LOG_INFO("方向盘回正点提前 %fm\n", fabsf(last_Curve[nIndex].y  - slotPosInfor_.safePointParkingIn.y));
                  }
              }else{}
          }else{}
      }
      else
#endif

          if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
          {
              if(!JudgeIfNeedCheckBadSafetyofRearMirror(slotPosInfor_, TotalSegs))
              {
                  return false;
              }
              for(size_t i = last_Curve.size(); i > 0; i--)
              {
                  tVehPos.x = last_Curve[i - 1].x;
                  tVehPos.y = last_Curve[i - 1].y;
                  tVehPos.yaw = last_Curve[i - 1].t;
                  CalMirrorPointOfVehicle(tVehPos, KeyPoint);

                  if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
                  {
                      if(MIN(KeyPoint[0].y, KeyPoint[1].y) - slotPosInfor_.fYSlotPosInMap > 0.3f)
                      {
                          nIndex = i - 1;
                          break;
                      }
                  }
                  else
                  {
                      if(MAX(KeyPoint[0].y, KeyPoint[1].y ) - slotPosInfor_.fYSlotPosInMap < -0.3f)
                      {
                          nIndex = i - 1;
                          break;
                      }
                  }
              }

              /**如果是最后一段降低阈值,起始点往后沿两个点**/
              if(bLastCurve)
              {
                  nIndex = MIN(nIndex + 2, last_Curve.size() - 1);
              }
              /*计算从该点到最后点轨迹的扭动次数*/
              fLastSteering = last_Curve[nIndex].fSteering;
              if(fabsf(fLastSteering) < 1e-3f)
              {
                  nSteeringTimes = 0;
              }
              else
              {
                  nSteeringTimes = 1;
              }
              for(size_t i = nIndex; i < last_Curve.size() ; ++i)
              {
                  if(fabsf(last_Curve[i].fSteering - fLastSteering) > 1e-3f)
                  {
                      if(fabsf(last_Curve[i].fSteering) > 1e-3f)
                      {
                          ++nSteeringTimes;
                      }
                      fLastSteering = last_Curve[i].fSteering;
                  }
              }
              if(nSteeringTimes == 2)
              {
                  bShotFail = true;
                  GAC_LOG_INFO("扭动截断\n");
              }else{}
          }else{}


      return bShotFail;
  }

  /**
   * @brief 判断是否需要进去后视镜安全检测逻辑
   * @param slotPosInfor_ 车位信息
   * @param TotalSegs 追溯的整条分段轨迹
   * @return 1-需要 0-不需要
   */
  bool HybridAStar::JudgeIfNeedCheckBadSafetyofRearMirror(const slotPosInfor &slotPosInfor_,
                                                          const PARTITION_PATH &TotalSegs)
  {
      bool bNeedCheck = false;
      bool bValidPath = false;
      bool bBreak = false;
      float fDis = 0.0f;
      float fSafeDis = 0.2f;
      vector<Node3D> last_Curve;
      LocationPoint tVehPos;
      LocationPoint KeyPoint[2];
      cpoint tF, tR;

      if(TotalSegs.empty())
      {
          return true;
      }
      tF.x = slotPosInfor_.fXFrontSlotCorner + slotPosInfor_.fXSlotPosInMap;
      tF.y = slotPosInfor_.fYFrontSlotCorner + slotPosInfor_.fYSlotPosInMap;
      tR.x = slotPosInfor_.fXRearSlotCorner + slotPosInfor_.fXSlotPosInMap;
      tR.y = slotPosInfor_.fYRearSlotCorner + slotPosInfor_.fYSlotPosInMap;

      last_Curve = TotalSegs.back();

      if(((last_Curve.front().nDirection == FORWARD) && (slotPosInfor_.nParkingType == TAIL_PARK_IN))
              || ((last_Curve.front().nDirection == BACKWARD) && (slotPosInfor_.nParkingType == HEAD_PARK_IN)))
      {
          bValidPath = false;
      }
      else
      {
          bValidPath = true;
      }

      if(!bValidPath)
      {
          return true;
      }
      for(size_t index = 0; index < last_Curve.size(); ++index)
      {
          tVehPos.x = last_Curve[index].x;
          tVehPos.y = last_Curve[index].y;
          tVehPos.yaw = last_Curve[index].t;
          CalMirrorPointOfVehicle(tVehPos, KeyPoint);

          /*只要有一个后视镜位置点和车位角点距离安全距离以内,那么必须进行后视镜安全策略判断*/
          for(int i = 0; i < 2; ++i)
          {
              fDis = hypotf(tR.x - KeyPoint[i].x, tR.y - KeyPoint[i].y);
              if(fDis < fSafeDis)
              {
                  bNeedCheck = true;
                  bBreak = true;
                  break;
              }else{}

              fDis = hypotf(tF.x - KeyPoint[i].x, tF.y - KeyPoint[i].y);
              if(fDis < fSafeDis)
              {
                  bNeedCheck = true;
                  bBreak = true;
                  break;
              }else{}
          }
          if(bBreak)
          {
              break;
          }
      }

      return bNeedCheck;
  }
  /**
   * @brief 根据Rs曲线连接结果追溯整条hybrid A*轨迹并进行分段
   * @param ShottingPoint RS起点
   * @param points reedsSheep曲线离散点
   * @param TotalSegs 追溯的整条分段轨迹
   */
  void HybridAStar::TracePathByRSAndShotPoint(const Node3D &ShottingPoint,
                                              const std::vector<Node3D> &points,
                                              PARTITION_PATH &TotalSegs)
  {
      int nTimes = 0;
      int8_t nD1 = points.front().nDirection;
      int8_t nlastDir = nD1;
      Node3D node = ShottingPoint;
      Node3D ChangeDirPoint;

      vector<Node3D> PathSeg;

      PathSeg.clear();
      TotalSegs.clear();
      /*RS曲线的曲线离散点分段存入*/
      for(size_t i = 0; i < points.size(); ++i)
      {
          if(points[i].nDirection == nlastDir)
          {
              PathSeg.push_back(points[i]);
          }
          else
          {
              TotalSegs.push_back(PathSeg);

              ChangeDirPoint = PathSeg.back();
              ChangeDirPoint.nDirection = points[i].nDirection;
              ChangeDirPoint.fSteering = points[i].fSteering;
              ChangeDirPoint.isStart = false;
              PathSeg.clear();
              PathSeg.push_back(ChangeDirPoint);
              PathSeg.push_back(points[i]);
              nlastDir = points[i].nDirection;
          }
      }

      TotalSegs.push_back(PathSeg);

      /*如果连接点不是起点，追溯前面所有扩展节点，分段存入*/
      if(!ShottingPoint.isStart)
      {
          PathSeg = TotalSegs.front();
          nlastDir = PathSeg.front().nDirection;
          TotalSegs.erase(TotalSegs.begin());
          while(nTimes < Constants::nMaxNodeNum)
          {
              if(node.nDirection == nlastDir)
              {
                  if(hypot(node.x - PathSeg.front().x, node.y - PathSeg.front().y) > 1e-3f)
                  {
                      /*考虑扩展节点和RS曲线起点重合*/
                      PathSeg.insert(PathSeg.begin(), node);
                  }
              }
              else
              {
                  /*增加当前段轨迹换向点*/
                  ChangeDirPoint = node;
                  ChangeDirPoint.nDirection = PathSeg.front().nDirection;
                  ChangeDirPoint.fSteering = PathSeg.front().fSteering;
                  ChangeDirPoint.isStart = false;
                  if(hypot(ChangeDirPoint.x - PathSeg.front().x, ChangeDirPoint.y - PathSeg.front().y) > 1e-3f)
                  {
                      PathSeg.insert(PathSeg.begin(), ChangeDirPoint);
                  }
                  TotalSegs.insert(TotalSegs.begin(),PathSeg);

                  /*换向点存入下一段轨迹*/
                  PathSeg.clear();
                  PathSeg.insert(PathSeg.begin(),node);
                  nlastDir = node.nDirection;
              }

              if(node.getPred() != nullptr)
              {
                  node = *node.getPred();
                  if(node.isStart)
                  {
                      node.nDirection = nlastDir;
                  }
              }
              else
              {
                  TotalSegs.insert(TotalSegs.begin(),PathSeg);
                  break;
              }
              nTimes++;
          }
      }
  }


void HybridAStar::TraceSearchPath(const Node3D& nNode, vector<PARTITION_PATH> &reservedPath)
{
    int nTimes = 0;
    int8_t nD1 = nNode.nDirection;
    int8_t nlastDir = nD1;
    Node3D node = nNode;
    Node3D ChangeDirPoint;

    vector<Node3D> PathSeg;
    PARTITION_PATH TotalSegs;

    PathSeg.clear();
    TotalSegs.clear();
    reservedPath.clear();

    PathSeg.push_back(nNode);
    while(nTimes < Constants::nMaxNodeNum)
    {
        if(node.nDirection == nlastDir)
        {
            if(hypot(node.x - PathSeg.front().x, node.y - PathSeg.front().y) > 1e-3f)
            {
                /*考虑扩展节点和RS曲线起点重合*/
                PathSeg.insert(PathSeg.begin(), node);
            }
        }
        else
        {
            /*增加当前段轨迹换向点*/
            ChangeDirPoint = node;
            ChangeDirPoint.nDirection = PathSeg.front().nDirection;
            ChangeDirPoint.fSteering = PathSeg.front().fSteering;
            ChangeDirPoint.isStart = false;
            if(hypot(ChangeDirPoint.x - PathSeg.front().x, ChangeDirPoint.y - PathSeg.front().y) > 1e-3f)
            {
                PathSeg.insert(PathSeg.begin(), ChangeDirPoint);
            }
            TotalSegs.insert(TotalSegs.begin(),PathSeg);

            /*换向点存入下一段轨迹*/
            PathSeg.clear();
            PathSeg.insert(PathSeg.begin(),node);
            nlastDir = node.nDirection;
        }

        if(node.getPred() != nullptr)
        {
            node = *node.getPred();
            if(node.isStart)
            {
                node.nDirection = nlastDir;
            }
        }
        else
        {
            TotalSegs.insert(TotalSegs.begin(),PathSeg);
            break;
        }
        nTimes++;
    }

    reservedPath.push_back(TotalSegs);
}
/**
 * @brief 使含有RS曲线片段的路径点间隔均匀,计算补全RS路径最后一个均匀点多产生的点数
 * @param path 原始路径
 * @param pathInterpolated 均匀路径
 * @return nUniform 补全RS路径最后一个均匀点多产生的点数
 */
void HybridAStar::MakingPathUniform(const std::vector<Node3D> &path,
                                    std::vector<Node3D>& pathInterpolated,
                                   int &nStartNum,
                                   int &nEndNum)
{
    size_t num = path.size();
    int8_t nDirection = 0;
    float fTraveledDistance = Constants::TravelDis;
    float fLastX = 0.0;
    float fLastY = 0.0;
    float fLastPhi = 0.0;
    float fNextX = 0.0;
    float fNextY = 0.0;
    float fNextPhi = 0.0;
    float fSteering=0.0;
    float fCurvature = 0.0;
    float fFirstTwoPointDis = 0.0;
    float fLastTwoPointDis = 0.0;
    std::vector<Node3D> pathBeforeInterpolated;
    Node3D FinalPoint;

    nEndNum = 0;
    nStartNum = 0;
    pathInterpolated.clear();
    pathBeforeInterpolated.clear();
    if(num < 3)
    {
        ;
    }
    else
    {
        fLastTwoPointDis = hypotf(path[num - 1].x - path[num - 2].x, path[num - 1].y - path[num - 2].y);
         fFirstTwoPointDis= hypotf(path[1].x - path[0].x, path[1].y - path[0].y);
        /*判断该段路径是不是均匀的搜索路径*/
         pathInterpolated = path;
         if(fFirstTwoPointDis < fTraveledDistance - 0.02f)
         {
             pathInterpolated.erase(pathInterpolated.begin());
             nDirection = path[1].nDirection;
             fSteering = path[1].fSteering;
             fTraveledDistance = -Constants::TravelDis * nDirection;
             fCurvature = tan(fSteering) / Constants::WheelBase; /*包含正负*/
             fLastX = path[1].x;
             fLastY = path[1].y;
             fLastPhi = path[1].t;/*此处使用最后一个点的偏航角*/

             fNextPhi = normalizeHeadingRad_0_2pi(fLastPhi + fTraveledDistance * fCurvature);
             fNextX = fLastX + fTraveledDistance * cos(fLastPhi + fTraveledDistance * fCurvature / 2);
             fNextY = fLastY + fTraveledDistance * sin(fLastPhi + fTraveledDistance * fCurvature / 2);

             FinalPoint.x = fNextX;
             FinalPoint.y = fNextY;
             FinalPoint.t = fNextPhi;
             FinalPoint.nDirection = nDirection;
             FinalPoint.fSteering = fSteering;
             pathInterpolated.insert(pathInterpolated.begin(), FinalPoint);
             nStartNum = int(hypot(FinalPoint.x - path.front().x, FinalPoint.y - path.front().y) / Constants::step_size);
         }

         if(fLastTwoPointDis  < fTraveledDistance - 0.02f)
         {
             pathInterpolated.pop_back();
             nDirection = path.back().nDirection;
             fSteering = path.back().fSteering;
             fTraveledDistance = Constants::TravelDis * nDirection;
             fCurvature = tan(fSteering) / Constants::WheelBase; /*包含正负*/
             fLastX = path[num - 2].x;
             fLastY = path[num - 2].y;
             fLastPhi = path[num - 2].t;/*此处使用最后一个点的偏航角*/

             fNextPhi = normalizeHeadingRad_0_2pi(fLastPhi + fTraveledDistance * fCurvature);
             fNextX = fLastX + fTraveledDistance * cos(fLastPhi + fTraveledDistance * fCurvature / 2);
             fNextY = fLastY + fTraveledDistance * sin(fLastPhi + fTraveledDistance * fCurvature / 2);

             FinalPoint.x = fNextX;
             FinalPoint.y = fNextY;
             FinalPoint.t = fNextPhi;
             FinalPoint.nDirection = nDirection;
             FinalPoint.fSteering = fSteering;
             pathInterpolated.push_back(FinalPoint);
             /*计算补全最后一个均匀点,多产生的点数*/
             nEndNum = int(hypot(FinalPoint.x - path.back().x, FinalPoint.y - path.back().y) / Constants::step_size);
         }
    }
}

/**
 * @brief 固定转向轨迹
 * @param path 路径
 */
bool HybridAStar::IsFixSteeringPath(const std::vector<Node3D> &path)
{
    bool bFixSteering = true;

    if(path.size() < 2)
    {
        bFixSteering = false;
    }
    else if(path.size() == 2)
    {
        bFixSteering = true;
    }
    else
    {
        if(!path.back().isRs)
        {
            for(size_t i = 2; i < path.size(); ++i)
            {
                if(fabsf(path[i].fSteering - path[i - 1].fSteering) > 1e-3)
                {
                    bFixSteering = false;
                    break;
                }else{}
            }
        }
        else
        {
            bFixSteering = false;
        }
    }
    return bFixSteering;
}

/**
 * @brief 路径后处理:每一片段的路径优化、生成控制需要的控制点
 * @param PartitionedResults 轨迹片段
 * @param partitioned_target_curvature_sets 分段控制点,包含坐标曲率等参数
 * @return 0-优化失败，1-优化成功
 */
void HybridAStar::OriginalHybridAStarTrajPostProcess(const std::vector<std::vector<Node3D> > &PartitionedResults,
                                                     std::vector<std::vector<CtrlPoint> >& partitioned_target_curvature_sets)
{

#ifdef DEGUG_TIME
    struct timeval start_Smoothing,end_Smoothing;
    float timeUseSmoothing = 0.0;
#endif
    float bsline_step_size = Constants::step_size;
    float min_steering_radius = Constants::min_steering_radius;
    float fSteering = 0.0;
    float kappa = 0.0;
    float fCurveLength = 0.0;

    int8_t nSteeringDirection = 0;
    int8_t nDrivingDirection = 0;
    int nStartNum = 0;
    int nEndNum = 0;
    int ExceedMaxCurvaturePointNum = 0;

    LocationPoint tPoint(0, 0, 0);
    std::vector<Node3D> UniformPath;
    std::vector<CtrlPoint> InterpolatedPath;
    std::vector<Node3D> optiPath;

    partitioned_target_curvature_sets.clear();
    InterpolatedPath.clear();
    UniformPath.clear();
    optiPath.clear();

    m_fYawOffsetOfOpti = 0.0;
    for(size_t j = 0; j < PartitionedResults.size(); ++j)
    {
        ExceedMaxCurvaturePointNum = 0;
        fCurveLength = 0.0f;
        if(IsFixSteeringPath(PartitionedResults[j]))
        {
            /*两个点为固定方向盘转向*/
            nDrivingDirection = PartitionedResults[j].back().nDirection;
            fSteering = PartitionedResults[j].back().fSteering;
            nSteeringDirection = SgnCus(fSteering);

            for(size_t index = 1; index < PartitionedResults[j].size(); ++index)
            {
                fCurveLength += hypotf(PartitionedResults[j][index].x - PartitionedResults[j][index - 1].x,
                        PartitionedResults[j][index].y - PartitionedResults[j][index - 1].y) ;
            }
            if(fabsf(fSteering) > 1e-3f)
            {
                kappa = tan(fSteering) / Constants::WheelBase;
            }
            else
            {
                kappa = 0.0;
            }
            if (PartitionedResults[j].size() >1)
            {
                tPoint.x = PartitionedResults[j][0].x;
                tPoint.y = PartitionedResults[j][0].y;
                tPoint.yaw = PartitionedResults[j][0].t;
            }
            /*按照圆弧生成定曲率轨迹点*/
            CircleInterpolate(nDrivingDirection, nSteeringDirection, kappa, fCurveLength, tPoint, InterpolatedPath);
        }
        else
        {

            MakingPathUniform(PartitionedResults[j],
                              UniformPath, nStartNum, nEndNum);

            bool bNeedAddPoint = false;
            for(int i = 0; i < 2; ++i)
            {
                ExceedMaxCurvaturePointNum = 0;
                lmfOpter.HybridAStarResultedTrajetoryOptimationByLMF(bNeedAddPoint, UniformPath, optiPath);
                BslineInterpolate(optiPath, bsline_step_size, InterpolatedPath);
                /*删除为了均匀路径产生的多余的点*/
                for(int i = 1; i <= nStartNum; i++)
                {
                    if(!InterpolatedPath.empty())
                    {
                        InterpolatedPath.erase(InterpolatedPath.begin());
                    }
                }
                for(int i = 1; i <= nEndNum; i++)
                {
                    if(!InterpolatedPath.empty())
                    {
                        InterpolatedPath.pop_back();
                    }
                }
                CalDiscretePointCurvature(InterpolatedPath);
                for(size_t i = 0; i < InterpolatedPath.size(); i++)
                {
                    if(fabsf(InterpolatedPath[i].kappa) > 1/min_steering_radius + 0.01)
                    {
                        GAC_LOG_DEBUG("exceeds max curvature is %f", fabsf(InterpolatedPath[i].kappa) - 1/min_steering_radius);
                        ExceedMaxCurvaturePointNum++;
                    }else{}
                }
#ifdef DEBUG_PRINT
                GAC_LOG_DEBUG("%d point curvature exceeds max curvature\n", ExceedMaxCurvaturePointNum);
                GAC_LOG_DEBUG("the %ld segment original start yaw is %f\n", j, normalizeHeadingRad_Npi_Ppi(PartitionedResults[j].front().t) * 57.3f);
                GAC_LOG_DEBUG("the %ld segment opti start yaw is %f\n", j, normalizeHeadingRad_Npi_Ppi(InterpolatedPath.front().yaw + PI * (InterpolatedPath.back().driving_direction == -1 ? 1 : 0)) * 57.3f);
                GAC_LOG_DEBUG("the %ld segment original end yaw is %f\n", j, normalizeHeadingRad_Npi_Ppi(PartitionedResults[j].back().t) * 57.3f);
                GAC_LOG_DEBUG("the %ld segment opti end yaw is %f\n", j, normalizeHeadingRad_Npi_Ppi(InterpolatedPath.back().yaw + PI * (InterpolatedPath.back().driving_direction == -1 ? 1 : 0)) * 57.3f);
#endif

                float fOrinialYaw = 0.0f;
                float fOptiYaw = 0.0f;
                float fDeltaYaw = 0.0f;
                if(j == 0)
                {
                    fOrinialYaw = normalizeHeadingRad_Npi_Ppi(PartitionedResults[j].back().t);
                    fOptiYaw = normalizeHeadingRad_Npi_Ppi(InterpolatedPath.back().yaw + PI * (InterpolatedPath.back().driving_direction == -1 ? 1 : 0));
                    fDeltaYaw = normalizeHeadingRad_Npi_Ppi(fOrinialYaw - fOptiYaw);
                    if(PartitionedResults.size() >= 2)
                    {
                        if(PartitionedResults[1].size() >= 3)
                        {
                            /*下一段轨迹足够长才可以赋值，防止控制无法跟上*/
                            m_fYawOffsetOfOpti = SgnCus(fDeltaYaw)* MIN(fabsf(fDeltaYaw), 2 * PI / 180);
                        }else{}
                    }else{}
                }
                if(ExceedMaxCurvaturePointNum > 3)
                {
                    bNeedAddPoint = true;
                }
                else
                {
                    break;
                }
            }
        }
        partitioned_target_curvature_sets.push_back(InterpolatedPath);
        InterpolatedPath.clear();
        optiPath.clear();
        InterpolatedPath.clear();
    }
    return;
}

/**
 * @brief b样条差值
 * @param 原始优化路径
 * @param 插值步长
 * @param 插值结果
 */
void HybridAStar::BslineInterpolate(const std::vector<Node3D> &path,
                       float bSlineStepSize,
                       std::vector<CtrlPoint>& InterpolatedPath)
{
    int nSegNum = 0.0;
    int nDrivingDirection = 0;
    float seg_length = 0.0;
    float  last_s0 = 0.0;
    float last_s1 = 0.0;
    float u = 0.0;
    CtrlPoint CtrlPoint_;
    Eigen::Vector2d s;
    Eigen::Vector2d xi2d;
    std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > TrajPoints;
    polyview::containers::Bspline<2> m_Bspline;

    InterpolatedPath.clear();

    if(path.empty())
    {
        ;
    }
    else
    {
        nDrivingDirection= path[0].nDirection;

        last_s0 = path[0].x;
        last_s1 = path[0].y;

        for(size_t i = 0; i < path.size(); ++i)
        {
            s[0] = path[i].x;
            s[1] = path[i].y;
            TrajPoints.push_back(s);
            seg_length += hypot(s[0] - last_s0, s[1] - last_s1);
            last_s0 = s[0];
            last_s1 = s[1];
        }

        // BSlineinterpolation
        nSegNum = int(seg_length / bSlineStepSize);

        /*如果原始轨迹点数目少于3个，那么不可以用二阶B样条插值*/
        if(path.size() <= 2)
        {
            for(size_t i = 0; i < path.size(); i++)
            {
                CtrlPoint_.x = path[i].x;
                CtrlPoint_.y = path[i].y;
                CtrlPoint_.yaw = normalizeHeadingRad_Npi_Ppi(path[i].t);
                CtrlPoint_.driving_direction = path[i].nDirection;
                InterpolatedPath.push_back(CtrlPoint_);
            }
        }
        else
        {
            int nBslineOrder = 2;
            if(path.size() < 4)
            {
                nBslineOrder = 2;
            }
            else
            {
                nBslineOrder = 3;
            }
            //update new bsline_step_size;
            bSlineStepSize = seg_length/nSegNum;

            m_Bspline.Interpolation(TrajPoints, nBslineOrder);

            for(int i=0; i <= nSegNum; i++)
            {
                u = MIN(i*bSlineStepSize/seg_length, 1.0f);

                /*按照生成b样条的线长度比例进行取点*/
                xi2d = m_Bspline.getPoint(u);
                CtrlPoint_.x = xi2d[0];
                CtrlPoint_.y = xi2d[1];
                CtrlPoint_.yaw = 0.0;
                CtrlPoint_.driving_direction = nDrivingDirection;
                InterpolatedPath.push_back(CtrlPoint_);
            }
        }
    }
}

/**
 * @brief 过滤出最优路径
 * @param configurationSpace 配置空间
 * @param reservedPath 搜索的成功路径
 * @param OptimalPath 最优路径
 */
void HybridAStar::filteringOptimalPath(CollisionDetection& configurationSpace,
                                       std::vector<PARTITION_PATH> &reservedPath,
                                       PARTITION_PATH &OptimalPath)
{
    float fTotalCost = 0.0f;
    float fMinCost = 0.0;
    float fYRange = 0.0f;
    size_t nIndex = 0;

    using PathCostPair = pair<PARTITION_PATH, float>;
    vector<PathCostPair> firstLevelFilteringPath;
    vector<PathCostPair> secondLevelFilteringPath;

    if(!reservedPath.empty())
    {
        /*一级过滤,取出最低代价*/
        for(size_t i = 0; i < reservedPath.size(); ++i)
        {
            fTotalCost = CalReservedPathTotalCost(configurationSpace, reservedPath[i]);
            firstLevelFilteringPath.push_back(PathCostPair(reservedPath[i], fTotalCost));
        }

        if(!firstLevelFilteringPath.empty())
        {
            sort(firstLevelFilteringPath.begin(), firstLevelFilteringPath.end(), [](const PathCostPair &a, const PathCostPair &b)
            {return a.second < b.second;});
            fMinCost  = firstLevelFilteringPath.front().second;
        }

        /*二级过滤,最低代价附近的路径，取出Y方向占据最小空间的轨迹*/
        for(size_t i = 0; i < firstLevelFilteringPath.size(); ++i)
        {
            if(firstLevelFilteringPath[i].second < fMinCost + 2.0f)
            {
                fYRange = CalYRangeOfPath(firstLevelFilteringPath[i].first);
                secondLevelFilteringPath.push_back(PathCostPair(firstLevelFilteringPath[i].first, fYRange));
            }
        }

        if(!secondLevelFilteringPath.empty())
        {
            sort(secondLevelFilteringPath.begin(), secondLevelFilteringPath.end(),
                 [](const PathCostPair &a, const PathCostPair &b){return a.second < b.second;});

            OptimalPath = secondLevelFilteringPath[nIndex].first;

            float fMinDis = 1e5;
            float fDis = 0.0;
            for(size_t i = 0; i < OptimalPath.size(); ++i)
            {
                fMinDis = 1e5;
                for(size_t j = 0; j < OptimalPath[i].size(); ++j)
                {
                    fDis = configurationSpace.GetMinObstacleDisOfVehicle(OptimalPath[i][j].x,
                                                                  OptimalPath[i][j].y,
                                                                  OptimalPath[i][j].t);
                    if(fDis < fMinDis)
                    {
                        fMinDis = fDis;
                    }
                }
                GAC_LOG_DEBUG("the %d seg min dis is %f", i, fMinDis);
            }
        }
    }
    return;
}

/**
 * @brief 计算路径在Y方向的占据范围值
 * @param path路径
 * @return Y方向的占据范围值
 * @note 泊车优先选取Y方向占据最小空间的轨迹，这样可以尽可能减少过道使用
 */
float HybridAStar::CalYRangeOfPath(const PARTITION_PATH &path)
{
    float fYRange = 0.0f;
    LocationPoint tE;
    LocationPoint tF;
    LocationPoint tG;
    LocationPoint tH;
    LocationPoint tVehPos;
    std::vector<float> vec;

    if(!path.empty())
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            for(size_t j = 0; j < path[i].size(); ++j)
            {
                tVehPos.x = path[i][j].x;
                tVehPos.y = path[i][j].y;
                tVehPos.yaw = path[i][j].t;
                CalCornerCoordinate(tVehPos, tE, tF, tG, tH);
                vec.push_back(tE.y);
                vec.push_back(tF.y);
                vec.push_back(tG.y);
                vec.push_back(tH.y);
            }
        }
        if(!vec.empty())
        {
            std::sort(vec.begin(), vec.end(), [](const float &a, const float &b) {return a < b;});
            fYRange = fabs(vec.back() - vec.front());
        }
    }
    return fYRange;
}

/**
 * @brief 计算搜索的成功路径总代价
 * @param configurationSpace 配置空间
 * @param reservedPath 搜索的成功路径
 * @return 总代价
 */
float HybridAStar::CalReservedPathTotalCost(CollisionDetection& configurationSpace,
                                           PARTITION_PATH &reservedPath)
{

    float fPiecewiseCost = 0.0f;
    float fStepSize = 0.0f;
    float fObsDis = 0.0f;
    int8_t nDirection = 0;

    for(size_t i = 0; i < reservedPath.size(); ++i)
    {
        nDirection = reservedPath[i].front().nDirection;
        for(size_t j = 1; j < reservedPath[i].size(); ++j)
        {
            fStepSize = hypotf(reservedPath[i][j].x - reservedPath[i][j - 1].x,
                               reservedPath[i][j].y - reservedPath[i][j - 1].y);
            if(nDirection == FORWARD)
            {
                /*前向行驶惩罚*/
                fPiecewiseCost += fStepSize*Constants::traj_forward_penalty;
            }
            else
            {
                /*后退行驶惩罚*/
                fPiecewiseCost += fStepSize*Constants::traj_back_penalty;
            }

            /*转向改变惩罚*/
            fPiecewiseCost += Constants::traj_steer_change_penalty * fabsf(reservedPath[i][j].fSteering - reservedPath[i][j - 1].fSteering);

            /* 不合理转向改变惩罚*/
            if(IllogicalSteeringCheck(&reservedPath[i][j]))
            {
                fPiecewiseCost += Constants::traj_illogical_steer_penalty * fabsf(reservedPath[i][j].fSteering);
            }

            fObsDis = configurationSpace.GetMinObstacleDisOfVehicle(reservedPath[i][j].x,
                                                                    reservedPath[i][j].y,
                                                                    reservedPath[i][j].t);
            fPiecewiseCost += CalObstacleDisCost(fObsDis);
        }
        if(reservedPath.size() > 1 && (i < reservedPath.size() - 1))
        {
            /*轨迹只有一段和最后一段轨迹都不要计算换挡惩罚*/
            fPiecewiseCost += Constants::traj_gear_switch_penalty;
        }else{}

        if(reservedPath[i].size() >=2 && reservedPath[i].size() <=4)
        {
            /*短轨迹惩罚*/
            fPiecewiseCost += 4.0f / (reservedPath[i].size() - 1);
        }
    }

    return fPiecewiseCost;
}

/**
 * @brief hybrid A Star路径搜索+路径平滑
 * @param start 起点
 * @param goal 终点
 * @param nodes3D node 3D节点
 * @param width 3D网格宽度
 * @param height 3D网格高度
 * @param configurationSpace 配置空间
 * @param HybridAResults hybrid A Star路径搜索结果
 * @param partitioned_target_curvature_sets 分段路径平滑结果
 * @return 1-搜索成功 0-搜索失败
 */
bool HybridAStar::plan(Node3D& start,
                       Node3D& goal,
                       Node3D* nodes3D,
                       CollisionDetection& configurationSpace,
                       std::vector<Node3D>& HybridAResults,
                       std::vector<std::vector<CtrlPoint> >& partitioned_target_curvature_sets)
{

    bool bPlanSuccess=true;
    std::vector<std::vector<Node3D> > PartitionedResults;
    vector<PARTITION_PATH> reservedPath;
    HybridAResults.clear();;
    partitioned_target_curvature_sets.clear();

#ifdef DEGUG_TIME
    QTime t;
    t.restart();
#endif
    bPlanSuccess = hybridAStar(start, goal, nodes3D, configurationSpace, reservedPath);
#ifdef DEGUG_TIME
    GAC_LOG_DEBUG("hybridAStar time is %d", t.restart());
#endif
    if (bPlanSuccess)
    {
        //2 hybrid A*原始路径点按照前进后退进行片段分割

        filteringOptimalPath(configurationSpace, reservedPath, PartitionedResults);

        for(size_t i = 0; i < PartitionedResults.size(); ++i)
        {
            for(size_t j = 0; j < PartitionedResults[i].size(); ++j)
            {
                HybridAResults.push_back(PartitionedResults[i][j]);
            }
        }

#ifdef DEGUG_TIME
        struct timeval start_Smoothing,end_Smoothing;
        float timeUseSmoothing = 0.0;
        gettimeofday(&start_Smoothing,NULL);
#endif
        //3 每一片段的路径优化

        if(!PartitionedResults.empty())
        {
            OriginalHybridAStarTrajPostProcess(PartitionedResults, partitioned_target_curvature_sets);
        }
        else
        {
            bPlanSuccess = false;
        }

#ifdef DEGUG_TIME
        gettimeofday(&end_Smoothing,NULL);
        timeUseSmoothing=(1000000*(end_Smoothing.tv_sec - start_Smoothing.tv_sec) + end_Smoothing.tv_usec-start_Smoothing.tv_usec)/1000.0;
        GAC_LOG_DEBUG("Total smoothing time is %f",timeUseSmoothing);
#endif
    }
    else
    {
        planErrCode = E10005;
    }
    return bPlanSuccess;
}

/**
 * @brief 初始化地图和输入信息
 * @param map 地图
 * @param InputParkingIn_ 输入信息
 * @return 1-初始化成功，0-初始化失败
 */
bool HybridAStar::InitMapInforAndVehicleInfor(unsigned char* map,
                                              InputParkingIn &InputParkingIn_)
{
    bool bValidity = false;
    float fVehContourSafeDis = 0.0f;
    float fMirrorSafeDis = 0.0f;
    float fSlotAreaSafeDis = 0.0f;
    float fOutSlotSafeDis = 0.0f;
    float fDisBetweenCurrentAndLastPlanning = 0.0;
    cpoint tF, tF2, tR, tR2;
    Polygon polygon;
    LocationPoint tSlotMiddlePoint;

//    if(InputParkingIn_.m_nTotalStep > 5)
//    {
//        int nMapIndex = round((InputParkingIn_.fDeltaXRearEdge + InputParkingIn_.fXSlotPosInGrid - 0.15f)*10)
//                        + round(InputParkingIn_.fYSlotPosInGrid*10)*250;
//        // int nMapIndex = round(InputParkingIn_.fXSlotPosInGrid*10) + round(InputParkingIn_.fYSlotPosInGrid*10)*250;
//        map[MIN(MAX(nMapIndex-1, 0),250*150-1)] = NO_FREE;
//        map[MIN(MAX(nMapIndex-2, 0),250*150-1)] = NO_FREE;
//        map[MIN(MAX(nMapIndex, 0),250*150-1)] = NO_FREE;
//        map[MIN(MAX(nMapIndex+250, 0),250*150-1)] = NO_FREE;
//        map[MIN(MAX(nMapIndex+500, 0),250*150-1)] = NO_FREE;
//    }
    Init();
    /*1 更新车位坐标信息*/
    m_slotPosInfor.nParkingType = InputParkingIn_.cParkType;/*1*/
    m_slotPosInfor.nSlotType = InputParkingIn_.cSlotType;/*2*/
    m_slotPosInfor.nSlotSide = InputParkingIn_.cSlotPosition;/*3*/
    m_slotPosInfor.nDetectType = InputParkingIn_.cDetectType;/*4*/
    m_slotPosInfor.fXSlotPosInMap = InputParkingIn_.fXSlotPosInGrid;/*5*/
    m_slotPosInfor.fYSlotPosInMap = Constants::ob_grid_height/10 - InputParkingIn_.fYSlotPosInGrid;/*6*/

    m_slotPosInfor.fSlotLength = InputParkingIn_.fSlotWidth;/*7*/

    m_slotPosInfor.fSlotWidth = InputParkingIn_.fSlotDepth;/*8*/
    m_slotPosInfor.cParkOutDir = InputParkingIn_.cParkOutDir;
    m_slotPosInfor.nUssSideType = InputParkingIn_.nUssSideType;
    m_OriginSlotYaw = InputParkingIn_.fSlotYaw;
    if(fabsf(m_OriginSlotYaw - 90*PI/180) < 6*PI/180)
    {
        m_slotPosInfor.fSlotAngle = InputParkingIn_.cSlotPosition*90*PI/180;
    }
    else
    {
        m_slotPosInfor.fSlotAngle = InputParkingIn_.cSlotPosition*m_OriginSlotYaw;
    }

    m_deltaRearX = InputParkingIn_.fDeltaXRearEdge;
    m_deltaRearY = InputParkingIn_.fDeltaYRearLine;
    m_deltaFrontX =  InputParkingIn_.fDeltaXFrontEdge;
    m_deltaFrontY = InputParkingIn_.fDeltaYFrontLine;

    m_slotPosInfor.fXRearSlotCorner = m_deltaRearX;/*10*/
    m_slotPosInfor.fYRearSlotCorner = m_deltaRearY;/*13*/

    m_slotPosInfor.fXFrontSlotCorner = m_deltaFrontX + InputParkingIn_.fSlotWidth;/*11*/
    m_slotPosInfor.fYFrontSlotCorner = m_deltaFrontY;/*12*/

    m_slotPosInfor.tFrontSlotCorner = InputParkingIn_.tFrontSlotCorner;
    m_slotPosInfor.tRearSlotCorner = InputParkingIn_.tRearSlotCorner;
    memcpy(m_slotPosInfor.SlotLineType, InputParkingIn_.SlotLineType, 16);

    if(m_slotPosInfor.fSlotLength > Constants::length)
    {
        m_slotPosInfor.fXRearSlotCorner = m_deltaRearX - 0.2f;/*10*/
        m_slotPosInfor.fXFrontSlotCorner = m_deltaFrontX + InputParkingIn_.fSlotWidth - 0.2f;/*11*/
    }

    m_slotPosInfor.startPointInSlot.x = InputParkingIn_.fVehCurX;/*14*/
    m_slotPosInfor.startPointInSlot.y = InputParkingIn_.fVehCurY;/*15*/

    if(InputParkingIn_.cARV_On != 0)
    {
        m_slotPosInfor.fGroundpinDepth = InputParkingIn_.m_VehTargetx;
    }else
    {
        m_slotPosInfor.fGroundpinDepth = 10.f;
    }

    /*偏航角补偿策略:如果本次规划为第一次规划或者本次规划和上一次规划距离小于一定长度，那么不进行补偿*/
    m_fYawOffsetOfOpti = InputParkingIn_.m_fYawOffsetOfOpti;
    m_lastPlanningPoint = InputParkingIn_.m_lastPlanningPoint;
    if(InputParkingIn_.m_bFirstPlan)
    {
        fDisBetweenCurrentAndLastPlanning = 0.0f;
    }
    else
    {
        fDisBetweenCurrentAndLastPlanning = hypotf(m_slotPosInfor.startPointInSlot.x - m_lastPlanningPoint.x,
                                                   m_slotPosInfor.startPointInSlot.y - m_lastPlanningPoint.y);
    }
//  if(fabs(InputParkingIn_.m_fYawOffsetOfOpti) > 3 * PI /180
       //|| (fDisBetweenCurrentAndLastPlanning < 1.0f))
    {
        m_fYawOffsetOfOpti = 0;
    }
    m_slotPosInfor.startPointInSlot.yaw = InputParkingIn_.fVehCurYaw + m_fYawOffsetOfOpti;/*16*/

    m_lastPlanningPoint = m_slotPosInfor.startPointInSlot;

    mParaSlotSteps = InputParkingIn_.mParaSlotSteps;
    m_bStartPathplanningInParaSLot = InputParkingIn_.m_bStartPathplanningInParaSLot;
    m_nDrivingDirection = InputParkingIn_.m_nDrivingDirection;
    m_nFirstPlanTotalSteps = InputParkingIn_.m_nFirstPlanTotalSteps;
    m_ParaHybridALastDriving = InputParkingIn_.m_ParaHybridALastDriving;
    m_ParkingCtrlStatus = InputParkingIn_.m_ParkingCtrlStatus;
    m_TotalGearSwitchTimes = InputParkingIn_.m_TotalGearSwitchTimes;
    m_bFirstPlan = InputParkingIn_.m_bFirstPlan;
    m_bUSSHasDetectFrontMargin = InputParkingIn_.m_bUSSHasDetectFrontMargin;
    m_bUSSHasDetectRearMargin = InputParkingIn_.m_bUSSHasDetectRearMargin;
    m_RotationCoordinate = InputParkingIn_.RotationCoordinate;
    m_bLastDriving = InputParkingIn_.m_LastDriving;

    m_nCurrentStep = InputParkingIn_.m_nCurrentStep;
    m_nTotalStep = InputParkingIn_.m_nTotalStep;

    if(m_nCurrentStep > MAX_PARKING_STEP - 1)
    {
        /*E10006超出最大轨迹段数*/
        planErrCode=E10006;
        return false;
    }

    if(m_slotPosInfor.nParkingType == REMOTE_IN_OUT)
    {
        return true;
    }
    m_bHitWheelBar = InputParkingIn_.m_bHitWheelBar;

    for(int i = 0; i < Constants::ob_grid_height; i++)
    {
        memcpy((void*)(occupyMap + (Constants::ob_grid_height - 1 - i) * Constants::ob_grid_width),
               (void*)(map + i * Constants::ob_grid_width), Constants::ob_grid_width * sizeof(unsigned char));
    }

    m_slotPosInfor.fRoadWidth = getRoadWidth(m_slotPosInfor, m_configurationSpace);

    m_slotPosInfor.targetPointInSlot = InputParkingIn_.TarVehPoseInSlot;
    m_slotPosInfor.isAvoidance = InputParkingIn_.nAvoidStatus > 0 ? true: false;
    /*2 设置3D网格起点和终点，以及车位坐标系起点和终点*/
    bValidity = SlotPosInforInputsCheck(m_slotPosInfor) && SetStartPointAndGoalPoint(m_slotPosInfor);

    m_slotPosInfor.bValidty = bValidity;

    if(bValidity)
    {
        tF.x = m_slotPosInfor.fXFrontSlotCorner + m_slotPosInfor.fXSlotPosInMap;
        tF.y = m_slotPosInfor.fYFrontSlotCorner + m_slotPosInfor.fYSlotPosInMap;

        tF2.x = tF.x - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
                * cosf(m_slotPosInfor.fSlotAngle);
        tF2.y = tF.y - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
                * sinf(m_slotPosInfor.fSlotAngle);

        tR.x = m_slotPosInfor.fXRearSlotCorner + m_slotPosInfor.fXSlotPosInMap;
        tR.y = m_slotPosInfor.fYRearSlotCorner + m_slotPosInfor.fYSlotPosInMap;

        tR2.x = tR.x - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
                * cosf(m_slotPosInfor.fSlotAngle);
        tR2.y = tR.y - m_slotPosInfor.fSlotWidth / fabsf(sinf(m_slotPosInfor.fSlotAngle))
                * sinf(m_slotPosInfor.fSlotAngle);


        /*3 计算车位中线方程*/
        tSlotMiddlePoint.x = m_slotPosInfor.fXSlotPosInMap + m_slotPosInfor.fSlotLength / 2;
        tSlotMiddlePoint.y = m_slotPosInfor.fYSlotPosInMap;
        tSlotMiddlePoint.yaw = m_slotPosInfor.fSlotAngle;
        CalLineEquation(tSlotMiddlePoint, m_slotPosInfor.LineEq_Goal);

        if(m_slotPosInfor.nSlotType == OBLIQUE)
        {
            /*4 设置库外\库内\角点碰撞安全距离*/
            CalSafeDis(tR, tF, m_slotPosInfor.LineEq_Goal, fVehContourSafeDis, fMirrorSafeDis);
            fSlotAreaSafeDis = Constants::obli_slot_area_dis;
            fOutSlotSafeDis = Constants::obli_out_slot_area_dis;

            /*5 计算车头泊入转弯半径*/
            if(m_slotPosInfor.nParkingType == HEAD_PARK_IN)
            {
                m_fHeadInRsRadius = CalHeadInParkingRadius(tR, tF, m_slotPosInfor.LineEq_Goal, fVehContourSafeDis);
            }
            else
            {
                m_fHeadInRsRadius = Constants::MaxROfHeadIn;
            }
            /*7 计算垂直车位或者斜列车位的车辆入库必须摆正的坐标点*/
#if 0
            SetObliSlotSafePoint(m_slotPosInfor);
#endif
        }
        else
        {
            /*4 设置库外\库内\角点碰撞安全距离*/
            fVehContourSafeDis = Constants::para_slot_corner_safety_dis;
            fSlotAreaSafeDis = Constants::para_slot_area_dis;
            fOutSlotSafeDis = Constants::para_out_slot_area_dis;
            fMirrorSafeDis = 0.01f;;
        }

        /*设置车位关键角点和安全距离*/
        m_configurationSpace.setSlotConer(tR, tR2, tF, tF2, fVehContourSafeDis, fMirrorSafeDis);

        /*设置车位外和车位内的不同安全距离*/
        CreatSlotAreaPolygon(m_slotPosInfor, polygon);
        m_configurationSpace.setAreaSafeDis(polygon, fOutSlotSafeDis, fSlotAreaSafeDis);
        SetVarialeCell(m_slotPosInfor, m_varialeCellBound);

#ifdef VARIABLE_CELL
        int nNodeNum = 0;
#ifdef SMALL_CELL_SEARCH

        nNodeNum = (Constants::Node3dWidth * Constants::Node3dHeight - Constants::varialeCellWidth * Constants::varialeCellHeight) * Constants::headings
                + 4 * Constants::varialeCellWidth * Constants::varialeCellHeight *Constants::variableCellheadings1;
#else
        nNodeNum = (Constants::Node3dWidth * Constants::Node3dHeight - Constants::varialeCellWidth * Constants::varialeCellHeight) * Constants::headings
                + Constants::varialeCellWidth * Constants::varialeCellHeight *Constants::variableCellheadings2;
#endif

        if(NODE3D_NUM !=  nNodeNum)
        {
            /*E10004可变栅格数量计算错误*/
            planErrCode=E10004;
            bValidity = false;
            GAC_LOG_ERROR("NODE3D_NUM is input error!!!");
        }
#endif
        InputParkingIn_.m_goalPointOffset =  m_slotPosInfor.goalPointInSlot;
    }

    return bValidity;
}


float HybridAStar::getRoadWidth(const slotPosInfor &slotPosInfor_,
                               CollisionDetection& configurationSpace)
{
    bool bHasStart = false;
    bool bHasEnd = false;
    int nRoadStart, nRoadEnd;
    int nStartX = 0;
    int nEndX = 0;
    int nStartY = 0;
    int nEndY = 0;
    int nIncre = 0;
    float fRoadWidth = 0.0;
    if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
    {
        nStartX = int((slotPosInfor_.fSlotLength + slotPosInfor_.fXSlotPosInMap) / Constants::obSize);
        nEndX = int((slotPosInfor_.fSlotLength + slotPosInfor_.fXSlotPosInMap + Constants::length) / Constants::obSize);
        nStartY = int((slotPosInfor_.fYSlotPosInMap) / Constants::obSize);
        nEndY = Constants::ob_grid_height - 1;
        nIncre = 1;
    }
    else
    {
        nStartX = int((slotPosInfor_.fSlotLength + slotPosInfor_.fXSlotPosInMap) / Constants::obSize);
        nEndX = int((slotPosInfor_.fSlotLength + slotPosInfor_.fXSlotPosInMap + Constants::length) / Constants::obSize);
        nStartY = int((slotPosInfor_.fYSlotPosInMap) / Constants::obSize);
        nEndY =  0;
        nIncre = -1;
    }
    nStartX = MAX(MIN(nStartX, Constants::ob_grid_width - 1), 0);
    nEndX = MAX(MIN(nEndX, Constants::ob_grid_width - 1), 0);
    nStartY = MAX(MIN(nStartY, Constants::ob_grid_height - 1), 0);
    nEndY = MAX(MIN(nEndY, Constants::ob_grid_height - 1), 0);

    for(int ny = nStartY; nIncre * (ny  - nEndY) < 0; ny += nIncre)
    {
        bHasStart = true;
        for(int nx = nStartX; nx < nEndX; ++nx)
        {
            if(!configurationSpace.gridIsFree(occupyMap[nx + ny * Constants::ob_grid_width]))
            {
                bHasStart = false;
                break;
            }else{}
        }
        if(bHasStart)
        {
            nRoadStart = ny;
            break;
        }
    }

    if(bHasStart)
    {
        for(int ny = nRoadStart; nIncre * (ny  - nEndY) < 0; ny += nIncre)
        {
            bHasEnd = false;
            for(int nx = nStartX; nx < nEndX; ++nx)
            {
                if(!configurationSpace.gridIsFree(occupyMap[nx + ny * Constants::ob_grid_width]))
                {
                    bHasEnd = true;
                    break;
                }else{}
            }
            if(bHasEnd)
            {
                nRoadEnd = ny;
                break;
            }
        }
    }

    if(bHasEnd && bHasStart)
    {
        fRoadWidth = MAX((fabs(nRoadEnd - nRoadStart) + 1) * Constants::obSize, 0);
    }
    else
    {
        fRoadWidth = 0.0;
    }
    GAC_LOG_INFO("road width is %f", fRoadWidth);

    return fRoadWidth;
}
/**
 * @brief 设置可变网格区域
 * @param slotPosInfor_ 车位信息
 * @param varialeCellBound 可变网格区域
 */
void HybridAStar::SetVarialeCell(const slotPosInfor &slotPosInfor_,
                                 VarialeCellBoundary &varialeCellBound)
{

    int nVarialeCellWidth = Constants::varialeCellWidth;
    int nVarialeCellHeight = Constants::varialeCellHeight;
    int nXLength = 0;
    int nYLength = 0;
    float fxs, fxe, fys, fye= 0.0;
    float fVarialeCellWidth = nVarialeCellWidth * Constants::cellSize;
    float fVarialeCellHeight = nVarialeCellHeight * Constants::cellSize;
    float fXLength = 0;
    float fYLength = 0;

    float fError = 1e-3f;
    float fMapWidth = Constants::ob_grid_width * Constants::obSize;
    float fMapHeight = Constants::ob_grid_height * Constants::obSize;

    if(slotPosInfor_.nSlotType == OBLIQUE)
    {
        /*垂直的可变栅格区域用最大宽度的一半足够了*/  
        nXLength = nVarialeCellWidth / 2;
        nYLength = nVarialeCellHeight;
        fXLength =  fVarialeCellWidth / 2;
        fYLength = fVarialeCellHeight;
        if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
        {
            fxs = /*slotPosInfor_.fSlotLength / 2 - fXLength / 2 +*/ slotPosInfor_.fXSlotPosInMap;
            fxe = fxs + fXLength;
            if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
            {
                fys = /*-1.0f + */slotPosInfor_.fYSlotPosInMap;
                fye = fys + fYLength;
            }
            else
            {
                fye = /*1.0f + */slotPosInfor_.fYSlotPosInMap;
                fys = fye - fYLength;
            }

#ifdef BYD_HAN
           varialeCellBound.bLargeHeadings = true;
#else
            varialeCellBound.bLargeHeadings = false;
#endif
//            varialeCellBound.bLargeHeadings = false;

            varialeCellBound.bLargeCell = true;

        }
        else
        {
            fxs = slotPosInfor_.fSlotLength / 2 - fXLength / 2 + slotPosInfor_.fXSlotPosInMap;
            fxe = fxs + fXLength;
            if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
            {
                fys = 2 + slotPosInfor_.fYSlotPosInMap;
                fye = fys + fYLength;
            }
            else
            {
                fye = -2 + slotPosInfor_.fYSlotPosInMap;
                fys = fye - fYLength;
            }
            varialeCellBound.bLargeHeadings = true;
            varialeCellBound.bLargeCell = true;
        }
    }
    else
    {
        nXLength = nVarialeCellWidth;
        nYLength = nVarialeCellHeight;
        fYLength = fVarialeCellHeight;
        fXLength = fVarialeCellWidth;

        fxs = slotPosInfor_.fSlotLength / 2 - fXLength / 2 + slotPosInfor_.fXSlotPosInMap;
        fxe = fxs + fXLength;
        if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
        {
            fys = -1.5f + slotPosInfor_.fYSlotPosInMap;
            fye = fys + fYLength;
        }
        else
        {
            fye = 1.5f + slotPosInfor_.fYSlotPosInMap;
            fys = fye - fYLength;
        }
#ifdef SMALL_CELL_SEARCH
        varialeCellBound.bLargeHeadings = false;
        varialeCellBound.bLargeCell = false;
#else
        varialeCellBound.bLargeHeadings = true;
        varialeCellBound.bLargeCell = true;
#endif

    }

    if(fxs < fError)
    {
        varialeCellBound.nxs = 0;
        varialeCellBound.nxe = varialeCellBound.nxs + nXLength - 1;
    }
    else if(fxe > fMapWidth - fError)
    {
        varialeCellBound.nxe = Constants::Node3dWidth - 1;
        varialeCellBound.nxs = varialeCellBound.nxe - (nXLength - 1);
    }
    else
    {
        varialeCellBound.nxs = int(fxs / Constants::cellSize);
        varialeCellBound.nxe = varialeCellBound.nxs + nXLength - 1;
    }

    if(fys < fError)
    {
        varialeCellBound.nys = 0;
        varialeCellBound.nye = varialeCellBound.nys + nYLength - 1;
    }
    else if(fye > fMapHeight - fError)
    {
        varialeCellBound.nye = Constants::Node3dHeight - 1;
        varialeCellBound.nys = varialeCellBound.nye - (nYLength - 1);
    }
    else
    {
        varialeCellBound.nys = int(fys / Constants::cellSize);
        varialeCellBound.nye = varialeCellBound.nys + nYLength - 1;
    }
}


/**
 * @brief 判断给出的车位的角点是否在车位两条平行线的外侧
 * @param slotPosInfor_ 车位信息
 * @return
 */
bool HybridAStar::SlotCornerCheck(slotPosInfor &slotPosInfor_)
{
    bool bCornerValid = false;
    bool  bFCornerValid = false;
    bool  bRCornerValid = false;
    float fRDis = 0.0;
    float fFDis = 0.0;
    LocationPoint tSlotRearTopPoint(0, 0, slotPosInfor_.fSlotAngle);
    LocationPoint tSlotRearBottomPoint(-(slotPosInfor_.fSlotWidth / fabsf(sin(slotPosInfor_.fSlotAngle))) * cos(slotPosInfor_.fSlotAngle),
                                       -slotPosInfor_.fSlotWidth / fabsf(sin(slotPosInfor_.fSlotAngle)) * sin(slotPosInfor_.fSlotAngle),
                                       slotPosInfor_.fSlotAngle);

    LocationPoint tSlotFrontTopPoint(slotPosInfor_.fSlotLength, 0, slotPosInfor_.fSlotAngle);
    LocationPoint tSlotFrontBottomPoint(slotPosInfor_.fSlotLength - slotPosInfor_.fSlotWidth / fabsf(sin(slotPosInfor_.fSlotAngle)) * cos(slotPosInfor_.fSlotAngle),
                                        -slotPosInfor_.fSlotWidth / fabsf(sin(slotPosInfor_.fSlotAngle)) * sin(slotPosInfor_.fSlotAngle),
                                        slotPosInfor_.fSlotAngle);

    LocationPoint tRearCorner(slotPosInfor_.fXRearSlotCorner, slotPosInfor_.fYRearSlotCorner, 0);
    LocationPoint tFrontCorner(slotPosInfor_.fXFrontSlotCorner, slotPosInfor_.fYFrontSlotCorner, 0);
    LocationPoint tP1, tP2, tP3, tP4;
    LocationPoint newRearCorner, newFrontCorner;
    LineEq line_slotRearSide;
    LineEq line_slotFrontSide;
    LineEq line_RearCorner;
    LineEq line_FrontCorner;

    CalLineEquation(tSlotRearTopPoint, line_slotRearSide);
    CalLineEquation(tSlotFrontTopPoint, line_slotFrontSide);
    CalLineEquation(tRearCorner, line_RearCorner);
    CalLineEquation(tFrontCorner, line_FrontCorner);

    fRDis = CalPointAndLineDis({tRearCorner.x, tRearCorner.y}, line_slotRearSide);
    fFDis = CalPointAndLineDis({tFrontCorner.x, tFrontCorner.y}, line_slotFrontSide);

    if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
    {
        tP1 = tSlotRearBottomPoint;
        tP2 = tSlotRearTopPoint;
        tP3 = tSlotFrontBottomPoint;
        tP4 = tSlotFrontTopPoint;
    }
    else
    {
        tP1 = tSlotRearTopPoint;
        tP2 = tSlotRearBottomPoint;
        tP3 = tSlotFrontTopPoint;
        tP4 = tSlotFrontBottomPoint;
    }
    if(!LeftOfLine(tP1, tP2, tRearCorner) && (fRDis > 0.01f))
    {
        bRCornerValid = false;
    }
    else
    {
        bRCornerValid = true;
    }
    if(LeftOfLine(tP3, tP4, tFrontCorner) && (fFDis > 0.01f))
    {
        bFCornerValid = false;
    }
    else
    {
        bFCornerValid = true;
    }

    bCornerValid = bFCornerValid && bRCornerValid;
    if(!bCornerValid)
    {
        GAC_LOG_INFO("融合给的车位角点有错误，侵入车位！！！找林孝明\n");
        if(!bFCornerValid)
        {
            /*输入保护策略:车位前角点不合理，取过前角点直线和前侧边直线的交点为前角点*/
            CalLineCrossPoint(line_slotFrontSide, line_FrontCorner, newFrontCorner);
            GAC_LOG_DEBUG("之前的前角点x坐标：%f, 之前的前角点y坐标：%f\n",slotPosInfor_.fXFrontSlotCorner, slotPosInfor_.fYFrontSlotCorner);
            slotPosInfor_.fXFrontSlotCorner = newFrontCorner.x;
            slotPosInfor_.fYFrontSlotCorner = newFrontCorner.y;
            GAC_LOG_DEBUG("之后的前角点x坐标：%f, 之后的前角点y坐标：%f\n",slotPosInfor_.fXFrontSlotCorner, slotPosInfor_.fYFrontSlotCorner);
        }else{}
        if(!bRCornerValid)
        {
            /*输入保护策略:车位后角点不合理，取过后角点直线和后侧边直线的交点为后角点*/
            CalLineCrossPoint(line_slotRearSide, line_RearCorner, newRearCorner);
            GAC_LOG_DEBUG("之前的后角点x坐标：%f, 之前的后角点y坐标：%f\n",slotPosInfor_.fXRearSlotCorner, slotPosInfor_.fYRearSlotCorner);
            slotPosInfor_.fXRearSlotCorner = newRearCorner.x;
            slotPosInfor_.fYRearSlotCorner = newRearCorner.y;
            GAC_LOG_DEBUG("之后的后角点x坐标：%f, 之后的后角点y坐标：%f\n",slotPosInfor_.fXRearSlotCorner, slotPosInfor_.fYRearSlotCorner);
        }else{}
    }
    return true;
}

/**
 * @brief 输入信息检测
 * @param[in] slotPosInfor 车位信息
 * @return bValidity 0-输入信息不合理 1-输入信息合理
 */
bool HybridAStar::SlotPosInforInputsCheck(slotPosInfor &slotPosInfor_)
{
    bool bValidity = false;
    bool bDetectTypeValid = false;
    bool bSlotTypeValid = false;
    bool bSlotSideValid = false;
    bool bParkOutDirValid = false;
    bool bUssSideTypeValid = false;
    bool bSlotSizeValid = false;
    bool bCornerValid = false;
    bool bParkingTypeAndSlotTypeValid = false;

    /*探测类型有效：超声、环视、自选*/
    bDetectTypeValid = (slotPosInfor_.nDetectType == USS_TYPE)
                       || (slotPosInfor_.nDetectType == MIX_TYPE)
                       || (slotPosInfor_.nDetectType == USS_CONFIRM_TYPE)
                       || (slotPosInfor_.nDetectType == USER_MANUL_TYPE);

    /*车位类型有效：垂直、平行*/
    bSlotTypeValid = (slotPosInfor_.nSlotType == OBLIQUE)
                     || (slotPosInfor_.nSlotType == PARALLEL);

    /*车位两侧位置有效：右侧、左侧*/
    bSlotSideValid = (slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
                     || (slotPosInfor_.nSlotSide == LEFTSIDESLOT);

    /*泊出车头朝向位置有效：车头向右、车头向左*/
    bParkOutDirValid = (slotPosInfor_.cParkOutDir == RIGHT_PARKING_OUT)
                       || (slotPosInfor_.cParkOutDir == LEFT_PARKING_OUT);

    /*超声检测车位单边类型有效：前单边、后单边、前后边*/
    bUssSideTypeValid = slotPosInfor_.nUssSideType == FRONT_SIDE
                        || (slotPosInfor_.nUssSideType == REAR_SIDE)
                        || (slotPosInfor_.nUssSideType == FRONT_REAR_SIDE);

    /*车位类型和泊车方式要匹配*/
    if(slotPosInfor_.nSlotType == PARALLEL)
    {
        if((slotPosInfor_.nParkingType == HEAD_PARK_IN)
           || (slotPosInfor_.nParkingType == TAIL_PARK_OUT))
        {
            /*平行泊车不存在车头泊入和车尾泊出*/
            bParkingTypeAndSlotTypeValid = false;
        }
        else
        {
            bParkingTypeAndSlotTypeValid = true;
        }
    }
    else
    {
        bParkingTypeAndSlotTypeValid = true;
    }
    /*车位尺寸类型有效*/
    if(bSlotTypeValid)
    {
        if(slotPosInfor_.nSlotType == OBLIQUE)
        {
            bSlotSizeValid = (fabsf(slotPosInfor_.fSlotLength * sin(slotPosInfor_.fSlotAngle)) > Constants::obli_min_slot_length)
                             && (slotPosInfor_.fSlotWidth > Constants::obli_min_slot_depth)
                             && (fabsf(slotPosInfor_.fSlotAngle) > Constants::obli_min_slot_yaw)
                             && (fabsf(slotPosInfor_.fSlotAngle) < Constants::obli_max_slot_yaw);
        }
        else
        {
            if(slotPosInfor_.nDetectType == USS_TYPE)
            {
                bSlotSizeValid = (slotPosInfor_.fXFrontSlotCorner - slotPosInfor_.fXRearSlotCorner > Constants::para_min_slot_length)
                                 && (slotPosInfor_.fSlotWidth > 1.f) && (fabs(fabsf(normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle)) - PI / 2) < 1e-3f) ;
            }
            else
            {
                bSlotSizeValid = (slotPosInfor_.fXFrontSlotCorner - slotPosInfor_.fXRearSlotCorner > Constants::para_min_slot_length)
                                 && (slotPosInfor_.fSlotWidth > 1.f) && (fabs(fabsf(normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle)) - PI / 2) < 1e-3f) ;
            }
        }
    }

    if (slotPosInfor_.nSlotType == OBLIQUE)
    {
        bCornerValid = SlotCornerCheck(slotPosInfor_);
    }else{bCornerValid = true;}

    bValidity = bDetectTypeValid && bSlotTypeValid
                && bSlotSideValid && bParkOutDirValid
                && bUssSideTypeValid && bSlotSizeValid
                && bCornerValid && bParkingTypeAndSlotTypeValid;

    if(!bSlotSizeValid)
    {
        /*E10001车位尺寸不够*/
        planErrCode = E10001;
        GAC_LOG_WARN("Slot size is invalid");
    }
    return bValidity;
}

/**
 * @brief 计算垂直车位或者斜列车位的车辆入库必须摆正的坐标点
 * @param slotPosInfor_ 车位信息
 */
void HybridAStar::SetObliSlotSafePoint(slotPosInfor &slotPosInfor_)
{
    LocationPoint RearCornerPoint, FrontCornerPoint, crossRearPoint, crossFrontPoint, crossPoint;
    LineEq  LineEqRearCornerPoint;
    LineEq  LineEqFrontCornerPoint;
    float fLen = 0.0;

    /*计算车位前角点垂直于直线的直线方程1*/
    RearCornerPoint.x =  slotPosInfor_.fXSlotPosInMap;
    RearCornerPoint.y =  slotPosInfor_.fYSlotPosInMap;
    RearCornerPoint.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.goalPointInMap.yaw + PI / 2);
    CalLineEquation(RearCornerPoint, LineEqRearCornerPoint);

    /*计算车位后角点垂直于直线的直线方程2*/
    FrontCornerPoint.x = slotPosInfor_.fSlotLength + slotPosInfor_.fXSlotPosInMap;
    FrontCornerPoint.y = 0 + slotPosInfor_.fYSlotPosInMap;
    FrontCornerPoint.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.goalPointInMap.yaw + PI / 2);
    CalLineEquation(FrontCornerPoint, LineEqFrontCornerPoint);

    /*计算两个直线和车位中线的交点*/
    CalLineCrossPoint(LineEqRearCornerPoint, slotPosInfor_.LineEq_Goal, crossRearPoint);
    CalLineCrossPoint(LineEqFrontCornerPoint, slotPosInfor_.LineEq_Goal, crossFrontPoint);

    /*选择靠外的交点*/
    if((crossRearPoint.y - crossFrontPoint.y) * slotPosInfor_.nSlotSide > 0)
    {
        crossPoint = crossRearPoint;
    }
    else
    {
        crossPoint = crossFrontPoint;
    }

    /*认为该交点在后视镜位置上，反推车辆后轴坐标*/
    fLen = Constants::RearViewMirrorToCenter - Constants::RearViewWidth;
    slotPosInfor_.safePointParkingIn.x = crossPoint.x - fLen * cos(slotPosInfor_.goalPointInSlot.yaw);
    slotPosInfor_.safePointParkingIn.y = crossPoint.y - fLen * sin(slotPosInfor_.goalPointInSlot.yaw);
    slotPosInfor_.safePointParkingIn.yaw = slotPosInfor_.goalPointInMap.yaw;
}

/**
 * @brief 设置垂直车位的终点
 * @param slotPosInfor 车位输入信息
 * @return 1-设置成功 0-设置失败
 */
bool HybridAStar::SetObliSlotGoalPoint(slotPosInfor &slotPosInfor_)
{
    float fNormalStopX = 0.0;
    float fNormalStopY = 0.0;
    float fStopXMove = 0.0;
    float fStopYMove = 0.0;
    /*斜列车位，将车辆的前边中心和车位中心重合时，车辆前角点凸出于车位线的距离*/
    float fDeltaMoveDis = 0.0;/*fabsf(Constants::width / 2 / tanf(slotPosInfor_.fSlotAngle))*/0;

    /*移动距离， 正为出车位，负为进车位*/
    float fObliTargetMoveDis = OBLI_TARGET_MOVE_DIS - fDeltaMoveDis;

    /*设置车位坐标系中车辆目标点位姿*/
    if(slotPosInfor_.nDetectType == USS_TYPE)
    {
        /*超声垂直*/
        if (slotPosInfor_.nUssSideType == FRONT_REAR_SIDE)
        {
            fNormalStopX = slotPosInfor_.fSlotLength / 2;
        }
        else if (slotPosInfor_.nUssSideType == FRONT_SIDE)
        {
            fNormalStopX = slotPosInfor_.fSlotLength - SINGLE_SIDE_SLOT_DIS - Constants::width / 2;
        }
        else
        {
            fNormalStopX = slotPosInfor_.fXRearSlotCorner + SINGLE_SIDE_SLOT_DIS + Constants::width / 2;
        }
        fNormalStopY = 0 - Constants::FrontEdgeToCenter * sin(slotPosInfor_.fSlotAngle);
        fStopXMove = fObliTargetMoveDis * cos(slotPosInfor_.fSlotAngle);
        fStopYMove = fObliTargetMoveDis * sin(slotPosInfor_.fSlotAngle);
        slotPosInfor_.goalPointInSlot.x = fNormalStopX + fStopXMove;
        slotPosInfor_.goalPointInSlot.y = fNormalStopY + fStopYMove;
        slotPosInfor_.goalPointInSlot.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle);
    }
    else  if(slotPosInfor_.nDetectType == USER_MANUL_TYPE)
    {
        if (slotPosInfor_.nParkingType == HEAD_PARK_IN)
        {
            fNormalStopX = slotPosInfor_.fSlotLength / 2 - Constants::RearEdgeToCenter * cos(slotPosInfor_.fSlotAngle);
            fNormalStopY = 0 - Constants::RearEdgeToCenter * sin(slotPosInfor_.fSlotAngle);
            fStopXMove = fObliTargetMoveDis * cos(slotPosInfor_.fSlotAngle);
            fStopYMove = fObliTargetMoveDis * sin(slotPosInfor_.fSlotAngle);
            slotPosInfor_.goalPointInSlot.x = fNormalStopX + fStopXMove;
            slotPosInfor_.goalPointInSlot.y = fNormalStopY + fStopYMove;
            slotPosInfor_.goalPointInSlot.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle - PI);
        }
        else
        {
            fNormalStopX = slotPosInfor_.fSlotLength / 2 - Constants::FrontEdgeToCenter * cos(slotPosInfor_.fSlotAngle);
            fNormalStopY = 0 - Constants::FrontEdgeToCenter * sin(slotPosInfor_.fSlotAngle);
            fStopXMove = fObliTargetMoveDis * cos(slotPosInfor_.fSlotAngle);
            fStopYMove = fObliTargetMoveDis * sin(slotPosInfor_.fSlotAngle);
            slotPosInfor_.goalPointInSlot.x = fNormalStopX + fStopXMove;
            slotPosInfor_.goalPointInSlot.y = fNormalStopY + fStopYMove;
            slotPosInfor_.goalPointInSlot.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle);
        }
    }
    else
    {
        /*环视垂直或者斜列*/
        if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
        {
            fNormalStopX = slotPosInfor_.fSlotLength / 2 - Constants::FrontEdgeToCenter * cos(slotPosInfor_.fSlotAngle);
            fNormalStopY = 0 - Constants::FrontEdgeToCenter * sin(slotPosInfor_.fSlotAngle);
            fStopXMove = fObliTargetMoveDis * cos(slotPosInfor_.fSlotAngle);
            fStopYMove = fObliTargetMoveDis * sin(slotPosInfor_.fSlotAngle);

            slotPosInfor_.goalPointInSlot.x = fNormalStopX + fStopXMove;
            slotPosInfor_.goalPointInSlot.y = fNormalStopY + fStopYMove;
            slotPosInfor_.goalPointInSlot.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle);
        }
        else if (slotPosInfor_.nParkingType == HEAD_PARK_IN)
        {
            fNormalStopX = slotPosInfor_.fSlotLength / 2 - Constants::RearEdgeToCenter * cos(slotPosInfor_.fSlotAngle);
            fNormalStopY = 0 - Constants::RearEdgeToCenter * sin(slotPosInfor_.fSlotAngle);
            fStopXMove = fObliTargetMoveDis * cos(slotPosInfor_.fSlotAngle);
            fStopYMove = fObliTargetMoveDis * sin(slotPosInfor_.fSlotAngle);

            slotPosInfor_.goalPointInSlot.x = fNormalStopX + fStopXMove;
            slotPosInfor_.goalPointInSlot.y = fNormalStopY + fStopYMove;
            slotPosInfor_.goalPointInSlot.yaw = normalizeHeadingRad_Npi_Ppi(slotPosInfor_.fSlotAngle - PI);
        }
        else if (slotPosInfor_.nParkingType == HEAD_PARK_OUT)
        {
            // if(slotPosInfor_.cParkOutDir == RIGHT_PARKING_OUT)
            // {
            //     slotPosInfor_.goalPointInSlot.x = LONG_DIS_TO_SLOT_HEAD;
            //     slotPosInfor_.goalPointInSlot.y = LAT_DIS_TO_SLOT_HEAD;
            //     slotPosInfor_.goalPointInSlot.yaw = 0;
            // }
            // else
            // {
            //     slotPosInfor_.goalPointInSlot.x = LONG_DIS_TO_SLOT_HEAD;
            //     slotPosInfor_.goalPointInSlot.y = - LAT_DIS_TO_SLOT_HEAD;
            //     slotPosInfor_.goalPointInSlot.yaw = 0;
            // }

            if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
            {
                slotPosInfor_.goalPointInSlot.x = OBLIQUE_HEAD_OUT_TAR_X;
                slotPosInfor_.goalPointInSlot.y = OBLIQUE_HEAD_OUT_TAR_Y;
                slotPosInfor_.goalPointInSlot.yaw = 0;
            }
            else
            {
                slotPosInfor_.goalPointInSlot.x = OBLIQUE_HEAD_OUT_TAR_X;
                slotPosInfor_.goalPointInSlot.y = -OBLIQUE_HEAD_OUT_TAR_Y;
                slotPosInfor_.goalPointInSlot.yaw = 0;
            }
        }
        else
        {
            if(slotPosInfor_.cParkOutDir == RIGHT_PARKING_OUT)
            {
                slotPosInfor_.goalPointInSlot.x = LONG_DIS_TO_SLOT_TAIL;
                slotPosInfor_.goalPointInSlot.y = - LAT_DIS_TO_SLOT_TAIL;
                slotPosInfor_.goalPointInSlot.yaw = 0;
            }
            else
            {
                slotPosInfor_.goalPointInSlot.x = LONG_DIS_TO_SLOT_TAIL;
                slotPosInfor_.goalPointInSlot.y = LAT_DIS_TO_SLOT_TAIL;
                slotPosInfor_.goalPointInSlot.yaw = 0;
            }
        }

    }

    slotPosInfor_.goalPointInSlot = slotPosInfor_.targetPointInSlot;//
    slotPosInfor_.goalPointInMap.x = slotPosInfor_.goalPointInSlot.x + slotPosInfor_.fXSlotPosInMap;
    slotPosInfor_.goalPointInMap.y = slotPosInfor_.goalPointInSlot.y + slotPosInfor_.fYSlotPosInMap;
    slotPosInfor_.goalPointInMap.yaw = normalizeHeadingRad_0_2pi(slotPosInfor_.goalPointInSlot.yaw);
    return true;
}

/**
 * @brief 设置平行车位的终点
 * @param slotPosInfor 车位输入信息
 * @return 1-设置成功 0-设置失败
 */
bool HybridAStar::SetParaSlotGoalPoint(slotPosInfor &slotPosInfor_)
{
    bool bSlotSizeSuccess = false;
    float fMinDis = Constants::para_min_slot_length - Constants::length;
    float fRearDis = 0.0f;
    float fFrontDis = 0.0f;
    float fXSlotCenter = 0.0f;
    LocationPoint tStep3KeyPoint;
    ParaSlotParameters ParaSlotParameters_;
    ParaSlotParameters_ = CalParaFrontAndRearMargin(slotPosInfor_);

    /*设置车位坐标系中车辆目标点位姿*/
    if(slotPosInfor_.nDetectType == USS_TYPE)
    {
        /*需要考虑车位是不是单边!!!!!!!!!!!*/
        if (slotPosInfor_.nUssSideType == FRONT_REAR_SIDE)
        {
            slotPosInfor_.goalPointInSlot.x = (slotPosInfor_.fXFrontSlotCorner - slotPosInfor_.fXRearSlotCorner - Constants::length) / 2 + Constants::RearEdgeToCenter + slotPosInfor_.fXRearSlotCorner;
        }
        else if (slotPosInfor_.nUssSideType == FRONT_SIDE)
        {
            slotPosInfor_.goalPointInSlot.x = slotPosInfor_.fXFrontSlotCorner - (Constants::length - Constants::RearEdgeToCenter + PARA_SINGLE_SIDE_SLOT_DIS);
        }
        else
        {
            slotPosInfor_.goalPointInSlot.x = Constants::RearEdgeToCenter + PARA_SINGLE_SIDE_SLOT_DIS + slotPosInfor_.fXRearSlotCorner;
        }

        slotPosInfor_.goalPointInSlot.y = ParaSlotParameters_.fParaTargetY * slotPosInfor_.nSlotSide;//(MIN_VEH_SIDE_TO_RIGHT_MARGIN + Constants::width / 2 + ParaSlotParameters_.fYParaRightMargin) * slotPosInfor_.nSlotSide;
        slotPosInfor_.goalPointInSlot.yaw = 0;
    }
    else
    {
        fXSlotCenter = (slotPosInfor_.fSlotLength - Constants::length) / 2 + Constants::RearEdgeToCenter;
        fRearDis = fXSlotCenter - Constants::RearEdgeToCenter - slotPosInfor_.fXRearSlotCorner;
        fFrontDis = slotPosInfor_.fXFrontSlotCorner - (fXSlotCenter + Constants::FrontEdgeToCenter);
        if(fRearDis >  fMinDis / 2)
        {
            if(fFrontDis > fMinDis / 2)
            {
              /*车位中心前后都比较安全，车辆可以停在车位中间*/
              slotPosInfor_.goalPointInSlot.x = fXSlotCenter;
            }
            else
            {
                /*车位中心后方空间大，前方空间很小，那么车辆停在车位中心靠后位置*/
                slotPosInfor_.goalPointInSlot.x = fXSlotCenter - (fMinDis / 2 - fFrontDis);
            }
        }
        else
        {
            if(fFrontDis > fMinDis / 2)
            {
              /*车位中心后方空间很小，前方空间大，那么车辆停在车位中心靠前位置*/
              slotPosInfor_.goalPointInSlot.x = fXSlotCenter + (fMinDis / 2 - fRearDis);
            }
            else
            {
                /*车位中心前后方空间都很小，那么车辆停在车位前后边界中心*/
                slotPosInfor_.goalPointInSlot.x = (slotPosInfor_.fXFrontSlotCorner - slotPosInfor_.fXRearSlotCorner - Constants::length) / 2 + Constants::RearEdgeToCenter;
                GAC_LOG_WARN("para slot size if invalid!!!");
            }
        }
        slotPosInfor_.goalPointInSlot.y = ParaSlotParameters_.fParaTargetY * slotPosInfor_.nSlotSide;
        slotPosInfor_.goalPointInSlot.yaw = 0;
    }
    
    slotPosInfor_.goalPointInSlot = slotPosInfor_.targetPointInSlot;//
    ParaSlotParameters_.fParaTargetY = slotPosInfor_.targetPointInSlot.y*slotPosInfor_.nSlotSide;

    if (!m_bStartPathplanningInParaSLot)
    {
        ParallelSlotPathPlanning_.CalStep3KeyPoint(ParaSlotParameters_.fXParaRearMargin,
                                                   ParaSlotParameters_.fYParaRightMargin,
                                                   ParaSlotParameters_.fXParaFrontMargin,
                                                   slotPosInfor_.nSlotSide *slotPosInfor_.fYFrontSlotCorner,
                                                   slotPosInfor_.goalPointInSlot.x,
                                                   ParaSlotParameters_.fParaTargetY,//MIN_VEH_SIDE_TO_RIGHT_MARGIN + Constants::width / 2 + ParaSlotParameters_.fYParaRightMargin,
                                                   bSlotSizeSuccess,
                                                   tStep3KeyPoint,
                                                   mParaSlotSteps); //notice
        if(!bSlotSizeSuccess)
        {
            planErrCode = ParallelSlotPathPlanning_.planErrCode;
            return false;
        }
        if(slotPosInfor_.nSlotSide == RIGHTSIDESLOT)
        {
            slotPosInfor_.ParkingInPointInSlot.x  = tStep3KeyPoint.x;
            slotPosInfor_.ParkingInPointInSlot.y  = tStep3KeyPoint.y;
            slotPosInfor_.ParkingInPointInSlot.yaw  = tStep3KeyPoint.yaw;
        }
        else
        {
            slotPosInfor_.ParkingInPointInSlot.x  = tStep3KeyPoint.x;
            slotPosInfor_.ParkingInPointInSlot.y  = -tStep3KeyPoint.y;
            slotPosInfor_.ParkingInPointInSlot.yaw  = -tStep3KeyPoint.yaw;
        }
        slotPosInfor_.goalPointInMap.x = slotPosInfor_.goalPointInSlot.x + slotPosInfor_.fXSlotPosInMap;
        slotPosInfor_.goalPointInMap.y = slotPosInfor_.goalPointInSlot.y + slotPosInfor_.fYSlotPosInMap;
        slotPosInfor_.goalPointInMap.yaw = normalizeHeadingRad_0_2pi(slotPosInfor_.goalPointInSlot.yaw);

        slotPosInfor_.ParkingInPointInMap.x = slotPosInfor_.ParkingInPointInSlot.x + slotPosInfor_.fXSlotPosInMap;
        slotPosInfor_.ParkingInPointInMap.y = slotPosInfor_.ParkingInPointInSlot.y + slotPosInfor_.fYSlotPosInMap;
        slotPosInfor_.ParkingInPointInMap.yaw = normalizeHeadingRad_0_2pi(slotPosInfor_.ParkingInPointInSlot.yaw);
    }

    if(slotPosInfor_.nParkingType == HEAD_PARK_OUT)
    {
        slotPosInfor_.goalPointInMap.x = slotPosInfor_.goalPointInSlot.x + slotPosInfor_.fXSlotPosInMap;
        slotPosInfor_.goalPointInMap.y = slotPosInfor_.goalPointInSlot.y + slotPosInfor_.fYSlotPosInMap;
        slotPosInfor_.goalPointInMap.yaw = normalizeHeadingRad_0_2pi(slotPosInfor_.goalPointInSlot.yaw);

        slotPosInfor_.ParkingInPointInMap.x = slotPosInfor_.goalPointInSlot.x + slotPosInfor_.fXSlotPosInMap;
        slotPosInfor_.ParkingInPointInMap.y = slotPosInfor_.goalPointInSlot.y + slotPosInfor_.fYSlotPosInMap;
        slotPosInfor_.ParkingInPointInMap.yaw = normalizeHeadingRad_0_2pi(slotPosInfor_.goalPointInSlot.yaw);
    }
    return true;
}

/**
 * @brief 设置起点和目标点
 * @param slotPosInfor 车位输入信息
 * @return 1-设置成功 0-设置失败
 */
bool HybridAStar::SetStartPointAndGoalPoint(slotPosInfor &slotPosInfor_)
{
    bool bSetSuccess = false;
    Vector2D StartPoint;
    Vector2D GoalPoint;

    /*设置3D网格坐标系中车辆起点位姿*/
    slotPosInfor_.startPointInMap.x = slotPosInfor_.startPointInSlot.x + slotPosInfor_.fXSlotPosInMap;
    slotPosInfor_.startPointInMap.y = slotPosInfor_.startPointInSlot.y + slotPosInfor_.fYSlotPosInMap;
    slotPosInfor_.startPointInMap.yaw = normalizeHeadingRad_0_2pi(slotPosInfor_.startPointInSlot.yaw);

    /*设置3D网格坐标系中车辆目标点位姿*/
    if(slotPosInfor_.nSlotType == OBLIQUE) //vertical parking
    {
        bSetSuccess = SetObliSlotGoalPoint(slotPosInfor_);
    }
    else if(slotPosInfor_.nSlotType == PARALLEL) //parallel parking
    {
        ParallelSlotPathPlanning_.InitMapAndSlotInfor(occupyMap, slotPosInfor_);
        GAC_LOG_INFO("车位可利用长度：%f m\n", m_slotPosInfor.fXFrontSlotCorner - m_slotPosInfor.fXRearSlotCorner - Constants::length);
        if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
        {
            bSetSuccess = SetParaSlotGoalPoint(slotPosInfor_);
        }
        else
        {
            bSetSuccess = true;
        }
        bSetSuccess = SetParaSlotGoalPoint(slotPosInfor_);
    }

    /*设置起点和终点的单位向量*/
    StartPoint.setXY(cos(slotPosInfor_.startPointInMap.yaw), sin(slotPosInfor_.startPointInMap.yaw));
    GoalPoint.setXY(cos(slotPosInfor_.goalPointInMap.yaw), sin(slotPosInfor_.goalPointInMap.yaw));
    /*叉乘大于0，逆时针，小于0顺时针*/
    slotPosInfor_.fCrossValueStartAndGoalPoint = StartPoint.cross(GoalPoint);

    return bSetSuccess;
}

/**
 * @brief hybrid A*规划的主函数，负责根据初始点和目标点规划一条满足车辆动力学约束、和障碍物无碰撞、曲率连续、速度连续的路径
 * @param[in] hybridAResults hybrid A*的搜索结果
 * @param[in] partitionedCtrlPointSets 对hybrid A*根据行驶方向进行片段分割，优化、插值后的控制点集合
 * @return nPlanningFlag 0-规划结束 1-规划成功 2 -规划失败
 * @note 不包含平行泊车库内调整轨迹搜索
 */
PlanningStatus HybridAStar::PathPlanning(const ulong nTotalGearSwitchTimes,
                               std::vector<Node3D>& hybridAResults,
                               std::vector<std::vector<CtrlPoint> >& partitionedCtrlPointSets)
{
    bool bPlanSuccess =false;
    int nMaxHybridAStarTimes = 0;
    LocationPoint goal;
    PlanningStatus nPlanningFlag = PlanningFail;/*0-规划结束 1-规划成功 2 -规划失败*/

    partitionedCtrlPointSets.clear();
    hybridAResults.clear();

    /*更新配置空间*/
    m_configurationSpace.updateGrid(occupyMap);

    /*node 3D网格初始化*/
    for(int i = 0; i < NODE3D_NUM; i++)
    {
        gNodes3D[i].Init();
    }

    if(m_slotPosInfor.nSlotType == OBLIQUE)
    {
        goal.x = m_slotPosInfor.goalPointInMap.x;
        goal.y = m_slotPosInfor.goalPointInMap.y;
        goal.yaw = m_slotPosInfor.goalPointInMap.yaw;
    }
    else
    {
        goal.x = m_slotPosInfor.ParkingInPointInMap.x;
        goal.y = m_slotPosInfor.ParkingInPointInMap.y;
        goal.yaw = normalizeHeadingRad_0_2pi(m_slotPosInfor.ParkingInPointInMap.yaw);
    }

    /*起点和终点*/
    Node3D nStart(m_slotPosInfor.startPointInMap.x, m_slotPosInfor.startPointInMap.y,
                  m_slotPosInfor.startPointInMap.yaw,
                  0, 0, nullptr,
                  m_nDrivingDirection ,true);
    Node3D nGoal(goal.x, goal.y, goal.yaw, 0, 0, nullptr);
    /*hybrid A Star路径搜索主程序*/
    bPlanSuccess = plan(nStart, nGoal, gNodes3D,
                        m_configurationSpace,
                        hybridAResults,
                        partitionedCtrlPointSets);

    /*根据已经完成的换挡次数和将要规划的轨迹片段数目判断是否超声正常泊车的最大泊车次数*/
    if(m_slotPosInfor.nSlotType == PARALLEL)
    {
        nMaxHybridAStarTimes = MAX_SEARCH_TIMES_PARA;
    }
    else
    {
        nMaxHybridAStarTimes = MAX_SEARCH_TIMES_VERTICAL;
    }
    if(nTotalGearSwitchTimes + partitionedCtrlPointSets.size() > nMaxHybridAStarTimes)
    {
        /*E10007两次规划轨迹段数差距太大*/
        planErrCode=E10007;

        bPlanSuccess = false;
        GAC_LOG_DEBUG("Exceed Max parking adjust times is %ld\n", partitionedCtrlPointSets.size());
        partitionedCtrlPointSets.clear();
        hybridAResults.clear();
    }

    /*输出控制点*/
    if(bPlanSuccess)
    {
        for(size_t j = 0; j < partitionedCtrlPointSets.size(); ++j)
        {
            for(size_t i = 0; i < partitionedCtrlPointSets[j].size(); ++i)
            {
                partitionedCtrlPointSets[j][i].x -= m_slotPosInfor.fXSlotPosInMap;
                partitionedCtrlPointSets[j][i].y -= m_slotPosInfor.fYSlotPosInMap;
                partitionedCtrlPointSets[j][i].yaw = normalizeHeadingRad_Npi_Ppi(partitionedCtrlPointSets[j][i].yaw);
            }
        }
        for(size_t i = 0; i < hybridAResults.size();i++)
        {
            hybridAResults[i].x = hybridAResults[i].x - m_slotPosInfor.fXSlotPosInMap;
            hybridAResults[i].y = hybridAResults[i].y - m_slotPosInfor.fYSlotPosInMap;
        }
    }

#if 1
    m_configurationSpace.deleteCurrentVehiclePositionOccupyMap(nGoal.x, nGoal.y, nGoal.t);
    m_configurationSpace.deleteCurrentVehiclePositionOccupyMap(nStart.x, nStart.y, nStart.t);
    int nColumn, nRow = 0;
    for(int i = 0; i < Constants::ob_grid_num; i++)
    {
        nRow = int(i / Constants::ob_grid_width);
        nColumn = i - nRow * Constants::ob_grid_width;
        occupyMap[i]= NO_FREE*m_configurationSpace.getGrid()[(Constants::ob_grid_height - nRow) * Constants::ob_grid_width + nColumn];
    }
#endif
    if(bPlanSuccess)
    {
        nPlanningFlag = PlanningSucc;
    }
    else
    {
        nPlanningFlag = PlanningFail;
    }

    return nPlanningFlag;
}

/**
 * @brief 给轨迹点赋测量状态
 * @param[in] aTracks 轨迹点
 * @return 0
 * @note
 */
int HybridAStar::AddMeasureStatusInTargetTrack(std::vector<TargetTrack>& aTracks)
{
    /*给轨迹点测量状态赋初值0*/
    for(size_t i = 0; i < aTracks.size(); ++i)
    {
        aTracks[i].nDetectStates = 0;
    }

    if(m_slotPosInfor.nDetectType == USS_TYPE)
    {
        // if (m_slotPosInfor.nSlotType == OBLIQUE)
        // {
        //     if(m_bLastDriving)
        //     {
        //         /*给超声垂直泊车最后一段轨迹点赋测量状态*/
        //          AddMeasureStatusInObliUSSSlot(m_slotPosInfor, aTracks);
        //     }else{}
        // }
        // else if (m_slotPosInfor.nSlotType == PARALLEL)
        // {
        //     if((m_ParaHybridALastDriving||m_bStartPathplanningInParaSLot)&&(m_nDrivingDirection == BACKWARD)&& (!m_bUSSHasDetectRearMargin))
        //     {
        //         /*给平行超声泊车最后一段hybrid A*轨迹点和车库内向后行驶赋测量状态*/
        //         m_bUSSHasDetectRearMargin = AddMeasureStatusInParaUssSlotWhenHybridALastDriving(m_slotPosInfor, aTracks);
        //     }
        //     else if (m_bStartPathplanningInParaSLot&&(m_nDrivingDirection == FORWARD) && (!m_bUSSHasDetectFrontMargin))
        //     {
        //         /*给平行超声泊车库内调整往前轨迹点赋测量状态*/
        //         m_bUSSHasDetectFrontMargin = AddMeasureStatusInParaUssSlotWhenForwardDriving(m_slotPosInfor, aTracks);
        //     }
        //     else
        //     {

        //     }
        // }else{}
    }
    else if(m_slotPosInfor.nDetectType== MIX_TYPE)
    {
        if (m_slotPosInfor.nSlotType == OBLIQUE)
        {
            if(m_bLastDriving)
            {
                /*给视觉划线车位垂直泊车最后一段轨迹点赋测量状态*/
                AddMeasureStatusInObliVisionSlot(m_slotPosInfor, aTracks);
            }else{}
        }
        else if (m_slotPosInfor.nSlotType == PARALLEL)
        {
            if((m_ParaHybridALastDriving||m_bStartPathplanningInParaSLot)&&(m_nDrivingDirection == BACKWARD) && (!m_bUSSHasDetectRearMargin))
            {
                /*给平行视觉泊车最后一段hybrid A*轨迹点赋测量状态和车库内向后行驶赋测量状态*/
                m_bUSSHasDetectRearMargin = AddMeasureStatusInParaUssSlotWhenHybridALastDriving(m_slotPosInfor, aTracks);
            }
            else if (m_bStartPathplanningInParaSLot&&(m_nDrivingDirection == FORWARD) && (!m_bUSSHasDetectFrontMargin))
            {
                /*给平行视觉泊车库内调整往前轨迹点赋测量状态*/
                m_bUSSHasDetectFrontMargin = AddMeasureStatusInParaUssSlotWhenForwardDriving(m_slotPosInfor, aTracks);
            }
            else
            {

            }
        }else{}
    }else{}

    return 0;
}


ParaSlotParameters HybridAStar::CalParaFrontAndRearMargin(const slotPosInfor &slotPosInfor_)
{
    float NewSlotWidth = 0.0f;
    float fMarginLeft = 0.0f;
    float fMarginRear = 0.2f;
    float fMarginFront = 0.2f;
    ParaSlotParameters ParaSlotParameters_;

    if(slotPosInfor_.nDetectType == USS_TYPE)
    {
        fMarginRear = USS_MIN_MARGIN;
        fMarginFront = USS_MIN_MARGIN;
    }
    else
    {
        fMarginRear = VISION_MIN_MARGIN;
        fMarginFront = VISION_MIN_MARGIN;
    }
    if (slotPosInfor_.nSlotType == PARALLEL)
    {
        fMarginRear += 0.1f;
        fMarginFront += 0.1f;
    }
    ParallelSlotPathPlanning_.CalLeftMarginAndSlotWidth(slotPosInfor_.fSlotLength - fMarginRear - fMarginFront,slotPosInfor_.fSlotWidth, fMarginLeft, NewSlotWidth);
    ParaSlotParameters_.fXParaRearMargin = slotPosInfor_.fXRearSlotCorner + fMarginRear;
    ParaSlotParameters_.fXParaFrontMargin = slotPosInfor_.fXFrontSlotCorner - fMarginFront;
    ParaSlotParameters_.fYParaRightMargin = -NewSlotWidth;

    if(NewSlotWidth/2.f - Constants::width/2.f > PARA_INSIDE_MIN_DIS)
    {
        ParaSlotParameters_.fParaTargetY = NewSlotWidth/-2.f;//max(NewSlotWidth/-2.f, Constants::width/-2.f - 0.2f);
    }
    else
    {
        ParaSlotParameters_.fParaTargetY = -NewSlotWidth + PARA_INSIDE_MIN_DIS + Constants::width/2.f;
    }
    if(slotPosInfor_.nDetectType == USS_TYPE)
    {
        ParaSlotParameters_.fParaTargetY = MAX(-0.1f - Constants::width/2.f, ParaSlotParameters_.fParaTargetY);
        ParaSlotParameters_.fYParaRightMargin = MAX(Constants::width*(-1.f) - 0.4f, ParaSlotParameters_.fYParaRightMargin);
    }
    return ParaSlotParameters_;

}

/**
 * @brief 平行泊车库内轨迹规划入口
 * @param nDrivingDirection 下一次车辆行驶方向
 * @param startPointInSlot 起点
 * @param goalPointInSlot 终点
 * @param PartitionedTargetCurvatureSets 规划路径
 * @return nPlanningFlag 0-规划结束 1-规划成功 2 -规划失败
 */
PlanningStatus HybridAStar::ParkingInPathPlanningInParaSlot(int8_t nDrivingDirection,
                                                            LocationPoint startPointInSlot,
                                                            LocationPoint goalPointInSlot,
                                                            std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets)
{
    LocationPoint ReveseStartPoint = startPointInSlot;
    LocationPoint ReveseGoalPoint = goalPointInSlot;

    bool bPlanSuccess = false;
    PlanningStatus nPlanningFlag = PlanningFail;/*0-规划结束 1-规划成功 2 -规划失败*/

    PartitionedTargetCurvatureSets.clear();
    ParaSlotParameters ParaSlotParameters_;
    ParaSlotParameters_ = CalParaFrontAndRearMargin(m_slotPosInfor);

    if(m_slotPosInfor.nSlotSide == LEFTSIDESLOT)
    {
        ReveseStartPoint.y = -startPointInSlot.y;
        ReveseStartPoint.yaw = -startPointInSlot.yaw;
        ReveseGoalPoint.y = -goalPointInSlot.y;
        ReveseGoalPoint.yaw = -goalPointInSlot.yaw;
    }

    if(fabsf(ReveseStartPoint.yaw) <= 3.0f*PI/180)
    {
        /*强制改变行驶方向*/
        if(ReveseStartPoint.x - ReveseGoalPoint.x > 0.2f)
        {
            nDrivingDirection = BACKWARD;
        }
        else if(ReveseStartPoint.x - ReveseGoalPoint.x < -0.2f)
        {
            nDrivingDirection = FORWARD;
        }
    }

    if(nDrivingDirection == 0)
    {
        return nPlanningFlag;
    }

    if(ReveseStartPoint.y > 0)
    {
        /*平行泊车库内重规划输入y大于0*/
        planErrCode=E20003;
        bPlanSuccess = false;
        GAC_LOG_ERROR("input error in PathPlanningInParallelSlot!!!\n");
    }
    else
    {
        bPlanSuccess= ParallelSlotPathPlanning_.PathPlanningInParallelSlot(m_slotPosInfor.nSlotSide,
                                                                           ParaSlotParameters_.fXParaRearMargin,
                                                                           ParaSlotParameters_.fXParaFrontMargin,
                                                                           ParaSlotParameters_.fYParaRightMargin,
                                                                           ReveseStartPoint,
                                                                           ReveseGoalPoint.x,
                                                                           ReveseGoalPoint.y,
                                                                           nDrivingDirection,
                                                                           PartitionedTargetCurvatureSets);
    }

    if(bPlanSuccess)
    {
        nPlanningFlag = PlanningSucc;
    }
    else
    {
        /*E10010平行泊车库内重规划失败*/
        planErrCode = ParallelSlotPathPlanning_.planErrCode;
        nPlanningFlag = PlanningFail;
    }

    return nPlanningFlag;
}


/**
 * @brief 平行泊车出库规划入口
 * @param nDrivingDirection 下一次车辆行驶方向
 * @param startPointInSlot 起点
 * @param PartitionedTargetCurvatureSets 规划路径
 * @return nPlanningFlag 0-规划结束 1-规划成功 2 -规划失败
 */
PlanningStatus HybridAStar::ParkingOutPathPlanningInParaSlot(int8_t nDrivingDirection,
                                                             LocationPoint startPointInSlot,
                                                             std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets)
{
    PlanningStatus nPlanningFlag = PlanningFail;/*0-规划结束 1-规划成功 2 -规划失败*/
    LocationPoint ReveseStartPoint = startPointInSlot;
    ParaSlotParameters ParaSlotParameters_;

    PartitionedTargetCurvatureSets.clear();
    ParaSlotParameters_ = CalParaFrontAndRearMargin(m_slotPosInfor);

    if(m_slotPosInfor.nSlotSide == LEFTSIDESLOT)
    {
        ReveseStartPoint.x = startPointInSlot.x;
        ReveseStartPoint.y = -startPointInSlot.y;
        ReveseStartPoint.yaw = -startPointInSlot.yaw;
    }

    if(ReveseStartPoint.y > 0)
    {
        GAC_LOG_ERROR("input error in ParkingOutPathPlanningInParaSlot!!!\n");
    }
    else
    {
        nPlanningFlag = ParallelSlotPathPlanning_.ParkingOutInParallelSlot(m_slotPosInfor.nSlotSide,
                                                                           ParaSlotParameters_.fXParaRearMargin,
                                                                           ParaSlotParameters_.fXParaFrontMargin,
                                                                           ParaSlotParameters_.fYParaRightMargin,
                                                                           ReveseStartPoint,
                                                                           nDrivingDirection,
                                                                           m_bFirstPlan,
                                                                           PartitionedTargetCurvatureSets);
    }

    return nPlanningFlag;

}


PlanningStatus HybridAStar::ParkingOutPathPlanningInVertSlot(int8_t nDrivingDirection,
                                                             LocationPoint startPointInSlot,
                                                             std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets)
{
    PlanningStatus nPlanningFlag = planningOver;/*0-规划结束 1-规划成功 2 -规划失败*/
    LocationPoint ReveseStartPoint = startPointInSlot;
    ParaSlotParameters ParaSlotParameters_;
    std::vector<CtrlPoint> TargetSeg;

    PartitionedTargetCurvatureSets.clear();
    ParaSlotParameters_ = CalParaFrontAndRearMargin(m_slotPosInfor);

    if(m_slotPosInfor.nSlotSide == LEFTSIDESLOT)
    {
        ReveseStartPoint.x = startPointInSlot.x;
        ReveseStartPoint.y = -startPointInSlot.y;
        ReveseStartPoint.yaw = -startPointInSlot.yaw;
    }

    if(ReveseStartPoint.y > 0)
    {
        /*垂直泊车泊出输入y大于0*/
        GAC_LOG_ERROR("input error in ParkingOutPathPlanningInParaSlot!!!\n");
    }
    else
    {
        if(m_slotPosInfor.nSlotSide == LEFTSIDESLOT)
        {
            ClothoidVerticalParkout(ReveseStartPoint, ParaSlotParameters_.fXParaFrontMargin, 0.0f,
                                    ParaSlotParameters_.fXParaRearMargin, 0.0f, -m_slotPosInfor.fSlotAngle, TargetSeg);
            for(size_t i = 0; i < TargetSeg.size(); i++)
            {
                TargetSeg[i].y = -TargetSeg[i].y;
                TargetSeg[i].yaw = normalizeHeadingRad_Npi_Ppi(-TargetSeg[i].yaw);
                TargetSeg[i].kappa = -TargetSeg[i].kappa;
            }
        }
        else
        {
            ClothoidVerticalParkout(ReveseStartPoint, ParaSlotParameters_.fXParaFrontMargin, 0.0f,
                                    ParaSlotParameters_.fXParaRearMargin, 0.0f, m_slotPosInfor.fSlotAngle, TargetSeg);
        }

        if (!TargetSeg.empty())
        {
            PartitionedTargetCurvatureSets.push_back(TargetSeg);
            nPlanningFlag = PlanningSucc;
        }
    }

    return nPlanningFlag;
}


/**
 * @brief 已知坐标系B相对于坐标系A的旋转坐标，将B坐标系中的轨迹转换成A中的轨迹
 * @param RotationCoordinate 坐标系B相对于坐标系A的旋转坐标
 * @param aTracks B坐标系中的轨迹转换成A中的轨迹
 */
void HybridAStar::GetFinalTracks(LocationPoint RotationCoordinate,
                                   const std::vector<std::vector<CtrlPoint> >& Segments,
                                   std::vector<TargetTrack> &aTracks)
{
    float fDeltaX = RotationCoordinate.x;
    float fDeltaY = RotationCoordinate.y;
    float fRotationYaw = RotationCoordinate.yaw;

    CtrlPoint  CtrlPoint_;
    TargetTrack TargetTrack_;
    LocationPoint tRotationCenter(0.0, 0.0, 0.0);
    LocationPoint tRotationPoint(0.0, 0.0, 0.0);
    LocationPoint tRotationNewPoint(0.0, 0.0, 0.0);
    std::vector<CtrlPoint> targetCtrlPointSets;

    aTracks.clear();

    if(!Segments.empty())
    {
        for(size_t i = 0; i < Segments[0].size(); ++i)
        {
            CtrlPoint_.veh_speed = 0.0;
            CtrlPoint_.kappa = 0.0;
            CtrlPoint_.driving_direction = Segments[0][i].driving_direction;

            tRotationPoint.x = Segments[0][i].x;
            tRotationPoint.y = Segments[0][i].y;
            RotateCoordinateOfPoint(tRotationCenter, tRotationPoint, fRotationYaw, tRotationNewPoint);
            CtrlPoint_.x  = tRotationNewPoint.x + fDeltaX;
            CtrlPoint_.y  = tRotationNewPoint.y + fDeltaY;

            targetCtrlPointSets.push_back(CtrlPoint_);
        }
        CalDiscretePointCurvature(targetCtrlPointSets);

        for(size_t i = 0; i < targetCtrlPointSets.size(); i++)
        {
            if(fabsf(targetCtrlPointSets[i].kappa) > Constants::max_steering_curvature)
            {
                targetCtrlPointSets[i].kappa = SgnCus(targetCtrlPointSets[i].kappa)*Constants::max_steering_curvature;
            }else{}
        }

        VelocityPlanning(targetCtrlPointSets);

        for(size_t i = 0;i < targetCtrlPointSets.size(); ++i)
        {
            TargetTrack_.point.setX(targetCtrlPointSets[i].x);
            TargetTrack_.point.setY(targetCtrlPointSets[i].y);
            TargetTrack_.yaw = targetCtrlPointSets[i].yaw;
            TargetTrack_.curvature = targetCtrlPointSets[i].kappa;
            TargetTrack_.speed = targetCtrlPointSets[i].veh_speed;
            TargetTrack_.nDetectStates = 0;
            aTracks.push_back(TargetTrack_);
        }
    }
}

/**
 * @brief 已知坐标系B相对于坐标系A的旋转坐标，将B坐标系中的轨迹转换成A中的轨迹
 * @param RotationCoordinate 坐标系B相对于坐标系A的旋转坐标
 * @param HybridAResults B坐标系中的轨迹转换成A中的轨迹
 */
void HybridAStar::RotateHybridAResults(LocationPoint RotationCoordinate, std::vector<Node3D>& HybridAResults)
{
    float fDeltaX = RotationCoordinate.x;
    float fDeltaY = RotationCoordinate.y;
    float fRotationYaw = RotationCoordinate.yaw;

    LocationPoint tRotationCenter(0, 0 ,0);
    LocationPoint tRotationPoint(0, 0 ,0);
    LocationPoint tRotationNewPoint(0,0,0);

    for(size_t i = 0; i < HybridAResults.size(); ++i)
    {
        tRotationPoint.x = HybridAResults[i].x;
        tRotationPoint.y = HybridAResults[i].y;
        RotateCoordinateOfPoint(tRotationCenter, tRotationPoint, fRotationYaw, tRotationNewPoint);

        tRotationNewPoint.x += fDeltaX;

        tRotationNewPoint.y += fDeltaY;

        HybridAResults[i].x = tRotationNewPoint.x;
        HybridAResults[i].y = tRotationNewPoint.y;
    }
}

/**
 * @brief 已知坐标系B相对于坐标系A的旋转坐标，将B坐标系中的轨迹转换成A中的轨迹
 * @param RotationCoordinate 坐标系B相对于坐标系A的旋转坐标
 * @param SegmentSets B坐标系中的轨迹转换成A中的轨迹
 */
void HybridAStar::RotateSegmentSets(LocationPoint RotationCoordinate, std::vector<std::vector<CtrlPoint> >& SegmentSets)
{
    float fDeltaX = RotationCoordinate.x;
    float fDeltaY = RotationCoordinate.y;
    float fRotationYaw = RotationCoordinate.yaw;

    LocationPoint tRotationCenter(0, 0 ,0);
    LocationPoint tRotationPoint(0, 0 ,0);
    LocationPoint tRotationNewPoint(0,0,0);

    for(size_t i = 0; i < SegmentSets.size(); ++i)
    {
        for(size_t j = 0; j < SegmentSets[i].size(); ++j)
        {

            tRotationPoint.x = SegmentSets[i][j].x;
            tRotationPoint.y = SegmentSets[i][j].y;
            RotateCoordinateOfPoint(tRotationCenter, tRotationPoint, fRotationYaw, tRotationNewPoint);

            tRotationNewPoint.x += fDeltaX;

            tRotationNewPoint.y += fDeltaY;

            SegmentSets[i][j].x = tRotationNewPoint.x;
            SegmentSets[i][j].y = tRotationNewPoint.y;
        }
    }
}

/**
 * @brief 判断是否能启动平行车库内的轨迹规划
 * @param nLastPlanningPathSegNum 上次规划的hybrid A轨迹段数
 * @param slotPosInfor_ 车位信息
 * @return 1-启动平行库内的轨迹规划，0-不启动
 */
bool HybridAStar::CheckIfStartPathPlanningInParaSlot(bool bParaHybridALastDriving,
                                                     const slotPosInfor &slotPosInfor_)
{
    bool bStart = false;
    bool bVehInSlot = false;
    int8_t nDrivingDirection = m_nDrivingDirection;
    float fVehStopy = 0.0;
    PlanningStatus nPlanningFlag = PlanningFail;/*0-规划结束 1-规划成功 2 -规划失败*/
    std::vector<std::vector<CtrlPoint> > PartitionedTargetCurvatureSets;

    bVehInSlot = slotPosInfor_.nSlotSide  * slotPosInfor_.startPointInSlot.y < -0.3f;
    if((slotPosInfor_.nSlotType == PARALLEL))
    {
        if(!bVehInSlot)
        {
            return false;
        }

        if(bParaHybridALastDriving)
        {
            bStart = true;
        }
        else
        {
            /*如果不是hybrid A*连接的最后一段，尝试用库内调整连接*/
            nPlanningFlag = ParkingInPathPlanningInParaSlot(-nDrivingDirection,
                                                            slotPosInfor_.startPointInSlot,
                                                            slotPosInfor_.goalPointInSlot,
                                                            PartitionedTargetCurvatureSets);
            if(nPlanningFlag == PlanningSucc)
            {
                fVehStopy = PartitionedTargetCurvatureSets.back().back().y;
                if(fabs(fVehStopy - (slotPosInfor_.goalPointInSlot.y)) < 0.1f)
                {
                    bStart = true;
                }
            }
        }
    }
    else
    {
        bStart = false;
    }
    return bStart;
}

bool HybridAStar::JudgeReachFixedGoalPoint(const slotPosInfor &slotPosInfor_)
{
    bool bReach = false;
    float fLateralError = 0.0;
    cpoint vehPos;
    if(slotPosInfor_.nSlotType == OBLIQUE)
    {
        if (fabsf(slotPosInfor_.startPointInSlot.yaw - slotPosInfor_.goalPointInSlot.yaw) < OBLI_STOP_YAW_ERROR)
        {
            vehPos.x = slotPosInfor_.startPointInMap.x;
            vehPos.y = slotPosInfor_.startPointInMap.y;
            fLateralError = CalPointAndLineDis(vehPos, slotPosInfor_.LineEq_Goal);
            if (fLateralError < OBLI_STOP_X_ERROR)
            {
                if (fabsf(slotPosInfor_.startPointInSlot.y - slotPosInfor_.goalPointInSlot.y) < OBLI_STOP_Y_ERROR)
                {
                    bReach = true;
                }
                else if((slotPosInfor_.nSlotSide * (slotPosInfor_.startPointInSlot.y - slotPosInfor_.goalPointInSlot.y) < 0))
                {
                    bReach = true;
                }
            }

        }
        if(m_bLastDriving)
        {
            if (fabsf(slotPosInfor_.startPointInSlot.y - slotPosInfor_.goalPointInSlot.y) < 1.f)//
            {
                bReach = true;
            }
        }
    }
    else
    {
        if(slotPosInfor_.nParkingType == TAIL_PARK_IN)
        {
            if (fabsf(slotPosInfor_.startPointInSlot.yaw - slotPosInfor_.goalPointInSlot.yaw) < PARA_STOP_YAW_ERROR)
            {
                if(fabsf(slotPosInfor_.startPointInSlot.x - slotPosInfor_.goalPointInSlot.x) < PARA_STOP_X_ERROR
                        && fabsf(slotPosInfor_.startPointInSlot.y - slotPosInfor_.goalPointInSlot.y) < PARA_STOP_Y_ERROR)
                {
                    bReach = true;
                }
            }
        }else{}
    }

    return bReach;
}

bool HybridAStar::JudgeVehReachGoalPoint(const slotPosInfor &slotPosInfor_)
{
    bool bReach = false;
    
    if (slotPosInfor_.nParkingType == HEAD_PARK_OUT || slotPosInfor_.nParkingType == REMOTE_IN_OUT)
    {
        if(m_bLastDriving)
        {
            bReach = true;
        }
    }
    else
    {
        bReach = JudgeReachFixedGoalPoint(slotPosInfor_);
    }

    return bReach;
}

PlanningStatus HybridAStar::PathPlanningByMode(ParkingPlanningMode PlanningMode)
{
    PlanningStatus nPlanningFlag = PlanningFail;/*0-规划结束 1-规划成功 2 -规划失败*/
    if(PlanningMode == HybridAStarMode)
    {
        struct timeval tpstartValidCheck,tpendValidCheck;
        float timeuseValidCheck;
        gettimeofday(&tpstartValidCheck,NULL);

        nPlanningFlag = PathPlanning(m_TotalGearSwitchTimes,
                                     m_HybridAResults,
                                     m_PartitionedTargetCurvatureSets);
        gettimeofday(&tpendValidCheck,NULL);
        timeuseValidCheck=(1000000*(tpendValidCheck.tv_sec-tpstartValidCheck.tv_sec) + tpendValidCheck.tv_usec-tpstartValidCheck.tv_usec)/1000.0;

        GAC_LOG_DEBUG("hybrid a search time is [%d] ms\n", timeuseValidCheck);
    }
    else if(PlanningMode == ParaParkingInMode)
    {
        nPlanningFlag = ParkingInPathPlanningInParaSlot(-m_nDrivingDirection,
                                                        m_slotPosInfor.startPointInSlot,
                                                        m_slotPosInfor.goalPointInSlot,
                                                        m_PartitionedTargetCurvatureSets);
    }
    else if(PlanningMode == ParaParkingOutMode)
    {
        if (m_slotPosInfor.nSlotType == PARALLEL)
        {
            nPlanningFlag = ParkingOutPathPlanningInParaSlot(-m_nDrivingDirection,
                                                             m_slotPosInfor.startPointInSlot,
                                                             m_PartitionedTargetCurvatureSets);
        }
        else
        {
            nPlanningFlag = ParkingOutPathPlanningInVertSlot(-m_nDrivingDirection,
                                                             m_slotPosInfor.startPointInSlot,
                                                             m_PartitionedTargetCurvatureSets);
        }

    }
    else
    {
        nPlanningFlag = RemoteInOutParkingPathPlanning(-m_nDrivingDirection,
                                                       m_slotPosInfor.startPointInSlot,
                                                       m_PartitionedTargetCurvatureSets);
    }

    return  nPlanningFlag;
}


/**
 * @brief 判断泊车规划模式
 * @return   0-hybridA*规划, 1-平行泊入规划, 2-平行泊出规划
 */
ParkingPlanningMode HybridAStar::JudgeParkingPlanningMode(void)
{
    ParkingPlanningMode PlanningMode = HybridAStarMode;

    if(m_slotPosInfor.nParkingType == REMOTE_IN_OUT)
    {
        PlanningMode = VerticalRemoteInOut;
        return PlanningMode;
    }
    if (m_slotPosInfor.nSlotType == PARALLEL)
    {
        if(m_slotPosInfor.nParkingType == TAIL_PARK_IN)
        {
            if(!m_bStartPathplanningInParaSLot)
            {
                m_bStartPathplanningInParaSLot = CheckIfStartPathPlanningInParaSlot(m_ParaHybridALastDriving, m_slotPosInfor);
            }
            if(m_bStartPathplanningInParaSLot)
            {
                PlanningMode = ParaParkingInMode;
            }
            else
            {
                m_ParaHybridALastDriving = false;
                PlanningMode = HybridAStarMode;
            }
        }
        else if(m_slotPosInfor.nParkingType == HEAD_PARK_OUT)
        {
            PlanningMode = ParaParkingOutMode;
            if(ifStartSearchPlanningParaOut(m_slotPosInfor))
            {
                PlanningMode = HybridAStarMode;
            }
        }else{}
    }
    else
    {
        if(m_slotPosInfor.nParkingType == TAIL_PARK_IN || m_slotPosInfor.nParkingType == HEAD_PARK_IN)
        {
            PlanningMode = HybridAStarMode;
        }
        else
        {
            PlanningMode = HybridAStarMode;
            if(m_bFirstPlan)
            {
            	PlanningMode = ParaParkingOutMode;
            }
        }
    }

    return PlanningMode;
}


/**
 * @brief 路径规划后的后处理
 * @param aTracks 发给控制模块的轨迹点
 */
void HybridAStar::PostProcessingTaskAfterPlanningSuccess(std::vector<TargetTrack>& aTracks)
{
    std::vector<std::vector<CtrlPoint> > PartitionedTargetCurvatureSets;
    /*0 判断是不是最后一段hybrid A*规划*/
    if((m_PartitionedTargetCurvatureSets.size() == 1)
        && (m_slotPosInfor.nSlotType == PARALLEL)
        && (!m_bStartPathplanningInParaSLot))
    {
        m_ParaHybridALastDriving = true;
    }else{}

    /*1 对于平行泊车入库，判断需不需要在hybridA*之后加轨迹*/
    if((m_slotPosInfor.nSlotType == PARALLEL )
        && (m_slotPosInfor.nParkingType == TAIL_PARK_IN)
        && (!m_bStartPathplanningInParaSLot))
    {
        PartitionedTargetCurvatureSets.clear();
        ParkingInPathPlanningInParaSlot(FORWARD,
                                        m_slotPosInfor.ParkingInPointInSlot,
                                        m_slotPosInfor.goalPointInSlot,
                                        PartitionedTargetCurvatureSets);
        m_PartitionedTargetCurvatureSets.insert(m_PartitionedTargetCurvatureSets.end(),
                                                PartitionedTargetCurvatureSets.begin(),
                                                PartitionedTargetCurvatureSets.end());
    }
    /*2 判断是不是最后一段规划轨迹*/
    if(m_PartitionedTargetCurvatureSets.size() == 1)
    {
        m_bLastDriving = true;
    }else{}

    /*3 获取下一段规划轨迹的行驶方向*/
    if(m_PartitionedTargetCurvatureSets.size() > 0)
    {
        m_nDrivingDirection = m_PartitionedTargetCurvatureSets[0][0].driving_direction;
    }else{}

    /*4 计算当前泊车步数和泊车总步数*/
    if(!m_PartitionedTargetCurvatureSets.empty())
    {
        m_TotalGearSwitchTimes++;
        m_nCurrentStep = m_TotalGearSwitchTimes;
        m_nTotalStep = m_PartitionedTargetCurvatureSets.size() + m_TotalGearSwitchTimes - 1;
        m_fResidualDis = 0.0;
        for(size_t i = 0; i < m_PartitionedTargetCurvatureSets.size(); ++i)
        {
            m_fResidualDis += m_PartitionedTargetCurvatureSets[i].size() * Constants::step_size;
        }

        GAC_LOG_DEBUG("m_nCurrentStep :%d, m_nTotalStep: %d, Residual Dis:%d\n",m_nCurrentStep, m_nTotalStep, m_fResidualDis);
    }
    else
    {
        m_nCurrentStep = 0;
        m_nTotalStep = 0;
    }

    /*5 轨迹坐标旋转*/
    GetFinalTracks(m_RotationCoordinate, m_PartitionedTargetCurvatureSets, aTracks);
    RotateHybridAResults(m_RotationCoordinate, m_HybridAResults);
    RotateSegmentSets(m_RotationCoordinate, m_PartitionedTargetCurvatureSets);

    /*6 轨迹点赋状态*/
    if (SIMULATION)
    {
        /*给轨迹点测量状态赋初值0*/
        for(size_t i = 0; i < aTracks.size(); ++i)
        {
            aTracks[i].nDetectStates = 0;
        }
    }
    else
    {
        if (m_slotPosInfor.nParkingType != HEAD_PARK_OUT)
        {
            AddMeasureStatusInTargetTrack(aTracks);
        }
    }

    if(aTracks.size() < 5)
    {
        m_PartitionedTargetCurvatureSets.clear();
        m_HybridAResults.clear();
        aTracks.clear();
    }else{}

    if (m_bFirstPlan)
    {
        m_bFirstPlan = false;
    }else{}
}
/**
 * @brief 路径的实时规划
 * @param InputParkingIn_
 * @param aTracks 控制点
 * @return 0-规划结束 1-规划成功 2 -规划失败
 */
PlanningStatus HybridAStar::PathPlanningRealTimeState(InputParkingIn &InputParkingIn_,
                                                      std::vector<TargetTrack>& aTracks)
{
    PlanningStatus nPlanningFlag = PlanningFail;/*0-规划结束 1-规划成功 2 -规划失败*/
    ParkingPlanningMode PlanningMode;

    m_HybridAResults.clear();
    m_PartitionedTargetCurvatureSets.clear();
    aTracks.clear();

    if (!m_slotPosInfor.bValidty && m_slotPosInfor.nParkingType != REMOTE_IN_OUT)
    {
        nPlanningFlag = PlanningFail;
    }
    else
    {
        if (JudgeVehReachGoalPoint(m_slotPosInfor) || (InputParkingIn_.m_bHitWheelBar))
        {
            if(InputParkingIn_.m_bHitWheelBar)
            {
                GAC_LOG_INFO("planning over for hitting the wheel bar!!!");
            }
            nPlanningFlag = planningOver;
        }
        else
        {
            m_bLastDriving = false;
            PlanningMode = JudgeParkingPlanningMode();
            nPlanningFlag = PathPlanningByMode(PlanningMode);
        }
    }

    if (nPlanningFlag == PlanningSucc)
    {
        PostProcessingTaskAfterPlanningSuccess(aTracks);
        if(aTracks.empty())
        {
            nPlanningFlag = PlanningFail;
        }
    }
    else
    {
        aTracks.clear();
        m_HybridAResults.clear();
        m_PartitionedTargetCurvatureSets.clear();
    }

    InputParkingIn_.cParkType = m_slotPosInfor.nParkingType;
    InputParkingIn_.cParkOutDir = m_slotPosInfor.cParkOutDir;
    InputParkingIn_.cSlotType = m_slotPosInfor.nSlotType;
    InputParkingIn_.cSlotPosition = m_slotPosInfor.nSlotSide;
    InputParkingIn_.cDetectType = m_slotPosInfor.nDetectType;
    InputParkingIn_.nUssSideType = m_slotPosInfor.nUssSideType;
    InputParkingIn_.fXSlotPosInGrid = m_slotPosInfor.fXSlotPosInMap;
    InputParkingIn_.fYSlotPosInGrid = Constants::ob_grid_height/10 - m_slotPosInfor.fYSlotPosInMap;
    InputParkingIn_.fSlotWidth = m_slotPosInfor.fSlotLength;
    InputParkingIn_.fSlotDepth = m_slotPosInfor.fSlotWidth;

    InputParkingIn_.fDeltaXRearEdge = m_deltaRearX;
    InputParkingIn_.fDeltaYRearLine = m_deltaRearY;
    InputParkingIn_.fDeltaXFrontEdge = m_deltaFrontX;
    InputParkingIn_.fDeltaYFrontLine = m_deltaFrontY;

    InputParkingIn_.fSlotYaw = m_OriginSlotYaw;

    InputParkingIn_.mParaSlotSteps = mParaSlotSteps;
    InputParkingIn_.m_bStartPathplanningInParaSLot = m_bStartPathplanningInParaSLot;

    InputParkingIn_.fVehCurX = m_slotPosInfor.startPointInSlot.x;
    InputParkingIn_.fVehCurY = m_slotPosInfor.startPointInSlot.y;
    InputParkingIn_.fVehCurYaw = m_slotPosInfor.startPointInSlot.yaw;

    InputParkingIn_.m_nDrivingDirection = m_nDrivingDirection;
    InputParkingIn_.m_nFirstPlanTotalSteps =  m_nFirstPlanTotalSteps;
    InputParkingIn_.m_ParaHybridALastDriving = m_ParaHybridALastDriving;
    InputParkingIn_.m_ParkingCtrlStatus = m_ParkingCtrlStatus;
    InputParkingIn_.m_TotalGearSwitchTimes = m_TotalGearSwitchTimes;
    InputParkingIn_.m_bFirstPlan = m_bFirstPlan;
    InputParkingIn_.m_bUSSHasDetectFrontMargin = m_bUSSHasDetectFrontMargin;
    InputParkingIn_.m_bUSSHasDetectRearMargin = m_bUSSHasDetectRearMargin ;
    InputParkingIn_.RotationCoordinate = m_RotationCoordinate;
    InputParkingIn_.m_LastDriving = m_bLastDriving;
    InputParkingIn_.m_nCurrentStep = m_nCurrentStep;
    InputParkingIn_.m_nTotalStep = m_nTotalStep;
    InputParkingIn_.m_fYawOffsetOfOpti = m_fYawOffsetOfOpti;
    InputParkingIn_.m_lastPlanningPoint = m_lastPlanningPoint;

    InputParkingIn_.m_goalPointOffset.x =  m_slotPosInfor.goalPointInSlot.x;
    InputParkingIn_.m_goalPointOffset.y = m_slotPosInfor.goalPointInSlot.y;
    InputParkingIn_.m_goalPointOffset.yaw = m_slotPosInfor.goalPointInSlot.yaw;

    InputParkingIn_.TarVehPoseInSlot =  m_slotPosInfor.targetPointInSlot;
    InputParkingIn_.tFrontSlotCorner =  m_slotPosInfor.tFrontSlotCorner;
    InputParkingIn_.tRearSlotCorner =  m_slotPosInfor.tRearSlotCorner;

    memcpy(InputParkingIn_.SlotLineType, m_slotPosInfor.SlotLineType, 16);
    return  nPlanningFlag;

}
PlanningStatus HybridAStar::RemoteInOutParkingPathPlanning(int8_t nDrivingDirection,
                                                           LocationPoint startPointInSlot,
                                                           std::vector<std::vector<CtrlPoint> > &PartitionedTargetCurvatureSets)
{
    PlanningStatus nPlanningFlag = PlanningFail;

    float fRemoteMoveDist = 5.0f;
    CtrlPoint tCtrlPoint;
    std::vector<CtrlPoint> TempTraj;
    TempTraj.clear();
    PartitionedTargetCurvatureSets.clear();

    int nDriveDir = nDrivingDirection;
    tCtrlPoint.driving_direction = nDriveDir;
    tCtrlPoint.kappa = 1.0f/2000.f;
    tCtrlPoint.yaw = startPointInSlot.yaw;
    float fCosYaw = cosf(startPointInSlot.yaw);
    float fSinYaw = sinf(startPointInSlot.yaw);

    for(float fPathLen = 0.f; fPathLen < fRemoteMoveDist; fPathLen += 0.05f)
    {
        tCtrlPoint.x = startPointInSlot.x + (nDriveDir) * fPathLen * fCosYaw;
        tCtrlPoint.y = startPointInSlot.y + (nDriveDir) * fPathLen * fSinYaw;

        if(fPathLen < fRemoteMoveDist/2.f)
        {
            tCtrlPoint.veh_speed = MAX(0.3f, sqrtf(2*1*fPathLen));
        }
        else
        {
            tCtrlPoint.veh_speed = MAX(0.3f, sqrtf(2*1*(fRemoteMoveDist - fPathLen)));
        }
        TempTraj.push_back(tCtrlPoint);
    }
    PartitionedTargetCurvatureSets.push_back(TempTraj);
    nPlanningFlag = PlanningSucc;
    m_bLastDriving = true;
    return nPlanningFlag;
}
bool HybridAStar::ifStartSearchPlanningParaOut(const slotPosInfor &slotPosInfor)
{
    bool bRet = false;
    LocationPoint tVehCorner[4];
    CalVehicleCornerPoint(slotPosInfor.startPointInSlot, tVehCorner);

    if((slotPosInfor.nSlotSide == RIGHTSIDESLOT && tVehCorner[1].y > 0.f)
            ||(slotPosInfor.nSlotSide == LEFTSIDESLOT && tVehCorner[2].y < 0.f))
    {
        bRet = true;
    }
    return bRet;
}

bool HybridAStar::CheckParkoutSearch(const Node3D* node)
{
    bool bRet = false;
    if(m_slotPosInfor.nSlotSide == RIGHTSIDESLOT)
    {
        bRet = (node->x > m_slotPosInfor.goalPointInMap.x) &&
                (node->y < m_slotPosInfor.goalPointInMap.y) &&
                (fabs(node->t) < M_PI/180.f);
    }
    else
    {
        bRet = (node->x > m_slotPosInfor.goalPointInMap.x) &&
                (node->y > m_slotPosInfor.goalPointInMap.y) &&
                (fabs(node->t) < M_PI/180.f);
    }
    return bRet;
}

_planErrCode HybridAStar::GetPlanErrCode(void)
{
    return planErrCode;
}
}

