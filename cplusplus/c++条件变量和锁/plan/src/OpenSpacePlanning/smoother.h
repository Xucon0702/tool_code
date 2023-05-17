///////////////////////////////////////////////////////////////////////////
//// Copyright (C) 2021-2026, by Motovis All Rights Reserved.
////
//// 本代码仅用于魔视智能与广州汽车集团股份有限公司汽车工程研究院合作的
//// X3V项目（以下简称本项目），魔视智能对本代码及基于本代码开发产生的
//// 所有内容拥有全部知识产权，任何人不得侵害或破坏，未经魔视智能授权许可
//// 或其他法律认可的方式，任何企业或个人不得随意复制、分发、下载和使用，
//// 以及用于非本项目的其他商业用途。
////
//// 本代码仅供指定接收人（包括但不限于      ）在魔视智能授权范围内使用，
//// 指定接收人必须征得魔视智能授权，才可在软件库中加入本代码。
////
//// 本代码是受法律保护的保密信息，如您不是指定接收人，请立即将本代码删除，
//// 法律禁止任何非法的披露、或以任何方式使用本代码。指定接收人应对本代码
//// 保密信息负有保密义务，未经允许，不得超出本项目约定的披露、复制、传播
//// 或允许第三方披露、复制、传播本代码部分或全部信息。
////
////
///////////////////////////////////////////////////////////////////////////
///**
//* @file smoother.h
//* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
//* @brief 共轭梯度轨迹优化器
//* @version V2.0
//* @author  xingyu zhang
//* @date 2020年7月2日
//* @note 通过建立曲率最小的优化函数,对轨迹采用CG(共轭梯度算法)数值优化技术进行轨迹平滑
//* 引用文献 :
//* 1 Tianyu Gu , Jarrod Snider , John M. Dolan. (2013) .Focused Trajectory Planning for Autonomous On-Road Driving.
//* 2 Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2010). Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments.
//* 3 Dolgov, Dmitri & Thrun, Sebastian & Montemerlo, Michael & Diebel, James. (2008). Practical Search Techniques in Path Planning for Autonomous Driving.
//* 4 高立, 数值最优化方法(P22 - P23, P105 - P109).
//* 5 袁亚湘, 最优化理论和方法(P75 - P79, P94 - P101).
//*/
//#ifndef SMOOTHER_H
//#define SMOOTHER_H

//#include <cmath>
//#include <vector>
//#include "node3d.h"
//#include "vector2d.h"
//#include "constants.h"
//#include"publicfunpathplan.h"
//#if PLATFORM == PC_SYSTEM
//#include "QDebug"
//#endif
//#include <sys/time.h>
//#include <iostream>

//#undef PrintDetails
//#define MAX_CG_ITERATIONS (1000)

//namespace HybridAStarPart {
//class Smoother {
// public:
//  Smoother(): m_CoeffKappa(0.0),m_CoeffKappaRate(0.0),m_CoeffMoving(0.0),m_CoeffUniform(0.0){}

//  double vecMole(std::vector<double>& x);

//  std::vector<double> numMultiply(double k, std::vector<double>& x);

//  std::vector<double> vecAdd(std::vector<double> &a, std::vector<double>& b);

//  double vecDot(std::vector<double>& a, std::vector<double>& b);

//  bool StopIteration(double GkMole,
//                     double newGkMultiDk,
//                     int iteration);

//  void NonLinearOptimizationByConjugateGradient(std::vector<Node3D>& OPath,
//                                                std::vector<Node3D>& SmoothedPath);


//  void CalGradient(std::vector<Vector2D>& p,
//                           std::vector<double>& lamda,
//                           std::vector<double> &gkLam);

//  double CalObjFun(std::vector<Vector2D>& p,
//                      std::vector<double>& lamda);

//  void updatePathkSample(std::vector<Vector2D>& pathi,
//                         std::vector<double>& lamda);


//  void updateLamda(std::vector<double>& lamda,
//                   double& arfak,
//                   std::vector<double>& dk);

//  void updateDk(std::vector<double>& gk,
//                       double &betakm1,
//                       std::vector<double> &dk);

//  double CalPhiAlpha(double fAlpha, std::vector<Vector2D>& xk,
//                    std::vector<double> &lamda,
//                    std::vector<double>& dk);

//  double CalPhiAlphaDiff(double fAlpha,
//                        std::vector<Vector2D>& xk,
//                        std::vector<double> &lamda,
//                        std::vector<double>& dk);

//  double LineSearchByWolfePowell(std::vector<Vector2D>& xk,
//                                 std::vector<double> &lamda,
//                                 std::vector<double>& dk, double GkMultiDk, double Obj);

//  void CalNormalVector(Vector2D& xi,Vector2D& xip1, Vector2D& ans);

//  void AddPointInTrajStartAndEnd(std::vector<Node3D>& OPath);

// private:
//  double m_CoeffKappa; /*曲率项系数*/
//  double m_CoeffKappaRate; /*曲率项系数*/
//  double m_CoeffMoving; /*移动项系数*/
//  double m_CoeffUniform; /*均匀系数*/

//  std::vector<Vector2D> NormVector; /*法向量*/

//};
//}
//#endif // SMOOTHER_H
