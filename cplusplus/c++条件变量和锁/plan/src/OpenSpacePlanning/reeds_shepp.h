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
//* @file reeds_shepp.h
//* @copyright 版权所有 2020 魔视智能科技(上海)有限公司
//* @brief ReedSheep曲线
//* @version V2.0
//* @author  zhou haining; xingyu zhang
//* @date 2020年7月2日
//* @note
//* 引用文献 :
//* 1 Reeds, J.A. & Shepp, L.A.. (1990). Optimal paths for a car that goes both forwards and backwards. Pacific Journal of Mathematics. 145. 367-393.
//* 2 Dubins, L.E.. (1957). On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents. American Journal of Mathematics. 79. 10.2307/2372560.
//*/

//#ifndef SPACES_REEDS_SHEPP_STATE_SPACE_
//#define SPACES_REEDS_SHEPP_STATE_SPACE_

//#include <vector>
//#include <cmath>
//#define NDEBUG
//#include <cassert>
//namespace REEDS_SHEPP_CURVE
//{
///** \brief The Reeds-Shepp path segment types */
//typedef enum
//{
//    RS_NOP = 0,
//    RS_LEFT = 1,
//    RS_STRAIGHT = 2,
//    RS_RIGHT = 3
//} ReedsSheppPathSegmentType;


///** \brief Reeds-Shepp path types */
//static const ReedsSheppPathSegmentType reedsSheppPathType[24][5] = {
//    { RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP },             // 0
//    { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP },            // 1
//    { RS_LEFT, RS_RIGHT, RS_LEFT, RS_RIGHT, RS_NOP },           // 2
//    { RS_RIGHT, RS_LEFT, RS_RIGHT, RS_LEFT, RS_NOP },           // 3
//    { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP },        // 4
//    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP },       // 5
//    { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },        // 6
//    { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },       // 7
//    { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP },       // 8
//    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP },        // 9
//    { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_LEFT, RS_NOP },       // 10
//    { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_RIGHT, RS_NOP },        // 11
//    { RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },         // 12
//    { RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },         // 13
//    { RS_LEFT, RS_STRAIGHT, RS_LEFT, RS_NOP, RS_NOP },          // 14
//    { RS_RIGHT, RS_STRAIGHT, RS_RIGHT, RS_NOP, RS_NOP },        // 15
//    { RS_LEFT, RS_RIGHT, RS_STRAIGHT, RS_LEFT, RS_RIGHT },      // 16
//    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_RIGHT, RS_LEFT },       // 17
//    { RS_LEFT,RS_RIGHT, RS_STRAIGHT, RS_NOP, RS_NOP },         // 18
//    { RS_RIGHT, RS_LEFT, RS_STRAIGHT, RS_NOP, RS_NOP },         // 19
//    { RS_STRAIGHT,RS_RIGHT, RS_LEFT, RS_NOP, RS_NOP },         // 20
//    { RS_STRAIGHT,RS_LEFT, RS_RIGHT, RS_NOP, RS_NOP },         // 21
//    { RS_STRAIGHT,RS_LEFT, RS_STRAIGHT, RS_NOP, RS_NOP },         // 22
//    { RS_STRAIGHT,RS_RIGHT, RS_STRAIGHT, RS_NOP, RS_NOP }         // 23

//};

///** \brief Complete description of a ReedsShepp path */
//typedef struct _ReedsSheppPath
//{
//    /** Path segment types */
//    const ReedsSheppPathSegmentType* type_;
//    /** The lengths of the five segments */
//    double length_[5];
//    /** Total length */
//    double totalLength_;
//    /** Turning radius, model forward velocity / model angular velocity */
//    double rho_;

//    _ReedsSheppPath():totalLength_(1e50), rho_(0.0){}
//} ReedsSheppPath;

//void GetReedsSheppPath(ReedsSheppPath *path,
//                      const ReedsSheppPathSegmentType* type = reedsSheppPathType[0],
//                      double t = 1e50, double u = 0., double v = 0.,
//                      double w = 0., double x = 0.);

//std::vector<ReedsSheppPathSegmentType> type(ReedsSheppPath *path);

///** \brief Return the shortest Reeds-Shepp path from q0 state to q1 state */
//ReedsSheppPath reedsShepp(double q0[3], double q1[3], double rho_);

//void ReedsShepp_interpolate(double q0[], ReedsSheppPath &path, double fUnitArcLength, double rho_, double s[]);

//void ReedsShepp_path_sample(double q0[3], double q1[3], double rho_, double step_size,
//                                std::vector<std::vector<double>> &points);

//}
//#endif
