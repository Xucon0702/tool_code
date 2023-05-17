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
#ifndef POLYVIEW_KNOTS_HPP
#define POLYVIEW_KNOTS_HPP

#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <iostream>

typedef unsigned long ulong;
typedef unsigned int uint;
typedef unsigned short ushort;

namespace polyview
{
namespace containers
{
    class Knots
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Basis
        {
            double dValue; // 基函数的值
            int nI; // 第 i 个
            int nP; // 次数 p

            Basis(double value, int i, int p) : dValue(value), nI(i), nP(p) {}

            Basis() : dValue(0), nI(0), nP(0) {}
        };

        Knots();

        Knots(int n);

        //Set N
        void setN(int n) { mnN = n; }

        //get N
        int getN() { return mnN; }

        //Find the span of u
        int FindSpan(double u, int p);

        //get the derivaty Basis of u
        void DersBasisFuns(int span, double u, int p, int n, std::vector<Basis> &vBasis);

        //get the basis function at u
        void BasisFuns(double u, int p, std::vector<Basis> &vBasis);

        //get the basis function at u with given span
        void BasisFuns(int span, double u, int p, std::vector<Basis> &vBasis);

        //Insert knot
        void addKnot(double U);

        //get knots vector
        std::vector<double> &getKnots() { return mvU; }

        size_t size() { return mvU.size(); }

    private:
        std::vector<double> mvU;
        int mnN;
    };
}
}
#endif //POLYVIEW_KNOTS_HPP
