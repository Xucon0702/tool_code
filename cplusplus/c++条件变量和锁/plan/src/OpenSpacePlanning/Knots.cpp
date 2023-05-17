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
#include "Knots.h"
namespace polyview{
namespace containers{

    Knots::Knots() {
        mvU.clear();
        mnN = 0;
    }

    Knots::Knots(int n):mnN(n)
    {

    }

    // 确定参数u所在的节点区间下标
    int Knots::FindSpan(double u, int p) {
        /* 特殊情况 */
        mnN = mvU.size() - p - 1;
        if ( u>= mvU[mnN])
            return mnN - 1;
        if ( u<mvU[p])
            return p;
//        if (fabs(u-mvU[mvU.size() - p - 1]) < 1e-4 ){
//            return mnN-1;
//        }
        int low = p;
        int high = mvU.size() - p - 1;
        int mid = floor((low+high) / 2);
        while(u < mvU[mid] || u >= mvU[mid + 1])/* 进行二分搜索 */
        {
            if (u < mvU[mid])
                high = mid;
            else
                low = mid;
            mid = floor((low+high) / 2);
        }
        return mid;
    }

    // 计算所有非零B样条基函数的值(根据u去求所在区间FindSpan)
    void Knots::BasisFuns(double u, int p, std::vector<polyview::containers::Knots::Basis> &vBasis) {
        vBasis.clear();
        vBasis.reserve(p+1);
        double N[p+1];
        double left[p+1];
        double right[p+1];
        N[0] = 1.0;

        int i = FindSpan(u,p);

        for (int j=1;j <=p;j++)
        {
            left[j] = u - mvU[i+1-j];
            right[j] = mvU[i+j] - u;
            double saved = 0.0;
            for (int r = 0; r < j; r++)
            {
                double temp = N[r] / (right[r+1] + left[j-r]);
                N[r] = saved +right[r+1] * temp;
                saved = left[j-r] * temp;
            }

            N[j] = saved;
        }

        for (int j=0;j<= p; j++)
        {
//            std::cout<<N[j]<<" ";
            Basis temp(N[j], i - p + j, p);
            vBasis.push_back(temp);
        }
    }

    // 计算所有非零B样条基函数的值(给定节点区间)
    void Knots::BasisFuns(int span, double u, int p, std::vector<polyview::containers::Knots::Basis> &vBasis) {
        vBasis.clear();
        vBasis.reserve(p+1);
        double N[p+1];
        double left[p+1];
        double right[p+1];
        N[0] = 1.0;

        int i = span;
        double saved = 0.0;
        double temp = 0.0;
        for (int j=1;j <=p;j++)
        {
            left[j] = u - mvU[i+1-j];
            right[j] = mvU[i+j] - u;
            saved = 0.0;
            for (int r = 0; r < j; r++)
            {
                temp = N[r] / (right[r+1] + left[j-r]);
                N[r] = saved +right[r+1] * temp;
                saved = left[j-r] * temp;
            }

            N[j] = saved;
        }

        for (int j=0;j<= p; j++)
        {
            Basis temp(N[j], i - p + j, p);
            vBasis.push_back(temp);
        }
    }

    // 计算非零B样条基函数及其导数
    void Knots::DersBasisFuns(int span, double u, int p, int n, std::vector<polyview::containers::Knots::Basis> &vBasis) {
        double ndu[p+1][p+1];
        ndu[0][0] = 1.0;
        double left[p+1],right[p+1];
        int i = span;
        for (int j=1; j <=p; j++)
        {
            left[j] = u - mvU[i+1-j];
            right[j] = mvU[i+j] - u;
            double saved = 0.0;
            for (int r = 0;r < j;r++)
            {
                ndu[j][r] = right[r+1] + left[j-r];// 下三角
                double temp = ndu[r][j-1] / ndu[j][r];
                ndu[r][j] = saved + right[r+1] * temp;// 上三角
                saved = left[j-r] * temp;
            }
            ndu[j][j] = saved;
        }
        double ders[p+1][p+1];

        for (int j=0;j <=p;j++)
        {
            ders[0][j] = ndu[j][p];// 载入基函数的值
        }
        /* 下面计算导数 */
        double a[2][p+1];
        for (int r = 0; r <= p;r++)// 对函数的下标进行循环
        {
            int s1 = 0, s2 = 1;// 改变数组a的行

            a[0][0] = 1.0;
            for (int k=1;k <= n; k++)// 循环计算k阶导数，k=1,2,3,...,n
            {
                double d = 0.0;
                int rk = r - k;
                int pk = p-k;
                if (r >=k)
                {
                    a[s2][0] = a[s1][0] / ndu[pk+1][rk];
                    d = a[s2][0] * ndu[rk][pk];
                }
                int j1,j2;
                if (rk >= -1) j1 = 1;
                else j1 = -rk;
                if (r-1 <= pk) j2 = k-1;
                else j2 = p-r;
                for (int j=j1; j<= j2; j++)
                {
                    a[s2][j] = (a[s1][j] - a[s1][j-1]) / ndu[pk+1][rk+j];
                    d += a[s2][j] * ndu[rk+j][pk];
                }

                if (r <= pk)
                {
                    a[s2][k] = -a[s1][k-1]/ndu[pk+1][r];
                    d += a[s2][k] * ndu[r][pk];
                }
                ders[k][r] = d;
                std::swap(s1,s2);// 转换行

            }
        }

        int r = p;
        for(int k=1; k<=n; k++)// 对结果乘以正确的因子
        {
            for (int j=0;j <=p; j++) ders[k][j] *= r;
            r *= (p-k);
        }

        for (uint j=0;j < p+1;j++)
        {
            Basis temp(ders[n][j], i - p + j, p);
            vBasis.push_back(temp);
        }
    }

    void Knots::addKnot(double U) {
        mvU.push_back(U);
    }
}
}
