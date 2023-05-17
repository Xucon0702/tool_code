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
#ifndef POLYVIEW_BSPLINE_HPP
#define POLYVIEW_BSPLINE_HPP

#include <Eigen/Eigen>
#include "Knots.h"
#include <iostream>
#include <memory>

namespace polyview
{
namespace containers
{
    template<int dim>
    class Bspline
    {
    public:
        typedef std::shared_ptr<polyview::containers::Bspline<dim> > Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Bspline(): mdScale(0.0), mnP(0){}
        Bspline(int p):mnP(p){}

        /* Interpolation a curve with given points with Order p,
         * 求控制点向量P, A * P = B, ==> P = B / A
         * */
        void Interpolation(const std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > &points, int p = 3)
        {
            if (points.size() < p + 1)
            {
                std::cout<<"Not enough points!\n";
                return;
            }

            mnP = p;
            int n = points.size() -1;
            int m = n + mnP + 1;
            mKnots.setN(n);
            std::vector<double> uk;
            InitializeKnots(points, uk);// 弦长参数化求uk 及 取平均值求节点矢量U

            // 建立系数矩阵A
            Eigen::MatrixXd A;
            A.resize(n+1, n+1);
            A.fill(0);
            for (uint i=0;i <= n; i++)
            {
                int span = mKnots.FindSpan(uk[i],mnP);
                std::vector<Knots::Basis> tempBasis;
                mKnots.BasisFuns(span, uk[i],mnP,tempBasis); // 获得第i行数据
                for (uint j=0;j < tempBasis.size();j++)
                {
                    A(i,span - mnP + j) = tempBasis[j].dValue;
                }
            }

            // 等式右边为需要处理的数据点
            Eigen::MatrixXd B;
            B.resize(points.size(), dim);
            for (uint i=0;i < points.size(); i++)
            {
                B.row(i) = points[i].transpose();
            }

            Eigen::MatrixXd P,Ptmp;
            P.resize(points.size(), dim);
            double x = A.determinant();
            Ptmp = A.inverse();
            P= A.inverse() * B;
            mvCPs.clear();
            for (uint i=0;i < points.size(); i++)
            {
                mvCPs.push_back(P.block<1,dim>(i,0).transpose());
            }
        }

        //Interpolation a curve with given points, begin derivate,end derivate  Order p
        void Interpolation(const std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > &points,
                           const Eigen::Matrix<double, dim, 1> & der_0,
                           const Eigen::Matrix<double, dim, 1> & der_n,
                           int p = 3)
        {
            if (points.size() < p)
            {
                std::cout<<"Not enough points!\n";
                return;
            }

            mnP = p;
            int n = points.size() -1;
            int m = n + mnP + 3;
            mKnots.setN(n);
            std::vector<double> uk;
            InitializeDerGivenKnots(points, uk);

            double sum = 0;
            for (uint i=1;i < points.size(); i++)
            {
                double dt = (points[i] - points[i-1]).norm();
                sum += dt;
            }

            std::vector<double> & U = mKnots.getKnots();

            Eigen::MatrixXd A;
            A.resize(n+3, n+3);
            A.fill(0);
            Eigen::MatrixXd B;
            B.resize(n+3, dim);
            A(0, 0) = 1;
            A(1, 0) = -1;
            A(1, 1) = 1;
            B.row(0) = points[0].transpose();
            B.row(1) = (U[mnP +1] / mnP * der_0).transpose() * sum;

            for (uint i=1;i < n; i++)
            {
                int span = mKnots.FindSpan(uk[i],mnP);
                std::vector<Knots::Basis> tempBasis;
                mKnots.BasisFuns(span, uk[i],mnP,tempBasis);
                for (uint j=0;j < tempBasis.size();j++)
                {
                    A(i + 1, span - mnP + j) = tempBasis[j].dValue;
                }
            }

            for (uint i=1;i < points.size()-1; i++)
            {
                B.row(i + 1) = points[i].transpose();
            }

            A(n+1,n + 1) = -1;
            A(n+1,n + 2) = 1;
            A(n+2,n + 2) = 1;
            B.row(n+1) = ((1-U[m- mnP - 1]) / mnP * der_n).transpose() *  sum;
            B.row(n+2) = points[n].transpose();

//            std::cout<<"A: \n"<<A<<std::endl<<std::endl;
//            std::cout<<"B: \n"<<B<<std::endl<<std::endl;

            Eigen::MatrixXd P;
            P.resize(n+3, dim);
            P= A.inverse() * B;
            mvCPs.clear();
            for (uint i=0;i < n+3; i++)
            {
                mvCPs.push_back(P.block<1,dim>(i,0).transpose());
            }

//            std::cout<<"begin Derivate: "<< (CurveDerivsAlg(0, 1).normalized()).transpose()<<std::endl;
//            std::cout<<"end Derivate: "<< (CurveDerivsAlg(1, 1).normalized()).transpose()<<std::endl;
        }

        //Do a fitting of B-spline to points with m control points and order p
        bool fitting(const std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > &points,
                     int n,
                     int p = 3)
        {
            //first we need to find Uk and Us
            mnP = p;
            int m = points.size() -1;
            double d = (m+1) / (n - p + 1);

            if (m <= n || n < p || p< 1)
            {
                std::cout<<"B-spline fitting Fault! Not enough points or Invalid number of control points!\n";
                return false;
            }

            std::vector<double> uk;
            uk.clear();
            uk.resize(points.size());
            std::vector<double> & U = mKnots.getKnots();
            uk[0] = 0;
            double sum = 0;
            for (uint i=1;i < points.size(); i++)
            {
                double dt = (points[i] - points[i-1]).norm();
                sum += dt;
                uk[i] = sum;
            }

            for (uint i=1;i < points.size();i++)
            {
                uk[i] = uk[i] / sum;
            }

            U.clear();
            U.reserve(n + mnP + 2);
            for (int i=0;i <=mnP;i++)
            {
                U.push_back(0.0);
            }

            for (uint j = 1; j <= n-mnP;j++)
            {
                int i = floor( j * d);
                double a = j * d - i;
                U.push_back((1-a) * uk[i-1] + a * uk[i]);
            }

            for (int i=0;i <=mnP;i++)
            {
                U.push_back(1.0);
            }

            //Estimate the nearest point
            std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > R_k;
            R_k.reserve(m + 1);
            R_k.push_back( points[0]);
            for (int k= 1; k < m ;k++)
            {
                double u = uk[k];
                std::vector<Knots::Basis> temp_Basis;
                mKnots.BasisFuns(u,mnP, temp_Basis);
                Eigen::Vector3d R_temp = points[k];
                for (int i=0; i < temp_Basis.size();i++)
                {
                    if (temp_Basis[i].nI == 0)
                    {
                        R_temp -= temp_Basis[i].dValue * points[0];
                    }

                    if (temp_Basis[i].nI == n)
                    {
                        R_temp -= temp_Basis[i].dValue * points[m];
                    }
                }
                R_k.push_back(R_temp);
            }
            R_k.push_back(points[m]);

            Eigen::MatrixXd N;
            N.resize(m-1, n-1);
            N.fill(0);
            for (int i=1; i < m; i++)
            {
                double u = uk[i];
                std::vector<Knots::Basis> temp_Basis;
                mKnots.BasisFuns(u,mnP, temp_Basis);
                for (uint j=0;j < temp_Basis.size();j++)
                {
                    if (temp_Basis[j].nI >=1 && temp_Basis[j].nI < n)
                        N(i-1, temp_Basis[j].nI -1) = temp_Basis[j].dValue;
                }
            }

//            std::cout<<"N: \n"<<N<<std::endl;

            Eigen::MatrixXd R;
            R.resize(n-1, dim);
            R.fill(0);
            for (int i=1; i < m; i++)
            {
                double u = uk[i];
                std::vector<Knots::Basis> temp_Basis;
                mKnots.BasisFuns(u,mnP, temp_Basis);
                for (uint j=0;j < temp_Basis.size();j++)
                {
                    if (temp_Basis[j].nI >=1 && temp_Basis[j].nI < n)
                        R.row(temp_Basis[j].nI -1) += temp_Basis[j].dValue * R_k[i].transpose();
                }
            }
//            std::cout<<"R: \n"<<R<<std::endl;
            Eigen::MatrixXd NTN = (N.transpose() * N);
//            std::cout<<"N^TN: \n"<<NTN<<std::endl;
            Eigen::MatrixXd P = NTN.inverse() * R;


            mvCPs.clear();
            mvCPs.push_back(points[0]);
            for (uint i=0;i < P.rows(); i++)
            {
                mvCPs.push_back(P.row(i).transpose());
            }
            mvCPs.push_back(points.back());

            return true;
        }

        //Initialize the Knots with given points(must be ordered), 弦长参数化求uk 及 取平均值求节点矢量U
        void InitializeKnots(const std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > &points,
                          std::vector<double> &uk)
        {
            int n = points.size() -1;
            int m = n + mnP + 1;
            uk.clear();
            uk.resize(points.size());
            std::vector<double> & U = mKnots.getKnots();
            uk[0] = 0;
            // 弦长参数化求uk 及 取平均值求节点矢量U
            double sum = 0;
            for (uint i=1;i < points.size(); i++)
            {
                double dt = (points[i] - points[i-1]).norm();
                sum += dt;
                uk[i] = sum;
            }
            for (uint i=1;i < points.size();i++)
            {
                uk[i] = uk[i] / sum;
            }

            U.clear();
            for (int i=0;i <=mnP;i++)
            {
                U.push_back(0);
            }

            for (int j = 1;j <= n-mnP;j++)
            {
                double temp_sum = 0;
                for (int i = j; i <= j+mnP-1; i++)
                {
                    temp_sum+= uk[i];
                }
                temp_sum /= mnP;

                U.push_back(temp_sum);
            }

            for (int i=0;i <=mnP;i++)
            {
                U.push_back(1);
            }
        }


        //Initialize the Knots with given points for given derivate
        void InitializeDerGivenKnots(const std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > &points,
                             std::vector<double> &uk)
        {
            int n = points.size() - 1;
            int m = n + mnP + 3;
            uk.clear();
            uk.resize(points.size());
            std::vector<double> & U = mKnots.getKnots();
            uk[0] = 0;
            double sum = 0;
            for (uint i=1;i < points.size(); i++)
            {
                double dt = (points[i] - points[i-1]).norm();
                sum += dt;
                uk[i] = sum;
            }
            for (uint i=1;i < points.size();i++)
            {
                uk[i] = uk[i] / sum;
            }

            U.clear();
            for (int i=0;i <=mnP;i++)
            {
                U.push_back(0);
            }

            for (int j = 0;j <= n- mnP + 1;j++)
            {
                double temp_sum = 0;
                for (int i = j; i <= j+mnP-1; i++)
                {
                    temp_sum+= uk[i];
                }
                temp_sum /= mnP;

                U.push_back(temp_sum);
            }

            for (int i=0;i <=mnP;i++)
            {
                U.push_back(1);
            }
        }


        //Compute a point on the curve at u
        Eigen::Matrix<double, dim,1> getPoint(double u)
        {
            std::vector<Knots::Basis> temp_Basis;
            mKnots.BasisFuns(u,mnP, temp_Basis);
            Eigen::Matrix<double, dim,1> result = Eigen::Matrix<double, dim,1>::Zero();
            for (uint i=0;i < temp_Basis.size(); i++)
            {
                Eigen::Matrix<double, dim,1> CP= getCP(temp_Basis[i].nI);
                result += temp_Basis[i].dValue * CP;// 将非零基函数的值与相应的控制点相乘并求和
            }
            return result;
        }

        //Get the N basis of a given u
        void getBasis(double u,  std::vector<Knots::Basis> & vBasis)
        {
            mKnots.BasisFuns(u,mnP, vBasis);
        }

        //Estimate the u of a given point
        double estimateU(const Eigen::Matrix<double, dim,1> &P)
        {
            std::vector<double> & knots = mKnots.getKnots();
            double minDist = 999999;
            int minKnot = 0;
            for (uint i=mnP;i <= mKnots.getN()+1;i++)
            {
                Eigen::Matrix<double, dim,1> tempP = getPoint(knots[i]);
                double dist = (tempP - P).norm();
                if (dist < minDist)
                {
                    minDist = dist;
                    minKnot = i;
                }
            }
            Eigen::Matrix<double, dim, 1> Der = CurveDerivsAlg(knots[minKnot], 1);
            Der.normalize();
            Eigen::Matrix<double, dim, 1> P_knot = getPoint(knots[minKnot]);

            double d = (P - P_knot).transpose() * Der;

            if (minKnot == mnP)
            {
                Eigen::Matrix<double, dim,1> lowerP = getPoint(knots[mnP]);
                Eigen::Matrix<double, dim,1> higherP = getPoint(knots[mnP+1]);
                double u = d / (higherP-lowerP).norm() * (knots[mnP+1] - knots[mnP]) + knots[minKnot];
                return u;
            }
            if (minKnot == mKnots.getN()+1)
            {
                Eigen::Matrix<double, dim,1> lowerP = getPoint(knots[minKnot-1]);
                Eigen::Matrix<double, dim,1> higherP = getPoint(knots[minKnot]);
                double u = d / (higherP-lowerP).norm() * (knots[minKnot] - knots[minKnot-1]) + knots[minKnot];
                return u;
            }
            {
                Eigen::Matrix<double, dim,1> tempP1 = getPoint(knots[minKnot-1]);
                Eigen::Matrix<double, dim,1> tempP2 = getPoint(knots[minKnot]);
                Eigen::Matrix<double, dim,1> tempP3 = getPoint(knots[minKnot+1]);

                if (d < 0)
                {
                    Eigen::Matrix<double, dim,1> lowerP = tempP1;
                    Eigen::Matrix<double, dim,1> higherP = tempP2;
                    double u = d / (higherP-lowerP).norm() * (knots[minKnot] - knots[minKnot-1]) + knots[minKnot];
                    return u;
                }
                else
                {
                    Eigen::Matrix<double, dim,1> lowerP = tempP2;
                    Eigen::Matrix<double, dim,1> higherP = tempP3;
                    double u = d / (higherP-lowerP).norm() * (knots[minKnot+1] - knots[minKnot]) + knots[minKnot];
                    return u;
                }
            }
        }

        double trajLength(double u0, double u1)
        {
            std::vector<double> & knots = mKnots.getKnots();
            int span0 = mKnots.FindSpan(u0,mnP);
            int span1 = mKnots.FindSpan(u1,mnP);
            double length = (getPoint(knots[span0+1]) - getPoint(u0)).norm();
            for (int i= span0+2; i < span1; i++)
            {
                length += (getPoint(knots[i]) - getPoint(knots[i-1])).norm();
            }
            length += (getPoint(u0) - getPoint(knots[span1])).norm();
            return length;
        }

        void FindURegion(const Eigen::Matrix<double, dim,1> &startP, const Eigen::Matrix<double, dim,1> &endP, double & startU, double &endU)
        {
            std::vector<int> list0;
            std::vector<int> list1;
            std::vector<double> & knots = mKnots.getKnots();
            for (int i=0;i < knots.size(); i++)
            {
                double dist0 = (getPoint(knots[i]) - startP).norm();
                double dist1 = (getPoint(knots[i]) - endP).norm();
                if (dist0 < 4.0)
                {
                    list0.push_back(i);
                }
                if (dist1 < 4.0)
                {
                    list1.push_back(i);
                }
            }

            double minDist = 0xffff;
            int _startUSpan, _endUSpan;
            for (int i = 0; i< list0.size();i++)
                for (int j=0;j < list1.size();j++)
                {

                    if (list0[i] > list1[j])
                        continue;
                    double u0 = knots[list0[i]];
                    double u1 = knots[list1[j]];
                    double length = trajLength(u0, u1);
                    length += (getPoint(u0) - startP).norm() * 50;
                    length += (getPoint(u1) - endP).norm() ;

                    if (length < minDist)
                    {
                        minDist = length;
                        _startUSpan = list0[i];
                        _endUSpan = list1[j];
                    }
                }

            Eigen::Vector2d t_start = startP - getPoint(knots[_startUSpan] ) ;
            Eigen::Vector2d der_start = CurveDerivsAlg(knots[_startUSpan], 1);
            der_start.normalize();
            double du_start = t_start.transpose() * der_start ;
            du_start *= mdScale;

            Eigen::Vector2d t_end = endP - getPoint(knots[_endUSpan] ) ;
            Eigen::Vector2d der_end = CurveDerivsAlg(knots[_endUSpan ],1);
            der_end.normalize();
            double du_end = t_end.transpose() * der_end;
            du_end *= mdScale;

            startU =knots[_startUSpan] + du_start + 4.0 * mdScale;
            endU =knots[_endUSpan] + du_end;

//            startU =knots[_startUSpan];
//            endU =knots[_endUSpan];
//            Eigen::Matrix<double, dim, 1> Der0 = CurveDerivsAlg(knots[_startUSpan], 1);
//            Der0.normalize();
//            Eigen::Matrix<double, dim, 1> P_knot0 = getPoint(knots[_startUSpan]);
//            double d0 = (startP - P_knot0).transpose() * Der0;
//            Eigen::Matrix<double, dim,1> lowerP0 = getPoint(knots[_startUSpan]);
//            Eigen::Matrix<double, dim,1> higherP0 = getPoint(knots[_startUSpan+1]);
//            startU = d0 / (higherP0-lowerP0).norm() * (knots[_startUSpan+1] - knots[_startUSpan]) + knots[_startUSpan];
//
//            Eigen::Matrix<double, dim, 1> Der1 = CurveDerivsAlg(knots[_endUSpan], 1);
//            Der1.normalize();
//            Eigen::Matrix<double, dim, 1> P_knot1 = getPoint(knots[_endUSpan]);
//            double d1 = (endP - P_knot1).transpose() * Der1;
//            Eigen::Matrix<double, dim,1> lowerP1 = getPoint(knots[_endUSpan-1]);
//            Eigen::Matrix<double, dim,1> higherP1 = getPoint(knots[_endUSpan]);
//            endU = d1 / (higherP1-lowerP1).norm() * (knots[_endUSpan] - knots[_endUSpan-1]) + knots[_endUSpan];

        }

        //Get the pointer to the raw data of the control point at Index
        double* getCPPtr(int Index)
        {
            return mvCPs[Index].data();
        }

        //Get the control point at Index
        Eigen::Matrix<double, dim,1> & getCP(int Index)
        {
            return mvCPs[Index];
        }

        //Compute the Curve derivs at order d
        Eigen::Matrix<double, dim,1> CurveDerivsAlg(double u, int d)
        {
            int du = d < mnP ? d:mnP;
            int span = mKnots.FindSpan(u, mnP);
            std::vector<Knots::Basis> vBasis;
            mKnots.DersBasisFuns(span, u, mnP, du, vBasis);
            Eigen::Matrix<double, dim,1> Ck = Eigen::Matrix<double, dim,1>::Zero();
            for (int j=0;j <= mnP;j++)
            {
                Eigen::Matrix<double, dim,1> CP = getCP(span - mnP + j);
                Ck = Ck + vBasis[j].dValue * CP;
            }
            return Ck;
        }

        //Find the repeat number of u
        int FindS(double u)
        {
            std::vector<double> &U = mKnots.getKnots();
            int count = 0;

            for (uint i=0;i < U.size();i++)
            {
                if (u == U[i])
                    count ++;
            }
            return count;
        }

        //Add a knot u into the B-spline
        void Addknot(double u)
        {
            int k = mKnots.FindSpan(u, mnP);
            std::vector<double> &U = mKnots.getKnots();


            int p = mnP;
//            mvCPs.insert(mvCPs.begin()+ k - p + 1, Eigen::Matrix<double, dim,1>::Zero());
            //Insert knot
            std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > Rw;
            for (int j=k-p+1; j<= k;j++)
            {
                double alpha = (u -U[j]) / (U[j+p] - U[j]);
                Rw.push_back(alpha * mvCPs[j] + (1-alpha) * mvCPs[j-1]);
            }
            mvCPs.insert(mvCPs.begin()+ k - p + 1, Eigen::Matrix<double, dim,1>::Zero());
            for (int j=k-p+1; j<= k;j++)
            {
                mvCPs[j] = Rw[j-k+p-1];
            }
            U.insert(U.begin() + k + 1, u);
        }

        uint CPsize() { return mvCPs.size(); }

        Knots & getKnots() { return mKnots; }
        int getP() { return mnP;}

    private:
        Knots mKnots;
        std::vector<Eigen::Matrix<double, dim,1>, Eigen::aligned_allocator<Eigen::Matrix<double, dim,1> > > mvCPs;
        double mdScale;
        int mnP;
    };
}
}
#endif //POLYVIEW_BSPLINE_HPP
