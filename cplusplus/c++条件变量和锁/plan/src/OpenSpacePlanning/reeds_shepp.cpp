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
//* @file reeds_shepp.cpp
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

//#include "reeds_shepp.h"

//// The comments, variable names, etc. use the nomenclature from the Reeds & Shepp paper.

//namespace REEDS_SHEPP_CURVE
//{
//const double pi = 3.14159265;
//const double twopi = 2. * pi;
//const double RS_EPS = 1e-6;
//const double ZERO = 1e-200;

//inline double mod2pi(double x)
//{
//    double v = fmod(x, twopi);
//    if (v < -pi)
//        v += twopi;
//    else
//        if (v > pi)
//            v -= twopi;
//    return v;
//}
//inline void polar(double x, double y, double &r, double &theta)
//{
//    r = sqrt(x*x + y*y);
//    theta = atan2(y, x);
//}
//inline void tauOmega(double u, double v, double xi, double eta, double phi, double &tau, double &omega)
//{
//    double delta = mod2pi(u-v), A = sin(u) - sin(delta), B = cos(u) - cos(delta) - 1.;
//    double t1 = atan2(eta*A - xi*B, xi*A + eta*B), t2 = 2. * (cos(delta) - cos(v) - cos(u)) + 3;
//    tau = (t2<0) ? mod2pi(t1+pi) : mod2pi(t1);
//    omega = mod2pi(tau - u + v - phi) ;
//}

//// formula 8.1 in Reeds-Shepp paper
//inline bool LpSpLp(double x, double y, double phi, double &t, double &u, double &v)
//{
//    polar(x - sin(phi), y - 1. + cos(phi), u, t);
//    if (t >= -ZERO)
//    {
//        v = mod2pi(phi - t);
//        if (v >= -ZERO)
//        {
//            assert(fabs(u*cos(t) + sin(phi) - x) < RS_EPS);
//            assert(fabs(u*sin(t) - cos(phi) + 1 - y) < RS_EPS);
//            assert(fabs(mod2pi(t+v - phi)) < RS_EPS);
//            return true;
//        }
//    }
//    return false;
//}
//// formula 8.2
//inline bool LpSpRp(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double t1, u1;
//    double theta = 0.0;
//    polar(x + sin(phi), y - 1. - cos(phi), u1, t1);
//    u1 = u1*u1;
//    if (u1 >= 4.)
//    {
//        u = sqrt(u1 - 4.);
//        theta = atan2(2., u);
//        t = mod2pi(t1 + theta);
//        v = mod2pi(t - phi);
//        assert(fabs(2*sin(t) + u*cos(t) - sin(phi) - x) < RS_EPS);
//        assert(fabs(-2*cos(t) + u*sin(t) + cos(phi) + 1 - y) < RS_EPS);
//        assert(fabs(mod2pi(t-v - phi)) < RS_EPS);
//        return t>=-ZERO && v>=-ZERO;
//    }
//    return false;
//}

//// Add Family LRS by zxy
// inline bool LpRmS(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x + sin(phi), eta = y - 1. - cos(phi), u1;
//    u1=(xi*sin(phi)-eta*cos(phi))/2;
//    double ts1, us1, vs1;
//    double ts2, us2, vs2;
//    bool validity=false;
//    if(u1>=-1&&u1<=1&&(fabs(phi-pi/2)>RS_EPS)&&(fabs(phi+pi/2)>RS_EPS))
//    {
//       ts1=mod2pi(acos(u1)+phi);
//       us1=mod2pi(ts1 - phi);
//       vs1=(xi-2*sin(ts1))/cos(phi);

//       ts2=mod2pi(-acos(u1)+phi);
//       us2=mod2pi(ts2 - phi);
//       vs2=(xi-2*sin(ts2))/cos(phi);
//       if(ts1>=0.2&&us1<=-ZERO&&vs1<=-ZERO)
//       {
//           t=ts1;
//           u=us1;
//           v=vs1;
//           validity= true;
//       }
//       else if(ts2>=0.2&&us2<=-ZERO&&vs2<=-ZERO)
//       {
//           t=ts2;
//           u=us2;
//           v=vs2;
//           validity= true;
//       }
//       else
//       {
//           ;
//       }

//    }
//    return validity;
//}

// inline bool LnRnSn(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x + sin(phi), eta = y - 1. - cos(phi), u1;
//    u1=(xi*sin(phi)-eta*cos(phi))/2;
//    double ts1, us1, vs1;
//    double ts2, us2, vs2;
//    bool validity=false;
//    if(u1>=-1&&u1<=1&&(fabs(phi-pi/2)>RS_EPS)&&(fabs(phi+pi/2)>RS_EPS))
//    {
//       ts1=mod2pi(acos(u1)+phi);
//       us1=mod2pi(ts1 - phi);
//       vs1=(xi-2*sin(ts1))/cos(phi);

//       ts2=mod2pi(-acos(u1)+phi);
//       us2=mod2pi(ts2 - phi);
//       vs2=(xi-2*sin(ts2))/cos(phi);
//       if(ts1<-ZERO&&us1<=-ZERO&&vs1<=-ZERO)
//       {
//           t=ts1;
//           u=us1;
//           v=vs1;
//           validity= true;
//       }
//       else if(ts2<-ZERO&&us2<=-ZERO&&vs2<=-ZERO)
//       {
//           t=ts2;
//           u=us2;
//           v=vs2;
//           validity= true;
//       }
//       else
//       {
//           ;
//       }

//    }
//    return validity;
//}

// // Add Family LRS by zxy
//  inline bool SpLpSp(double x, double y, double phi, double &t, double &u, double &v)
// {
//      u=mod2pi(phi);
//      if(u!=0)
//      {
//          v=(y-1+cos(u))/sin(u);
//          t=x-sin(u)-v*cos(u);
//          float e1=(t+sin(u)+v*cos(u)-x);
//          float e2=(1-cos(u)+v*sin(u)-y);
//          assert((e1) < RS_EPS) ;
//          assert((e2) < RS_EPS) ;
//          return (t<=-ZERO&&u<=-ZERO&&v<=-ZERO);
//      }
//      return false;

// }

//void CSC(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_, L;
//    if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[14], t, u, v);
//        Lmin = L;
//    }
//    if (LpSpLp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[14], -t, -u, -v);
//        Lmin = L;
//    }
//    if (LpSpLp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[15], t, u, v);
//        Lmin = L;
//    }
//    if (LpSpLp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[15], -t, -u, -v);
//        Lmin = L;
//    }
//    if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[12], t, u, v);
//        Lmin = L;
//    }
//    if (LpSpRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[12], -t, -u, -v);
//        Lmin = L;
//    }
//    if (LpSpRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[13], t, u, v);
//        Lmin = L;
//    }
//    if (LpSpRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//        GetReedsSheppPath(&path, reedsSheppPathType[13], -t, -u, -v);
//}


//void SCS(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_, L;
//    if (SpLpSp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[22], t, u, v);
//        Lmin = L;
//    }
//    if (SpLpSp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[22], -t, -u, -v);
//        Lmin = L;
//    }
//    if (SpLpSp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[23], t, u, v);
//        Lmin = L;
//    }
//    if (SpLpSp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[23], -t, -u, -v);
//        Lmin = L;
//    }
//}

//void CpCnSn(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_, L;
//    if (LpRmS(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[18], t, u, v);
//        Lmin = L;
//    }
//    if (LpRmS(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[18], -t, -u, -v);
//        Lmin = L;
//    }
//    if (LpRmS(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[19], t, u, v);
//        Lmin = L;
//    }
//    if (LpRmS(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[19], -t, -u, -v);
//        Lmin = L;
//    }
//}



//void CnCnSn(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_, L;
//    if (LnRnSn(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[18], t, u, v);
//        Lmin = L;
//    }
//    if (LnRnSn(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[18], -t, -u, -v);
//        Lmin = L;
//    }
//    if (LnRnSn(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[19], t, u, v);
//        Lmin = L;
//    }
//    if (LnRnSn(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[19], -t, -u, -v);
//        Lmin = L;
//    }

//    // backwards
//    double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
//    if (LnRnSn(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[20], v, u, t);
//        Lmin = L;
//    }
//    if (LnRnSn(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[20], -v, -u, -t);
//        Lmin = L;
//    }
//    if (LnRnSn(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[21], v, u, t);
//        Lmin = L;
//    }
//    if (LnRnSn(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[21], -v, -u, -t);
//        Lmin = L;
//    }
//}


//// formula 8.3 / 8.4  *** TYPO IN PAPER ***
//inline bool LpRmL(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x - sin(phi), eta = y - 1. + cos(phi), u1, theta;
//    polar(xi, eta, u1, theta);
//    if (u1 <= 4.)
//    {
//        u = -2.*asin(.25 * u1);
//        t = mod2pi(theta + .5 * u + pi);
//        v = mod2pi(phi - t + u);
//        assert(fabs(2*(sin(t) - sin(t-u)) + sin(phi) - x) < RS_EPS);
//        assert(fabs(2*(-cos(t) + cos(t-u)) - cos(phi) + 1 - y) < RS_EPS);
//        assert(fabs(mod2pi(t-u+v - phi)) < RS_EPS);
//        return t>=-ZERO && u<=ZERO;
//    }
//    return false;
//}
//void CCC(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_, L;
//    if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[0], t, u, v);
//        Lmin = L;
//    }
//    if (LpRmL(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[0], -t, -u, -v);
//        Lmin = L;
//    }
//    if (LpRmL(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[1], t, u, v);
//        Lmin = L;
//    }
//    if (LpRmL(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[1], -t, -u, -v);
//        Lmin = L;
//    }

//    // backwards
//    double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
//    if (LpRmL(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[0], v, u, t);
//        Lmin = L;
//    }
//    if (LpRmL(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[0], -v, -u, -t);
//        Lmin = L;
//    }
//    if (LpRmL(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[1], v, u, t);
//        Lmin = L;
//    }
//    if (LpRmL(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//        GetReedsSheppPath(&path, reedsSheppPathType[1], -v, -u, -t);
//}
//// formula 8.7
//inline bool LpRupLumRm(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = .25 * (2. + sqrt(xi*xi + eta*eta));
//    if (rho <= 1.)
//    {
//        u = acos(rho);
//        tauOmega(u, -u, xi, eta, phi, t, v);
//        assert(fabs(2*(sin(t)-sin(t-u)+sin(t-2*u))-sin(phi) - x) < RS_EPS);
//        assert(fabs(2*(-cos(t)+cos(t-u)-cos(t-2*u))+cos(phi)+1 - y) < RS_EPS);
//        assert(fabs(mod2pi(t-2*u-v - phi)) < RS_EPS);
//        return t>=-ZERO && v<=ZERO;
//    }
//    return false;
//}
//// formula 8.8
//inline bool LpRumLumRp(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho = (20. - xi*xi - eta*eta) / 16.;
//    if (rho>=0 && rho<=1)
//    {
//        u = -acos(rho);
//        if (u >= -.5 * pi)
//        {
//            tauOmega(u, u, xi, eta, phi, t, v);
//            assert(fabs(4*sin(t)-2*sin(t-u)-sin(phi) - x) < RS_EPS);
//            assert(fabs(-4*cos(t)+2*cos(t-u)+cos(phi)+1 - y) < RS_EPS);
//            assert(fabs(mod2pi(t-v - phi)) < RS_EPS);
//            return t>=-ZERO && v>=-ZERO;
//        }
//    }
//    return false;
//}
//void CCCC(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_, L;
//    if (LpRupLumRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[2], t, u, -u, v);
//        Lmin = L;
//    }
//    if (LpRupLumRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[2], -t, -u, u, -v);
//        Lmin = L;
//    }
//    if (LpRupLumRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[3], t, u, -u, v);
//        Lmin = L;
//    }
//    if (LpRupLumRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[3], -t, -u, u, -v);
//        Lmin = L;
//    }

//    if (LpRumLumRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[2], t, u, u, v);
//        Lmin = L;
//    }
//    if (LpRumLumRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[2], -t, -u, -u, -v);
//        Lmin = L;
//    }
//    if (LpRumLumRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[3], t, u, u, v);
//        Lmin = L;
//    }
//    if (LpRumLumRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + 2.*fabs(u) + fabs(v))) // timeflip + reflect
//        GetReedsSheppPath(&path, reedsSheppPathType[3], -t, -u, -u, -v);
//}
//// formula 8.9
//inline bool LpRmSmLm(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x - sin(phi), eta = y - 1. + cos(phi), rho, theta;
//    polar(xi, eta, rho, theta);
//    if (rho >= 2.)
//    {
//        double r = sqrt(rho*rho - 4.);
//        u = 2. - r;
//        t = mod2pi(theta + atan2(r, -2.));
//        v = mod2pi(phi - .5*pi - t);
//        assert(fabs(2*(sin(t)-cos(t))-u*sin(t)+sin(phi) - x) < RS_EPS);
//        assert(fabs(-2*(sin(t)+cos(t))+u*cos(t)-cos(phi)+1 - y) < RS_EPS);
//        assert(fabs(mod2pi(t+pi/2+v-phi)) < RS_EPS);
//        return t>=-ZERO && u<=ZERO && v<=ZERO;
//    }
//    return false;
//}
//// formula 8.10
//inline bool LpRmSmRm(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
//    polar(-eta, xi, rho, theta);
//    if (rho >= 2.)
//    {
//        t = theta;
//        u = 2. - rho;
//        v = mod2pi(t + .5*pi - phi);
//        assert(fabs(2*sin(t)-cos(t-v)-u*sin(t) - x) < RS_EPS);
//        assert(fabs(-2*cos(t)-sin(t-v)+u*cos(t)+1 - y) < RS_EPS);
//        assert(fabs(mod2pi(t+pi/2-v-phi)) < RS_EPS);
//        return t>=-ZERO && u<=ZERO && v<=ZERO;
//    }
//    return false;
//}
//void CCSC(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_ - .5*pi, L;
//    if (LpRmSmLm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[4], t, -.5*pi, u, v);
//        Lmin = L;
//    }
//    if (LpRmSmLm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[4], -t, .5*pi, -u, -v);
//        Lmin = L;
//    }
//    if (LpRmSmLm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[5], t, -.5*pi, u, v);
//        Lmin = L;
//    }
//    if (LpRmSmLm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[5], -t, .5*pi, -u, -v);
//        Lmin = L;
//    }

//    if (LpRmSmRm(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[8], t, -.5*pi, u, v);
//        Lmin = L;
//    }
//    if (LpRmSmRm(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[8], -t, .5*pi, -u, -v);
//        Lmin = L;
//    }
//    if (LpRmSmRm(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[9], t, -.5*pi, u, v);
//        Lmin = L;
//    }
//    if (LpRmSmRm(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[9], -t, .5*pi, -u, -v);
//        Lmin = L;
//    }

//    // backwards
//    double xb = x*cos(phi) + y*sin(phi), yb = x*sin(phi) - y*cos(phi);
//    if (LpRmSmLm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[6], v, u, -.5*pi, t);
//        Lmin = L;
//    }
//    if (LpRmSmLm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[6], -v, -u, .5*pi, -t);
//        Lmin = L;
//    }
//    if (LpRmSmLm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[7], v, u, -.5*pi, t);
//        Lmin = L;
//    }
//    if (LpRmSmLm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[7], -v, -u, .5*pi, -t);
//        Lmin = L;
//    }

//    if (LpRmSmRm(xb, yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[10], v, u, -.5*pi, t);
//        Lmin = L;
//    }
//    if (LpRmSmRm(-xb, yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[10], -v, -u, .5*pi, -t);
//        Lmin = L;
//    }
//    if (LpRmSmRm(xb, -yb, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[11], v, u, -.5*pi, t);
//        Lmin = L;
//    }
//    if (LpRmSmRm(-xb, -yb, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//        GetReedsSheppPath(&path, reedsSheppPathType[11], -v, -u, .5*pi, -t);
//}
//// formula 8.11 *** TYPO IN PAPER ***
//inline bool LpRmSLmRp(double x, double y, double phi, double &t, double &u, double &v)
//{
//    double xi = x + sin(phi), eta = y - 1. - cos(phi), rho, theta;
//    polar(xi, eta, rho, theta);
//    if (rho >= 2.)
//    {
//        u = 4. - sqrt(rho*rho - 4.);
//        if (u <= ZERO)
//        {
//            t = mod2pi(atan2((4-u)*xi -2*eta, -2*xi + (u-4)*eta));
//            v = mod2pi(t - phi);
//            assert(fabs(4*sin(t)-2*cos(t)-u*sin(t)-sin(phi) - x) < RS_EPS);
//            assert(fabs(-4*cos(t)-2*sin(t)+u*cos(t)+cos(phi)+1 - y) < RS_EPS);
//            assert(fabs(mod2pi(t-v-phi)) < RS_EPS);
//            return t>=-ZERO && v>=-ZERO;
//        }
//    }
//    return false;
//}
//void CCSCC(double x, double y, double phi, ReedsSheppPath &path)
//{
//    double t, u, v, Lmin = path.totalLength_ - pi, L;
//    if (LpRmSLmRp(x, y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v)))
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[16], t, -.5*pi, u, -.5*pi, v);
//        Lmin = L;
//    }
//    if (LpRmSLmRp(-x, y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[16], -t, .5*pi, -u, .5*pi, -v);
//        Lmin = L;
//    }
//    if (LpRmSLmRp(x, -y, -phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // reflect
//    {
//        GetReedsSheppPath(&path, reedsSheppPathType[17], t, -.5*pi, u, -.5*pi, v);
//        Lmin = L;
//    }
//    if (LpRmSLmRp(-x, -y, phi, t, u, v) && Lmin > (L = fabs(t) + fabs(u) + fabs(v))) // timeflip + reflect
//        GetReedsSheppPath(&path, reedsSheppPathType[17], -t, .5*pi, -u, .5*pi, -v);
//}

//void GetReedsSheppPath(ReedsSheppPath *path, const ReedsSheppPathSegmentType* type,
//                              double t, double u, double v, double w, double x)
//{
//    path->type_ = type;
//    path->length_[0] = t;
//    path->length_[1] = u;
//    path->length_[2] = v;
//    path->length_[3] = w;
//    path->length_[4] = x;
//    path->totalLength_ = fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x);
//}

//ReedsSheppPath reedsShepp(double q0[3], double q1[3], double rho_)
//{
//    ReedsSheppPath path;
//    double dx = q1[0] - q0[0], dy = q1[1] - q0[1], dth = q1[2] - q0[2];
//    double c = cos(q0[2]), s = sin(q0[2]);
//    double x = (c*dx + s*dy) / rho_, y = (-s*dx + c*dy) / rho_;

//    CSC(x, y, dth, path);
////    CpCnSn(x, y, dth, path);
////    CnCnSn(x, y, dth, path);
////    SCS(x, y, dth, path);
//    CCC(x, y, dth, path);
//    CCCC(x, y, dth, path);
//    CCSC(x, y, dth, path);
//    CCSCC(x, y, dth, path);
//    path.rho_ = rho_;

//    return path;
//}

//std::vector<ReedsSheppPathSegmentType> type(ReedsSheppPath *path)
//{
//    std::vector<ReedsSheppPathSegmentType> type_list;
//    for ( int i = 0; i < 5; ++i)
//        type_list.push_back(path->type_[i]);

//    return type_list;
//}

//void ReedsShepp_interpolate(double q0[3], ReedsSheppPath &path, double fUnitArcLength, double rho_, double s[4])
//{

//    if (fUnitArcLength < 0.0) fUnitArcLength = 0.0;
//    if (fUnitArcLength > path.totalLength_) fUnitArcLength = path.totalLength_;

//    double phi, v;

//    s[0] =0.;
//    s[1] =0.;
//    s[3] =0.;
//    s[2] =q0[2];

//    for ( int i=0; i<5 && fUnitArcLength>1e-3; ++i)
//    {
//        if (path.length_[i]<0)
//        {
//            v = std::max(-fUnitArcLength, path.length_[i]);
//            fUnitArcLength += v;
//        }
//        else
//        {
//            v = std::min(fUnitArcLength, path.length_[i]);
//            fUnitArcLength -= v;
//        }
//        phi = s[2];
//        switch(path.type_[i])
//        {
//            case RS_LEFT:
//                s[0] += ( sin(phi+v) - sin(phi));
//                s[1] += (-cos(phi+v) + cos(phi));
//                s[2] = phi + v;
//                s[3] = 1 / rho_;
//                break;
//            case RS_RIGHT:
//                s[0] += (-sin(phi-v) + sin(phi));
//                s[1] += ( cos(phi-v) - cos(phi));
//                s[2] = phi - v;
//                s[3] = 1 / rho_;
//                break;
//            case RS_STRAIGHT:
//                s[0] += (v * cos(phi));
//                s[1] += (v * sin(phi));
//                s[3] = 0.0;
//                break;
//            case RS_NOP:
//                break;
//        }
//    }

//    s[0] = s[0] * rho_ + q0[0];
//    s[1] = s[1] * rho_ + q0[1];
//}

//void ReedsShepp_path_sample(double q0[3], double q1[3], double rho_, double step_size,
//                            std::vector<std::vector<double> > &points)
//{
//    double qnew[4] = {};
//    ReedsSheppPathSegmentType PathSegment[5];
//    double Length[5];
//    ReedsSheppPath path = REEDS_SHEPP_CURVE::reedsShepp(q0, q1, rho_);
//    int ndimen1;
//    int ndimen2;
//    int ndimen;
//    for(int i=0;i<5;i++)
//    {
//        PathSegment[i]=path.type_[i];
//        Length[i]=path.length_[i];
//    }
//    double length = rho_ * path.totalLength_;

//    for (double seg=step_size; seg<=length; seg+=step_size)
//    {
//        REEDS_SHEPP_CURVE::ReedsShepp_interpolate(q0, path, seg/rho_, rho_, qnew);
//        ndimen1=sizeof qnew ;
//        ndimen2=sizeof qnew[0];
//        ndimen=ndimen1/ndimen2;
//        std::vector<double> v(qnew, qnew + ndimen);
//        points.push_back(v);
//    }
//    qnew[0]=q1[0];
//    qnew[1]=q1[1];
//    qnew[2]=q1[2];
//    ndimen1=sizeof qnew ;
//    ndimen2=sizeof qnew[0];
//    ndimen=ndimen1/ndimen2;
//    std::vector<double> v(qnew, qnew + ndimen);
//    points.push_back(v);
//}

//}
