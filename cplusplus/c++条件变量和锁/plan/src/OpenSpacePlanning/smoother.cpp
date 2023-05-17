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

//#include "smoother.h"


//namespace HybridAStarPart {

//double Smoother::vecMole(std::vector<double>& x)
//{
//    double fSum = 0;
//    for(int i = 0; i < x.size(); i++)
//    {
//        fSum += x[i] * x[i];
//    }
//    fSum = sqrt(fSum);
//    return fSum;
//}

//std::vector<double> Smoother::numMultiply(double k, std::vector<double>& x)
//{
//    std::vector<double> y = x;
//    for(int i = 0; i < x.size(); i++)
//    {
//        y[i] = k * x[i];
//    }
//    return y;
//}

//std::vector<double> Smoother::vecAdd(std::vector<double>& a, std::vector<double>& b)
//{
//    std::vector<double> c = a;
//    for(int i = 0; i < a.size(); i++)
//    {
//        c[i] = a[i] + b[i];
//    }
//    return c;
//}

//double Smoother::vecDot(std::vector<double>& a, std::vector<double>& b)
//{
//    double fSum = 0;
//    for(int i = 0; i < a.size(); i++)
//    {
//        fSum += a[i] * b[i];
//    }
//    return fSum;
//}


//void Smoother::CalGradient(std::vector<Vector2D>& p,
//                           std::vector<double>& lamda,
//                           std::vector<double>& gkLam)
//{
//    int N = p.size() - 1;
//    Vector2D detaXim1, detaXi, detaXip1, detaXip2;
//    Vector2D zeros(0, 0);

//    Vector2D kim1, ki, kip1;
//    Vector2D Dki,Dkim1,Dkip1;
//    Vector2D uim1, ui, uip1, uip2;
//    Vector2D Duim1, Dui, Duip1, Duip2;
//    Vector2D DdeltaXiLamdai, DdeltaXip1Lamdai;
//    double DdeltaNormXiLamdai, DdeltaNormXip1Lamdai;
//    double normdetaXim1, normdetaXi, normdetaXip1, normdetaXip2;
//    double DObjLamdai =0.0;
//    double fkim1, fki, fkip1 = 0.0;
//    double g_curvatureTerm, g_movingTerm;

//    gkLam.clear();
//    gkLam.push_back(0.0);
//    gkLam.push_back(0.0);
//    for(int i = 2; i <= N - 2; i++)
//    {
//        detaXim1 = p[i - 1] - p[i - 2] + lamda[i - 1] * NormVector[i - 1] - lamda[i - 2] * NormVector[i - 2];
//        detaXi = p[i] - p[i - 1] + lamda[i] * NormVector[i] - lamda[i - 1] * NormVector[i - 1];
//        detaXip1 = p[i + 1] - p[i] + lamda[i + 1] * NormVector[i + 1] - lamda[i] * NormVector[i];
//        detaXip2 = p[i + 2] - p[i + 1] + lamda[i + 2] * NormVector[i + 2] - lamda[i + 1] * NormVector[i + 1];
//        normdetaXim1 = detaXim1.length();
//        normdetaXi = detaXi.length();
//        normdetaXip1 = detaXip1.length();
//        normdetaXip2 = detaXip2.length();

//        uim1 = detaXim1 / normdetaXim1;
//        ui = detaXi / normdetaXi;
//        uip1 = detaXip1 / normdetaXip1;
//        uip2 = detaXip2 / normdetaXip2;

//        kim1 = 1 / normdetaXim1 *(ui - uim1);
//        ki = 1 / normdetaXi *(uip1 - ui);
//        kip1 = 1 / normdetaXip1 *(uip2 - uip1);

//        DdeltaXiLamdai = NormVector[i];
//        DdeltaXip1Lamdai = -NormVector[i];

//        DdeltaNormXiLamdai = ui.dot(DdeltaXiLamdai);
//        DdeltaNormXip1Lamdai = uip1.dot(DdeltaXip1Lamdai);

//        Duim1 = zeros;

//        Dui = 1 / normdetaXi * DdeltaXiLamdai
//                - detaXi / (normdetaXi * normdetaXi) * ((detaXi / normdetaXi).dot(DdeltaXiLamdai));

//        Duip1 = 1 / normdetaXip1 * DdeltaXip1Lamdai
//                - detaXip1 / (normdetaXip1 * normdetaXip1) * ((detaXip1 / normdetaXip1).dot(DdeltaXip1Lamdai));

//        Duip2 = zeros;

//        Dkim1 = 1 / normdetaXim1 * (Dui - Duim1);
//        Dki = (-(uip1 - ui) * DdeltaNormXiLamdai) / (normdetaXi * normdetaXi)
//                + 1 / normdetaXi *(Duip1 - Dui);
//        Dkip1 = (-(uip2 - uip1) * DdeltaNormXip1Lamdai) / (normdetaXip1 * normdetaXip1)
//                + 1 / normdetaXip1 *(Duip2 - Duip1);

//        fkim1 = 2 * kim1.length() * (kim1 / kim1.length()).dot(Dkim1);
//        fki = 2 * ki.length() * (ki / ki.length()).dot(Dki);
//        fkip1 = 2 * kip1.length() * (kip1 / kip1.length()).dot(Dkip1);
//        if(kim1.length() < CAL_ERROR)
//        {
//            fkim1 = 0;
//        }
//        if(ki.length() < CAL_ERROR)
//        {
//            fki = 0;
//        }
//        if(kip1.length() < CAL_ERROR)
//        {
//            fkip1 = 0;
//        }

//        g_curvatureTerm = fkim1 + fki + fkip1;

//        g_movingTerm = 2 * (lamda[i]);

//        DObjLamdai = m_CoeffKappa * g_curvatureTerm + m_CoeffMoving * g_movingTerm;

//        if (std::isnan(DObjLamdai) || std::isinf(DObjLamdai))
//        {
//            gkLam.push_back(0.0);
//        }else{
//            gkLam.push_back(DObjLamdai);
//        }
//    }
//    gkLam.push_back(0.0);
//    gkLam.push_back(0.0);
//}

//void Smoother::CalNormalVector(Vector2D& xi,Vector2D& xip1, Vector2D& ans)
//{
//    Vector2D detaxi = xip1 - xi;
//    ans.setXY(- detaxi.getY(), detaxi.getX());

//    ans = ans / ans.length();
//}


//double Smoother::CalObjFun(std::vector<Vector2D>& p,
//                              std::vector<double>& lamda){
//    double ans = 0.0;
//    double normdetaXip1;
//    double normdetaXi;
//    Vector2D detaXi;
//    Vector2D detaXip1;
//    Vector2D ki,li;
//    int N = p.size() - 1;
//    for(int i = 1; i <= N - 1; i++)
//    {
//        li=lamda[i] * NormVector[i];

//        detaXi = p[i] - p[i - 1] + lamda[i] * NormVector[i] - lamda[i - 1] * NormVector[i - 1];
//        detaXip1 = p[i + 1] - p[i] + lamda[i + 1] * NormVector[i + 1] - lamda[i] * NormVector[i];

//        normdetaXi = detaXi.length();
//        normdetaXip1 = detaXip1.length();

//        ki = 1 / normdetaXi *(detaXip1 / normdetaXip1 - detaXi / normdetaXi);


//        ans += m_CoeffKappa *  ki.length()*ki.length() + m_CoeffMoving * li.length()*li.length()
//                + m_CoeffUniform * (normdetaXip1 - normdetaXi) * (normdetaXip1 - normdetaXi);

//    }
//    return ans;
//}



//double Smoother::CalPhiAlpha(double fAlpha,
//                             std::vector<Vector2D>& xk,
//                             std::vector<double> &lamda,
//                             std::vector<double>& dk)
//{

//    std::vector<double> vecTemp;
//    std::vector<double> lamdap1;

//    double fPhiAlpha;
//    vecTemp = numMultiply(fAlpha, dk);
//    lamdap1 = vecAdd(lamda, vecTemp);
//    fPhiAlpha = CalObjFun(xk, lamdap1);
//    return fPhiAlpha;

//}

//double Smoother::CalPhiAlphaDiff(double fAlpha,
//                                 std::vector<Vector2D>& xk,
//                                 std::vector<double> &lamda,
//                                 std::vector<double>& dk)
//{
//    double fDiffPhiAlpha;
//    std::vector<double> vecTemp;
//    std::vector<double> lamdap1;
//    std::vector<double> gkp1;

//    vecTemp = numMultiply(fAlpha, dk);
//    lamdap1 = vecAdd(lamda, vecTemp);
//    CalGradient(xk, lamdap1, gkp1);
//    fDiffPhiAlpha = vecDot(gkp1, dk);
//    return fDiffPhiAlpha;
//}


///**
// * @brief wolfe-powell准则求步长
// * @param xk 轨迹原始点
// * @param lamda 自变量，沿着法向量的移动长度
// * @param dk 迭代方向
// * @param GkMultiDk 目标函数在步长 = 0处的导数
// * @param Obj 目标函数在步长 = 0目标函数值
// * @return fAlpha 步长
// */
//double Smoother::LineSearchByWolfePowell(std::vector<Vector2D>& xk,
//                                         std::vector<double> &lamda,
//                                         std::vector<double>& dk,
//                                         double GkMultiDk,
//                                         double Obj)

//{
//    double fAlpha1 = 0.0;
//    double fAlpha2 = 0.01;
//    double fAlpha = fAlpha2;
//    double fAlphaNew = 0.0;
//    double fPhi0 = 0.0;
//    double fDPhi0 = 0.0;
//    double fPhiAlpha1 = 0.0;
//    double fPhiAlpha2 = 0.0;
//    double fPhiAlpha = 0.0;
//    double fDiffPhiAlpha1 = 0.0;
//    double fDiffPhiAlpha2 = 0.0;
//    double fDiffPhiAlpha = 0.0;
//    double rho = 0.1;
//    double sigma = 0.4;
//    double fVal1, fVal2, fVal3, fVal4 = 0;
//    int nIteraions= 0 ;

//    bool bCondition2 = false;
//    fPhi0 = Obj;
//    fDPhi0 = GkMultiDk;

//    fPhiAlpha = CalPhiAlpha(fAlpha, xk, lamda, dk);
//    fPhiAlpha1 = Obj;
//    fPhiAlpha2 = fPhiAlpha;

//    fDiffPhiAlpha = CalPhiAlphaDiff(fAlpha, xk, lamda, dk);
//    fDiffPhiAlpha1 = GkMultiDk;
//    fDiffPhiAlpha2 = fDiffPhiAlpha;

//    if (fDPhi0 >= 0)
//    {
//        fAlpha = 0;
//    }
//    else
//    {
//        while(nIteraions < 50 && (fDPhi0 < 0))
//        {

//            fVal1 = fPhiAlpha;
//            fVal2 = fPhi0 +  rho * fDPhi0 * fAlpha;

//            if(fVal1 <= fVal2 || bCondition2)
//            {
//                fVal3 = fabs(fDiffPhiAlpha);
//                fVal4 = -sigma *  fDPhi0;
//                if(fVal1 <= fVal2)
//                {
//                    bCondition2 = false;
//                    if(fVal3 <= fVal4)
//                    {
//                        break;
//                    }
//                    else
//                    {
//                        /*alpha对应导数为负和Alpha1点对应导数为负，说明之间不存在局部最小值，此时进行二点二次外插*/
//                        fAlphaNew = fAlpha + (fDiffPhiAlpha * (fAlpha - fAlpha1))/(fDiffPhiAlpha1 - fDiffPhiAlpha);

//                        fAlpha1 = fAlpha;
//                        fPhiAlpha1 = fPhiAlpha;
//                        fDiffPhiAlpha1 = fDiffPhiAlpha;

//                        fAlpha = fAlphaNew;
//                        fPhiAlpha = CalPhiAlpha(fAlpha, xk, lamda, dk);
//                        fDiffPhiAlpha = CalPhiAlphaDiff(fAlpha, xk, lamda, dk);

//                        bCondition2 = true;
//                    }
//                }
//                else
//                {
//                    if(bCondition2)
//                    {
//                        fAlpha = fAlpha1;
//                        qDebug("外插失败，退出本轮步长迭代");
//                        break;
//                    }

//                }

//            }
//            else //alpha对应的函数值大于value1
//            {
//                if(fDiffPhiAlpha1 < 0)
//                {
//                    if(fDiffPhiAlpha > 0)
//                    {
//                        /*alpha对应导数为正和Alpha1点对应导数为负，说明之间存在局部最小值，此时进行二点二次内插*/
//                        fAlphaNew = fAlpha1 - 0.5 * (fDiffPhiAlpha1 * pow(fAlpha - fAlpha1, 2)) / (fPhiAlpha - fPhiAlpha1 - fDiffPhiAlpha1*(fAlpha - fAlpha1));
//                        fAlpha2 = fAlpha;
//                        fPhiAlpha2 = fPhiAlpha;
//                        fDiffPhiAlpha2 = fDiffPhiAlpha;

//                        fAlpha = fAlphaNew;
//                        fPhiAlpha = CalPhiAlpha(fAlpha, xk, lamda, dk);
//                        fDiffPhiAlpha = CalPhiAlphaDiff(fAlpha, xk, lamda, dk);
//                    }
//                    else if(fDiffPhiAlpha > -CAL_ERROR)
//                    {
//                        /*alpha对应导数为很小的负值和Alpha1点对应导数为负，说明此时函数值很难下降，共轭梯度迭代已经接近停止，退出*/
//                        break;
//                    }
//                    else
//                    {
//                        /*alpha对应导数为较大负值和Alpha1点对应导数为负，说明之间不存在局部最小值，退出*/
//                        fAlpha2 = fAlpha;
//                        fPhiAlpha2 = fPhiAlpha;
//                        fDiffPhiAlpha2 = fDiffPhiAlpha;

//                        fAlpha = (fAlpha1 + fAlpha) / 2;
//                        fPhiAlpha = CalPhiAlpha(fAlpha, xk, lamda, dk);
//                        fDiffPhiAlpha = CalPhiAlphaDiff(fAlpha, xk, lamda, dk);
//                    }
//                }
//                else
//                {
//                    /*区间上边界大于等于0，此时进行步长求解无意义，退出*/
//                    fAlpha = 0;
//                    break;
//                }

//            }

//            if((std::isnan(fAlpha)) || (std::isinf(fAlpha)) || fabs(fAlpha) < 1e-7)
//            {
//                qDebug("fAlpha1 %f, fAlpha2 %f, fAlpha %f", fAlpha1, fAlpha2, fAlpha);
//                fAlpha = 0;
//                break;
//            }

//            nIteraions++;
//        }
//    }
//    return fAlpha;

//}


//void Smoother::updatePathkSample(std::vector<Vector2D>& pathi,
//                                 std::vector<double>& lamda){
//    for( int i = 1;i<pathi.size() - 1;i++){

//        pathi[i] = pathi[i] + lamda[i] * NormVector[i];

//    }
//}
//void Smoother::updateLamda(std::vector<double>& lamda,
//                           double& arfak,
//                           std::vector<double>& dk)
//{
//    for( int i = 2;i<lamda.size() - 2;i++)
//    {
//        lamda[i] = lamda[i] + arfak * dk[i];
//    }
//}

//void Smoother::updateDk(std::vector<double>& gk,
//                               double &betakm1,
//                               std::vector<double> &dk)
//{
//    for( int i = 0;i<gk.size();i++){
//        dk[i] =  - gk[i] + betakm1 * dk[i];
//    }
//    return;
//}


//void Smoother::AddPointInTrajStartAndEnd(std::vector<Node3D>& OPath)
//{

//    int8_t direction;
//    Node3D XiFirst;
//    Node3D XiLast;
//    double stepSize;
//    double x;
//    double y;
//    double yaw;

//    if(OPath.size() > 0)
//    {
//        direction = OPath[0].nDirection;
//        stepSize = sqrt(pow(OPath[0].x - OPath[1].x,2)+ pow(OPath[0].y-OPath[1].y,2));
//        yaw = OPath[0].t;
//        x= OPath[0].x - direction*stepSize*cos(yaw);
//        y= OPath[0].y - direction*stepSize*sin(yaw);
//        XiFirst.x = x;
//        XiFirst.y = y;
//        XiFirst.t = yaw;
//        XiFirst.nDirection = direction;

//        yaw = OPath.back().t;
//        x= OPath.back().x + direction*stepSize*cos(yaw);
//        y= OPath.back().y + direction*stepSize*sin(yaw);
//        XiLast.x = x;
//        XiLast.y = y;
//        XiLast.t = yaw;
//        XiLast.nDirection = direction;
//        OPath.insert(OPath.begin(),XiFirst);
//        OPath.insert(OPath.end(),XiLast);
//    }
//    else
//    {

//    }

//}

///**
// * @brief 终止准则
// * @param GkMole gk模
// * @param newGkMultiDk 目标函数在步长 = 0处的导数
// * @param iteration 迭代次数
// * @return 1-停止 0-继续
// */
//bool Smoother::StopIteration(double GkMole,
//                             double newGkMultiDk,
//                             int iteration)
//{
//    bool bJudge1 = false;
//    bool bJudge2 = false;
//    double c1 = 1e-3;
//    double c2 = 1e-3;

//    bJudge1 = (GkMole < c1) && (iteration < MAX_CG_ITERATIONS);

//    bJudge2 = (fabs(newGkMultiDk) < c2) && (iteration < MAX_CG_ITERATIONS);

//    if(bJudge1 || bJudge2)
//    {
//        return true;
//    }
//    else
//    {
//        return false;
//    }

//}

///**
// * @brief 共轭梯度非线性优化曲率
// * @param OPath 原始路径
// * @param SmoothedPath 输出优化路径
// */
//void Smoother::NonLinearOptimizationByConjugateGradient(std::vector<Node3D>& OPath,
//                                                        std::vector<Node3D>& SmoothedPath)
//{

//    int iteration = 0;
//    bool bFRCG = false;
//    bool bPRP = true;
//    double arfak = 0.0;
//    double betak;
//    double minans = 1e10;
//    double objVal = 0;
//    double GkMole = 0.0;
//    double Gkm1Mole = 0.0;

//    double lastObjVal = 0.0;
//    double newObjVal = 0.0;
//    double lastGkMultiDk = 0.0;
//    double newGkMultiDk = 0.0;


//    std::vector<Vector2D> xk,xkmin;
//    std::vector<double> lamda,lamdamin;
//    std::vector<double> gk;
//    std::vector<double> gkm1;
//    std::vector<double> dk;
//    std::vector<double> dkm1;
//    std::vector<double> gkTemp;
//    Vector2D dirI;
//    Vector2D Zeros(0,0);
//    Vector2D xki;
//    Vector2D xkip1;
//    Node3D xki3D;
//    std::vector<Node3D> OPath2 = OPath;
//    std::vector<CtrlPoint> InterpolatedPath;
//    CtrlPoint CtrlPoint_;
//    //init
//    m_CoeffKappa = 0.2;
//    m_CoeffMoving = 0.5;
//    m_CoeffKappaRate = 0.;
//    m_CoeffUniform = 0;
//    NormVector.clear();
//    if(OPath2.size() == 0)
//    {
//        ;
//    }
//    else
//    {
//        SmoothedPath.clear();

//        AddPointInTrajStartAndEnd(OPath2);

//        for( int i = 0;i < OPath2.size(); i++)
//        {
//            xki.setXY(OPath2[i].x, OPath2[i].y);
//            xkip1.setXY(OPath2[i + 1].x, OPath2[i + 1].y);
//            xk.push_back(xki);
//            if((i <= 1) || (i >= (OPath2.size() - 2)))
//            {
//                lamda.push_back(0.0);
//                NormVector.push_back(Zeros);
//            }
//            else
//            {
//                lamda.push_back(0.0);
//                CalNormalVector(xki, xkip1, dirI);
//                NormVector.push_back(dirI);
//            }
//        }
//        /*计算初始梯度*/
//        CalGradient(xk, lamda, gk);

//        /*计算初始迭代方向*/
//        dk = numMultiply(-1, gk);

//        /*计算gk模*/
//        GkMole = vecMole(gk);

//        /*计算目标函数在步长=0处的导数*/
//        newGkMultiDk = vecDot(gk, dk);

//        /*计算初始目标函数值*/
//        newObjVal = CalObjFun(xk, lamda);

//#ifdef PrintDetails

//        std::cout<<"iteration: "<<iteration<<" ,arfak:      "<<arfak<<" , gk mole:      "<<vecMole(gk)<<" ,objVal:"<<objVal<<std::endl;
//#endif

//        lamdamin = lamda;

//        while((!StopIteration(GkMole, newGkMultiDk, iteration)) || (iteration == 0))
//        {
//            lastObjVal = newObjVal;
//            lastGkMultiDk = newGkMultiDk;
//            Gkm1Mole = GkMole;
//            gkm1 = gk;
//            dkm1 = dk;

//            //1 采用wolfe-powell准则求步长
//            arfak = LineSearchByWolfePowell(xk, lamda, dkm1, lastGkMultiDk, lastObjVal);

//            //2 更新自变量lamda
//            updateLamda(lamda, arfak, dkm1);

//            //3 更新梯度gk
//            CalGradient(xk, lamda, gk);
//            GkMole = vecMole(gk);

//            //4 更新迭代方向dk
////            if(bFRCG)
//            {
//                /*FR非线性共轭梯度公式*/
//                betak = (GkMole * GkMole) / (Gkm1Mole * Gkm1Mole);
//            }
////            else if(bPRP)
////            {
////                /*PRP非线性共轭梯度公式*/
////                gkTemp = numMultiply(-1, gkm1);
////                gkTemp = vecAdd(gk, gkTemp);
////                betak = vecDot(gk, gkTemp) / (Gkm1Mole * Gkm1Mole);
////                betak = MAX(betak, 0);
////            }
////            else
////            {
////                /*共轭梯度下降(Conjugate Desent)非线性共轭梯度公式*/
////                betak = -(GkMole * GkMole) / lastGkMultiDk;
////            }

//            if(vecDot(gk, dk) > 0)
//            {
//                dk = numMultiply(-1, gk);
//            }
//            else
//            {
//                updateDk(gk, betak, dk);
//            }

//            /*5 计算目标函数在步长=0处的导数*/
//            newGkMultiDk = vecDot(gk, dk);

//            iteration ++ ;

//            //6 计算目标函数值objvalue
//            newObjVal = CalObjFun(xk,lamda);

//#ifdef PrintDetails
//            std::cout<<"iteration: "<<iteration<<",   arfak:"<<arfak<<",    gk mole:"<<GkMole<<",    gk*dk:"<<newGkMultiDk<<",     objVal:"<<newObjVal<<std::endl;
//#endif
//            if(newObjVal<minans)
//            {
//                minans = newObjVal;
//                lamdamin = lamda;
//            }
//        }
//        xkmin=xk;
//        updatePathkSample(xkmin,lamdamin);

//#ifdef  PrintDetails
//        for(int i=0; i<lamdamin.size(); i++)
//        {
//            qDebug("lamdamini %d: %f",i,lamdamin[i]);
//        }
//#endif

//        for( int i = 1; i< xkmin.size() - 1; i++)
//        {
//            CtrlPoint_.x = xkmin[i].getX();
//            CtrlPoint_.y = xkmin[i].getY();
//            InterpolatedPath.push_back(CtrlPoint_);
//            xki3D.x = xkmin[i].getX();
//            xki3D.y = xkmin[i].getY();
////            if(i == 0)
////            {
////                xki3D.setT(normalizeHeadingRad_0_2pi(atan2(xkmin[i + 1].getY() - xkmin[i].getY(), xkmin[i + 1].getX() - xkmin[i].getX())));
////            }
////            else
////            {
//                xki3D.t = normalizeHeadingRad_0_2pi(atan2(xkmin[i].getY() - xkmin[i - 1].getY(), xkmin[i].getX() - xkmin[i - 1].getX()));
////            }
//            xki3D.nDirection = OPath2[i].nDirection;
//            SmoothedPath.push_back(xki3D);
//        }

//        //CalDiscretePointCurvature(InterpolatedPath);
//#ifdef  PrintDetails

//        for( int i = 1; i < InterpolatedPath.size(); i++)
//        {
//            qDebug("k: %f, detaX: %f",  InterpolatedPath[i].kappa, hypot(InterpolatedPath[i].x - InterpolatedPath[i - 1].x, InterpolatedPath[i].y - InterpolatedPath[i - 1].y));
//        }
//#endif

//    }

//}

//}//namespace
