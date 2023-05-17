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
#include"lmf.h"
namespace HybridAStarPart
{
void LMF_Optimizer::Init()
{
    TrajPointsNum = 0;
    V = 0.0;
    GammaRate = 0.0;
    RadiusTrustRegion = 0.0;
    ObjValCurrent = 0.0;
    ObjValUpdate = 0.0;
    DeltaObjVal = 0.0;
}

void LMF_Optimizer::init_lmf(vector<Node3D>& OPath)
{
    TrajPoint temp;

    TrajPointsNum = OPath.size();
    ObjValCurrent = 1.;
    DeltaObjVal = 10.;
    RadiusTrustRegion = 1.;
    GammaRate = 0.;

    DescentDirection.resize(TrajPointsNum);
    DescentDirection.fill(0.);

    LamdaCurrent.resize(TrajPointsNum);
    LamdaCurrent.fill(0);

    HesseMatrixObj.resize(TrajPointsNum,TrajPointsNum);
    HesseMatrixObj.fill(0.);

    GradVectorObj.resize(TrajPointsNum);
    GradVectorObj.fill(0.);

    TrajectoryToBeSmoothed.clear();
    TrajectoryCurrent.clear();

    /*calculate the pos of each trajectory point*/
    for(int i = 0; i < TrajPointsNum; i++)
    {
        temp.pos.setXY(OPath.at(i).x,OPath.at(i).y);
        temp.yawAng = OPath.at(i).t;
        temp.fowdbck = OPath.at(i).nDirection;

        TrajectoryToBeSmoothed.push_back(temp);
    }
    /*calculate the Directions of each trajectory point*/
    NormalVectorOfTrajPointsCal(OPath);

    /*calculate k of each trajectory point*/
    TrajectoryCurrent = TrajectoryToBeSmoothed;

    TrajectoryPointsCurvatureCal();
    GradientsOfLaterOffsetsCal();
    GradientsOfCurvaturesCal();
    GradientsOfLaterOffsetsCal();
    JacobianMatrixOfResidueFunctionCal();
    ResidueFunsVectorCal();

    // J^T * R
    GradientOfObjectiveCal();
    // J^T * J
    HesseMatrixOfObjectiveCal();

    DescentDirectionCal_DogLeg();
    GammaRateCal();
    UpdateRadiusTrustRegion();
    UpdateIterLamda();
}

/* calculate the unit normal vector of each trajectory point*/
void LMF_Optimizer::GetNormalVectorOfPi(Vector2D& xi,Vector2D& xip1, Vector2D& ans)
{
    Vector2D detaxi = xip1 - xi;
    ans.setXY(- detaxi.getY(), detaxi.getX());
    ans = ans / ans.length();
}

void LMF_Optimizer::NormalVectorOfTrajPointsCal(vector<Node3D>& OPath)
{
    Vector2D Zeros(0,0);
    Vector2D xki, xkip1, dirI;

    Directions.clear();

    for( int i = 0; i < TrajPointsNum; i++)
    {
        xki.setXY(OPath[i].x,  OPath[i].y);
        xkip1.setXY(OPath[i + 1].x, OPath[i + 1].y);

        if((i <= 1) || (i >= (TrajPointsNum - 2)))
        {
            /*p0 + p1, pN-2 + pN-1 are fixed points, not optimized !*/
            Directions.push_back(Zeros);
        }
        else
        {
            /*dirI is vertiacl to (xki--xkip1)*/
            GetNormalVectorOfPi(xki, xkip1, dirI);
            Directions.push_back(dirI);
        }
    }
}

void LMF_Optimizer::UpdateTrajPoints()
{
    // s1: update trajpoints
    LamdaCurrent(0) = 0;
    LamdaCurrent(1) = 0;
    LamdaCurrent(TrajPointsNum - 1) = 0;
    LamdaCurrent(TrajPointsNum - 2) = 0;

    for(int i = 0; i < TrajPointsNum; i++)
    {
        // update pos: unknowed variables are lamdas
        TrajectoryCurrent.at(i).pos = TrajectoryToBeSmoothed.at(i).pos + LamdaCurrent(i) * Directions.at(i);
    }
}

void LMF_Optimizer::TrajectoryPointsCurvatureCal()
{
    float ds = 0.0;
    Vector2D temp;
    for(int i = 1; i < TrajPointsNum; i++)
    {
        temp = TrajectoryCurrent.at(i).pos - TrajectoryCurrent.at(i - 1).pos;
        ds = temp.length();
        TrajectoryCurrent.at(i).yaw = temp / ds;

        TrajectoryCurrent.at(i).yawAng = atan2(TrajectoryCurrent.at(i).yaw.getY(), TrajectoryCurrent.at(i).yaw.getX());
    }
    TrajectoryCurrent.at(0).yaw = TrajectoryCurrent.at(1).yaw;

    for(int i = 1; i < TrajPointsNum - 1; i++)
    {
        temp = TrajectoryCurrent.at(i).pos - TrajectoryCurrent.at(i - 1).pos;
        ds = temp.length();

        temp = TrajectoryCurrent.at(i + 1).yaw - TrajectoryCurrent.at(i).yaw;
        TrajectoryCurrent.at(i).k = temp / ds;
    }
    TrajectoryCurrent.at(0).k = TrajectoryCurrent.at(1).k;
    TrajectoryCurrent.at(TrajPointsNum - 1).k = TrajectoryCurrent.at(TrajPointsNum - 2).k;
}

/*i -- i+1 -- i+2: patial k(i) / lamda(i)*/
/*i-1 -- i -- i+1: patial k(i) / lamda(i-1)*/
void LMF_Optimizer::PartialDifferential_ki_lamdaiM1(Vector2D &ki_lamdai, int i)
{
    /// (i == i - 1) + 1
    float x0i = TrajectoryToBeSmoothed.at(i).pos.getX(), y0i = TrajectoryToBeSmoothed.at(i).pos.getY();
    float x0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getX(), y0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getY();
    float lamdi = LamdaCurrent(i), lamdi1 = LamdaCurrent(i + 1);
    float xdi = Directions.at(i).getX(), ydi = Directions.at(i).getY();
    float xdi1 = Directions.at(i + 1).getX(), ydi1 = Directions.at(i + 1).getY();
    float rectangle = x0i1 - x0i + lamdi1 * xdi1;
    float triangle = y0i1 - y0i + lamdi1 * ydi1;
    float Den = pow(rectangle - lamdi * xdi, 2) + pow(triangle - lamdi * ydi, 2);
    float DerivDen = 2 *(rectangle - lamdi * xdi)*(-xdi) + 2 * (triangle - lamdi * ydi) * (-ydi);
    float NumeX = rectangle - lamdi * xdi, NumeY = triangle - lamdi * ydi;
    float DerivNumeX = -xdi, DerivNumeY = -ydi;
    /*Forgot the iterm of delta(Pi+2 - Pi+1) is also relevant with lamdi-1*/
    float deltaPoint_i1_x = x0i1 - x0i + lamdi1 * xdi1 - lamdi * xdi;
    float deltaPoint_i1_y = y0i1 - y0i + lamdi1 * ydi1 - lamdi * ydi;
    float NormLength_i1 = hypot(deltaPoint_i1_x, deltaPoint_i1_y);
    float theta_i1_x = deltaPoint_i1_x / NormLength_i1;
    float theta_i1_y = deltaPoint_i1_y / NormLength_i1;
    x0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getX(), y0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getY();
    float x0i2 = TrajectoryToBeSmoothed.at(i + 2).pos.getX(), y0i2 = TrajectoryToBeSmoothed.at(i + 2).pos.getY();
    lamdi1 = LamdaCurrent(i + 1);
    float lamdi2 = LamdaCurrent(i + 2);
    xdi1 = Directions.at(i + 1).getX(), ydi1 = Directions.at(i + 1).getY();
    float xdi2 = Directions.at(i + 2).getX(), ydi2 = Directions.at(i + 2).getY();
    float deltaPoint_i2_x = x0i2 - x0i1 + lamdi2 * xdi2 - lamdi1 * xdi1;
    float deltaPoint_i2_y = y0i2 - y0i1 + lamdi2 * ydi2 - lamdi1* ydi1;
    float NormLength_i2 = hypot(deltaPoint_i2_x, deltaPoint_i2_y);
    float theta_i2_x = deltaPoint_i2_x / NormLength_i2;
    float theta_i2_y = deltaPoint_i2_y / NormLength_i2;
    float ki_lamdai_add_x = theta_i2_x * (- 1. / NormLength_i1) * (theta_i1_x * (- xdi) + theta_i1_y * (- ydi));
    float ki_lamdai_add_y = theta_i2_y * (- 1. / NormLength_i1) * (theta_i1_x * (- xdi) + theta_i1_y * (- ydi));
    float ki_lamdai_x = (DerivNumeX * Den - NumeX * DerivDen ) / pow(Den,2) + ki_lamdai_add_x;
    float ki_lamdai_y = (DerivNumeY * Den - NumeY * DerivDen ) / pow(Den,2) + ki_lamdai_add_y;

    ki_lamdai.setX(-ki_lamdai_x * sqrt(Wk));
    ki_lamdai.setY(-ki_lamdai_y * sqrt(Wk));
}

/*i -- i+1 -- i+2: patial k(i) / lamda(i+2)*/
/*i-1 -- i -- i+1: patial k(i) / lamda(i+1)*/
void LMF_Optimizer::PartialDifferential_ki_lamdaiP1(Vector2D &ki_lamdai2, int i)
{
    float x0i = TrajectoryToBeSmoothed.at(i).pos.getX(), y0i = TrajectoryToBeSmoothed.at(i).pos.getY();
    float x0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getX(), y0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getY();
    float lamdi = LamdaCurrent(i), lamdi1 = LamdaCurrent(i + 1);
    float xdi = Directions.at(i).getX(), ydi = Directions.at(i).getY();
    float xdi1 = Directions.at(i + 1).getX(), ydi1 = Directions.at(i + 1).getY();
    float rectangle1 = x0i1 - x0i + lamdi1 * xdi1;
    float triangle1 = y0i1 - y0i + lamdi1 * ydi1;
    float Den1 = pow(rectangle1 - lamdi * xdi, 2) + pow(triangle1 - lamdi * ydi, 2);
    Den1 = sqrt(Den1);
    float x0i2 = TrajectoryToBeSmoothed.at(i + 2).pos.getX(), y0i2 = TrajectoryToBeSmoothed.at(i + 2).pos.getY();
    float lamdi2 = LamdaCurrent(i + 2);
    float xdi2 = Directions.at(i + 2).getX(), ydi2 = Directions.at(i + 2).getY();
    float rectangle2 = x0i2 - x0i1 - lamdi1 * xdi1;
    float triangle2 = y0i2 - y0i1 - lamdi1 * ydi1;
    float Den = pow(rectangle2 + lamdi2 * xdi2, 2) + pow(triangle2 + lamdi2 * ydi2, 2);
    float DerivDen = 2 *(rectangle2 + lamdi2 * xdi2)*(xdi2) + 2 * (triangle2 + lamdi2 * ydi2) * (ydi2);
    DerivDen = 0.5 * pow(Den, -0.5) * DerivDen;
    Den = sqrt(Den);
    float NumeX = rectangle2 + lamdi2 * xdi2, NumeY = triangle2 + lamdi2 * ydi2;
    float DerivNumeX = xdi2, DerivNumeY = ydi2;
    float ki_lamdai_x = (DerivNumeX * Den - NumeX * DerivDen)/pow(Den,2);
    float ki_lamdai_y = (DerivNumeY * Den - NumeY * DerivDen)/pow(Den,2);

    ki_lamdai2.setX(ki_lamdai_x * (1. / Den1) * sqrt(Wk));
    ki_lamdai2.setY(ki_lamdai_y * (1. / Den1) * sqrt(Wk));
}

/*i -- i+1 -- i+2: patial k(i) / lamda(i + 1) is the most complex*/
/*i-1 -- i -- i+1: patial k(i) / lamda(i)*/
void LMF_Optimizer::PartialDifferential_ki_lamdai(Vector2D &ki_lamdai1, int i)
{

    float x0i = TrajectoryToBeSmoothed.at(i).pos.getX(), y0i = TrajectoryToBeSmoothed.at(i).pos.getY();
    float x0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getX(), y0i1 = TrajectoryToBeSmoothed.at(i + 1).pos.getY();
    float lamdi = LamdaCurrent(i), lamdi1 = LamdaCurrent(i + 1);
    float xdi = Directions.at(i).getX(), ydi = Directions.at(i).getY();
    float xdi1 = Directions.at(i + 1).getX(), ydi1 = Directions.at(i + 1).getY();
    float rectangle = x0i1 - x0i - lamdi * xdi;
    float triangle = y0i1 - y0i - lamdi * ydi;
    float Den1 = pow(rectangle + lamdi1 * xdi1, 2) + pow(triangle + lamdi1 * ydi1, 2);
    float DerivDen1 = 2 *(rectangle + lamdi1 * xdi1) * (xdi1) + 2 * (triangle + lamdi1 * ydi1) * (ydi1);
    float NumeX1 = rectangle + lamdi1 * xdi1, NumeY1 = triangle + lamdi1 * ydi1;
    float DerivNumeX1 = xdi1, DerivNumeY1 = ydi1;
    float ki_lamdai_x1 = (DerivNumeX1 * Den1 - NumeX1 * DerivDen1) / pow(Den1,2);
    float ki_lamdai_y1 = (DerivNumeY1 * Den1 - NumeY1 * DerivDen1) / pow(Den1,2);
    float x0i2 = TrajectoryToBeSmoothed.at(i + 2).pos.getX(), y0i2 = TrajectoryToBeSmoothed.at(i + 2).pos.getY();
    float lamdi2 = LamdaCurrent(i + 2);
    float xdi2 = Directions.at(i + 2).getX(), ydi2 = Directions.at(i + 2).getY();
    float rectangle2 = x0i2 - x0i1 + lamdi2 * xdi2;
    float triangle2 = y0i2 - y0i1 + lamdi2 * ydi2;
    float Den2 = pow(rectangle2 - lamdi1 * xdi1, 2) + pow(triangle2 - lamdi1 * ydi1, 2);
    float DerivDen2 = 2 *(rectangle2 - lamdi1 * xdi1) * (-xdi1) + 2 * (triangle2 - lamdi1 * ydi1) * (-ydi1);
    DerivDen2 = 0.5 * pow(Den2, -0.5) * DerivDen2;
    float Den = pow(Den2, 0.5) * pow(Den1, 0.5);
    float DerivDen = 0.5 * pow(Den2, -0.5) * DerivDen2 * pow(Den1, 0.5) + pow(Den2, 0.5) * 0.5 * pow(Den1, -0.5) * DerivDen1;
    float NumeX = rectangle2 - lamdi1 * xdi1, NumeY = triangle2 - lamdi1 * ydi1;
    float DerivNumeX = -xdi1, DerivNumeY = -ydi1;
    float ki_lamdai_x2 = (DerivNumeX * Den - NumeX * DerivDen)/pow(Den,2);
    float ki_lamdai_y2 = (DerivNumeY * Den - NumeY * DerivDen)/pow(Den,2);
    ki_lamdai1.setX(-ki_lamdai_x1 * sqrt(Wk) + ki_lamdai_x2 * sqrt(Wk));
    ki_lamdai1.setY(-ki_lamdai_y1 * sqrt(Wk) + ki_lamdai_y2 * sqrt(Wk));
}

/*dK/dLamda: K is curvatures vetcor*/
void LMF_Optimizer::GradientsOfCurvaturesCal()
{
    vector<Vector2D> ARow;
    Vector2D Zero(0.0, 0.0);

    GradientsOfCurvatures.clear();

    for(int i = 0; i < TrajPointsNum; i++)
    {
        ARow.clear();
        /*PartialDifferential_ki_lamda is a vector, in which only PD_ki_lamdai, PD_ki_lamdai1,
        PD_ki_lamdai2 are not zeros, since ki is only relevent with (lamdai, lamdai1, lamdai2);*/
        for(int j = 0; j < TrajPointsNum; j++)
        {
            ARow.push_back(Zero);
        }
        if(i < TrajPointsNum - 1 && i > 0)
        {
            PartialDifferential_ki_lamdaiM1(ARow.at(i - 1), i - 1);
            PartialDifferential_ki_lamdai(ARow.at(i), i - 1);
            PartialDifferential_ki_lamdaiP1(ARow.at(i + 1), i - 1);
        }else{}
        GradientsOfCurvatures.push_back(ARow);
    }
}


/* dO/dLamda : O is later offset vector*/
void LMF_Optimizer::GradientsOfLaterOffsetsCal()
{
    GradientsOfLatOffs.clear();

    for(int i = 0; i < TrajPointsNum; i++)
    {
        GradientsOfLatOffs.push_back(Directions.at(i) * sqrt(Wo));
    }
}

/*Jk = dRk*/
void LMF_Optimizer::JacobianMatrixOfResidueFunctionCal()
{
    JacoMatrix.clear();

    vector<Vector2D> ARowOfJacoMatrix;
    Vector2D Zero(0.0, 0.0);
    for(int i = 0; i < 2 * TrajPointsNum; i++)
    {
        ARowOfJacoMatrix.clear();
        for(int j = 0; j < TrajPointsNum; j++)
        {
            ARowOfJacoMatrix.push_back(Zero);
        }

        if(i < TrajPointsNum)
        {
            ARowOfJacoMatrix = GradientsOfCurvatures.at(i);
        }
        else
        {
            ARowOfJacoMatrix.at(i - TrajPointsNum) = GradientsOfLatOffs.at(i - TrajPointsNum);
        }
        /*JacoMatrix: (2 * TrajPointsNum) * (TrajPointsNum * vector2D);
        The first TrajPointsNum rows are Gradients of Curvatures, in which the diagnoal elemetns of vector2D are non-Zeros;
        The Later TrajPointsNum rows are ones of Later Offsets and also is diagnoal matrix of vector2D*/
        JacoMatrix.push_back(ARowOfJacoMatrix);
    }
}

/*Rk*/
void LMF_Optimizer::ResidueFunsVectorCal()
{
    /*R = [K_T, O_T]T; 2N*/
    Residues.clear();

    for(int i = 0; i < TrajPointsNum; i++)
    {
        Residues.push_back(sqrt(Wk) * TrajectoryCurrent.at(i).k);
    }
    for(int i = 0; i < TrajPointsNum; i++)
    {
        Residues.push_back(sqrt(Wo) * LamdaCurrent(i) * Directions.at(i));
    }
}

/*Objective Function f's Gradient : gk = JkT * Rk; (TrajPointsNum * 2N) * (2N * 1)*/
void LMF_Optimizer::GradientOfObjectiveCal()
{
    float sum = 0.0;
    for(int i = 0; i < TrajPointsNum; i++)
    {
        sum = 0.0;
        for(int j = 0; j < 2 * TrajPointsNum; j++)
        {
            /*Transpose JacoMatrix : (2 * TrajPointsNum) * TrajPointsNum --> TrajPointsNum * (2 * TrajPointsNum)*/
            sum += JacoMatrix.at(j).at(i).dot(Residues.at(j));
        }
        GradVectorObj(i) = sum;
    }
}

/*Hesse Matrix of Objective Function f : Gk = JkT * Jk;*/
void LMF_Optimizer::HesseMatrixOfObjectiveCal()
{
    for(int i = 0; i < TrajPointsNum; i++)
    {
        for(int j = 0; j < TrajPointsNum; j++)
        {
            float sum = 0.;
            for(int kr = 0; kr < 2 * TrajPointsNum; kr++)
            {
                sum += JacoMatrix.at(kr).at(i).dot(JacoMatrix.at(kr).at(j));
            }
            HesseMatrixObj(i,j) = sum;
        }
    }
}

/*Decending Direction*/
void LMF_Optimizer::DescentDirectionCal()
{
    /*vector transform to matrix: HesseMatrixObj*/
    Eigen::MatrixXf Iden;
    Eigen::MatrixXf A;
    Eigen::VectorXf b;

    Iden.setIdentity(TrajPointsNum,TrajPointsNum);
    A = HesseMatrixObj + V * Iden;
    b = -GradVectorObj;
    DescentDirection = A.inverse() * b;
}

/*Apply Dogleg method to determine the Descent Direction dk*/
void LMF_Optimizer::DescentDirectionCal_DogLeg()
{
    float norm_sum = 0.0;
    float step_SD = 0.0;
    float beta = 0.0;
    float a, b, c = 0.0;
    float root1 = 0.0;
    float root2 = 0.0;
    Vector2D temp2D;
    Eigen::VectorXf temp;
    Eigen::VectorXf DescentDirection_GN; /*GaussNewton*/
    Eigen::VectorXf DescentDirection_SD = -GradVectorObj; /*SteepestDescent*/

    DescentDirection_GN = HesseMatrixObj.colPivHouseholderQr().solve(-GradVectorObj);

    for(int i = 0; i < 2 * TrajPointsNum; i++)
    {
        for(int j = 0; j < TrajPointsNum; j++)
        {
            /*cumulative ith row*/
            temp2D = temp2D + JacoMatrix.at(i).at(j) * DescentDirection_SD(j);
        }

        norm_sum += temp2D.sqlength();
    }
    step_SD = DescentDirection_SD.squaredNorm() / norm_sum;

    /*DogLeg Algorithm Start*/
    if(DescentDirection_GN.norm() <= RadiusTrustRegion)
    {
        DescentDirection = DescentDirection_GN;
    }
    else if(step_SD * DescentDirection_SD.norm() >= RadiusTrustRegion)
    {
        DescentDirection = RadiusTrustRegion * DescentDirection_SD / DescentDirection_SD.norm();
    }
    else
    {
        temp = DescentDirection_GN - step_SD * DescentDirection_SD;
        a = temp.squaredNorm();
        b = 2 * step_SD * DescentDirection_SD.transpose() * (DescentDirection_GN - step_SD * DescentDirection_SD);
        c = pow(step_SD, 2) * DescentDirection_SD.squaredNorm() - pow(RadiusTrustRegion, 2);

        if(fabs(a) > 1e-10 && (pow(b, 2) - 4 * a *c) >= 0)
        {
            root1 = (-b + sqrt(pow(b, 2) - 4 * a *c)) / (2 * a);
            root2 = (-b - sqrt(pow(b, 2) - 4 * a *c)) / (2 * a);
        }
        else
        {
            GAC_LOG_WARN("WRONG: Coeffient a is ZERO !\n");
        }

        if(root1 > 0 && root1 < 1)
        {
            beta = root1;
        }
        else
        {
            beta = root2;
        }
        DescentDirection = (1 - beta) * step_SD * DescentDirection_SD + beta * DescentDirection_GN;
    }
}


/*Objective function value calculate*/
float LMF_Optimizer::ObjFunValCal_CurLamda()
{
    float ObjVal = 0.0;
    /* fk = RkT*Rk
    i = 0; i < 2 * TrajPointsNum;*/
    for(int i = 1; i < 2 * TrajPointsNum - 1; i++)
    {
        if(i < TrajPointsNum - 1)
        {
            ObjVal += Residues.at(i).sqlength();
        }
        if(i > TrajPointsNum)
        {
            ObjVal += Residues.at(i).sqlength();
        }
    }

    return ObjVal;
}

/* calculate the objective value at (lamdaCur + dk)*/
float LMF_Optimizer::ObjFunValCal_CurLamda_RadiuRT()
{
    float ds = 0.0;
    float ObjVal = 0.0;

    Vector2D temp;
    TrajPoint posUpdate;

    // (lamdaCur + dk)
    LamdaCurrent_CurLamda_RadiusRT = LamdaCurrent + DescentDirection;

    LamdaCurrent_CurLamda_RadiusRT(0) = 0;
    LamdaCurrent_CurLamda_RadiusRT(1) = 0;
    LamdaCurrent_CurLamda_RadiusRT(TrajPointsNum - 1) = 0;
    LamdaCurrent_CurLamda_RadiusRT(TrajPointsNum - 2) = 0;

    // update trajpoints
    TrajectoryLamdaUpdate.clear();
    for(int i = 0; i < TrajPointsNum; i++)
    {
        // update pos: unknowed variables are lamdas
        posUpdate.pos.setX(TrajectoryToBeSmoothed.at(i).pos.getX() + LamdaCurrent_CurLamda_RadiusRT(i) * Directions.at(i).getX());
        posUpdate.pos.setY(TrajectoryToBeSmoothed.at(i).pos.getY() + LamdaCurrent_CurLamda_RadiusRT(i) * Directions.at(i).getY());
        TrajectoryLamdaUpdate.push_back(posUpdate);
    }

    /*TrajectoryLamdaUpdate's curvatures*/
    for(int i = 0; i < TrajPointsNum - 1; i++)
    {
        temp = TrajectoryToBeSmoothed.at(i).fowdbck * (TrajectoryLamdaUpdate.at(i + 1).pos - TrajectoryLamdaUpdate.at(i).pos);
        ds = temp.length();
        TrajectoryLamdaUpdate.at(i).yaw = temp / ds;
        TrajectoryLamdaUpdate.at(i).yawAng = atan2(TrajectoryLamdaUpdate.at(i).yaw.getY(), TrajectoryLamdaUpdate.at(i).yaw.getX());
    }

    for(int i = 0; i < TrajPointsNum - 2; i++)
    {
        temp = TrajectoryLamdaUpdate.at(i + 1).pos - TrajectoryLamdaUpdate.at(i).pos;
        ds = temp.length();
        temp = TrajectoryLamdaUpdate.at(i + 1).yaw - TrajectoryLamdaUpdate.at(i).yaw;
        TrajectoryLamdaUpdate.at(i).k = temp / ds;
    }

    // R = [K_T, O_T]T; 2N
    Residues_CurLamda_RadiusRT.clear();

    for(int i = 0; i < TrajPointsNum; i++)
    {
        Residues_CurLamda_RadiusRT.push_back(sqrt(Wk) * TrajectoryLamdaUpdate.at(i).k);
    }
    for(int i = 0; i < TrajPointsNum; i++)
    {
        Residues_CurLamda_RadiusRT.push_back(sqrt(Wo) * LamdaCurrent_CurLamda_RadiusRT(i) * Directions.at(i));
    }
    /* fk = RkT * Rk*/
    for(int i = 0; i < 2 * TrajPointsNum; i++)
    {
        ObjVal += Residues_CurLamda_RadiusRT.at(i).sqlength();
    }

    return ObjVal;
}


// Gamma
void LMF_Optimizer::GammaRateCal()
{
    float deltaqk = 0.0;
    float dTJTJd, dTJTR = 0.0;

    /*1, calculate deltafk*/
    ObjValCurrent = ObjFunValCal_CurLamda();
    ObjValUpdate = ObjFunValCal_CurLamda_RadiuRT();
    DeltaObjVal = ObjValCurrent - ObjValUpdate;

    /*2, calculate deltaqk
     general method calculating deltaqk*/
    dTJTJd = DescentDirection.transpose() * HesseMatrixObj * DescentDirection;
    dTJTR = DescentDirection.transpose() * GradVectorObj;
    deltaqk = -0.5 * dTJTJd - dTJTR;

    /*3, calculate GammaRate*/
    if(deltaqk <= 0)
    {
        GAC_LOG_WARN("WRONGS: deltaqk <= 0 !\n");
    }
    else
    {
        GammaRate = DeltaObjVal / deltaqk;
    }
}

/*Update v value*/
void LMF_Optimizer::UpdateV()
{
    if(GammaRate < GammaRateLow)
    {
        V = 4.0 * V;
    }
    else if (GammaRate > GammaRateHigh)
    {
        V = V / 2.0;
    }
    else
    {
        ;
    }
}

/*update Radius of Trust Region*/
void LMF_Optimizer::UpdateRadiusTrustRegion()
{
    if(GammaRate < GammaRateLow)
    {
        RadiusTrustRegion /= 4.0;
    }
    else if ((GammaRate > GammaRateHigh) && fabs(DescentDirection.norm() - RadiusTrustRegion) < 1e-10)
    {
        RadiusTrustRegion *= 2.0;
    }
    else
    {
        ;
    }
}


/*Update Lamda */
void LMF_Optimizer::UpdateIterLamda()
{
    if(GammaRate <= 0.0)
    {
        ;
    }
    else
    {
        LamdaCurrent += DescentDirection;
    }
}

/*CheckIfGradVectSmallEnough*/
bool LMF_Optimizer::CheckIfGradVectSmallEnough()
{
    /*||gk|| < threshold */
    double sum = GradVectorObj.norm();
    if (sum > ObjGradVectSize)
    {
        return false;
    }

    return true;
}

// Call Entry
void LMF_Optimizer::HybridAStarResultedTrajetoryOptimationByLMF(bool bNeedAddPoint,
                                                                vector<Node3D>& OPath,
                                                                vector<Node3D>& SmoothedPath)
{
    int iter = 0;
    Node3D temp3D;
    Vector2D temp2D;

    int nDeltaNum = 1;
//    bool bNeedAddPoint = true;

//    if(OPath.size() <= 7)
//    {
//        bNeedAddPoint = true;
//        nDeltaNum = 1;
//    }
//    else
//    {
//        bNeedAddPoint = false;
//        nDeltaNum = 0;
//    }
    if(bNeedAddPoint)
    {
         nDeltaNum = 1;
    }
    else
    {
         nDeltaNum = 0;
    }

    if(bNeedAddPoint)
    {
        AddPointInTrajStartAndEnd(OPath);
    }


    init_lmf(OPath);
    while(iter < 1e3 && !CheckIfGradVectSmallEnough()
          && fabs(DeltaObjVal / ObjValCurrent) > 1e-3) // Stop condition
    {
        // Calculation Prcoess
        UpdateTrajPoints();

        TrajectoryPointsCurvatureCal();
        GradientsOfCurvaturesCal();
        GradientsOfLaterOffsetsCal();
        JacobianMatrixOfResidueFunctionCal();

        ResidueFunsVectorCal();
        GradientOfObjectiveCal();
        HesseMatrixOfObjectiveCal();
        DescentDirectionCal_DogLeg();

        GammaRateCal();
        UpdateRadiusTrustRegion();
        UpdateIterLamda();
        iter++;

        GAC_LOG_DEBUG("iteration: %d, gk mole: %f, objVal: %f\n", iter, GradVectorObj.norm(), ObjValCurrent);
    }

    SmoothedPath.clear();

    for(int i = nDeltaNum; i < TrajPointsNum - nDeltaNum; i++)
    {
        /*TrajPoints are Transformed to Node3D;*/
        temp3D.x = TrajectoryCurrent.at(i).pos.getX();
        temp3D.y = TrajectoryCurrent.at(i).pos.getY();

        /* TrajectoryToBeSmoothed.at(i).fowdbck*/
        if(i == TrajPointsNum - 1)
        {
            temp3D.t = normalizeHeadingRad_0_2pi(OPath.back().t);
        }
        else
        {
            temp2D = (TrajectoryCurrent.at(i + 1).pos - TrajectoryCurrent.at(i).pos);
            temp3D.t = normalizeHeadingRad_0_2pi(atan2(temp2D.getY(), temp2D.getX()));
        }

        temp3D.nDirection = TrajectoryCurrent.at(i).fowdbck;

        SmoothedPath.push_back(temp3D);
    }
}

void LMF_Optimizer::AddPointInTrajStartAndEnd(std::vector<Node3D>& OPath)
{
    int8_t direction = 0;
    float stepSize = 0.0;
    float x = 0.0;
    float y = 0.0;
    float yaw = 0.0;

    Node3D XiFirst;
    Node3D XiLast;

    direction = OPath[0].nDirection;
    stepSize = sqrtf(powf(OPath[0].x-OPath[1].x,2)+ powf(OPath[0].y -OPath[1].y,2));
    yaw = OPath[0].t;
    x= OPath[0].x - direction*stepSize*cos(yaw);
    y= OPath[0].y - direction*stepSize*sin(yaw);
    XiFirst.x = x;
    XiFirst.y = y;
    XiFirst.t = yaw;
    XiFirst.nDirection = direction;
    OPath.insert(OPath.begin(),XiFirst);

    yaw = OPath.back().t;
    x= OPath.back().x + direction*stepSize*cos(yaw);
    y= OPath.back().y + direction*stepSize*sin(yaw);
    XiLast.x = x;
    XiLast.y = y;
    XiLast.t = yaw;
    XiLast.nDirection = direction;
    OPath.insert(OPath.end(),XiLast);
}
}
