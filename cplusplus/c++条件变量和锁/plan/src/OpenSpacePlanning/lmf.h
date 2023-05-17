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
#ifndef LMF_H
#define LMF_H

#include <vector>
#include <iostream>
#include "vector2d.h"
#include "node3d.h"
#include "Eigen/Eigen"
#include"publicfunpathplan.h"

// define part
#define ObjGradVectSize (1e-3)
#define GammaRateLow (0.25)
#define GammaRateHigh (0.75)

//WK = 2 * WK_zxy, Wo = 2 * Wo_zxy;
#define Wk (0.4)
#define Wo (1)

using namespace std;

namespace HybridAStarPart
{
struct TrajPoint
{
    Vector2D pos;
    Vector2D yaw;
    Vector2D k;
    double yawAng;
    int fowdbck;
    TrajPoint():yawAng(0), fowdbck(0){}
};


class LMF_Optimizer
{
public:
    LMF_Optimizer()
    {
        Init();
    }
    void Init();

    void init_lmf(vector<Node3D>& OPath);

    void GetNormalVectorOfPi(Vector2D& xi,Vector2D& xip1, Vector2D& ans);

    void NormalVectorOfTrajPointsCal(vector<Node3D>& OPath);

    void UpdateTrajPoints();

    void TrajectoryPointsCurvatureCal();

    void GradientsOfLaterOffsetsCal();

    void GradientsOfCurvaturesCal();

    void PartialDifferential_ki_lamdaiM1(Vector2D &ki_lamdai, int i);

    void PartialDifferential_ki_lamdai(Vector2D &ki_lamdai1, int i);

    void PartialDifferential_ki_lamdaiP1(Vector2D &ki_lamdai2, int i);

    void ResidueFunsVectorCal();

    void JacobianMatrixOfResidueFunctionCal();

    void GradientOfObjectiveCal();

    void HesseMatrixOfObjectiveCal();

    void DescentDirectionCal();

    void DescentDirectionCal_DogLeg();

    float ObjFunValCal_CurLamda();

    float ObjFunValCal_CurLamda_RadiuRT();

    void GammaRateCal();

    void UpdateV();

    void UpdateIterLamda();

    void UpdateRadiusTrustRegion();

    bool CheckIfGradVectSmallEnough();

    void AddPointInTrajStartAndEnd(vector<Node3D>& OPath);

    void HybridAStarResultedTrajetoryOptimationByLMF(bool bNeedAddPoint,
                                                     vector<Node3D>& OPath,
                                                     vector<Node3D>& SmoothedPath);
    vector<TrajPoint>* GetTrajtoryCurrent()
    {
        return &TrajectoryCurrent;
    }

private:
    int TrajPointsNum;
    float V;
    float GammaRate;
    float RadiusTrustRegion;
    float ObjValCurrent;
    float ObjValUpdate;
    float DeltaObjVal;

    vector<TrajPoint> TrajectoryToBeSmoothed; // Hybrid A* results
    vector<TrajPoint> TrajectoryCurrent;
    vector<TrajPoint> TrajectoryLamdaUpdate;

    vector<Vector2D> Directions;
    vector<Vector2D> Residues, Residues_CurLamda_RadiusRT;
    vector<Vector2D> GradientsOfLatOffs;

    vector<vector<Vector2D>> GradientsOfCurvatures;
    vector<vector<Vector2D>> JacoMatrix;

    Eigen::MatrixXf HesseMatrixObj;
    Eigen::VectorXf GradVectorObj;

    Eigen::VectorXf LamdaCurrent, LamdaCurrent_CurLamda_RadiusRT;
    Eigen::VectorXf DescentDirection;


};

}

#endif // LMF_H
