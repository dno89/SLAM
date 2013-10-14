/**
 * \file PolarPointLandmark.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include "PolarPointLandmark.h"
#include "../Base/DMDebug.h"
//std
#include <cmath>

IMPORT_DEBUG_LOG()

using namespace SLAM;
using namespace std;

VectorType SLAM::Models::PolarPointLandmark::H(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> H called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    VectorType res(2);
    res << sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1])), atan2(Xm[1] - Xv[1], Xm[0] - Xv[0]) - Xv[2];

    return res;
}

MatrixType SLAM::Models::PolarPointLandmark::dH_dXv(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXv called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
    MatrixType J(2, 3);
    double rho = sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1]-Xm[1])*(Xv[1]-Xm[1]));

    J <<    (Xv[0] - Xm[0])/rho,            (Xv[1] - Xm[1])/rho,        0,
            -(Xv[1] - Xm[1])/(rho*rho),     (Xv[0] - Xm[0])/(rho*rho),  -1;
    
    return J;
}

MatrixType SLAM::Models::PolarPointLandmark::dH_dXm(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXm called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    MatrixType J(2, 2);
    double rho = sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1]));
    
    J <<    (-Xv[0] + Xm[0])/rho,          (-Xv[1] + Xm[1])/rho,
            (Xv[1] - Xm[1])/(rho*rho),     -(Xv[0] - Xm[0])/(rho*rho);
    
    return J;
}

VectorType SLAM::Models::PolarPointLandmark::G(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> G called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;    
    VectorType Xm(2);
    Xm << Xv[0] + Z[0]*cos(Xv[2]+Z[1]), Xv[1] + Z[0]*sin(Xv[2]+Z[1]);
    
    return Xm;
}

MatrixType SLAM::Models::PolarPointLandmark::dG_dXv(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dXc called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl; 
    MatrixType J(2, 3);
    J <<    1,  0,  -Z[0]*sin(Xv[2]+Z[1]),
            0,  1,  Z[0]*cos(Xv[2]+Z[1]);
    
    return J;
}

MatrixType SLAM::Models::PolarPointLandmark::dG_dZ(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dZ called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;        
    MatrixType J(2, 2);
    J <<    cos(Xv[2]+Z[1]),  -Z[0]*sin(Xv[2]+Z[1]),
            sin(Xv[2]+Z[1]),  Z[0]*cos(Xv[2]+Z[1]);
    
    return J;
}

VectorType SLAM::Models::PolarPointLandmark::Difference(const VectorType& z1, const VectorType& z2) {
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
    static const double PI2 = 2*M_PI;
    VectorType res(2);
    res[0] = z1[0] - z2[0];
    
    //angular distance
    ScalarType theta1 = z1[1], theta2 = z2[1];
//         cout << "z1: " << theta1 << ", z2: " << theta2 << endl;
    
    while(theta1 < 0.0) theta1 += PI2;
    theta1 = fmod(theta1, PI2);
    while(theta2 < 0.0) theta2 += PI2;
    theta2 = fmod(theta2, PI2);
//         cout << "theta1: " << theta1 << ", theta2: " << theta2 << endl;
    
    ScalarType d1 = fmod(theta1 - theta2 + PI2, PI2), d2 = fmod(PI2 + theta2 - theta1, PI2);
//         cout << "d1: " << d1 << ", d2: " << d2 << endl;
    
    if(d1 <= d2) {
        if(theta1 > theta2) {
            res[1] = d1;
        } else {
            res[1] = -d1;
        }
    } else {
        if(theta1 > theta2) {
            res[1] = d2;
        } else {
            res[1] = -d2;
        }
    }
    
//     cout << "Final distance: (" << res.transpose() << ")" << endl;
//     DPRINT("PPL Difference between (" << z1.transpose() << ") and (" << z2.transpose() << ") is [" << res.transpose() << "]")
    return res;
}

ScalarType SLAM::Models::PolarPointLandmark::Distance(const VectorType& z1, const VectorType& z2) {
    VectorType diff(SLAM::Models::PolarPointLandmark::Difference(z1, z2));
//     DPRINT("PPL Distance: " << sqrt(diff(0)*diff(0) + diff(1)*diff(1)))
    return sqrt(diff(0)*diff(0) + 10*diff(1)*diff(1));
    
    ///FIXME: this distance yields worse performances.. WHY??
//     const ScalarType dx = z1[0]*cos(z1[1]) - z2[0]*cos(z2[1]), dy = z1[0]*sin(z1[1]) - z2[0]*sin(z2[1]);
//     return sqrt(dx*dx + dy*dy);
}

bool Models::PolarPointLandmark::Sort ( const SLAM::VectorType& z1, const SLAM::VectorType& z2 ) {
    return z1[1] < z2[1];
}

VectorType SLAM::Models::PolarPointLandmark::Normalize(const VectorType& z) {
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
    static const double PI2 = 2*M_PI;
    VectorType res(z);
    
    while(res(1) < -M_PI) res(1) += PI2;
    while(res(1) > M_PI) res(1) -= PI2;
    
    return res;
}

const LandmarkModel SLAM::Models::PolarPointLandmarkModel(LandmarkPerceptionModel(PolarPointLandmark::H, PolarPointLandmark::dH_dXv, PolarPointLandmark::dH_dXm, PolarPointLandmark::Difference, PolarPointLandmark::Distance, PolarPointLandmark::Sort, PolarPointLandmark::Normalize), LandmarkInitializationModel(PolarPointLandmark::G, PolarPointLandmark::dG_dXv, PolarPointLandmark::dG_dZ));