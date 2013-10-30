/**
 * \file PolarLineLandmark.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include <SLAM/Landmark/PolarLineLandmark.h>
#include <SLAM/Base/DMDebug.h>
// #include "../DataAssociation/DataAssociation.h"
//std
#include <cmath>

IMPORT_DEBUG_LOG()

using namespace SLAM;
using namespace std;

VectorType SLAM::Models::PolarLineLandmark::H(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> H called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    VectorType res(2);
    res[1] = Xm[1] - Xv[0]*cos(Xm[0]) - Xv[1]*sin(Xm[0]);
    res[0] = Xm[0] - Xv[2];
    
    ///FIXME
//     if(res[1] < 0.0) {
//         res[0] += M_PI;
//     }
//     res << Xm[0] - Xv[2], Xm[1] - Xv[0]*cos(Xm[0]) - Xv[1]*sin(Xm[0]);

    return res;
}

MatrixType SLAM::Models::PolarLineLandmark::dH_dXv(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXv called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
    MatrixType J(2, 3);
    J <<    0,              0,              -1,
            -cos(Xm[0]),    -sin(Xm[0]),    0;
    
    return J;
}

MatrixType SLAM::Models::PolarLineLandmark::dH_dXm(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXm called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    MatrixType J(2, 2);
    J <<    1,                                  0,
            Xv[0]*sin(Xm[0])-Xv[1]*cos(Xm[0]),  1;
    
    return J;
}

VectorType SLAM::Models::PolarLineLandmark::G(const VectorType& Xv, const VectorType& Z) {
    ///FIXME: TEST
//         cout << ">> G called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;    
    VectorType Xm(2);
    const double gamma = Xv[2]+Z[0];

    Xm[1] = Z[1] + Xv[0]*cos(gamma) + Xv[1]*sin(gamma);
    Xm[0] = gamma;
    
    ///FIXME
//     if(Xm[1] < 0) {
//         Xm[0] -= M_PI;
//     }
    
    return Xm;
}

MatrixType SLAM::Models::PolarLineLandmark::dG_dXv(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dXc called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl; 
    MatrixType J(2, 3);
    const double gamma = Xv[2]+Z[0];
    J <<    0,          0,          1,
            cos(gamma), sin(gamma), -Xv[0]*sin(gamma)+Xv[1]*cos(gamma);
    
    return J;
}

MatrixType SLAM::Models::PolarLineLandmark::dG_dZ(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dZ called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;        
    MatrixType J(2, 2);
    const double gamma = Xv[2]+Z[0];

    J <<    1,                                  0,
            -Xv[0]*sin(gamma)+Xv[1]*cos(gamma), 1;
    
    return J;
}

VectorType SLAM::Models::PolarLineLandmark::Difference(const VectorType& z1, const VectorType& z2) {
//     assert(false);
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
    static const double PI2 = 2*M_PI;
    VectorType res(2);
    if(z1[1] < 0 || z2[1] < 0) {
//         res[1] = std::numeric_limits<ScalarType>::max();
        DERROR("PolarLineLandmark::Difference ERROR: one 'r' is negative!")
    } else {
        res[1] = z1[1] - z2[1];
    }
    
    //angular distance
    ScalarType theta1 = z1[0], theta2 = z2[0];
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
            res[0] = d1;
        } else {
            res[0] = -d1;
        }
    } else {
        if(theta1 > theta2) {
            res[0] = d2;
        } else {
            res[0] = -d2;
        }
    }
    
//         cout << "Final distance: (" << res.transpose() << ")" << endl;
//     DPRINT("PPL Difference between (" << z1.transpose() << ") and (" << z2.transpose() << ") is [" << res.transpose() << "]")
    return res;
}

ScalarType SLAM::Models::PolarLineLandmark::Distance(const VectorType& z1, const VectorType& z2) {
    if(z1[1]*z2[1] < 0.0) {
//         DINFO("Polar Line Distance between (" << z1.transpose() << ") e (" << z2.transpose() << ") is: INFINITE")
        
        return numeric_limits<ScalarType>::max();
//         return 100.0;
    }
    VectorType diff(SLAM::Models::PolarLineLandmark::Difference(z1, z2));

    return sqrt(diff(0)*diff(0) + 10.0*diff(1)*diff(1));
}

bool Models::PolarLineLandmark::Sort ( const SLAM::VectorType& z1, const SLAM::VectorType& z2 ) {
    assert(false);
    return z1[1] < z2[1];
}

VectorType SLAM::Models::PolarLineLandmark::Normalize(const VectorType& z) {
//     assert(false);
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
    static const double PI2 = 2*M_PI;
    VectorType res(z);
    
    while(res(0) < -M_PI) res(0) += PI2;
    while(res(0) > M_PI) res(0) -= PI2;
    
    return res;
}

const LandmarkModel SLAM::Models::PolarLineLandmarkModel(LandmarkPerceptionModel(PolarLineLandmark::H, PolarLineLandmark::dH_dXv, PolarLineLandmark::dH_dXm, PolarLineLandmark::Difference, PolarLineLandmark::Distance, 2.0, PolarLineLandmark::Sort, PolarLineLandmark::Normalize), LandmarkInitializationModel(PolarLineLandmark::G, PolarLineLandmark::dG_dXv, PolarLineLandmark::dG_dZ));