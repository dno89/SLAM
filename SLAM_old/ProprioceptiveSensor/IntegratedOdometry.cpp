/**
 * @file IntegratedOdometry.cpp
 * @author Daniele Molinari -- 238168
 * @version 1.0
 */

#include "IntegratedOdometry.h"

using namespace SLAM;
using namespace std;

VectorType SLAM::Models::IntegratedOdometry::H(const VectorType& Xv) {
//         cout << ">> H called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    VectorType res(3);
    res << Xv[0], Xv[1], Xv[2];

    return res;
}

MatrixType SLAM::Models::IntegratedOdometry::dH_dXv(const VectorType& Xv) {
//         cout << ">> dH_dXv called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
    MatrixType J(3, 3);

    J <<    1,            0,        0,
            0,            1,        0,
            0,            0,        1;
    
    return J;
}

VectorType SLAM::Models::IntegratedOdometry::Difference(const VectorType& z1, const VectorType& z2) {
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
    static const double PI2 = 2*M_PI;
    VectorType res(3);
    res[0] = z1[0] - z2[0];
    res[1] = z1[1] - z2[1];
    
    //angular distance
    ScalarType theta1 = z1[2], theta2 = z2[2];
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
            res[2] = d1;
        } else {
            res[2] = -d1;
        }
    } else {
        if(theta1 > theta2) {
            res[2] = d2;
        } else {
            res[2] = -d2;
        }
    }
    
//     cout << "Final distance: (" << res.transpose() << ")" << endl;
//     DPRINT("PPL Difference between (" << z1.transpose() << ") and (" << z2.transpose() << ") is [" << res.transpose() << "]")
    return res;
}

const ProprioceptiveModel SLAM::Models::IntegratedOdometryModel(IntegratedOdometry::H, IntegratedOdometry::dH_dXv, IntegratedOdometry::Difference);