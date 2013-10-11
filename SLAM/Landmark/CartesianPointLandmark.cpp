/**
 * \file CartesianPointLandmark.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include "CartesianPointLandmark.h"
#include "../Base/DMDebug.h"
//std
#include <cmath>

IMPORT_DEBUG_LOG()

using namespace SLAM;
using namespace std;

VectorType SLAM::Models::CartesianPointLandmark::H(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> H called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    VectorType res(2);
    const double ct = cos(Xv[2]), st = sin(Xv[2]);
    res << ct*(Xm[0] - Xv[0]) + st*(Xm[1] - Xv[1]), -st*(Xm[0] - Xv[0]) + ct*(Xm[1] - Xv[1]);

    return res;
}

MatrixType SLAM::Models::CartesianPointLandmark::dH_dXv(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXv called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
    MatrixType J(2, 3);
    const double ct = cos(Xv[2]), st = sin(Xv[2]);

    J <<    -ct,    -st,    st*(Xv[0] - Xm[0]) - ct*(Xv[1] - Xm[1]),
            st,     -ct,    ct*(Xv[0] - Xm[0])  + st*(Xv[1] - Xv[1]);
    
    return J;
}

MatrixType SLAM::Models::CartesianPointLandmark::dH_dXm(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXm called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
    MatrixType J(2, 2);
    const double ct = cos(Xv[2]), st = sin(Xv[2]);
    
    J <<    ct,     st,
            -st,    ct;
    
    return J;
}

VectorType SLAM::Models::CartesianPointLandmark::G(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> G called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;    
    VectorType Xm(2);
    const double ct = cos(Xv[2]), st = sin(Xv[2]);
    
    Xm <<   ct*Z[0] - st*Z[1] + Xv[0],
            st*Z[0] + ct*Z[1] + Xv[1];
    
    return Xm;
}

MatrixType SLAM::Models::CartesianPointLandmark::dG_dXv(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dXc called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl; 
    MatrixType J(2, 3);
    const double ct = cos(Xv[2]), st = sin(Xv[2]);
    
    J <<    1,  0,  -st*Z[0] - ct*Z[1],
            0,  1,  ct*Z[0] - st*Z[1];
    
    return J;
}

MatrixType SLAM::Models::CartesianPointLandmark::dG_dZ(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dZ called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;        
    MatrixType J(2, 2);
    const double ct = cos(Xv[2]), st = sin(Xv[2]);
    
    J <<    ct, -st,
            st, ct;
    
    return J;
}

bool Models::CartesianPointLandmark::Sort ( const SLAM::VectorType& z1, const SLAM::VectorType& z2 ) {
    throw std::runtime_error("CarterianPointLandmark::Sort ERROR: no consistent sort defined!");
}

const LandmarkModel SLAM::Models::CartesianPointLandmarkModel(LandmarkPerceptionModel(CartesianPointLandmark::H, CartesianPointLandmark::dH_dXv, CartesianPointLandmark::dH_dXm, DefaultDifference, DefaultDistance, CartesianPointLandmark::Sort, DefaultNormalize), LandmarkInitializationModel(CartesianPointLandmark::G, CartesianPointLandmark::dG_dXv, CartesianPointLandmark::dG_dZ));