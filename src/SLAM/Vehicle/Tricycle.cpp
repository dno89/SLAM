/**
 * \file Tricycle.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include <SLAM/Vehicle/Tricycle.h>

using namespace SLAM;
using namespace Eigen;

////model parameters
double SLAM::Models::Tricycle::Parameters::TimeIncrement = 50e-3; //50ms
double SLAM::Models::Tricycle::Parameters::L = 1.0; //1m

////model functions
VectorType SLAM::Models::Tricycle::F(const VectorType& Xv, const VectorType& U) {
    using namespace SLAM::Models::Tricycle::Parameters;
	
    Vector3d res(Xv);
    res[0] += U[0]*TimeIncrement*cos(U[1])*cos(Xv[2]);
    res[1] += U[0]*TimeIncrement*cos(U[1])*sin(Xv[2]);
    res[2] += U[0]*TimeIncrement/L*sin(U[1]);
    
    return res;
}
MatrixType SLAM::Models::Tricycle::dF_dXv(const VectorType& Xv, const VectorType& U) {
    using namespace SLAM::Models::Tricycle::Parameters;

    MatrixType J(3, 3);
    J <<    1, 0, -U[0]*TimeIncrement*cos(U[1])*sin(Xv[2]),
            0, 1, U[0]*TimeIncrement*cos(U[1])*cos(Xv[2]),
            0, 0, 1;
            
    return J;
}

////model definition
const VehicleModel SLAM::Models::TricycleModel(Tricycle::F, Tricycle::dF_dXv);