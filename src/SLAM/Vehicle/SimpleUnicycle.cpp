/**
 * \file SimpleUnicycle.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include <SLAM/Vehicle/SimpleUnicycle.h>

using namespace SLAM;
using namespace Eigen;

////model parameters
double SLAM::Models::SimpleUnicycle::Parameters::TimeIncrement = 50e-3; //50ms

////model functions
VectorType SLAM::Models::SimpleUnicycle::F(const VectorType& Xv, const VectorType& U) {
    using namespace SLAM::Models::SimpleUnicycle::Parameters;
//         cout << ">> F called with Xv = (" << Xv.transpose() << "), U = (" << U.transpose() << ")" << endl;
    Vector3d res(Xv);
    res[0] += U[0]*TimeIncrement*cos(Xv[2]+U[1]*TimeIncrement/2.0);
    res[1] += U[0]*TimeIncrement*sin(Xv[2]+U[1]*TimeIncrement/2.0);
    
    res[2] += U[1]*TimeIncrement;
    
    return res;
}
MatrixType SLAM::Models::SimpleUnicycle::dF_dXv(const VectorType& Xv, const VectorType& U) {
    using namespace SLAM::Models::SimpleUnicycle::Parameters;
//         cout << ">> dF_dXv called with Xv = (" << Xv.transpose() << "), U = (" << U.transpose() << ")" << endl;
    MatrixType J(3, 3);
    J <<    1, 0, (-U[0]*TimeIncrement*sin(Xv[2]+U[1]*TimeIncrement/2.0)),
            0, 1, (U[0]*TimeIncrement*cos(Xv[2]+U[1]*TimeIncrement/2.0)),
            0, 0, 1;
            
    return J;
}

////model definition
const VehicleModel SLAM::Models::SimpleUnicycleModel(SimpleUnicycle::F, SimpleUnicycle::dF_dXv);