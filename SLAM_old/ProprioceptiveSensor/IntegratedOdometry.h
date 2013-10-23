/**
 * @file IntegratedOdometry.h
 * @author Daniele Molinari -- 238168
 * @version 1.0
 */

#pragma once

////include
//SLAM
#include "../Base/types.h"

namespace SLAM { namespace Models { namespace IntegratedOdometry {
    ////model functions
    /**
     * This model make the following assumpions:
     * Xv = (X, Y, THETA)^t
     * Z = (x, y, theta)^t
     * 
     * Where x, y and theta is the robot state estimation performed by an integrated odometry
     */
    ////perception
    VectorType H(const VectorType& Xv);
    MatrixType dH_dXv(const VectorType& Xv);
    ////distance, sorting and normalization
    VectorType Difference(const VectorType& z1, const VectorType& z2);
    
}
    ////model
    extern const ProprioceptiveModel IntegratedOdometryModel;
} }