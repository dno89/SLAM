/**
 * \file Tricycle.h
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

#pragma once

////include
//SLAM
#include "../Base/types.h"

namespace SLAM { namespace Models { namespace Tricycle {
    ////model constants
    namespace Parameters {
        //the time increment in the discretized model
        extern double TimeIncrement;
        //the distance between the front and the rear wheel
        extern double L;
    }
    /**
     * Assuming that the following hold:
     * Xv = (X, Y, Theta)^t
     * U = (V, Delta)^t
     * 
     * Where V is the linear speed and Delta is the steering angle of the motor wheel.
     */
    ////model functions
    VectorType F(const VectorType& Xv, const VectorType& U);
    MatrixType dF_dXv(const VectorType& Xv, const VectorType& U);
}
    ////model
    extern const VehicleModel TricycleModel;
} }
