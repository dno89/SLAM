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
        extern double TimeIncrement;
    }
    
    ////model functions
    VectorType F(const VectorType& Xv, const VectorType& U);
    MatrixType dF_dXv(const VectorType& Xv, const VectorType& U);
}
    ////model
    extern const VehicleModel SimpleUnicycleModel;
} }
