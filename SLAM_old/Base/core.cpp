/**
 * \file core.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include "core.h"

namespace SLAM { namespace Models {

VectorType DefaultDifference(const VectorType& v1, const VectorType& v2)  {
    return v1 - v2;
}

ScalarType DefaultDistance(const VectorType& v1, const VectorType& v2)  {
    return DefaultDifference(v1, v2).norm();
}

bool DefaultSort ( const VectorType& v1, const VectorType& v2 ) {
    return v1[0] < v2[0];
}

VectorType DefaultNormalize( const VectorType& v ) {
    return v;
}

} }