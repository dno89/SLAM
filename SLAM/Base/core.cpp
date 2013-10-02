////include
//SLAM
#include "core.h"

namespace SLAM {

VectorType DefaultDistance(const VectorType& v1, const VectorType& v2)  {
    return v1 - v2;
}

bool DefaultSort ( const VectorType& v1, const VectorType& v2 ) {
    return v1[0] < v2[0];
}


}