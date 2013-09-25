////include
//SLAM
#include "core.h"
// #include "types.h"

using namespace SLAM;

VectorType DefaultDistance(const VectorType& v1, const VectorType& v2)  {
    return v1 - v2;
}