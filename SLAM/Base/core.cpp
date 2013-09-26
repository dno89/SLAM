////include
//SLAM
#include "core.h"
// #include "types.h"

using namespace SLAM;

VectorType SLAM::DefaultDistance(const VectorType& v1, const VectorType& v2)  {
    return v1 - v2;
}