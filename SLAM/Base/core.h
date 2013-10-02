// #ifndef CORE_H
// #define CORE_H
#pragma once

////include
//Eigen
#include <Eigen/Dense>

namespace SLAM {
    ////typedef
    typedef double ScalarType;
    
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>                VectorType;
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>   MatrixType;
    
    
    ////functions
    //the default distance function
    /**
     * @brief return the distance vector as @p v1 - @p v2
     * @p v1 first vector
     * @p v2 second vector
     * @return @p v1 - @p v2
     */
    VectorType DefaultDistance(const VectorType& v1, const VectorType& v2);
    //the default (fake) sort function
    /**
     * @brief strict weak ordering for std algorithm application
     * @p v1 first vector
     * @p v2 second vector
     * @return true if @p v1 comes first than @p v2
     */
    bool DefaultSort(const VectorType& v1, const VectorType& v2);
}
// #endif  //CORE_H