/**
 * \file core.h
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

#pragma once

////include
//Eigen
#include <Eigen/Dense>

namespace SLAM {
    ////typedef
    typedef double ScalarType;
    
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>                VectorType;
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>   MatrixType;
    
    namespace Models {
        ////functions
        //the default distance function
        /**
        * @brief return the distance vector as @p v1 - @p v2
        * @p v1 first vector
        * @p v2 second vector
        * @return @p v1 - @p v2
        */
        VectorType DefaultDifference(const VectorType& v1, const VectorType& v2);
        /**
        * @brief return the distance vector as @p v1 - @p v2
        * @p v1 first vector
        * @p v2 second vector
        * @return @p v1 - @p v2
        */
        ScalarType DefaultDistance(const VectorType& v1, const VectorType& v2);
        //the default (fake) sort function
        /**
        * @brief strict weak ordering for std algorithm application
        * @p v1 first vector
        * @p v2 second vector
        * @return true if @p v1 comes first than @p v2
        */
        bool DefaultSort(const VectorType& v1, const VectorType& v2);
        //the default (fake) normalization function
        /**
        * @brief return the normalization of vector @p v
        * @p v the observation vector
        * @return the normalized vector @p v
        */
        VectorType DefaultNormalize(const VectorType& v);
    }
}