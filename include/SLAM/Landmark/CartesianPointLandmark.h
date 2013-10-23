/**
 * \file CartesianPointLandmark.h
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

#pragma once

////include
//SLAM
#include "../Base/types.h"

namespace SLAM { namespace Models { namespace CartesianPointLandmark {
    ////model functions
    /**
     * This model make the following assumpions:
     * Xv = (X, Y, THETA)^t
     * Xm = (x, y)^t
     * Z = (xr, yr)^t
     * 
     * Where xr and yr are the relative coordinate of the landmark expressed wrt to the robot reference system.
     */
    ////perception
    VectorType H(const VectorType& Xv, const VectorType& Xm);
    MatrixType dH_dXv(const VectorType& Xv, const VectorType& Xm);
    MatrixType dH_dXm(const VectorType& Xv, const VectorType& Xm);
    ////initialization
    VectorType G(const VectorType& Xv, const VectorType& Z);
    MatrixType dG_dXv(const VectorType& Xv, const VectorType& Z);
    MatrixType dG_dZ(const VectorType& Xv, const VectorType& Z);
    ////distance, sorting and normalization
//     VectorType Difference(const VectorType& z1, const VectorType& z2);
//     ScalarType Distance(const VectorType& z1, const VectorType& z2);
    bool Sort(const VectorType& z1, const VectorType& z2);
//     VectorType Normalize(const VectorType& z);
    
}
    ////model
    extern const LandmarkModel CartesianPointLandmarkModel;
} }