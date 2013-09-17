// #ifndef EKFSLAMENGINE_H
// #define EKFSLAMENGINE_H
#pragma once

////include
//std
#include <stdexcept>
#include <cassert>
//SLAM
#include "../base/core.h"
#include "../base/types.h"

namespace SLAM {
    class SLAMEngine {
    public:
        ////CONSTRUCTOR
        SLAMEngine() {
        }
        
        ////INTERFACE
        /**
         * @brief the function setup the foundamental parameters
         * @p vehicle_state_size the size of the vehicle state
         * @p initial_state_estimation the initial state
         * @p initial_covariance_estimation the initial covariance matrix of the state
         * @p vehicle_model the vehicle model according to 
         */
        void Setup(int vehicle_state_size, const VectorType& initial_state_estimation, const MatrixType& initial_covariance_estimation, const VehicleModel& vehicle_model) {
            //set the size
            m_XvSize = vehicle_state_size;
            
            //set the remainig parameters with some debug assertions
            assert(initial_state_estimation.rows() == m_XvSize);
            m_Xv = initial_state_estimation;
            
            assert(initial_covariance_estimation.cols() == initial_covariance_estimation.rows() == m_XvSize);
            m_Pvv = initial_covariance_estimation;
            
            assert(vehicle_model);
            m_vModel = vehicle_model;
            
            m_init = true;
        }
        
        /**
         * @brief make the prediction step of the EKF filter
         * @p u the control input
         * @p Q the process noise covariance matrix
         */
        void Predict(VectorType u, MatrixType Q) {
            check();
            
            //use the old state to the the Taylor 1st order approximation
            MatrixType df_dXv = m_vModel.dF_dXv(m_Xv, u);
            
            //predict the vehicle state
            m_Xv = m_vModel.F(m_Xv, u);
            //Xv is now Xv-(k)
            
            //landmarks are supposed to be static: no update needed
            
            //update the covariance matrices
            //Pvv
            m_Pvv = df_dXv * m_Pvv * df_dXv.transpose() + Q;
            assert(m_Pvv.cols() == m_Pvv.rows() == m_XvSize);
            
            //Pvm
            int eta = m_Pvm.cols();
            m_Pvm = df_dXv * m_Pvm;
            assert(m_Pvm.rows() == m_XvSize && m_Pvm.cols() == eta);
        }
        
        
    private:
        ////DATA
        //FLAGS
        bool m_init = false;
        
        //VEHICLE STATE RELATED
        //the size of the vehicle state
        int m_XvSize;
        //the actual vehicle state
        VectorType m_Xv;
        //the vehicle model with Jacobian
        VehicleModel m_vModel;
        
        //LANDMARKS
        std::vector<Landmark> m_landmarks;
        
        //COVARIANCE
        //the vehicle-vehicle covariance matrix
        MatrixType m_Pvv;
        //the vehicle-landmarks covariance matrix
        MatrixType m_Pvm;
        //the landmarks-landmarks covariance matrix
        MatrixType m_Pmm;
        
        ////SUPPORT FUNCTIONS
        void check() {
            if(!m_init) throw std::runtime_error("SLAMEngine ERROR: a function has been called with the object not propertly initialize (call Setup before using the object)\n");
        }
    };
}

// #endif  //EKFSLAMENGINE_H