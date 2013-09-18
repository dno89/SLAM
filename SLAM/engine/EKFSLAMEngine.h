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
         * @p vehicle_model the vehicle model according to @class VehicleModel
         */
        void Setup(const VectorType& initial_state_estimation, const MatrixType& initial_covariance_estimation, const VehicleModel& vehicle_model) {
            //set the size
            m_XvSize = initial_state_estimation.rows();
            
            //set the remainig parameters with some debug assertions
            assert(initial_state_estimation.rows() == m_XvSize);
            m_Xv = initial_state_estimation;
            
            assert(initial_covariance_estimation.cols() == m_XvSize && initial_covariance_estimation.rows() == m_XvSize);
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
            assert(m_Xv.rows() == m_XvSize);
            //Xv is now Xv-(k)
            
            //landmarks are supposed to be static: no update needed
            
            //update the covariance matrices
            //Pvv
            m_Pvv = df_dXv * m_Pvv * df_dXv.transpose() + Q;
            assert(m_Pvv.cols() == m_XvSize && m_Pvv.rows() == m_XvSize);
            
            //Pvm
            if(m_Pvm.rows() > 0) {
                int eta = m_Pvm.cols();
                m_Pvm = df_dXv * m_Pvm;
                assert(m_Pvm.rows() == m_XvSize && m_Pvm.cols() == eta);
            }
        }
        
        /**
         * @brief add a new landmark to the tracked list
         * @p observation the observation associated with the new feature
         * @p observation_model the observation model of the feature according to @class LandmarkModel
         * @p initialization_model the initialization model accordin to @class LandmarkInitializationModel
         * @p R the covariance matrix for the observation noise (in this case restricted to the new landmark)
         * @return the index at which the landmark has been inserted
         */
        int AddNewLandmark(const VectorType& observation, const LandmarkModel& observation_model, const LandmarkInitializationModel& initialization_model, const MatrixType R) {
            //
            check();
            //check the model
            assert(observation_model);
            assert(initialization_model);
            
            //get the initial landmark state estimation
            VectorType Xm = initialization_model.G(m_Xv, observation);
            //the new landmark
            Landmark new_lm(Xm.rows(), Xm, observation_model);
            ///TODO add here if the incremental count (colum in Pvm/Pmm matrix) is needed
            //add it to the list
            m_landmarks.push_back(new_lm);
            
            //the new landmark state size
            const int Nj = Xm.rows();
            
            //add Nj columns and rows to Pmm
            MatrixType A, B;
            //A is the leftmost column and bottom row, excluded the Nj x Nj square diagonal piece (bottom right corner)
            A.noalias() = m_Pvm.transpose() * initialization_model.dG_dXv(m_Xv, observation).transpose();
            //B is the square Nj x Nj bottom right block
            B.noalias() = initialization_model.dG_dXv(m_Xv, observation) * m_Pvv * initialization_model.dG_dXv(m_Xv, observation).transpose() + initialization_model.dG_dZ(m_Xv, observation) * R * initialization_model.dG_dZ(m_Xv, observation).transpose();
            
            //add the columns
            m_Pmm.conservativeResize(Eigen::NoChange, m_Pmm.cols()+Nj);
            m_Pmm.rightCols(Nj) = A;
            //add the rows
            m_Pmm.conservativeResize(m_Pmm.rows()+Nj, Eigen::NoChange);
            m_Pmm.bottomLeftCorner(/*A.cols()*/Nj, A.rows()) = A.transpose();
            m_Pmm.bottomRightCorner(Nj, Nj) = B;
            
            //add Nj columns to Pvm
            m_Pvm.conservativeResize(Eigen::NoChange_t(), m_Pvm.cols() + Nj);
            m_Pvm.rightCols(Nj).noalias() = m_Pvv.transpose() * initialization_model.dG_dXv(m_Xv, observation).transpose();
            
            return m_landmarks.size()-1;
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
        
        //STATE OBSERVATION
//         std::vector<>
        
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