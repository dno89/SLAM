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
#include "../base/DMDebug.h"
//Eigen
#include <Eigen/Sparse>

////DEBUG
CREATE_PRIVATE_DEBUG_LOG("/tmp/SlamEngine.log",)
// INIT()

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
                const int eta = m_Pvm.cols();
                m_Pvm = df_dXv * m_Pvm;
                assert(m_Pvm.rows() == m_XvSize && m_Pvm.cols() == eta);
            }
        }
        
        /**
         * @brief perform update/correction based on the current perception
         * @p perceptions a vector of associated perception: the current observation
         */
        void Update(std::vector<AssociatedPerception>& perceptions, MatrixType R) {
            const int eta_p = preprocess_perceptions(perceptions);
            const int p = perceptions.size();
            const int eta = m_Pvm.cols();
            
            //the innovation vector
            VectorType ni(eta_p);
            for(int ii = 0; ii < p; ++ii) {
                ni.segment(perceptions[ii].AccumulatedSize, perceptions[ii].Z.rows()) = perceptions[ii].Z - (m_landmarks[perceptions[ii].AssociatedIndex].Model.H(m_Xv));
            }
            
            //the jacobian matrix
            Eigen::SparseMatrix<ScalarType> dH_dX(eta_p, eta + m_XvSize);
            //fill the matrix per block-row
            for(int kk = 0; kk < p; ++kk) {
                //the jacobian wrt the vehicle state: a (Mjkk)x(Nv) matrix
                MatrixType dH_dXv = m_landmarks[perceptions[kk].AssociatedIndex].Model.dH_dXv(m_Xv);
                //the jacobian wrt the landmark state: a (Mjkk)x(Njkk) matrix
                MatrixType dH_dXm = m_landmarks[perceptions[kk].AssociatedIndex].Model.dH_dXm(m_Xv);
                
                //set it on the sparse Jacobian
                //these are the spanned rows
                for(int ii = perceptions[kk].AccumulatedSize; ii < perceptions[kk].AccumulatedSize+perceptions[kk].Z.rows(); ++ii) {
                    //set the Xv dependent term
                    //these are the spanned columns
                    for(int jj = 0; jj < m_XvSize; ++jj) {
                        dH_dX.insert(ii, jj) = dH_dXv(ii - perceptions[kk].AccumulatedSize, jj);
                    }
                    
                    //set the Xm dependent term
                    //the starting column in dH_dX
                    const int start_col = m_landmarks[perceptions[kk].AssociatedIndex].AccumulatedSize+m_XvSize;
                    //the size of the landmark status
                    const int N = m_landmarks[perceptions[kk].AssociatedIndex].Xm.rows();
                    for(int jj = start_col; jj < start_col + N; ++jj) {
                        dH_dX.insert(ii, jj) = dH_dXm(ii - perceptions[kk].AccumulatedSize, jj - start_col);
                    }
                }
            }
            
            //the total P matrix
            MatrixType P(m_XvSize+eta, m_XvSize+eta);
            P << m_Pvv, m_Pvm, m_Pvm.transpose(), m_Pmm;
            
            //the S matrix
            MatrixType S(eta_p, eta_p);
            S.noalias() = dH_dX * P * dH_dX.transpose() + R;
            
            //the Kalman gain
            MatrixType W(eta+m_XvSize, eta_p);
            W = P * dH_dX.transpose() * S.inverse();
            
            //the complete state update
            VectorType dX = W * ni;
            //update the vehicle state
            m_Xv += dX.head(m_XvSize);
            //update the landmark state
            for(int ii = 0; ii < m_landmarks.size(); ++ii) {
                m_landmarks[ii].Xm += dX.segment(m_landmarks[ii].AccumulatedSize+m_XvSize, m_landmarks[ii].Xm.rows());
            }
            
            //update the total covariance matrix
            P = P - W * S * W.transpose();
            //propagates the update to the 3 components
            m_Pvv = P.topLeftCorner(m_XvSize, m_XvSize);
            m_Pvm = P.topRightCorner(m_XvSize, eta);
            m_Pmm = P.bottomRightCorner(eta, eta);
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
            Landmark new_lm(Xm, observation_model);
            //set the accumulated size
            if(m_landmarks.size()) {
                new_lm.AccumulatedSize = m_landmarks.back().AccumulatedSize + m_landmarks.back().Xm.rows();
            }
            //add it to the list
            m_landmarks.push_back(new_lm);
            
            //the new landmark state size
            const int Nj = Xm.rows();
            
            if(m_Pmm.cols() != 0) {
                //this is not the first landmark
                
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
            } else {
                //this is the first landmark
                
                //create Pmm
                m_Pmm = initialization_model.dG_dXv(m_Xv, observation) * m_Pvv * initialization_model.dG_dXv(m_Xv, observation).transpose() + initialization_model.dG_dZ(m_Xv, observation) * R * initialization_model.dG_dZ(m_Xv, observation).transpose();
                
                //create Pvm
                //add Nj columns to Pvm
                m_Pvm.conservativeResize(m_XvSize, Nj);
                m_Pvm.rightCols(Nj).noalias() = m_Pvv.transpose() * initialization_model.dG_dXv(m_Xv, observation).transpose();
            }
            
            return m_landmarks.size()-1;
        }
        
        /**
         * @brief get the current state estimation
         * @return the current state estimation
         */
        VectorType GetStateEstimation() const {
            return m_Xv;
        }
        
        /**
         * @brief get the current state estimation for landmark @p i
         * @p i the index of the landmark
         * @return the current state estimation for landmark @p i
         */
        VectorType GetLandmarkEstimation(int i) {
            return m_landmarks[i].Xm;
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
        ///TODO
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
        /**
         * @brief set the acculumated size for @p p
         * @p p the vector of associated perceptions
         * @return the total size of the observation vector
         */
        int preprocess_perceptions(std::vector<AssociatedPerception>& p) {
            int accum = 0;
            for(int ii = 0; ii < p.size(); ++ii) {
                p[ii].AccumulatedSize = accum;
                accum += p[ii].Z.rows();
            }
            
            return accum;
        }
    };
}

// #endif  //EKFSLAMENGINE_H