/**
 * \file EKFSLAMEngine.cpp
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

////include
//SLAM
#include "EKFSLAMEngine.h"
#include "../Base/DMDebug.h"
//std
#include <stdexcept>
#include <cassert>
#include <vector>
#include <fstream>
#include <chrono>
//Eigen
#include <Eigen/Sparse>

////DEBUG
CREATE_PUBLIC_DEBUG_LOG("/tmp/SlamEngine.log",)

using namespace SLAM;
using namespace std;
using namespace Eigen;

////DEBUG
ofstream dl_update("/tmp/Xupdate.dat");
ofstream dl_uncertainty("/tmp/Xuncertainty.dat");

#ifdef  EKFSLAM_ENABLE_TEST
static unsigned int iteration_count = 0;
#endif  //EKFSLAM_ENABLE_TEST

EKFSLAMEngine::EKFSLAMEngine() {}

void EKFSLAMEngine::Setup(const VectorType& initial_state_estimation, const MatrixType& initial_covariance_estimation, const VehicleModel& vehicle_model) {
        DOPEN_CONTEXT("Setup")
        
        //set the size
        m_XvSize = initial_state_estimation.rows();
        
        //set the remainig parameters with some debug assertions
        assert(initial_state_estimation.rows() == m_XvSize);
        m_Xv = initial_state_estimation;
        
        DPRINT("State initial estimation " << m_Xv.transpose())
        
        assert(initial_covariance_estimation.cols() == m_XvSize && initial_covariance_estimation.rows() == m_XvSize);
        m_Pvv = initial_covariance_estimation;
        
        DTRACE_L(m_Pvv)
        
        assert(vehicle_model);
        m_vModel = vehicle_model;
        
        DCLOSE_CONTEXT("Setup")
        
        m_init = true;
}

void EKFSLAMEngine::Predict(const VectorType& u, const MatrixType& Q) {
#ifdef EKFSLAM_ENABLE_TEST
    DINFO("** ITERATION " << iteration_count++ << " **")
#endif  //EKFSLAM_ENABLE_TEST
    DOPEN_CONTEXT("Predict")
#ifndef NDEBUG
    auto t_start = chrono::high_resolution_clock::now();
#endif  //NDEBUG
    
    check();
    
    //use the old state to the the Taylor 1st order approximation
    MatrixType df_dXv = m_vModel.dF_dXv(m_Xv, u);
    
    DTRACE_L(u)
    DTRACE_L(Q)
    
    DTRACE(m_Xv.transpose())
    //predict the vehicle state
    m_Xv = m_vModel.F(m_Xv, u);
    assert(m_Xv.rows() == m_XvSize);
    //Xv is now Xv-(k)
    DPRINT("After prediction: " << m_Xv.transpose())
    
    //landmarks are supposed to be static: no update needed
    
    //update the covariance matrices
    //Pvv
//     DTRACE_L(df_dXv)
//     DTRACE_L(m_Pvv)
    m_Pvv = df_dXv * m_Pvv * df_dXv.transpose() + Q;
    
    assert(m_Pvv.cols() == m_XvSize && m_Pvv.rows() == m_XvSize);            
    DPRINT("after prediction:\n" << m_Pvv)
    
    //Pvm
    if(m_Pvm.cols() > 0) {
        const int eta = m_Pvm.cols();
        
//         DTRACE_L(m_Pvm)
        m_Pvm = df_dXv * m_Pvm;
        assert(m_Pvm.rows() == m_XvSize && m_Pvm.cols() == eta);
        
//         DPRINT("after prediction:\n" << m_Pvm)                
    }
    
#ifndef NDEBUG
    auto dt = chrono::high_resolution_clock::now()-t_start;
    DINFO("Predict took " << chrono::duration_cast<chrono::microseconds>(dt).count() << " us.")
#endif  //NDEBUG
    
    DCLOSE_CONTEXT("Predict")
}

void EKFSLAMEngine::Update(std::vector<ProprioceptiveObservation>& proprioceptive_observations, std::vector<AssociatedPerception>& perceptions, const MatrixType& R) {
    DOPEN_CONTEXT("Update")
#ifndef NDEBUG
    auto t_start = chrono::high_resolution_clock::now();
#endif  //NDEBUG
    
    check();
    
    int eta_p = preprocess_proprioceptive_perceptions(proprioceptive_observations);
    eta_p += preprocess_perceptions(perceptions, eta_p);
    const int p = perceptions.size();
    const int pp = proprioceptive_observations.size();
    const int eta = m_Pvm.cols();
    
    DTRACE(p)
    DTRACE(pp)
    DTRACE(eta_p)
    DTRACE(eta)
    DTRACE(m_Xv.transpose())
    
    //the innovation vector
//     VectorType std_ni(eta_p);
    VectorType ni(eta_p);
    for(int ii = 0; ii < pp; ++ii) {
        DPRINT("Predicted proprioceptive observation : " << proprioceptive_observations[ii].PM.H(m_Xv).transpose())
        DPRINT("Actual proprioceptive observation : " << proprioceptive_observations[ii].Z.transpose())
        
        ni.segment(proprioceptive_observations[ii].AccumulatedSize, proprioceptive_observations[ii].Z.rows()) = proprioceptive_observations[ii].PM.Difference(proprioceptive_observations[ii].Z, proprioceptive_observations[ii].PM.H(m_Xv));
    }
    for(int ii = 0; ii < p; ++ii) {
        DPRINT("Estimated landmark " << perceptions[ii].AssociatedIndex << " state: " << m_landmarks[perceptions[ii].AssociatedIndex].Xm.transpose())
        DPRINT("Predicted observation : " << m_landmarks[perceptions[ii].AssociatedIndex].Model.H(m_Xv).transpose())
        DPRINT("Actual observation : " << perceptions[ii].Z.transpose())
        
//         std_ni.segment(perceptions[ii].AccumulatedSize, perceptions[ii].Z.rows()) = perceptions[ii].Z - (m_landmarks[perceptions[ii].AssociatedIndex].Model.H(m_Xv));
        
        ni.segment(perceptions[ii].AccumulatedSize, perceptions[ii].Z.rows()) = m_landmarks[perceptions[ii].AssociatedIndex].Model.Difference(perceptions[ii].Z, m_landmarks[perceptions[ii].AssociatedIndex].Model.H(m_Xv));
    }
//     DTRACE_L(std_ni)
    DTRACE_L(ni)
    
    //the jacobian matrix
    Eigen::SparseMatrix<ScalarType> dH_dX(eta_p, eta + m_XvSize);
    //fill the matrix per block-row
    for(int kk = 0; kk < pp; ++kk) {
        MatrixType dH_dXv = proprioceptive_observations[kk].PM.dH_dXv(m_Xv);
        
        //set it on the sparse Jacobian
        //these are the spanned rows
        for(int ii = proprioceptive_observations[kk].AccumulatedSize; ii < proprioceptive_observations[kk].AccumulatedSize+proprioceptive_observations[kk].Z.rows(); ++ii) {
            //set the Xv dependent term
            //these are the spanned columns
            for(int jj = 0; jj < m_XvSize; ++jj) {
                dH_dX.insert(ii, jj) = dH_dXv(ii - proprioceptive_observations[kk].AccumulatedSize, jj);
            }
        }
    }
    for(int kk = 0; kk < p; ++kk) {
        //the jacobian wrt the vehicle state: a (Mjkk)x(Nv) matrix
        MatrixType dH_dXv = m_landmarks[perceptions[kk].AssociatedIndex].Model.dH_dXv(m_Xv);
//         DTRACE_L(dH_dXv)
        //the jacobian wrt the landmark state: a (Mjkk)x(Njkk) matrix
        MatrixType dH_dXm = m_landmarks[perceptions[kk].AssociatedIndex].Model.dH_dXm(m_Xv);
//         DTRACE_L(dH_dXm)
        
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
    
#ifndef NDEBUG
    MatrixType dense_dH_dX = dH_dX;
//     DTRACE_L(dense_dH_dX)
#endif
    
    //the total P matrix
    MatrixType P(m_XvSize+eta, m_XvSize+eta);
    P << m_Pvv, m_Pvm, m_Pvm.transpose(), m_Pmm;
//     DTRACE_L(m_Pvv)
//     DTRACE_L(m_Pvm)
//     DTRACE_L(m_Pmm)
//     DTRACE_L(P)
    
    //the S matrix
    MatrixType S(eta_p, eta_p);
    S.noalias() = dH_dX * P * dH_dX.transpose() + R;
    
//     DTRACE_L(S)
    
    //the Kalman gain
    MatrixType W(eta+m_XvSize, eta_p);
    W = P * dH_dX.transpose() * S.inverse();
    
//     DTRACE_L(W)

    //the complete state update
    VectorType dX = W * ni;
//     DTRACE_L(dX)
    
    ///DEBUG
    for(int ii = 0; ii < dX.rows(); ++ii) {
        dl_update << abs(dX[ii]) << " ";
    }
    dl_update << endl;
    
    //update the vehicle state
    DTRACE_L(m_Xv)
    m_Xv += dX.head(m_XvSize);
    DPRINT("after update:\n" << m_Xv)
    
    //update the landmark state
    for(int ii = 0; ii < m_landmarks.size(); ++ii) {
//         DPRINT("landmark " << ii << ": " << m_landmarks[ii].Xm.transpose())
        m_landmarks[ii].Xm += dX.segment(m_landmarks[ii].AccumulatedSize+m_XvSize, m_landmarks[ii].Xm.rows());
//         DPRINT("after update: " << m_landmarks[ii].Xm.transpose())
    }
    
    //update the total covariance matrix
    P = P - W * S * W.transpose();
    
    ///DEBUG
    for(int ii = 0; ii < P.cols(); ++ii) {
        dl_uncertainty << sqrt(P(ii, ii)) << " ";
    }
    dl_uncertainty << endl;
    
//     DTRACE_L(P)
    
    //propagates the update to the 3 components
    m_Pvv = P.topLeftCorner(m_XvSize, m_XvSize);
    m_Pvm = P.topRightCorner(m_XvSize, eta);
    m_Pmm = P.bottomRightCorner(eta, eta);
    
//     DTRACE_L(m_Pvv)
//     DTRACE_L(m_Pvm)
//     DTRACE_L(m_Pmm)
    
#ifndef NDEBUG
    auto dt = chrono::high_resolution_clock::now()-t_start;
    DINFO("Preassociated Update took " << chrono::duration_cast<chrono::microseconds>(dt).count() << " us.")
#endif  //NDEBUG
    
    DCLOSE_CONTEXT("Update")
}

void EKFSLAMEngine::Update(std::vector<ProprioceptiveObservation>& proprioceptive_observations, const std::vector<Observation>& observations, AssociationFunction AF) {
    DOPEN_CONTEXT("Update")
#ifndef NDEBUG
    auto t_start = chrono::high_resolution_clock::now();
#endif  //NDEBUG
    
    if(observations.empty()) {
        DWARNING("Empty observations")
        DCLOSE_CONTEXT("Update")
        
        return;
    }
    
    std::vector<LandmarkAssociation> assoc = (*AF)(observations, *this);
    
    std::vector<AssociatedPerception> ap;
    ap.reserve(assoc.size());
    
    std::vector<Observation> new_landmark;
    MatrixType R;
    
    int ob_size = 0;
    
    //add the covariance of proprioceptive observations
    for(int ii = 0; ii < proprioceptive_observations.size(); ++ii) {
        ob_size = proprioceptive_observations[ii].Z.rows();
        
        R.conservativeResize(R.rows() + ob_size, R.cols() + ob_size);
        R.rightCols(ob_size) = MatrixXd::Zero(R.rows(), ob_size);
        R.bottomRows(ob_size) = MatrixXd::Zero(ob_size, R.cols());
        R.bottomRightCorner(ob_size, ob_size) = proprioceptive_observations[ii].Pz;
    }
        
    for(auto la : assoc) {
        if(la.LandmarkIndex >= 0) {
            //known landmark
            ap.push_back(AssociatedPerception(observations[la.ObservationIndex].Z, la.LandmarkIndex));
            
            ob_size = observations[la.ObservationIndex].Z.rows();
            
            R.conservativeResize(R.rows() + ob_size, R.cols() + ob_size);
            R.rightCols(ob_size) = MatrixXd::Zero(R.rows(), ob_size);
            R.bottomRows(ob_size) = MatrixXd::Zero(ob_size, R.cols());
            R.bottomRightCorner(ob_size, ob_size) = observations[la.ObservationIndex].Pz;
        } else {
            //new landmark
            new_landmark.push_back(observations[la.ObservationIndex]);
        }
    }
    
    //update with the given observations
    if(!ap.empty() || !proprioceptive_observations.empty()) {
        Update(proprioceptive_observations, ap, R);
    }
    
    //add the new landmarks
    for(auto o : new_landmark) {
        AddNewLandmark(o.Z, o.LM, o.Pz);
    }
    
#ifndef NDEBUG
    auto dt = chrono::high_resolution_clock::now()-t_start;
    DINFO("Update with association took " << chrono::duration_cast<chrono::microseconds>(dt).count() << " us.")
#endif  //NDEBUG
    
    DCLOSE_CONTEXT("Update")
}

int EKFSLAMEngine::AddNewLandmark(const VectorType& raw_observation, const LandmarkPerceptionModel& observation_model, const LandmarkInitializationModel& initialization_model, const MatrixType& R) {
    DOPEN_CONTEXT("AddNewLandmark")
#ifndef NDEBUG
    auto t_start = chrono::high_resolution_clock::now();
#endif  //NDEBUG
    
    check();
    //check the model
    assert(observation_model);
    assert(initialization_model);
    
    //get the initial landmark state estimation
    VectorType Xm = initialization_model.G(m_Xv, raw_observation);
    //the new landmark
    Landmark new_lm(Xm, observation_model);
    //set the accumulated size
    if(m_landmarks.size()) {
        new_lm.AccumulatedSize = m_landmarks.back().AccumulatedSize + m_landmarks.back().Xm.rows();
    }
    //add it to the list
    m_landmarks.push_back(new_lm);
    
    DPRINT("New Landmark added with state (" << new_lm.Xm.transpose() << "), covariance:\n" << R << "\n and AcculumatedSize: " << new_lm.AccumulatedSize)
    
    //the new landmark state size
    const int Nj = Xm.rows();
    
    if(m_Pmm.cols() != 0) {
        //this is not the first landmark
        
        //add Nj columns and rows to Pmm
        MatrixType A, B;
        //A is the leftmost column and bottom row, excluded the Nj x Nj square diagonal piece (bottom right corner)
        A.noalias() = m_Pvm.transpose() * initialization_model.dG_dXv(m_Xv, raw_observation).transpose();
        //B is the square Nj x Nj bottom right block
        B.noalias() = initialization_model.dG_dXv(m_Xv, raw_observation) * m_Pvv * initialization_model.dG_dXv(m_Xv, raw_observation).transpose() + initialization_model.dG_dZ(m_Xv, raw_observation) * R * initialization_model.dG_dZ(m_Xv, raw_observation).transpose();
        
//         DTRACE_L(A)
//         DTRACE_L(B)
//         DTRACE_L(m_Pmm)
        
        //add the columns
        m_Pmm.conservativeResize(Eigen::NoChange, m_Pmm.cols()+Nj);
        m_Pmm.rightCols(Nj) = A;
        //add the rows
        m_Pmm.conservativeResize(m_Pmm.rows()+Nj, Eigen::NoChange);
        m_Pmm.bottomLeftCorner(/*A.cols()*/Nj, A.rows()) = A.transpose();
        m_Pmm.bottomRightCorner(Nj, Nj) = B;
        
//         DTRACE_L(m_Pmm)
        
        //add Nj columns to Pvm
//         DTRACE_L(m_Pvm)
        m_Pvm.conservativeResize(Eigen::NoChange_t(), m_Pvm.cols() + Nj);
        m_Pvm.rightCols(Nj).noalias() = m_Pvv.transpose() * initialization_model.dG_dXv(m_Xv, raw_observation).transpose();
//         DTRACE_L(m_Pvm)
    } else {
        //this is the first landmark
        
        //create Pmm
        m_Pmm = initialization_model.dG_dXv(m_Xv, raw_observation) * m_Pvv * initialization_model.dG_dXv(m_Xv, raw_observation).transpose() + initialization_model.dG_dZ(m_Xv, raw_observation) * R * initialization_model.dG_dZ(m_Xv, raw_observation).transpose();
        
//         DTRACE_L(m_Pmm)
        
        //create Pvm
        //add Nj columns to Pvm
        m_Pvm.conservativeResize(m_XvSize, Nj);
        m_Pvm.rightCols(Nj).noalias() = m_Pvv.transpose() * initialization_model.dG_dXv(m_Xv, raw_observation).transpose();
        
//         DTRACE_L(m_Pvm)
    }
    
#ifndef NDEBUG
    auto dt = chrono::high_resolution_clock::now()-t_start;
    DINFO("AddNewLandmark took " << chrono::duration_cast<chrono::microseconds>(dt).count() << " us.")
#endif  //NDEBUG
    
    DCLOSE_CONTEXT("AddNewLandmark")
    
    return m_landmarks.size()-1;
}

int EKFSLAMEngine::AddNewLandmark ( const SLAM::VectorType& raw_observation, const LandmarkModel& landmark_model, const MatrixType& R ) {
    AddNewLandmark(raw_observation, landmark_model.LPM, landmark_model.LIM, R);
}


inline void EKFSLAMEngine::check() {
    if(!m_init) throw std::runtime_error("SLAMEngine ERROR: a function has been called with the object not propertly initialize (call Setup before using the object)\n");
}

inline int EKFSLAMEngine::preprocess_perceptions(std::vector<AssociatedPerception>& p, int initial_offset) const {
    int accum = initial_offset;
    for(int ii = 0; ii < p.size(); ++ii) {
        p[ii].AccumulatedSize = accum;
        accum += p[ii].Z.rows();
    }
    
    return accum;
}

inline int EKFSLAMEngine::preprocess_proprioceptive_perceptions(std::vector<ProprioceptiveObservation>& pp) const {
    int accum = 0;
    for(int ii = 0; ii < pp.size(); ++ii) {
        pp[ii].AccumulatedSize = accum;
        accum += pp[ii].Z.rows();
    }
    
    return accum;
}