/**
 * \file EKFSLAMEngine.h
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */


#pragma once

////include
//SLAM
#include "../Base/core.h"
#include "../Base/types.h"
#include "../DataAssociation/DataAssociation.h"
//std
#include <vector>

namespace SLAM {
    class EKFSLAMEngine {
    public:
        ////TYPEDEF
        /**
         * @brief function that associate some perceptions to known landmark
         * @return a vector of associated perception. Perceptions corresponding to new landmark are associated to -1
         */
        typedef std::vector<LandmarkAssociation>(*AssociationFunction)(const std::vector<Observation>&, const EKFSLAMEngine&);
        
        ////CONSTRUCTOR
        EKFSLAMEngine();
        //no copy or assignment
        EKFSLAMEngine(const EKFSLAMEngine&) = delete;
        EKFSLAMEngine& operator=(const EKFSLAMEngine&) = delete;
        
        ////INTERFACE
        /**
         * @brief the function setup the foundamental parameters
         * @p vehicle_state_size the size of the vehicle state
         * @p initial_state_estimation the initial state
         * @p initial_covariance_estimation the initial covariance matrix of the state
         * @p vehicle_model the vehicle model according to @class VehicleModel
         */
        void Setup(const VectorType& initial_state_estimation, const MatrixType& initial_covariance_estimation, const VehicleModel& vehicle_model);
        
        /**
         * @brief make the prediction step of the EKF filter
         * @p u the control input
         * @p Q the process noise covariance matrix
         */
        void Predict(const VectorType& u, const MatrixType& Q);
        
        /**
         * @brief perform update/correction based on the current perception
         * @p perceptions a vector of perception associated with known landmarks
         * @p R the TOTAL covariance matrix for all the perceptions
         */
        void Update(std::vector<AssociatedPerception>& perceptions, const MatrixType& R);
        
        /**
         * @brief function that associate the observations with the given function @p AF and call the other version of update
         * @p observations vector containing the raw observation and their covariance
         * @p AF the association function
         */
        void Update(const std::vector<Observation>& observations, AssociationFunction AF = Association::GreedyDataAssociation);
        
        /**
         * @brief add a new landmark to the tracked list
         * @p observation the observation associated with the new feature
         * @p observation_model the observation model of the feature according to @class LandmarkModel
         * @p initialization_model the initialization model accordin to @class LandmarkInitializationModel
         * @p R the covariance matrix for the observation noise (in this case restricted to the new landmark)
         * @return the index at which the landmark has been inserted
         */
        int AddNewLandmark(const VectorType& raw_observation, const LandmarkPerceptionModel& observation_model, const LandmarkInitializationModel& initialization_model, const MatrixType& R);
        /**
         * Overload that accept the LandmarkModel struct
         */
        int AddNewLandmark(const VectorType& raw_observation, const LandmarkModel& landmark_model, const MatrixType& R);
        
        /**
         * @brief get the current state estimation
         * @return the current state estimation
         */
        const VectorType& GetStateEstimation() const {
            return m_Xv;
        }
        
        /**
         * @brief get the current state estimation for landmark @p i
         * @p i the index of the landmark
         * @return the current state estimation for landmark @p i
         */
        const VectorType& GetLandmarkEstimation(int i) const {
            return m_landmarks[i].Xm;
        }
        
        /**
         * @brief the number of tracked landmarks
         * @return the number of tracked landmarks
         */
        int GetTrackedLandmarksSize() const {
            return m_landmarks.size();
        }
        
        /**
         * @brief get the @p i -th landmark
         * @return a const reference to the i-th landmark
         */
        const Landmark& GetLandmark(int i) const {
            return m_landmarks[i];
        }
        
        /**
		 * @brief get the landmarks list
		 * @return a const reference to the landmarks list
		 */
        const std::vector<Landmark>& GetLandmarks() const {
			return m_landmarks;
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
        void check();
        /**
         * @brief set the acculumated size for @p p
         * @p p the vector of associated perceptions
         * @return the total size of the observation vector
         */
        int preprocess_perceptions(std::vector<AssociatedPerception>& p);
    };
}