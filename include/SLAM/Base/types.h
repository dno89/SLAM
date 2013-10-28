/**
 * \file types.h
 * \author Daniele Molinari -- 238168
 * \version 1.0
 */

#pragma once

////include
//SLAM
#include "core.h"
//std
#include <stdexcept>


namespace SLAM {
    
    ////support classes
    /**
     * @brief a class that incapsulate the vehicle model and its Jacobian
     */
    class VehicleModel {
    public:
        ////typedef
        typedef VectorType StateType;
        typedef VectorType ControlInputType;
        /**
         * @note the first argument is the vehicle state, the second is the control input
         */
        typedef VectorType (*VehicleFunction)(const StateType&, const ControlInputType&);
        typedef MatrixType (*VehicleJacobian)(const StateType&, const ControlInputType&);
        
        ////constructor
        /**
         * @brief proper initialization for the structure
         * @p f the vehicle model function that update the state in function of previous state and control input
         * @p df_dxv the Jacobian of @p f computed wrt the vehicle state Xv
         */
        VehicleModel(VehicleFunction f, VehicleJacobian df_dxv);
        /**
         * @brief empty constructor
         */
        VehicleModel();
        /**
         * @brief default copy constructor
         */
        VehicleModel(const VehicleModel&) = default;
        /**
         * @brief default copy operator
         */
        VehicleModel& operator=(const VehicleModel&) = default;
        
        /**
         * @brief check whether the object has been properly initialized
         */
        operator bool() const;
        
        /**
         * @note Wrapper for F and dF_dXv function
         */
        VectorType F(const VectorType& Xv, const VectorType& U) const;
        MatrixType dF_dXv(const VectorType& Xv, const VectorType& U) const;
        
    private:
        ////data
        VehicleFunction m_F = nullptr;
        VehicleJacobian m_dF_dXv = nullptr;
    };
    
    /**
     * @class LandmarkModel a class that incapsulate the vehicle model and its Jacobian
     */
    class LandmarkPerceptionModel {
    public:
        ////typedef
        typedef VectorType VehicleStateType;
        typedef VectorType ObservationType;
        typedef VectorType LandmarkStateType;
        /**
         * @note the first argument is the vehicle state, the second is the control input
         */
        typedef ObservationType (*ObservationFunction)(const VehicleStateType&, const LandmarkStateType&);
        typedef MatrixType (*ObservationJacobian)(const VehicleStateType&, const LandmarkStateType&);
        typedef VectorType (*DifferenceFunction)(const ObservationType&, const ObservationType&);
        typedef ScalarType (*DistanceFunction)(const ObservationType&, const ObservationType&);
        typedef bool (*SortFunction)(const ObservationType&, const ObservationType&);
        typedef VectorType (*NormalizeFunction)(const ObservationType&);
        
        ////constructor
        /**
         * @brief proper initialization for the structure
         * @p h the observation model function that gives the observation given the vehicle and landmark state
         * @p dh_dxv the Jacobian of @p h computed wrt the vehicle state Xv
         * @p dh_dxm the Jacobian of @p h computed wrt the landmark state Xm
         * @p distance a function that, given 2 perceptions returns a vector representing the distance, component by component, between the two.
         */
        LandmarkPerceptionModel(ObservationFunction h, ObservationJacobian dh_dxv, ObservationJacobian dh_dxm, DifferenceFunction difference = Models::DefaultDifference, DistanceFunction distance = Models::DefaultDistance, SortFunction sort = Models::DefaultSort, NormalizeFunction normalize = Models::DefaultNormalize);
        /**
         * @brief empty constructor
         */
        LandmarkPerceptionModel();
        /**
         * @brief default copy constructor
         */
        LandmarkPerceptionModel(const LandmarkPerceptionModel&) = default;
        /**
         * @brief default copy operator
         */
        LandmarkPerceptionModel& operator=(const LandmarkPerceptionModel&) = default;
        
        bool operator==(const LandmarkPerceptionModel& m);
        
        /**
         * @brief check whether the object has been properly initialized
         */
        operator bool() const;
        
        /**
         * @brief compare operator required by the std::map
         */
        friend bool operator<(const LandmarkPerceptionModel& m1, const LandmarkPerceptionModel& m2)  {
            return m1.m_H < m2.m_H;
        }
        
        /**
         * @note Wrapper for H, dH_dXv and dH_dXm functions
         */
        ObservationType H(const VehicleStateType& Xv, const LandmarkStateType& Xm) const;
        MatrixType dH_dXv(const VehicleStateType& Xv, const LandmarkStateType& Xm) const;
        MatrixType dH_dXm(const VehicleStateType& Xv, const LandmarkStateType& Xm) const;
        VectorType Difference(const ObservationType& v1, const ObservationType& v2) const;
        ScalarType Distance(const ObservationType& v1, const ObservationType& v2) const;
        bool Sort(const ObservationType& v1, const ObservationType& v2) const;
        VectorType Normalize(const ObservationType& v) const;
        
    private:
        ////data
        ObservationFunction m_H = nullptr;
        ObservationJacobian m_dH_dXv = nullptr;
        ObservationJacobian m_dH_dXm = nullptr;
        DistanceFunction m_distance = nullptr;
        DifferenceFunction m_difference = nullptr;
        SortFunction m_sort = nullptr;
        NormalizeFunction m_normalize = nullptr;
        
        ////function
        void check(const char* str) const;
    };
    
    /**
     * @class LandmarkInitializationModel a class that incapsulate the vehicle model and its Jacobian
     */
    class LandmarkInitializationModel {
    public:
        ////typedef
        typedef VectorType VehicleStateType;
        typedef VectorType ObservationType;
        typedef VectorType LandmarkStateType;
        /**
         * @note the first argument is the vehicle state, the second is the control input
         */
        typedef LandmarkStateType (*InitializationFunction)(const VehicleStateType&, const ObservationType&);
        typedef MatrixType (*InitializationJacobian)(const VehicleStateType&, const ObservationType&);
        
        ////constructor
        /**
         * @brief proper initialization for the structure
         * @p g the landmark state initialization function: a function that return the estimated feature state given the observation and vehicle state
         * @p dg_dxv the Jacobian of @p g computed wrt the vehicle state Xv
         * @p dg_dz the Jacobian of @p g computed wrt the observation Z
         */
        LandmarkInitializationModel(InitializationFunction g, InitializationJacobian dg_dxv, InitializationJacobian dg_dz);
        /**
         * @brief empty constructor
         */
        LandmarkInitializationModel();
        /**
         * @brief default copy constructor
         */
        LandmarkInitializationModel(const LandmarkInitializationModel&) = default;
        /**
         * @brief default copy operator
         */
        LandmarkInitializationModel& operator=(const LandmarkInitializationModel&) = default;
        
        /**
         * @brief check whether the object has been properly initialized
         */
        operator bool() const;
        
        /**
         * @note Wrapper for G, dG_dXv and dG_dZ functions
         */
        LandmarkStateType G(const VehicleStateType& Xv, const ObservationType& Z) const;
        MatrixType dG_dXv(const VehicleStateType& Xv, const ObservationType& Z) const;
        MatrixType dG_dZ(const VehicleStateType& Xv, const ObservationType& Z) const;
        
    private:
        ////data
        InitializationFunction m_G = nullptr;
        InitializationJacobian m_dG_dXv = nullptr;
        InitializationJacobian m_dG_dZ = nullptr;
    };
    
    /**
     * Just a convenient struct that hold both a LandmarkPerceptionModel and a LandmarkInitializationModel
     */
    struct LandmarkModel {
        ////constructors
        LandmarkModel();
        LandmarkModel(const LandmarkPerceptionModel& lpm, const LandmarkInitializationModel& lim);
        LandmarkModel(const LandmarkModel&) = default;
        ////assignment
        LandmarkModel& operator=(const LandmarkModel&) = default;
        
        ////automatic conversion
        operator LandmarkInitializationModel&() { return LIM; }
        operator const LandmarkInitializationModel&() const { return LIM; }
        operator LandmarkPerceptionModel&() { return LPM; }
        operator const LandmarkPerceptionModel&() const { return LPM; }
        
        //member data
        LandmarkPerceptionModel LPM;
        LandmarkInitializationModel LIM;
    };
    
    /**
     * Just a convenient class for force the evaluation of the model on the relative landmark state.
     */
    class RestrainedLandmarkModel {
    private:
        const LandmarkPerceptionModel m_lm;
        const VectorType& m_ls;
    public:
        RestrainedLandmarkModel(const LandmarkPerceptionModel& landmark_model, const VectorType& landmark_state);
        RestrainedLandmarkModel(const RestrainedLandmarkModel&) = delete;
        RestrainedLandmarkModel& operator=(const RestrainedLandmarkModel&) = delete;
            
        ////typedef
        typedef VectorType VehicleStateType;
        typedef VectorType ObservationType;
        
        ////INTERFACE
        ObservationType H(const VehicleStateType& Xv) const;
        MatrixType dH_dXv(const VehicleStateType& Xv) const;
        MatrixType dH_dXm(const VehicleStateType& Xv) const;
        ScalarType Distance(const ObservationType& v1, const ObservationType& v2) const;
        VectorType Difference(const ObservationType& v1, const ObservationType& v2) const;
        VectorType Normalize(const ObservationType& v) const;
        
        const LandmarkPerceptionModel& GetModel() const;
        
        operator const LandmarkPerceptionModel&() const;
    };
    
    /**
     * @struct Landmark
     * @brief keep together various information related to a single landmark
     */
    struct Landmark {
        Landmark(const VectorType& state, const LandmarkPerceptionModel& model);
            
        Landmark(const Landmark& l);
        
        Landmark& operator=(const Landmark& l) = delete;
        
        //the landmark state size
        int AccumulatedSize;
        //the landmark state
        VectorType Xm;
        //the landmark observation model
        RestrainedLandmarkModel Model;
    };
    
    /**
     * @brief simple struct that contains a perception and the associated covariance.
     */
    struct Observation {
        Observation();
        Observation(const VectorType& z, const MatrixType& pz, const LandmarkModel& lm);
        Observation(const VectorType& z, const MatrixType& pz, const LandmarkPerceptionModel& lpm, const LandmarkInitializationModel& lim);
        
        //the raw perception
        VectorType Z;
        //the covariance matrix for this perception
        MatrixType Pz;
        //the landmark model
        LandmarkModel LM;
//         LandmarkPerceptionModel LM;
//         //the initialization model
//         LandmarkInitializationModel LIM;
    };
    
    /**
     * @struct AssociatedPerception
     * @brief an observation vector with an associated index
     * @var Observation
     * @brief The vector with the actual observation
     * 
     * @var AssociatedIndex
     * @brief The index of the tracked feature associated with this observation
     * 
     * @var AccumulatedSize
     * @brief Support variable used by the update function, DO NOT SET
     */
    struct AssociatedPerception {
//         AssociatedPerception() {}
        AssociatedPerception(const VectorType& observation, int associated_index);
        
        //the perception
        VectorType Z;
        //the index of the associated landmark
        int AssociatedIndex;
        //accumulated size, used by the engine
        int AccumulatedSize;
    };
    
    /**
     * @brief simple pair with more meaningful names
     */
    struct LandmarkAssociation {
        LandmarkAssociation(int observation_index = -1, int landmark_index = -1);
        
        int ObservationIndex;
        int LandmarkIndex;
    };
    
    /**
     * @class ProprioceptiveModel a class that incapsulate the vehicle model and its Jacobian
     */
    class ProprioceptiveModel {
    public:
        ////typedef
        typedef VectorType VehicleStateType;
        typedef VectorType ObservationType;
        /**
         * @note the first argument is the vehicle state, the second is the control input
         */
        typedef ObservationType (*ObservationFunction)(const VehicleStateType&);
        typedef MatrixType (*ObservationJacobian)(const VehicleStateType&);
        typedef VectorType (*DifferenceFunction)(const ObservationType&, const ObservationType&);
        
        ////constructor
        /**
         * @brief proper initialization for the structure
         * @p h the observation model function that gives the observation given the vehicle and landmark state
         * @p dh_dxv the Jacobian of @p h computed wrt the vehicle state Xv
         * @p dh_dxm the Jacobian of @p h computed wrt the landmark state Xm
         * @p distance a function that, given 2 perceptions returns a vector representing the distance, component by component, between the two.
         */
        ProprioceptiveModel(ObservationFunction h, ObservationJacobian dh_dxv, DifferenceFunction difference = Models::DefaultDifference);
        /**
         * @brief empty constructor
         */
        ProprioceptiveModel();
        /**
         * @brief default copy constructor
         */
        ProprioceptiveModel(const ProprioceptiveModel&) = default;
        /**
         * @brief default copy operator
         */
        ProprioceptiveModel& operator=(const ProprioceptiveModel&) = default;
        
        bool operator==(const ProprioceptiveModel& m);
        
        /**
         * @brief check whether the object has been properly initialized
         */
        operator bool() const;
        
        /**
         * @brief compare operator required by the std::map
         */
        friend bool operator<(const ProprioceptiveModel& m1, const ProprioceptiveModel& m2) {
            return m1.m_H < m2.m_H;
        }
        
        /**
         * @note Wrapper for H, dH_dXv and dH_dXm functions
         */
        ObservationType H(const VehicleStateType& Xv) const;
        MatrixType dH_dXv(const VehicleStateType& Xv) const;
        VectorType Difference(const ObservationType& v1, const ObservationType& v2) const;
    private:
        ////data
        ObservationFunction m_H = nullptr;
        ObservationJacobian m_dH_dXv = nullptr;
        DifferenceFunction m_difference = nullptr;
        
        ////function
        void check(const char* str) const;
    };
    
    /**
     * @brief simple struct that contains a perception and the associated covariance.
     */
    struct ProprioceptiveObservation {
        ProprioceptiveObservation();
        ProprioceptiveObservation(const VectorType& z, const MatrixType& pz, const ProprioceptiveModel& pm);
        
        //the raw perception
        VectorType Z;
        //the covariance matrix for this perception
        MatrixType Pz;
        //the landmark model
        ProprioceptiveModel PM;
        //the accumulated size: used by the engine
        int AccumulatedSize;
    };
}