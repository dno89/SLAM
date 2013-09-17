// #ifndef TYPES_H
// #define TYPES_H
#pragma once

////include
//SLAM
#include "core.h"
//Eigen
#include <Eigen/Dense>
//std
#include <stdexcept>


namespace SLAM {
    ////typedef
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1>                VectorType;
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>   MatrixType;
    
    ////support classes
    /**
     * @class VehicleModel a class that incapsulate the vehicle model and its Jacobian
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
        VehicleModel(VehicleFunction f, VehicleJacobian df_dxv) : m_F(f), m_dF_dXv(df_dxv) {
            if(!(m_F && m_dF_dXv)) {
                throw std::runtime_error("VehicleModel::VehicleModel(VehicleFunction,VehicleJacobian) ERROR: f and df_dxv must be non-NULL!\n");
            }
        }
        /**
         * @brief empty constructor
         */
        VehicleModel() /*: m_F(nullptr), m_dF_dXv(nullptr)*/ {}
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
        operator bool() const {
            return m_F && m_dF_dXv;
        }
        
        /**
         * @note Wrapper for F and dF_dXv function
         */
        VectorType F(const VectorType& Xv, const VectorType& U) {
             if(!(m_F && m_dF_dXv)) {
                throw std::runtime_error("VehicleModel::F ERROR: structure not initialized\n");
            }
            
            return (*m_F)(Xv, U);
        }
        VectorType dF_dXv(const VectorType& Xv, const VectorType& U) {
             if(!(m_F && m_dF_dXv)) {
                throw std::runtime_error("VehicleModel::F ERROR: structure not initialized\n");
            }
            
            return (*m_dF_dXv)(Xv, U);
        }
        
    private:
        ////data
        VehicleFunction m_F = nullptr;
        VehicleJacobian m_dF_dXv = nullptr;
    };
    
    /**
     * @class LandmarkModel a class that incapsulate the vehicle model and its Jacobian
     */
    class LandmarkModel {
    public:
        ////typedef
        typedef VectorType VehicleStateType;
        typedef VectorType LandmarkStateType;
        /**
         * @note the first argument is the vehicle state, the second is the control input
         */
        typedef VectorType (*ObservationFunction)(const VehicleStateType&, const LandmarkStateType&);
        typedef MatrixType (*ObservationJacobian)(const VehicleStateType&, const LandmarkStateType&);
        
        ////constructor
        /**
         * @brief proper initialization for the structure
         * @p h the observation model function that gives the observation given the vehicle and landmark state
         * @p dh_dxv the Jacobian of @p h computed wrt the vehicle state Xv
         * @p dh_dxm the Jacobian of @p h computed wrt the landmark state Xm
         */
        LandmarkModel(ObservationFunction h, ObservationJacobian dh_dxv, ObservationJacobian dh_dxm) : m_H(h), m_dH_dXv(dh_dxv), m_dH_dXm(dh_dxm) {
            if(!(m_H && m_dH_dXm && m_dH_dXv)) {
                throw std::runtime_error("LandmarkModel::LandmarkModel(ObservationFunction,ObservationJacobian,ObservationJacobian) ERROR: h, dh_dxv and df_dxm must be non-NULL!\n");
            }
        }
        /**
         * @brief empty constructor
         */
        LandmarkModel() {}
        /**
         * @brief default copy constructor
         */
        LandmarkModel(const LandmarkModel&) = default;
        /**
         * @brief default copy operator
         */
        LandmarkModel& operator=(const LandmarkModel&) = default;
        
        /**
         * @brief check whether the object has been properly initialized
         */
        operator bool() const {
            return m_H && m_dH_dXm && m_dH_dXv;
        }
        
        /**
         * @note Wrapper for H, dH_dXv and dH_dXm functions
         */
        VectorType H(const VehicleStateType& Xv, const LandmarkStateType& Xm) {
             if(!(m_H && m_dH_dXm && m_dH_dXv)) {
                throw std::runtime_error("LandmarkModel::H ERROR: functions not initialized\n");
            }
            
            return (*m_H)(Xv, Xm);
        }
        MatrixType dH_dXv(const VehicleStateType& Xv, const LandmarkStateType& Xm) {
             if(!(m_H && m_dH_dXm && m_dH_dXv)) {
                throw std::runtime_error("LandmarkModel::dH_dXv ERROR: functions not initialized\n");
            }
            
            return (*m_dH_dXv)(Xv, Xm);
        }
        MatrixType dH_dXm(const VehicleStateType& Xv, const LandmarkStateType& Xm) {
             if(!(m_H && m_dH_dXm && m_dH_dXv)) {
                throw std::runtime_error("LandmarkModel::dH_dXm ERROR: functions not initialized\n");
            }
            
            return (*m_dH_dXm)(Xv, Xm);
        }
        
    private:
        ////data
        ObservationFunction m_H = nullptr;
        ObservationJacobian m_dH_dXv = nullptr;
        ObservationJacobian m_dH_dXm = nullptr;
    };
    
    struct Landmark {
        Landmark() {}
        Landmark(int size, const VectorType& state, const LandmarkModel& model) :
            XmSize(size), Xm(state), Model(model) {}
        
        //the landmark state size
        int XmSize;
        //the landmark state
        VectorType Xm;
        //the landmark observation model
        LandmarkModel Model;
    };
}

// #endif  //TYPES_H