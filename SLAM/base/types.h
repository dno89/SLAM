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
    private: 
        ////subtypes
        class Wrapper {
        public:
            virtual VectorType F(const VectorType& Xv, const VectorType& U) = 0;
            virtual MatrixType dF_dXv(const VectorType& Xv, const VectorType& U) = 0;
        };
        template<int N, int M>
        class WrapperImpl : public Wrapper {
        public:
            static const int XvSize = N;
            static const int USize = M;
            
            ////typedef
            //vectors and matrices
            typedef Eigen::Matrix<ScalarType, XvSize, 1> StateVectorType;
            typedef Eigen::Matrix<ScalarType, USize, 1> InputVectorType;
            typedef Eigen::Matrix<ScalarType, XvSize, XvSize> JacobianMatrixType;
            //function pointers
            typedef StateVectorType (*FType)(const StateVectorType&, const InputVectorType&);
            typedef JacobianMatrixType (*dF_dXvType)(const StateVectorType&, const InputVectorType&);
            
            ////constructor
            WrapperImpl(FType f, dF_dXvType df_dxv) : m_F(f), m_dF_dXv(df_dxv)
                {}
                
            ////functions
            VectorType F ( const VectorType& Xv, const VectorType& U ) {
                StateVectorType xv(Xv);
                InputVectorType u(U);
                
                return (*m_F)(xv, u);
            }
            MatrixType dF_dXv ( const VectorType& Xv, const VectorType& U ) {
            }
            
        private:
            FType const m_F;
            dF_dXvType const m_dF_dXv;
        };
        
        ///TODO INTEGRATE
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
        VectorType F(const VectorType& Xv, const VectorType& U) const {
             if(!(m_F && m_dF_dXv)) {
                throw std::runtime_error("VehicleModel::F ERROR: structure not initialized\n");
            }
            
            return (*m_F)(Xv, U);
        }
        MatrixType dF_dXv(const VectorType& Xv, const VectorType& U) const {
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
        VectorType H(const VehicleStateType& Xv, const LandmarkStateType& Xm) const {
             if(!(m_H && m_dH_dXm && m_dH_dXv)) {
                throw std::runtime_error("LandmarkModel::H ERROR: functions not initialized\n");
            }
            
            return (*m_H)(Xv, Xm);
        }
        MatrixType dH_dXv(const VehicleStateType& Xv, const LandmarkStateType& Xm) const {
             if(!(m_H && m_dH_dXm && m_dH_dXv)) {
                throw std::runtime_error("LandmarkModel::dH_dXv ERROR: functions not initialized\n");
            }
            
            return (*m_dH_dXv)(Xv, Xm);
        }
        MatrixType dH_dXm(const VehicleStateType& Xv, const LandmarkStateType& Xm) const {
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
        LandmarkInitializationModel(InitializationFunction g, InitializationJacobian dg_dxv, InitializationJacobian dg_dz) : m_G(g), m_dG_dXv(dg_dxv), m_dG_dZ(dg_dz) {
            if(!(m_G && m_dG_dZ && m_dG_dXv)) {
                throw std::runtime_error("LandmarkInitializationModel::LandmarkInitializationModel(InitializationFunction,InitializationJacobian,InitializationJacobian) ERROR: g, dg_dxv and dg_dz must be non-NULL!\n");
            }
        }
        /**
         * @brief empty constructor
         */
        LandmarkInitializationModel() {}
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
        operator bool() const {
            return m_G && m_dG_dZ && m_dG_dXv;
        }
        
        /**
         * @note Wrapper for G, dG_dXv and dG_dZ functions
         */
        LandmarkStateType G(const VehicleStateType& Xv, const ObservationType& Z) const {
             if(!(m_G && m_dG_dZ && m_dG_dXv)) {
                throw std::runtime_error("LandmarkInitializationModel::H ERROR: functions not initialized\n");
            }
            
            return (*m_G)(Xv, Z);
        }
        MatrixType dG_dXv(const VehicleStateType& Xv, const ObservationType& Z) const {
             if(!(m_G && m_dG_dZ && m_dG_dXv)) {
                throw std::runtime_error("LandmarkInitializationModel::dH_dXv ERROR: functions not initialized\n");
            }
            
            return (*m_dG_dXv)(Xv, Z);
        }
        MatrixType dG_dZ(const VehicleStateType& Xv, const ObservationType& Z) const {
             if(!(m_G && m_dG_dZ && m_dG_dXv)) {
                throw std::runtime_error("LandmarkInitializationModel::dH_dZ ERROR: functions not initialized\n");
            }
            
            return (*m_dG_dZ)(Xv, Z);
        }
        
    private:
        ////data
        InitializationFunction m_G = nullptr;
        InitializationJacobian m_dG_dXv = nullptr;
        InitializationJacobian m_dG_dZ = nullptr;
    };
    
    /**
     * @struct Landmark
     * @brief keep together various information related to a single landmark
     */
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