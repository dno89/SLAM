////include
//std
#include <iostream>
#include <stdexcept>
#include <map>
#include <chrono>
#include <cstdlib>
#include <random>
//SLAM
#include "../SLAM/SLAM.h"
//Eigen
#include <Eigen/Sparse>
#include <Eigen/Dense>

using namespace std;
using namespace chrono;
using namespace Eigen;
using namespace SLAM;

/******************************************************************************
 *          GLOBAL VARS
 * ***************************************************************************/

////typedef
typedef int (*TestFunction)(int, char**);

////global var
std::map<std::string, TestFunction> tests;



/******************************************************************************
 *          TEST
 * ***************************************************************************/

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

/******************************************************************************
 *          
 * ***************************************************************************/

VectorType f(const VectorType& Xv, const VectorType& U)  {
    cerr << "f" << endl;
    return Xv;
}

MatrixType df_dxv(const VectorType& Xv, const VectorType& U)  {
    cerr << "df_dxv" << endl;
    
    MatrixType m(3, 3);
    m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    return m;
}

Vector3d typed_f(const Vector3d& Xv, const Vector2d& U)  {
    cerr << "typed f" << endl;
    return Xv;
}

Matrix3d typed_df_dxv(const Vector3d& Xv, const Vector2d& U)  {
    cerr << "typed df_dxv" << endl;
    
    Matrix3d m;
    m << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    return m;
}

int base_test(int argc, char **argv) {
    std::cout << "Hello, world!\nThis is a big SLAM test\n" << std::endl;
    
    SLAMEngine e;
    
    VectorType v(3);
    VectorType u(2);
    v << 0, 1, 2;
    u << 2, 3;
    
    MatrixType p(3, 3);
    p << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
    
    e.Setup(v, p, VehicleModel(f, df_dxv));
    
    e.Predict(u, p);
    
    Wrapper* w = new WrapperImpl<3, 2>(typed_f, typed_df_dxv);
    
    Vector3d typed_U;
    typed_U << 100, -100, 0;
    
    cout << w->F(v, typed_U) << endl;
    
    delete w;
    
    return 0;
}

int speed_test(int argc, char **argv) {
    std::cout << "Eigen speed test: dense vs sparse matrices" << std::endl;
    
    srand(time(NULL));
    
    const int sigma = 30;   //Nv+eta
    MatrixXd P = MatrixXd::Random(sigma, sigma);
    
    const int p = 11;   //number of observation
    MatrixXd dense_dH = MatrixXd::Zero(p, sigma);
    SparseMatrix<double> sparse_dH(p, sigma);
    
    for(int ii = 0; ii < p; ++ii) {
        double dxv = (double(rand())/RAND_MAX)*50.0 - 25.0;
        
        //first element
        dense_dH(ii, 0) = dxv;
        sparse_dH.insert(ii, 0) = dxv;
        
        int column = rand()%(sigma-1)+1;
        double dxm = (double(rand())/RAND_MAX)*50.0 - 25.0;
        
        //other element
        dense_dH(ii, column) = dxm;
        sparse_dH.insert(ii, column) = dxm;
    }
    
    const int n_tests = 10000;
    
    std::cout << "Starting with DENSE test" << endl;
    
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    for(int ii = 0; ii < n_tests; ++ii) {
        MatrixXd res;
        res.noalias() = dense_dH * P * dense_dH.transpose();
    }
    high_resolution_clock::duration dt = high_resolution_clock::now() - t1;
    
    std::cout << n_tests << " moltiplications with dense matrices took " << duration_cast<milliseconds>(dt).count() << "ms." << endl;
    
    std::cout << "\nStarting with SPARSE test" << endl;
    
    t1 = high_resolution_clock::now();
    for(int ii = 0; ii < n_tests; ++ii) {
        MatrixXd res;
        res.noalias() = sparse_dH * P * sparse_dH.transpose();
    }
    dt = high_resolution_clock::now() - t1;
    
    std::cout << n_tests << " moltiplications with sparse matrices took " << duration_cast<milliseconds>(dt).count() << "ms." << endl;
    
    time_t t = high_resolution_clock::to_time_t(high_resolution_clock::now());
    std::cout << "Test done at: " << ctime(&t) << endl;
    
//     std::cout << "Test done at: " << ctime(&high_resolution_clock::to_time_t(high_resolution_clock::now())) << endl;
    
    return 0;
}

namespace engine_test {
    /******************************************************************************
    *           MODEL       
    * ***************************************************************************/
    //discretization
    static const double time_increment = 200e-3; //200ms
    //noise generator
    static default_random_engine eng;
    static const double mu = 0.0;
    static const double state_sigma = 0.05;
    static const double observation_sigma = 0.03;
    static normal_distribution<double> state_noise(mu, state_sigma);
    static normal_distribution<double> observation_noise(mu, observation_sigma);
    
    ////vehicle model
    //Xv = (x, y, theta)
    //U = (v, omega)
    VectorType F(const VectorType& Xv, const VectorType& U) {
        Vector3d res(Xv);
        
        res[0] += U[0]*time_increment*cos(Xv[2]+U[1]*time_increment/2.0);
        res[1] += U[0]*time_increment*sin(Xv[2]+U[1]*time_increment/2.0);
        res[2] += U[1]*time_increment;
        
        return res;
    }
    VectorType noisy_F(const VectorType& Xv, const VectorType& U) {
        Vector3d res(Xv);
        
        res[0] += U[0]*time_increment*cos(Xv[2]+U[1]*time_increment/2.0) + state_noise(eng);
        res[1] += U[0]*time_increment*sin(Xv[2]+U[1]*time_increment/2.0) + state_noise(eng);
        res[2] += U[1]*time_increment + state_noise(eng);
        
        return res;
    }
    MatrixType dF_dXv(const VectorType& Xv, const VectorType& U) {
        MatrixType J(3, 3);
        J <<    1, 0, (-U[0]*time_increment*sin(Xv[2]+U[1]*time_increment/2.0)),
                0, 1, (U[0]*time_increment*cos(Xv[2]+U[1]*time_increment/2.0)),
                0, 0, 1;
                
        return J;
    }
    
    ////landmark model
    //Xm = (mx, my)
    //Z = (rho, theta)
    VectorType H(VectorType Xv, VectorType Xm) {
        VectorType res(2);
        res << sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1])), atan2(Xm[1]-Xv[1], Xm[0] - Xv[0]);
        
        return res;
    }
    MatrixType dH_dXv(VectorType Xv, VectorType Xm) {
        MatrixType J(2, 3);
        double rho = sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1]));
        J <<    (Xv[0] - Xm[0])/rho,            (Xv[1] - Xm[1])/rho,        0,
                -(Xv[1] - Xm[1])/(rho*rho),     (Xv[0] - Xm[0])/(rho*rho),  -1;
        
        return J;
    }
    MatrixType dH_dXm(VectorType Xv, VectorType Xm) {
        MatrixType J(2, 2);
        double rho = sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1]));
        J <<    (-Xv[0] + Xm[0])/rho,          (-Xv[1] + Xm[1])/rho,
                (Xv[1] - Xm[1])/(rho*rho),     -(Xv[0] - Xm[0])/(rho*rho);
        
        return J;
    }
    
    int slam_engine_test(int argc, char **argv) {
        return 0;
    }
}

////utility functions
void register_function(std::string name, TestFunction fn_ptr) {
    tests[name] = fn_ptr;
}
void RegisterFunctions() {
    register_function("base_test",      base_test);
    register_function("speed_test",     speed_test);
}


//// MAIN
int main(int argc, char** argv) {
    ////using
    using namespace std;
    
    RegisterFunctions();
    
    if(argc < 2) {
error:  cerr << "Usage: " << argv[0] << " test {args, ...}\nAvailable test: ";
        for(auto it = tests.begin(); it != tests.end(); ++it) {
            cerr << "\n\t" << it->first;
        }
        cerr << endl;
        return 1;
    }
    
    string test = argv[1];
    TestFunction fn;
    
    if(tests.count(test) == 0) {
        goto error;
    } else {
        fn = tests[test];
    }
    
    return (*fn)(argc-1, argv+1);
}