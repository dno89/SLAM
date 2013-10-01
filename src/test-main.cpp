////include
//std
#include <iostream>
#include <stdexcept>
#include <map>
#include <chrono>
#include <cstdlib>
#include <random>
#include <strstream>
#include <fstream>
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
    
//     SLAMEngine e;
//     
//     VectorType v(3);
//     VectorType u(2);
//     v << 0, 1, 2;
//     u << 2, 3;
//     
//     MatrixType p(3, 3);
//     p << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
//     
//     e.Setup(v, p, VehicleModel(f, df_dxv));
//     
//     e.Predict(u, p);
//     
//     Wrapper* w = new WrapperImpl<3, 2>(typed_f, typed_df_dxv);
//     
//     Vector3d typed_U;
//     typed_U << 100, -100, 0;
//     
//     cout << w->F(v, typed_U) << endl;
    
    cout << endl << "The -1.5 mod 1.0 is " << fmod(-1.5, 1.0) << endl;
    cout << endl << "The 2.5 mod 1.0 is " << fmod(2.5, 1.0) << endl;
    
    MatrixType m(2, 2);
    m << 1, 2, 3, 4;
    cout << "m: " << m << endl;
    m.conservativeResize(5, 5);
    cout << "m after resizing: " << m << endl;
    
//     delete w;
    
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
    static const double time_increment = 50e-3; //200ms
    //noise generator
    static default_random_engine eng_state(/*time(NULL)*/1);
    static default_random_engine eng_ob(/*time(NULL)*/1);
    static const double mu = 0.0;
    
    static const double state_pos_sigma = 0.01;
    static const double state_ang_sigma = 0.005;
	static const double observation_rho_sigma = 0.01;
	static const double observation_alpha_sigma = 0.004;
    
//     static const double state_pos_sigma = 0.001;
//     static const double state_ang_sigma = 0.001;
//     static const double observation_sigma = 0.01;
    
//     static const double state_pos_sigma = 0.05;
//     static const double state_ang_sigma = 0.001;    //this noise make the whole estimation diverge
//     static const double observation_sigma = 0.0;
    
//     static const double state_pos_sigma = 0.1;
//     static const double state_ang_sigma = 0.05;
//     static const double observation_sigma = 0.05;
    
//     static const double state_pos_sigma = 0.00;
//     static const double state_ang_sigma = 0.00;
//     static const double observation_sigma = 0.00;
    
    static normal_distribution<double> state_pos_noise(mu, state_pos_sigma);
    static normal_distribution<double> state_ang_noise(mu, state_ang_sigma);
	static normal_distribution<double> observation_rho_noise(mu, observation_rho_sigma);
	static normal_distribution<double> observation_alpha_noise(mu, observation_alpha_sigma);
    
    ////vehicle model
    //Xv = (x, y, theta)
    //U = (v, omega)
    VectorType F(const VectorType& Xv, const VectorType& U) {
//         cout << ">> F called with Xv = (" << Xv.transpose() << "), U = (" << U.transpose() << ")" << endl;
        
        Vector3d res(Xv);
        res[0] += U[0]*time_increment*cos(Xv[2]+U[1]*time_increment/2.0);
        res[1] += U[0]*time_increment*sin(Xv[2]+U[1]*time_increment/2.0);
        
        res[2] += U[1]*time_increment;
        //wrap in [-PI, PI]
        
        
        return res;
    }
    MatrixType dF_dXv(const VectorType& Xv, const VectorType& U) {
//         cout << ">> dF_dXv called with Xv = (" << Xv.transpose() << "), U = (" << U.transpose() << ")" << endl;
        
        MatrixType J(3, 3);
        J <<    1, 0, (-U[0]*time_increment*sin(Xv[2]+U[1]*time_increment/2.0)),
                0, 1, (U[0]*time_increment*cos(Xv[2]+U[1]*time_increment/2.0)),
                0, 0, 1;
                
        return J;
    }
    
    ////landmark model
    //Xm = (mx, my)
    //Z = (rho, phi)
    VectorType H(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> H called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
        VectorType res(2);
        res << sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1])), atan2(Xm[1] - Xv[1], Xm[0] - Xv[0]) - Xv[2];
        
        return res;
    }
    MatrixType dH_dXv(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXv called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
        MatrixType J(2, 3);
        double rho = sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1]-Xm[1])*(Xv[1]-Xm[1]));

        J <<    (Xv[0] - Xm[0])/rho,            (Xv[1] - Xm[1])/rho,        0,
                -(Xv[1] - Xm[1])/(rho*rho),     (Xv[0] - Xm[0])/(rho*rho),  -1;
        
        return J;
    }
    MatrixType dH_dXm(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> dH_dXm called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
        MatrixType J(2, 2);
        double rho = sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1]));
        
        J <<    (-Xv[0] + Xm[0])/rho,          (-Xv[1] + Xm[1])/rho,
                (Xv[1] - Xm[1])/(rho*rho),     -(Xv[0] - Xm[0])/(rho*rho);
        
        return J;
    }
    
    ////initialization model
    VectorType G(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> G called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;
        
        VectorType Xm(2);
        Xm << Xv[0] + Z[0]*cos(Xv[2]+Z[1]), Xv[1] + Z[0]*sin(Xv[2]+Z[1]);
        
        return Xm;
    }
    MatrixType dG_dXv(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dXc called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;
        
        MatrixType J(2, 3);
        J <<    1,  0,  -Z[0]*sin(Xv[2]+Z[1]),
                0,  1,  Z[0]*cos(Xv[2]+Z[1]);
        
        return J;
    }
    MatrixType dG_dZ(const VectorType& Xv, const VectorType& Z) {
//         cout << ">> dG_dZ called with Xv = (" << Xv.transpose() << "), Z = (" << Z.transpose() << ")" << endl;
        
        MatrixType J(2, 2);
        J <<    cos(Xv[2]+Z[1]),  -Z[0]*sin(Xv[2]+Z[1]),
                sin(Xv[2]+Z[1]),  Z[0]*cos(Xv[2]+Z[1]);
        
        return J;
    }
    
    VectorType PointLandmarkDistance(const VectorType& z1, const VectorType& z2) {
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
        
        static const double PI2 = 2*M_PI;
        VectorType res(2);
        res[0] = z1[0] - z2[0];
        
        //angular distance
        ScalarType theta1 = z1[1], theta2 = z2[1];
//         cout << "z1: " << theta1 << ", z2: " << theta2 << endl;
        
        while(theta1 < 0.0) theta1 += PI2;
        theta1 = fmod(theta1, PI2);
        while(theta2 < 0.0) theta2 += PI2;
        theta2 = fmod(theta2, PI2);
//         cout << "theta1: " << theta1 << ", theta2: " << theta2 << endl;
        
        ScalarType d1 = fmod(theta1 - theta2 + PI2, PI2), d2 = fmod(PI2 + theta2 - theta1, PI2);
//         cout << "d1: " << d1 << ", d2: " << d2 << endl;
        
        if(d1 <= d2) {
            if(theta1 > theta2) {
                res[1] = d1;
            } else {
                res[1] = -d1;
            }
        } else {
            if(theta1 > theta2) {
                res[1] = d2;
            } else {
                res[1] = -d2;
            }
        }
        
//         cout << "Final distance: (" << res.transpose() << ")" << endl;
        
        return res;
    }
    VectorType StateDistance(const VectorType& z1, const VectorType& z2) {
//         cout << ">> PointLandmarkDistance called with z1: " << z1.transpose() << ", z2: " << z2.transpose() << endl;
        
        static const double PI2 = 2*M_PI;
        VectorType res(3);
        res[0] = z1[0] - z2[0];
        res[1] = z1[1] - z2[1];
        
        //angular distance
        ScalarType theta1 = z1[2], theta2 = z2[2];
//         cout << "z1: " << theta1 << ", z2: " << theta2 << endl;
        
        while(theta1 < 0.0) theta1 += PI2;
        theta1 = fmod(theta1, PI2);
        while(theta2 < 0.0) theta2 += PI2;
        theta2 = fmod(theta2, PI2);
//         cout << "theta1: " << theta1 << ", theta2: " << theta2 << endl;
        
        ScalarType d1 = fmod(theta1 - theta2 + PI2, PI2), d2 = fmod(PI2 + theta2 - theta1, PI2);
//         cout << "d1: " << d1 << ", d2: " << d2 << endl;
        
        if(d1 <= d2) {
            if(theta1 > theta2) {
                res[2] = d1;
            } else {
                res[2] = -d1;
            }
        } else {
            if(theta1 > theta2) {
                res[2] = d2;
            } else {
                res[2] = -d2;
            }
        }
        
//         cout << "Final distance: (" << res.transpose() << ")" << endl;
        
        return res;
    }
    /******************************************************************************
    *           MODEL   END
    * ***************************************************************************/
    ////model declaration
    VehicleModel VM(F, dF_dXv);
    LandmarkModel LM(H, dH_dXv, dH_dXm, PointLandmarkDistance);
    LandmarkInitializationModel LIM(G, dG_dXv, dG_dZ);
    
    /******************************************************************************
    *           SIMULATOR
    * ***************************************************************************/
    VectorType noisy_F(const VectorType& Xv, const VectorType& U) {
//         cout << ">> noisy_F called with Xv = (" << Xv.transpose() << "), U = (" << U.transpose() << ")" << endl;
        Vector3d res(Xv);
        
        res[0] += U[0]*time_increment*cos(Xv[2]+U[1]*time_increment/2.0) + state_pos_noise(eng_state);
        res[1] += U[0]*time_increment*sin(Xv[2]+U[1]*time_increment/2.0) + state_pos_noise(eng_state);
        res[2] += U[1]*time_increment + state_ang_noise(eng_state);
        
        return res;
    }
    VectorType Observation_generator(const VectorType& Xv, const VectorType& Xm) {
//         cout << ">> Observation_generator called with Xv = (" << Xv.transpose() << "), Xm = (" << Xm.transpose() << ")" << endl;
        
        VectorType res(2);
        res << sqrt((Xv[0]-Xm[0])*(Xv[0]-Xm[0]) + (Xv[1] - Xm[1])*(Xv[1] - Xm[1])) + observation_rho_noise(eng_ob), atan2(Xm[1]-Xv[1], Xm[0] - Xv[0]) - Xv[2] + observation_alpha_noise(eng_ob);
        
        return res;
    }
    VectorType Control_input_generator(int tick) {
        VectorType res(2);
        res << 4*pow(sin(tick/300.0 * 2 * M_PI), 2), 0.8*sin(tick/500.0 * 2 * M_PI);
//         res << 4*pow(sin(tick/300.0 * 2 * M_PI), 2), -0.005 + 0.8*sin(tick/500.0 * 2 * M_PI);
//         res << 4.0, 0.005 + 0.2*cos(tick/300.0* 2 * M_PI);
//         res << tick*0.001, 0.2;
//         cout << "arg: " << tick/100.0*2*M_PI;
//         res << 1.0, 2*sin(tick/50.0*2*M_PI);
//         res << 0.5 + sqrt(tick) / 100.0, 0.04;
        return res;
    }
    
    static const int LANDMARK_NUMBER = 50;
    static const double SENSOR_RANGE_MAX = 70;
    static const double SENSOR_RANGE_MIN = 0.05;
    static const double SENSOR_ANGLE_MAX = 2.1*M_PI;
    static default_random_engine lre(1);
    static uniform_int_distribution<int> un_x(-50, 500);
    static uniform_int_distribution<int> un_y(-100, 50);
    
    int slam_engine_test(int argc, char **argv) {
//         const double observation_sigma = 0.01;
        
        //real vehicle position
        VectorType Xv(3);
// 		Xv << state_pos_noise(eng_state), state_pos_noise(eng_state), state_ang_noise(eng_state);
		Xv << 0.0, 0.0, 0.0;
        
        ofstream out_Xv("/tmp/Xv.dat");
        ofstream out_XvE("/tmp/XvE.dat");
        ofstream real_Xm("/tmp/real_Xm.dat");
        ofstream tracked_Xm("/tmp/tracked_Xm.dat");
        
        ///SECTION: landmark initialization
        vector<VectorType> landmarks;
        map<int, int> associations;
        
        for(int ii = 0; ii < LANDMARK_NUMBER; ++ii) {
            VectorType Xm(2);
            Xm << un_x(lre), un_y(lre);
            
            real_Xm << Xm.transpose() << " ";
            
            landmarks.push_back(Xm);
        }
        real_Xm.close();
        
//         VectorType Xm(2);
//         Xm << 300, 0;
//         landmarks.push_back(Xm);
//         real_Xm << Xm.transpose() << " ";
        
//         Xm << 0, 10;
//         landmarks.push_back(Xm);
//         real_Xm << Xm.transpose() << " ";
//         
//         Xm << 0, -10;
//         landmarks.push_back(Xm);
//         real_Xm << Xm.transpose() << " ";
//         
//         Xm << -10, 0;
//         landmarks.push_back(Xm);
//         real_Xm << Xm.transpose() << " ";
        
        MatrixType R(2, 2);
//         R = MatrixXd::Identity(2, 2)*observation_sigma*observation_sigma;
		R << observation_rho_sigma*observation_rho_sigma, 0.0,
		0.0, observation_alpha_sigma*observation_alpha_sigma;
			
        MatrixType Q(3, 3);
        Q << state_pos_sigma*state_pos_sigma, 0, 0,
            0, state_pos_sigma*state_pos_sigma, 0,
            0, 0, state_ang_sigma*state_ang_sigma;
        
        //the SLAM engine
        EKFSLAMEngine se;
        //setup the state model
        se.Setup(Vector3d(0, 0, 0), Q, VM);
        cout << "Initial estimated Xv: " << se.GetStateEstimation().transpose() << endl;
        
        ///PLOTTING SCRIPT
        strstream script;
// 		script << "#!/usr/bin/gnuplot -persistent\nset datafile missing \"?\"\nunset colorbox\n";
		script << "#!/usr/bin/gnuplot -persistent\nset datafile missing \"?\"\n";
        for(int ii = 0; ii < landmarks.size(); ++ii) {
            script << "count" << ii << " = 0\n";
        }
        script << "plot '/tmp/Xv.dat' u 1:2 w l t 'real', '' u 4:5 w l t 'tracked', '/tmp/real_Xm.dat' u 1:2 t ''";
        for(int ii = 1; ii < landmarks.size(); ++ii) {
            script << ", '' u " << 2*ii+1 << ":" << 2*ii+2 << " t ''";
        }
        script << ", '/tmp/tracked_Xm.dat' u 1:2:(count0 = count0 + 1) palette t ''";
        for(int ii = 1; ii < landmarks.size(); ++ii) {
            script << ", '' u " << 2*ii+1 << ":" << 2*ii+2 << ":(count" << ii << " = count" << ii << " + 1) palette t ''";
        }
        script << endl << '\0';
        ofstream out_script("/tmp/script.gnu");
        out_script << script.str() << flush;
        out_script.close();
        
        const unsigned long int TOTAL_TICK = 1e4;
        for(int ii = 0; ii < TOTAL_TICK; ++ii) {
            auto t_start = high_resolution_clock::now();
            
            cout << "--[[ ITERATION " << ii << " ]]--\n";
            //initial condition
            cout << "Real Xv: " << Xv.transpose() << endl;
            cout << "Estimated Xv: " << se.GetStateEstimation().transpose() << endl;
            //the control input
            VectorType U(Control_input_generator(ii));
            cout << "--< prediction >--\nU: " << U.transpose() << endl;
            //real state update
            Xv = noisy_F(Xv, U);
            se.Predict(U, Q);
            cout << "Real Xv: " << Xv.transpose() << endl;
            cout << "Estimated Xv: " << se.GetStateEstimation().transpose() << endl;
            
            cout << "--< update >--\n";
            
            std::vector<AssociatedPerception> percs;
            std::vector<int> toAdd;
            for(int jj = 0; jj < landmarks.size(); ++jj) {
                //check if the robot sees the landmark ii
                Vector2d z = H(Xv, landmarks[jj]);
//                 double distance = sqrt((landmarks[ii][0] - Xv[0])*(landmarks[ii][0] - Xv[0]) + (landmarks[ii][1] - Xv[1])*(landmarks[ii][1] - Xv[1]));
                
                if(z[0] <= SENSOR_RANGE_MAX && z[0] > SENSOR_RANGE_MIN && abs(z[1]) <= SENSOR_ANGLE_MAX) {
                    //check if the landmarks ii has been seen before
                    if(associations.count(jj)) {
                        percs.push_back(AssociatedPerception(Observation_generator(Xv, landmarks[jj]), associations[jj]));
                    } else {
                        toAdd.push_back(jj);
//                         associations[ii] = se.AddNewLandmark(Observation_generator(Xv, landmarks[ii]), LM, LIM, R);
                    }
                }
            }
            
            if(!percs.empty()) {
				MatrixXd r = MatrixXd::Zero(percs.size()*2, percs.size()*2);
				for(int jj = 0; jj < percs.size(); ++jj) {
					r(2*jj, 2*jj) = observation_rho_sigma*observation_rho_sigma;
					r(2*jj+1, 2*jj+1) = observation_alpha_sigma*observation_alpha_sigma;
				}
// 				cerr << r;
                se.Update(percs, r);
            }
            if(!toAdd.empty()) {
                for(auto l_index : toAdd) {
                    associations[l_index] = se.AddNewLandmark(Observation_generator(Xv, landmarks[l_index]), LM, LIM, R);
                }
            }
            cout << "Landmark perceived: " << percs.size() << endl;
            cout << "Landmark tracked: " << se.GetTrackedLandmarksSize() << endl;
            cout << "Real Xv: " << Xv.transpose() << endl;
            cout << "Estimated Xv: " << se.GetStateEstimation().transpose() << endl;
            
            out_Xv << Xv.transpose() << " " << se.GetStateEstimation().transpose() << endl;
            
            for(int ii = 0; ii < landmarks.size(); ++ii) {
                if(associations.count(ii)) {
                    //tracked
                    VectorType Xm(se.GetLandmarkEstimation(associations[ii]));
                    tracked_Xm << " " << Xm.transpose();
                } else {
                    //not tracked
                    tracked_Xm << " ? ?";
                }
            }
            tracked_Xm << endl;
            
            double error = StateDistance(Xv, se.GetStateEstimation()).norm();
            cout << "--<< SUMMARY: Xv error: " << error << endl;
            out_XvE << error << endl;
            
            auto dt = high_resolution_clock::now() - t_start;
            cout << "--<< " << duration_cast<microseconds>(dt).count() << " us >>--" << endl << endl;
            cerr << '+';
        }
        
        return 0;
    }
    
    int full_slam_engine_test(int argc, char **argv) {
        //real vehicle position
        VectorType Xv(3);
        Xv << state_pos_noise(eng_state), state_pos_noise(eng_state), state_ang_noise(eng_state);
        
        ofstream out_Xv("/tmp/Xv.dat");
        ofstream out_XvE("/tmp/XvE.dat");
        ofstream real_Xm("/tmp/real_Xm.dat");
        ofstream tracked_Xm("/tmp/tracked_Xm.dat");
        
        ///SECTION: landmark initialization
        vector<VectorType> landmarks;
        
        for(int ii = 0; ii < LANDMARK_NUMBER; ++ii) {
            VectorType Xm(2);
            Xm << un_x(lre), un_y(lre);
            
            real_Xm << Xm.transpose() << " ";
            
            landmarks.push_back(Xm);
        }
        real_Xm.close();
        
		MatrixType R(2, 2);
		//         R = MatrixXd::Identity(2, 2)*observation_sigma*observation_sigma;
		R << observation_rho_sigma*observation_rho_sigma, 0,
		0.0, observation_alpha_sigma*observation_alpha_sigma;
		
        MatrixType Q(3, 3);
        Q << state_pos_sigma*state_pos_sigma, 0, 0,
            0, state_pos_sigma*state_pos_sigma, 0,
            0, 0, state_ang_sigma*state_ang_sigma;
        
        //the SLAM engine
        EKFSLAMEngine se;
        //setup the state model
        se.Setup(Vector3d(0, 0, 0), Q, VM);
        cout << "Initial estimated Xv: " << se.GetStateEstimation().transpose() << endl;
        
        ///PLOTTING SCRIPT
        strstream script;
        script << "#!/usr/bin/gnuplot -persistent\nset datafile missing \"?\"\nunset colorbox\n";
        for(int ii = 0; ii < landmarks.size(); ++ii) {
            script << "count" << ii << " = 0\n";
        }
        script << "plot '/tmp/Xv.dat' u 1:2 w l t 'real', '' u 4:5 w l t 'tracked', '/tmp/real_Xm.dat' u 1:2 t ''";
        for(int ii = 1; ii < landmarks.size(); ++ii) {
            script << ", '' u " << 2*ii+1 << ":" << 2*ii+2 << " t ''";
        }
        script << ", '/tmp/tracked_Xm.dat' u 1:2:(count0 = count0 + 1) palette t ''";
        for(int ii = 1; ii < landmarks.size(); ++ii) {
            script << ", '' u " << 2*ii+1 << ":" << 2*ii+2 << ":(count" << ii << " = count" << ii << " + 1) palette t ''";
        }
        script << endl << '\0';
        ofstream out_script("/tmp/script.gnu");
        out_script << script.str() << flush;
        out_script.close();
        
        const int TOTAL_TICK = 1e4;
        for(int ii = 0; ii < TOTAL_TICK; ++ii) {
            auto t_start = high_resolution_clock::now();
            
            cout << "--[[ ITERATION " << ii << " ]]--\n";
            //initial condition
            cout << "Real Xv: " << Xv.transpose() << endl;
            cout << "Estimated Xv: " << se.GetStateEstimation().transpose() << endl;
            //the control input
            VectorType U(Control_input_generator(ii));
            cout << "--< prediction >--\nU: " << U.transpose() << endl;
            //real state update
            Xv = noisy_F(Xv, U);
            se.Predict(U, Q);
            cout << "Real Xv: " << Xv.transpose() << endl;
            cout << "Estimated Xv: " << se.GetStateEstimation().transpose() << endl;
            
            cout << "--< update >--\n";
            
            ///SECTION: perceptions
            std::vector<Observation> observations;
            for(int ii = 0; ii < landmarks.size(); ++ii) {
                //check if the robot sees the landmark ii
                double distance = sqrt((landmarks[ii][0] - Xv[0])*(landmarks[ii][0] - Xv[0]) + (landmarks[ii][1] - Xv[1])*(landmarks[ii][1] - Xv[1]));
                
                if(distance <= SENSOR_RANGE_MAX && distance > 0.05) {
                    //check if the landmarks ii has been seen before
                    observations.push_back(Observation(Observation_generator(Xv, landmarks[ii]), R, LM, LIM));
                }
            }
            
//             if(!percs.empty()) {
//                 se.Update(percs, MatrixXd::Identity(percs.size()*2.0, percs.size()*2.0)*observation_sigma*observation_sigma);
//             }
            se.Update(observations);
            cout << "Landmark perceived: " << observations.size() << endl;
            cout << "Landmark tracked: " << se.GetTrackedLandmarksSize() << endl;
            cout << "Real Xv: " << Xv.transpose() << endl;
            cout << "Estimated Xv: " << se.GetStateEstimation().transpose() << endl;
            
            out_Xv << Xv.transpose() << " " << se.GetStateEstimation().transpose() << endl;
            
//             for(int ii = 0; ii < landmarks.size(); ++ii) {
//                 if(associations.count(ii)) {
//                     //tracked
//                     VectorType Xm(se.GetLandmarkEstimation(associations[ii]));
//                     tracked_Xm << " " << Xm.transpose();
//                 } else {
//                     //not tracked
//                     tracked_Xm << " ? ?";
//                 }
//             }
//             tracked_Xm << endl;
            
            double error = StateDistance(Xv, se.GetStateEstimation()).norm();
            cout << "--<< SUMMARY: Xv error: " << error << endl;
            out_XvE << error << endl;
            
            auto dt = high_resolution_clock::now() - t_start;
            cout << "--<< " << duration_cast<microseconds>(dt).count() << " us >>--" << endl << endl;
            cerr << '+';
        }
        
        return 0;
    }
}

////utility functions
void register_function(std::string name, TestFunction fn_ptr) {
    tests[name] = fn_ptr;
}
void RegisterFunctions() {
    register_function("base_test",          base_test);
    register_function("speed_test",         speed_test);
    register_function("slam_test",          engine_test::slam_engine_test);
    register_function("full_slam_test",     engine_test::full_slam_engine_test);
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

//plot "./Xv.dat" u 1:2 w l t "real", "./Xv.dat" u 4:5 w l t "tracked", "./Xm1.dat" u 3:4 w p t "estimated L1", "./Xm2.dat" u 3:4 w p t "estimated L2", "./Xm1.dat" u 1:2 w p t "L1", "./Xm2.dat" u 1:2 w p t "L2"