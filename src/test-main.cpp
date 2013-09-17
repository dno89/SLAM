////include
//std
#include <iostream>
#include <stdexcept>
//SLAM
#include "../SLAM/SLAM.h"

using namespace std;
using namespace Eigen;
using namespace SLAM;

VectorType f(const VectorType& Xv, const VectorType& U)  {
}

MatrixType df_dxv(const VectorType& Xv, const VectorType& U)  {
}

int main(int argc, char **argv) {
    std::cout << "Hello, world!\nThis is a big SLAM test\n" << std::endl;
    
    SLAMEngine e;
    
    VectorType v;
    v << 0, 1, 2;
    
    MatrixType p(3, 3);
    p << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
    
    e.Setup(3, v, p, VehicleModel(f, df_dxv));
    return 0;
}

////FAILED EXPERIMENT
// ////test support
// void F(Vector2d& x, const Vector2d& u) {
//     x += u;
// }
// 
// ////test class
// class SLAM {
//     //the pose
// public:
//     ////subclasses
//     //placeholder
//     class PosePlaceholder {
//     public:
//         //sizes
//         virtual int GetPoseSize() = 0;
//         virtual int GetInputSize() = 0;
//         
//         //prediction
//         template<int InputSize>
//         void Predict(const Matrix<ScalarType, InputSize, 1>& input);
//     };
//     //acutal pose
//     template<int PoseSize, int InputSize>
//     class Pose : public PosePlaceholder {
//     public:
// //         static const int poseSize = PoseSize;
// //         static const int inputSize = InputSize;
//         ////typedef
//         typedef void (*FType)(Matrix<ScalarType, PoseSize, 1>&, const Matrix<ScalarType, InputSize, 1>&);
//         typedef Matrix<ScalarType, PoseSize, 1> PoseType;
//     private:
// //         static const int poseSize = PoseSize;
// //         static const int inputSize = InputSize;
//         
//         //data
//         PoseType m_pose;
//         FType const m_F;
//     public:
//         ////constructor
//         Pose(PoseType starting_pose, FType f) : m_F(f), m_pose(starting_pose) {
//             cout << "Pose constructed with PoseSize: " << PoseSize << ", InputSize: " << InputSize << endl;
//         } 
//         
//         ////
//         int GetPoseSize() { return PoseSize;}
//         int GetInputSize() { return InputSize;}
//         
//     };
//     
//     
//     SLAM() {}
//     //pose
//     template<int PoseSize, int InputSize>
//     void addVehiclePose(const Matrix<ScalarType, PoseSize, 1>& starting_pose, void (*F)(Matrix<ScalarType, PoseSize, 1>&, const Matrix<ScalarType, InputSize, 1>&)) {
//         cout << "PoseSize: " << PoseSize << ", InputSize: " << InputSize << endl;
//         PosePlaceholder* pph = new Pose<PoseSize, InputSize>(starting_pose, F);
//         delete pph;
//     }
// };