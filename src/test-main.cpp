////include
//std
#include <iostream>
#include <stdexcept>
//SLAM
#include "../SLAM/SLAM.h"

using namespace std;
using namespace Eigen;
using namespace SLAM;

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

int main(int argc, char **argv) {
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