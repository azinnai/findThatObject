#include "geometry.h"


Matrix4d v2t(const VectorXd& x){

    Matrix4d T;
    Matrix3d R;
    T.setIdentity();

    R = Rx(x[3])*Ry(x[4])*Rz(x[5]);

    T.block(0,0,3,3) = R;
    T(0,3) = x[0];
    T(1,3) = x[1];
    T(2,3) = x[2];

    return T;
}

Matrix3d Rx(double alpha){
    Matrix3d R;
    double c(cos(alpha));
    double s(sin(alpha));

    R <<  1,0,0,
          0,c,-s,
          0,s,c;

    return R;
}

Matrix3d Ry(double alpha){
    Matrix3d R;
    double c(cos(alpha));
    double s(sin(alpha));

    R << c,0,s,
         0,1,0,
         -s,0,c;

    return R;

}

Matrix3d Rz(double alpha){
    Matrix3d R;
    double c(cos(alpha));
    double s(sin(alpha));

    R <<  c,-s,0,
          s,c,0,
          0,0,1;

    return R;

}


