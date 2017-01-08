#include "icp.h"


icp::icp(const Ref<const MatrixXd>& cloud1, const Ref<const MatrixXd>& cloud2):
                                            firstCloud(cloud1),
                                            secondCloud(cloud2)
{
    std::cout << "icp class created" << std::endl;
}


icp::~icp(){
    std::cout << "icp object destructed" << std::endl;
}

void icp::setFirstCloud(const Ref<const MatrixXd>& cloud){

    firstCloud = cloud;
    std::cout << "first cloud set" << std::endl;

}

void icp::setSecondCloud(const Ref<const MatrixXd>& cloud){

    secondCloud = cloud;
    std::cout << "second cloud set" << std::endl;
}

void icp::setInitialGuess(const Ref<const Matrix4d>& x_guess){
    initialGuess = x_guess;
    std::cout << "initial guess set." << std::endl;
}
icp::icpResults icp::allignClouds(int n_it, double kernel_threshold){

    VectorXd chi_stats(n_it);
    VectorXd num_inliers(n_it);
    VectorXd dx(6), b(6);
    MatrixXd H(6,6), X(4,4);
    double chi;
    eJs ej; //error and jacobian struct
    chi_stats.setZero();
    num_inliers.setZero();

    X = initialGuess;

    std::cout << "initial guess  \n\n" << X << std::endl;

    for (int it = 0; it < n_it ; it++){

        H.setZero();
        b.setZero();
        chi_stats(it) = 0;

        //std::cout << "it : " << it << "\n newGues \n" << newGuess << "\n chi \n" <<chi << std::endl;

        for (int i = 0; i < secondCloud.cols(); i++){

            ej = errorAndJacobianManifold(X, firstCloud.col(i), secondCloud.col(i));
            chi = ej.e.transpose() * ej.e;
            //std::cout<< "col difference: " << firstCloud.col(i)- secondCloud.col(i)<< std::endl;
            if(chi>kernel_threshold){
                ej.e = ej.e*sqrt(kernel_threshold/chi);
                chi = kernel_threshold;
            } else {
                num_inliers(it)++;
            }

            chi_stats(it) += chi;
            H+=ej.J.transpose() * ej.J;
            b+=ej.J.transpose() * ej.e;
            //std::cout << "chi : " << chi << std::endl;
        }

        dx = -H.colPivHouseholderQr().solve(b);
        X = v2t(dx) * X;
        std::cout << "inliers % " << num_inliers(it)/firstCloud.cols() << std::endl;

    }
    icpResults results;
    results.chi = chi_stats;
    results.newGuess = X;

    return results;
}

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





icp::eJs icp::errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                const Vector3d& p,
                                const Vector3d& z)
{
    Vector3d z_hat, e, t;
    eJs ej;
    MatrixXd J(3,6), skew_z_hat(3,3);

    J.setZero();

    t = x.col(3).head(3);

    z_hat = x.block(0, 0, 3, 3)*p + t;

    e = z_hat - z;

    skew_z_hat << 0, -z_hat(2), z_hat(1),
    z_hat(2), 0, -z_hat(0),
    -z_hat(1), z_hat(0), 0;

    J.leftCols(3).setIdentity();
    J.rightCols(3) = -skew_z_hat;

    //std::cout<<"J \n"<< J <<std::endl;

    ej.e = e;
    ej.J = J;

    return ej;

}



