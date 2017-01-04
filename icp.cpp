#include "icp.h"


icp::icp(const Ref<const MatrixXd>& cloud1, const Ref<const MatrixXd>& cloud2):
                                            firstCloud(cloud1),
                                            secondCloud(cloud2)
                                            //transformMatrix(Matrix4d::Identity())
{
    std::cout << "icp class created" << std::endl;
    newGuess = initialGuess;
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
    newGuess = initialGuess;
    std::cout << "initial guess set." << std::endl;
}
icp::icpRes icp::allignClouds(int n_it){

    VectorXd chi_stats(n_it);
    VectorXd dx(6), b(6);
    MatrixXd H(6,6);
    double chi;
    eJs ej; //error and jacobian struct
    chi_stats.setZero();
    std::cout << "initial guess  \n\n" << newGuess << std::endl;

    for (int it = 0; it < n_it ; it++){

        H.setZero();
        b.setZero();
        chi = 0;

        //std::cout << "it : " << it << "\n newGues \n" << newGuess << "\n chi \n" <<chi << std::endl;

        for (int i = 0; i < secondCloud.cols(); i++){

            ej = errorAndJacobianManifold(newGuess, firstCloud.col(i), secondCloud.col(i));
            H+=ej.J.transpose() * ej.J;
            b+=ej.J.transpose() * ej.e;
            chi+=ej.e.transpose() * ej.e;
        }

        chi_stats(it) = chi;
        dx = H.colPivHouseholderQr().solve(b);
        newGuess = v2t(dx) * newGuess;

        //std::cout << "it : " << it << "\n newGues \n" << newGuess << "\n chi \n" <<chi << std::endl;
    }
    icpRes results;
    results.chi = chi_stats;
    results.newGuess = newGuess;

    return results;
}

Matrix4d icp::v2t(const VectorXd& x){

    Matrix4d T;
    Matrix3d R;
    T.setIdentity();

    R = AngleAxisd(x[3], Vector3d::UnitX())
    * AngleAxisd(x[4],  Vector3d::UnitY())
    * AngleAxisd(x[5], Vector3d::UnitZ());

    T.block<3,3>(0,0) = R;
    T(0,3) = x[0];
    T(1,3) = x[1];
    T(2,3) = x[2];

    return T;
}



icp::eJs icp::errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                const Vector3d& p,
                                const Vector3d& z)
{
    Vector3d z_hat, e, t;
    eJs ej;
    MatrixXd J(3,6), skew_z_hat(3,3);

    J.setZero();

    t << x(0,3), x(1,3), x(2,3);

    z_hat = x.block<3,3>(0,0)*p + t;
    e = z_hat - z;

    skew_z_hat << 0, -z_hat(2), z_hat(1),
    z_hat(2), 0, -z_hat(0),
    -z_hat(1), z_hat(0), 0;

    J.block<3,3>(0,0).setIdentity();
    J.block<3,3>(0,3) = skew_z_hat;

    ej.e = e;
    ej.J = J;

    return ej;

}



