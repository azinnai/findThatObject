#include "icp.h"


icp::icp()
{
    std::cout << "icp object created" << std::endl;
}


icp::~icp(){
    std::cout << "icp object destructed" << std::endl;
}


icp::icpResults icp::relaxAllignClouds(const Ref<const MatrixXd>& set1,
                                        const Ref<const MatrixXd>& set2){


    VectorXd dx(12), b(12), x(12), e(3);
    MatrixXd H(12,12), X(4,4), Hi(3,12), A(3,3), R(3,3), M(3,9);
    icpResults results;
    double chi=0;


    X.setIdentity();

    x << 1,0,0,   //initial guess linear relaxed
         0,1,0,
         0,0,1,
         0,0,0;


    H.setZero();
    b.setZero();

    for (int i = 0; i < set1.cols(); i++){
        M.setZero();

        M.block(0,0,1,3) = set2.col(i).transpose();
        M.block(1,3,1,3) = set2.col(i).transpose();
        M.block(2,6,1,3) = set2.col(i).transpose();


        Hi.leftCols(9) = M;
        Hi.rightCols(3).setIdentity();

        e = M * x.segment(0,9) + x.segment(9,3) - set1.col(i);
        

        H+=Hi.transpose()*Hi;
        b+=Hi.transpose()*e;

    }

    dx = -H.colPivHouseholderQr().solve(b);
    x += dx;


    A.row(0) = x.segment(0,3).transpose();
    A.row(1) = x.segment(3,3).transpose();
    A.row(2) = x.segment(6,3).transpose();

    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

    R = svd.matrixU() * svd.matrixV().transpose();

    X.block(0,0,3,3) = R;
    X.block(0,3,3,1) = x.tail(3);

    results.newGuess = X;
    results.sigma = svd.singularValues();

    eJzs ejz; //error and jacobian struct

    for (int j = 0; j < set1.cols(); j++){
        ejz = errorAndJacobianManifold(X, set1.col(j), set2.col(j));
        chi += ejz.e.transpose() * ejz.e;
    }

    results.totalError = chi/((double)set1.cols());

    return results; //the relaxed solution for the initial correspondences.
}


icp::icpResults icp::allignClouds(const Ref<const MatrixXd>& firstCloud,
                                        const Ref<const MatrixXd>& secondCloud,
                                        const Ref<const Matrix4d>& initialGuess,
                                        int n_it, double kernel_threshold){

    VectorXd chi_stats(n_it);
    VectorXd num_inliers(n_it);
    VectorXd dx(6), b(6);
    MatrixXd H(6,6), X(4,4);
    double chi;
    eJzs ejz; //error and jacobian struct
    chi_stats.setZero();
    num_inliers.setZero();

    X = initialGuess;

    for (int it = 0; it < n_it ; it++){

        H.setZero();
        b.setZero();
        chi_stats(it) = 0;
      
        for (int i = 0; i < secondCloud.cols(); i++){

            ejz = errorAndJacobianManifold(X, firstCloud.col(i), secondCloud.col(i));
            chi = ejz.e.transpose() * ejz.e;

            //std::cout<< "col difference: " << firstCloud.col(i)- secondCloud.col(i)<< std::endl;
            if(chi>kernel_threshold){
                ejz.e = ejz.e*sqrt(kernel_threshold/chi);
                chi = kernel_threshold;
            } else {
                num_inliers(it)++;
            }

            chi_stats(it) += chi;
            H+=ejz.J.transpose() * ejz.J;
            b+=ejz.J.transpose() * ejz.e;

        }
        dx = -H.ldlt().solve(b);
        X = v2t(dx) * X ;
        //Vector3d rgb1, rgb2;
        //rgb1 << 255,0,0;
        //rgb2 << 0,255,0;
        //pointCloudVis(firstCloud, secondCloud, X, rgb1, rgb2);

      
    }


    icpResults results;
    results.chi = chi_stats;
    results.newGuess = X;

    return results;
}

icp::eJzs icp::errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                const Vector3d& z,
                                const Vector3d& p)
{
    Vector3d z_hat, e, t;
    eJzs ej;
    MatrixXd J(3,6), skew_z_hat(3,3);

    J.setZero();

    t = x.col(3).head(3);

    z_hat = x.block(0, 0, 3, 3)*p + t;

    e = z_hat - z;

    skew_z_hat << 0, -z_hat(2), z_hat(1),
    z_hat(2), 0, -z_hat(0),
    -z_hat(1), z_hat(0), 0;

    J.leftCols(3).setIdentity();
    J.rightCols(3) = -2*skew_z_hat;

    ej.e = e;
    ej.J = J;
    ej.z = z_hat;

    return ej;

}
