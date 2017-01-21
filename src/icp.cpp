#include "icp.h"

icpResults relaxAllignClouds(const Ref<const MatrixXd>& set1,
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

    results.chi = chi;

    return results; //the relaxed solution for the initial correspondences.
}


icpResults allignClouds(const Ref<const MatrixXd>& firstCloud,
                                        const Ref<const MatrixXd>& secondCloud,
                                        const Ref<const Matrix4d>& initialGuess,
                                        double kernel_threshold){

    VectorXd dx(6), b(6);
    MatrixXd H(6,6), X(4,4);
    double chi(0), error;
    eJzs ejz; //error and jacobian struct

    X = initialGuess;

    H.setZero();
    b.setZero();

    for (int i = 0; i < secondCloud.cols(); i++){

        ejz = errorAndJacobianManifold(X, firstCloud.col(i).head(3), secondCloud.col(i).head(3));
        error = ejz.e.transpose() * ejz.e;
        if(error>kernel_threshold) {
            error = error * sqrt(kernel_threshold / error);
            chi += error;
        } else chi+=error;
        H+=ejz.J.transpose() * ejz.J;
        b+=ejz.J.transpose() * ejz.e;

    }
    dx = -H.ldlt().solve(b);
    X = v2t(dx) * X ;

    icpResults results;
    results.chi = chi;
    results.newGuess = X;

    return results;
}

eJzs errorAndJacobianManifold(const Ref<const Matrix4d>& x,
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

Matrix4d coarseAllignment(const MatrixXd & referenceCloud, const MatrixXd & secondCloud, int n_it, BaseTreeNode* kdTree, double maxDistance){

    Vector3d comReferenceMean, comSecondMean, comReferenceSTD, comSecondSTD;
    Matrix4d bestGuess, guess, initialGuess, identity;
    MatrixXd tmpCloud, initialCloud;
    double best_chi(DBL_MAX);
    int max_correspondences(0);
    identity.setIdentity();
    bestGuess.setIdentity();

    matrix_container correspondences;
    icpResults results;

    getCenterOfMass(referenceCloud, comReferenceMean, comReferenceSTD);
    getCenterOfMass(secondCloud, comSecondMean, comSecondSTD);
    //std::cerr << "mean " << comReferenceMean.transpose() << "\nstd  " << comReferenceSTD.transpose() << std::endl;
    initialGuess.setIdentity();
    initialGuess.col(3).head(3) = comReferenceMean - comSecondMean;
    initialCloud = initialGuess * secondCloud;
    std::cerr << "initial guess \n " << initialGuess << std::endl;

    /*building KDtree
    double leaf_range = 0.02;
    VectorXdVector points(referenceCloud.cols());
    fromEigenToKD(referenceCloud, points);
    BaseTreeNode* rootTree = buildTree(points, leaf_range);
    */

    std::random_device rd;
    std::mt19937 mt(rd());
    std::normal_distribution<double> distribution_x(0.0, 3*comReferenceSTD(0));
    std::normal_distribution<double> distribution_y(0.0, 4*comReferenceSTD(1));
    std::normal_distribution<double> distribution_z(0.0, 3*comReferenceSTD(2));

    correspondences = findAllNeighbors(initialCloud, maxDistance, kdTree);
    results = allignClouds(correspondences.correspondences1, correspondences.correspondences2, identity, 0.1);


    for(int it = 0; it < n_it; ++it){
        double rndx, rndy, rndz;
        rndx = distribution_x(mt);
        rndy = distribution_y(mt);
        rndz = distribution_z(mt);

        guess << 1,0,0,rndx,
                0,1,0,rndy,
                0,0,1,rndz,
                0,0,0,1;

        tmpCloud = guess * initialCloud;
        correspondences = findAllNeighbors(tmpCloud, maxDistance, kdTree);
        //results = allignClouds(correspondences.correspondences1, correspondences.correspondences2, identity, 0.1);
        //if(results.chi < best_chi){
        if(correspondences.correspondences2.cols() > max_correspondences ){
            bestGuess = guess;
            //best_chi = results.chi;
            max_correspondences = correspondences.correspondences2.cols();
            std::cerr << "Max correnspondeces" << correspondences.correspondences2.cols() <<
                      std::endl;
        }
    }

    bestGuess = bestGuess * initialGuess;

    return bestGuess;
}

void getCenterOfMass(const Ref<const MatrixXd>& cloud, Vector3d& com, Vector3d& std){

    com.setZero();
    std.setZero();

    for(int i = 0; i < cloud.cols(); ++i){
        com(0) += cloud.col(i)(0);
        com(1) += cloud.col(i)(1);
        com(2) += cloud.col(i)(2);
    }
    com = com/cloud.cols();

    for(int j=0; j < cloud.cols(); ++j){
        std(0) += (cloud.col(j)(0) - com(0))*(cloud.col(j)(0) - com(0));
        std(1) += (cloud.col(j)(1) - com(1))*(cloud.col(j)(1) - com(1));
        std(2) += (cloud.col(j)(2) - com(2))*(cloud.col(j)(2) - com(2));
    }

    std = std/cloud.cols();

    std::cerr << "mean " << com.transpose() << "\nstd  " << std.transpose() << std::endl;
}
