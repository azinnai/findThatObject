#ifndef ICP_H_INCLUDED
#define ICP_H_INCLUDED

#include <vector>
#include <istream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;



class icp {
    private:
        MatrixXd firstCloud, secondCloud;
        Matrix4d transformMatrix, initialGuess;


    public:
        struct eJs {
            Vector3d e;
            MatrixXd J;
        };

        struct icpResults {
            MatrixXd newGuess;
            VectorXd chi;
        };

        void setFirstCloud(const Ref<const MatrixXd>& cloud);
        void setSecondCloud(const Ref<const MatrixXd>& cloud);
        void setInitialGuess(const Ref<const Matrix4d>& x_guess);
        icpResults allignClouds(int n_it, double kernel_threshold);
        eJs errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                        const Vector3d& p,
                                        const Vector3d& z);


        icp(const Ref<const MatrixXd>&  cloud1, const Ref<const MatrixXd>& cloud2);
        ~icp();
};

Matrix4d v2t(const VectorXd& x);
Matrix3d Rx(double alpha);
Matrix3d Ry(double alpha);
Matrix3d Rz(double alpha);


#endif // ICP_H_INCLUDED
