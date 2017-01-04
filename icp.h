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
        Matrix4d transformMatrix, initialGuess, newGuess;


    public:
        struct eJs {
            Vector3d e;
            MatrixXd J;
        };

        struct icpRes {
            MatrixXd newGuess;
            VectorXd chi;
        };

        void setFirstCloud(const Ref<const MatrixXd>& cloud);
        void setSecondCloud(const Ref<const MatrixXd>& cloud);
        void setInitialGuess(const Ref<const Matrix4d>& x_guess);
        icpRes allignClouds(int n_it);
        Matrix4d v2t(const VectorXd& x);
        eJs errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                        const Vector3d& p,
                                        const Vector3d& z);


        icp(const Ref<const MatrixXd>&  cloud1, const Ref<const MatrixXd>& cloud2);
        ~icp();
};



#endif // ICP_H_INCLUDED
