#ifndef ICP_H_INCLUDED
#define ICP_H_INCLUDED

#include <vector>
#include <iostream>
#include <fstream>
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/kdtree/kdtree_flann.h>
#include "geometry.h"
#include "pclVisualizer.h"

using namespace Eigen;




class icp {
    public:
        struct eJzs {
            Vector3d e,z;
            MatrixXd J;

        };

        struct icpResults {
            MatrixXd newGuess;
            VectorXd chi;
            VectorXd sigma;
            double totalError;
        };

        struct ransacResults {
            MatrixXd bestCloud1, bestCloud2;
            Matrix4d bestTransformation;
        };

        struct inliers {
            MatrixXd inliers1;
            MatrixXd inliers2;
        };
        

        icpResults allignClouds(const Ref<const MatrixXd>& firstCloud,
                                        const Ref<const MatrixXd>& secondCloud,
                                        const Ref<const Matrix4d>& initialGuess,
                                        int n_it, double kernel_threshold);
        eJzs errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                        const Vector3d& z,
                                        const Vector3d& p);
        icpResults relaxAllignClouds(const Ref<const MatrixXd>& set1,
                                        const Ref<const MatrixXd>& set2);
        ransacResults ransac(const Ref<const MatrixXd> cloud1, const Ref<const MatrixXd> cloud2,
                    int n_it, double sigmaThreshold, int minNumberOfCorrespondences);

        inliers resizeClouds(const Ref<const MatrixXd>& cloud1, const Ref<const MatrixXd>& cloud2, 
                        std::map<int, std::pair<int, double>>& correspondences);
        std::map<int,std::pair<int, double>> findCorrespondences(const Ref<const MatrixXd>& cloud, 
                                                double squaredDistanceThreshold,
                                                pcl::KdTreeFLANN<pcl::PointXYZ> kdtree);
        MatrixXd mapToMatrix(const std::map<int, Vector3d>& map);
        pcl::KdTreeFLANN<pcl::PointXYZ> kdTreeBuild(const Ref<const MatrixXd>& cloudEigen);


        icp();
        ~icp();
};



#endif // ICP_H_INCLUDED