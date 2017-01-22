#ifndef ICP_H_INCLUDED
#define ICP_H_INCLUDED

#include <vector>
#include <iostream>
#include <fstream>
#include <float.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/uniform_sampling.h>


#include "geometry.h"
#include "converters.h"

using namespace Eigen;

struct eJzs {
    Vector3d e,z;
    MatrixXd J;

};

struct icpResults {
    MatrixXd newGuess;
    double chi;
    VectorXd sigma;
};

struct matrix_container{
    MatrixXd correspondences1;
    MatrixXd correspondences2;

};

matrix_container findAllNeighbors(const Ref<const MatrixXd>& queryM, double max_distance, BaseTreeNode* rootTree);


icpResults allignClouds(const Ref<const MatrixXd>& firstCloud,
                                const Ref<const MatrixXd>& secondCloud,
                                const Ref<const Matrix4d>& initialGuess,
                                double kernel_threshold);

eJzs errorAndJacobianManifold(const Ref<const Matrix4d>& x,
                                const Vector3d& z,
                                const Vector3d& p);

icpResults relaxAllignClouds(const Ref<const MatrixXd>& set1,
                                const Ref<const MatrixXd>& set2);

void exctractUSkeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,   pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints,
                                float radiusSearch);

Matrix4d coarseAllignment(const MatrixXd & referenceCloud, const MatrixXd & secondCloud, int n_it, BaseTreeNode* kdTree, double maxDistance);

void getCenterOfMass(const Ref<const MatrixXd>& cloud, Vector3d& com, Vector3d& std);

#endif // ICP_H_INCLUDED
