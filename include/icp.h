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
#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
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
                                double kernel_threshold);
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
                                        pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree);
MatrixXd mapToMatrix(const std::map<int, Vector3d>& map);
pcl::KdTreeFLANN<pcl::PointXYZ> kdTreeBuild(const Ref<const MatrixXd>& cloudEigen);
pcl::RangeImage exctractNARFkeyPoints(const Ref<const MatrixXd>& cloud);
void exctractUSkeyPoints(const Ref<const MatrixXd>& cloud,   pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints,
                                float radiusSearch);
Matrix4d coarseAllignment(const MatrixXd & referenceCloud, const MatrixXd & secondCloud, int n_it, BaseTreeNode* kdTree, double maxDistance);

void getCenterOfMass(const Ref<const MatrixXd>& cloud, Vector3d& com, Vector3d& std);



#endif // ICP_H_INCLUDED
