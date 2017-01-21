//
// Created by ale on 18/01/17.
//

#ifndef FINDTHATOBJECT_CONVERTERS_H
#define FINDTHATOBJECT_CONVERTERS_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include "kdTree.h"

struct matrix_container{
    MatrixXd correspondences1;
    MatrixXd correspondences2;

};

matrix_container findAllNeighbors(const Ref<const MatrixXd>& queryM, double max_distance, BaseTreeNode* rootTree);
void fromPclToKD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                 VectorXdVector& points);
template<typename M>
M load_csv (const std::string & path);

void fromCsvToPcd(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);
MatrixXd fromPclToEigenM(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);
void fromEigenToKD(const MatrixXd & cloud,
                   VectorXdVector& points);
#endif //FINDTHATOBJECT_CONVERTERS_H
