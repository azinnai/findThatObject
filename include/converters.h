#ifndef FINDTHATOBJECT_CONVERTERS_H
#define FINDTHATOBJECT_CONVERTERS_H
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/uniform_sampling.h>

#include "kdTree.h"


void fromPclToKD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                 VectorXdVector& points);
template<typename M>
M load_csv (const std::string & path);

void fromCsvToPcd(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr);

MatrixXd fromPclToEigenM(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud);

#endif //FINDTHATOBJECT_CONVERTERS_H
