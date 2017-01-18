#include <iostream>
#include <fstream>
#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include "icp.h"
#include "kdTree.h"
#include "converters.h"


int main(int argc, char* argv[])
{
    //getting the two point cloud, can receive either csv or pcd files.
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    std::string path1 = argv[1];
    std::string path2 = argv[2];

    if (path1.substr(path1.size() - 3) == "txt") {
        fromCsvToPcd(path1, reference_cloud);
    }  else if (path1.substr(path1.size() - 3) == "pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (path1, *reference_cloud) == -1){
            PCL_ERROR ("Couldn't read pcd files \n");
            return (-1);
        }
    } else {
        std::cerr << "invalid first point cloud file\n";
        return(-1);
    }

    if (path2.substr(path2.size() - 3) == "txt") {
        fromCsvToPcd(path2, second_cloud);
    }  else if (path2.substr(path2.size() - 3) == "pcd") {
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (path2, *second_cloud) == -1){
            PCL_ERROR ("Couldn't read pcd files \n");
            return (-1);
        }
    } else {
        std::cerr << "invalid second point cloud file\n";
        return (-1);
    }

    //downsampling both clouds
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setLeafSize(0.04, 0.04, 0.04);
    filter.setInputCloud(reference_cloud);
    filter.filter(*reference_cloud_filtered);
    filter.setInputCloud(second_cloud);
    filter.filter(*second_cloud_filtered);


    //creating the pcl viewer with some properties
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, -0.4, 0.0, -1.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            red (reference_cloud_filtered, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            green (second_cloud_filtered, 0, 255, 0);
    viewer->addPointCloud(reference_cloud_filtered, red, "reference_cloud");
    viewer->addPointCloud(second_cloud_filtered, green, "second_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "second_cloud");

    //building KDtree
    double leaf_range = 0.02;
    double max_distance = 0.15;

    VectorXdVector points(reference_cloud_filtered->width);
    fromPclToKD(reference_cloud_filtered, points);

    BaseTreeNode* rootTree = buildTree(points, leaf_range);

    matrix_container correspondences;
    MatrixXd eigen_reference_cloud_filtered;
    MatrixXd eigen_second_cloud_filtered;
    eigen_reference_cloud_filtered = fromPclToEigenM(reference_cloud_filtered);
    eigen_second_cloud_filtered = fromPclToEigenM(second_cloud_filtered);

    icp leastSquare;
    icp::icpResults leastSquaresResults;

    int n_it_max = 200;
    int n_it = 0;
    double kernel_threshold(1.0);
    Matrix4d guess, identity;

    Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();

    // Define a translation of 2.5 meters on the x axis.
    transform_2.translation() << 0.2, 0.0, 0.0;

    // The same rotation matrix as before; theta radians arround Z axis
    transform_2.rotate (Eigen::AngleAxisd (0.1, Eigen::Vector3d::UnitZ()));

    // Print the transformation
    guess = transform_2.matrix();
    //eigen_second_cloud_filtered = guess * eigen_second_cloud_filtered;

    guess.setIdentity();
    identity.setIdentity();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    double best_chi(DBL_MAX);

    while (!viewer->wasStopped() && n_it < n_it_max)
    {
        correspondences = findAllNeighbors(eigen_second_cloud_filtered, max_distance, rootTree);

        //correspondences.correspondences2 = guess.inverse() * correspondences.correspondences2;
        leastSquaresResults = leastSquare.allignClouds(correspondences.correspondences1,
                                                       correspondences.correspondences2,
                                                       identity, kernel_threshold);
        std::cerr << correspondences.correspondences1.cols() << "  " << correspondences.correspondences2.cols() <<
                  "chi:  " << leastSquaresResults.chi << std::endl;
        //if (leastSquaresResults.chi<best_chi) {
            eigen_second_cloud_filtered = leastSquaresResults.newGuess * eigen_second_cloud_filtered;
            guess = leastSquaresResults.newGuess * guess;
            pcl::transformPointCloud(*second_cloud_filtered, *transformed_cloud, guess);
            best_chi = leastSquaresResults.chi;
        //}

        viewer->removeAllPointClouds();

        viewer->addPointCloud(reference_cloud_filtered, red, "reference_cloud");
        viewer->addPointCloud(transformed_cloud, green, "second_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, "reference_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.5, "second_cloud");

        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        ++n_it;
    }

    return 0;
}