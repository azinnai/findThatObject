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

#include "converters.h"
#include "kdTree.h"
#include "icp.h"


static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
              << "\tInsert exactly two point cloud sources\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-v,--voxel\t\tSet voxel grid size to downsample data, default=0.04\n"
              << "\t-d,--distance\t\tSpecify the max distance between tow points to be associated, default=0.15\n"
              << "\t-i,--iterations\t\tNumber of least squares iterations, default=200\n"
              << std::endl;
}

int main(int argc, char* argv[])
{
    std::string path1("void"), path2("void");
    double max_distance = 0.02f;
    double voxelSize = 0.03f;
    int n_it_max = 100;
    bool USampling = false;
    bool VoxelGrid = true;
    bool normals = false;
    bool moveCloud = false;


    if (argc < 3) {
        show_usage(argv[0]);
        return 1;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        } else if ((arg == "-v") || (arg == "--voxel")) {
            if (i + 2 < argc) { // Make sure we aren't at the end of argv!
                std::string value = argv[++i];
                voxelSize = std::stod(value); // Increment 'i' so we don't get the argument as the next argv[i].
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "There are no sources" << std::endl;
                return 1;
            }

        } else if ((arg == "-d") || (arg == "--distance")) {
            if (i + 2 < argc) { // Make sure we aren't at the end of argv!
                std::string value = argv[++i];
                max_distance = std::stod(value); // Increment 'i' so we don't get the argument as the next argv[i].
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "There are no sources" << std::endl;
                return 1;
            }
        } else if ((arg == "-i") || (arg == "--iterations")) {
            if (i + 2 < argc) { // Make sure we aren't at the end of argv!
                std::string value = argv[++i];
                n_it_max = std::stoi(value); // Increment 'i' so we don't get the argument as the next argv[i].
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "There are no sources" << std::endl;
                return 1;
            }
        } else {
            if (path1 == "void") path1 = argv[i];
            else path2 = argv[i];
        }
    }

    //reading the two point cloud, can receive either csv or pcd files.
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());


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




    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    if(VoxelGrid == true) {
        //downsampling both clouds using voxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        filter.setInputCloud(reference_cloud);
        filter.filter(*reference_cloud_filtered);
        filter.setInputCloud(second_cloud);
        filter.filter(*second_cloud_filtered);
    }




    //prova uskeypoint
    if(USampling == true) {
        double radiusSearch = 0.07;
        exctractUSkeyPoints(fromPclToEigenM(reference_cloud), reference_cloud_filtered, radiusSearch);
        exctractUSkeyPoints(fromPclToEigenM(second_cloud), second_cloud_filtered, radiusSearch);
    }


    //building KDtree
    double leaf_range = 0.02;
    VectorXdVector points(reference_cloud_filtered->width);
    fromPclToKD(reference_cloud_filtered, points);
    BaseTreeNode* rootTree = buildTree(points, leaf_range);


    //converting PointCloudXYZ to 4*N Eigen Matrix needed by leastsquares
    MatrixXd eigen_reference_cloud_filtered;
    MatrixXd eigen_second_cloud_filtered;
    eigen_reference_cloud_filtered = fromPclToEigenM(reference_cloud_filtered);
    eigen_second_cloud_filtered = fromPclToEigenM(second_cloud_filtered);



    Matrix4d guess, identity;

    //move the cloud
    if(moveCloud== true) {

        Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();

        // Define a translation of 2.5 meters on the x axis.
        transform_2.translation() << 0.2, 0.0, 0.0;

        // The same rotation matrix as before; theta radians arround Z axis
        transform_2.rotate(Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()));

        // Print the transformation
        guess = transform_2.matrix();
        eigen_second_cloud_filtered = guess * eigen_second_cloud_filtered;
        pcl::transformPointCloud(*second_cloud_filtered, *transformed_cloud, guess);
    }
    guess.setIdentity();
    identity.setIdentity();

    //creating the pcl viewer with some properties
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.0, 0.0, -0.4, 0.0, -1.0, 0.0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(reference_cloud_filtered, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(transformed_cloud, 0, 255, 0);

    //variables for the loop
    int n_it = 0;
    double kernel_threshold(0.1);
    matrix_container correspondences;
    //icp leastSquare; moved above for testing
    icpResults leastSquaresResults;
    std::cerr << "point_reference : " << reference_cloud_filtered->width <<
              "\npoint_second : " << second_cloud_filtered->width <<std::endl;

    //com try

    guess = coarseAllignment(eigen_reference_cloud_filtered, eigen_second_cloud_filtered, 100000, rootTree, max_distance);
    eigen_second_cloud_filtered = guess * eigen_second_cloud_filtered;

    while (!viewer->wasStopped() && n_it < n_it_max)
    {
        correspondences = findAllNeighbors(eigen_second_cloud_filtered, max_distance, rootTree);

        leastSquaresResults = allignClouds(correspondences.correspondences1,
                                                       correspondences.correspondences2,
                                                       identity, kernel_threshold);
        if(n_it%100==0) {
            std::cerr << "inliers: %" << float(correspondences.correspondences1.cols())/reference_cloud_filtered->width <<
                      " error:  " << float(leastSquaresResults.chi) << std::endl;
        }
        eigen_second_cloud_filtered = leastSquaresResults.newGuess * eigen_second_cloud_filtered;
        guess = leastSquaresResults.newGuess * guess;
        pcl::transformPointCloud(*second_cloud_filtered, *transformed_cloud, guess);
        viewer->removeAllPointClouds();
        viewer->addPointCloud(reference_cloud_filtered, red, "reference_cloud");
        viewer->addPointCloud(transformed_cloud, green, "second_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5, "reference_cloud");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5, "second_cloud");

        viewer->spinOnce (1000);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        ++n_it;
    }

    return 0;
}
