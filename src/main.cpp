#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>

#include "kdTree.h"
#include "converters.h"
#include "icp.h"


static void show_usage(std::string name)
{
    std::cerr << "Usage: " << name << " <option(s)> SOURCES"
              << "\nInsert exactly two point cloud sources as last arguments, better if first the scene and then the object.\n"
              << "At least -r or -v with values has to be inserted\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-r,--radius\t\tEnable Uniform Sampling, set the radius and disable Voxel Grid, normally 0.02 is a good choice\n"
              << "\t-v,--voxel\t\tEnable downsampling with Voxel Grid and set the voxel size, normally 0.02 is a good choice\n"
              << "\t-d,--distance\t\tSpecify the max distance between two points to be associated, default=0.008\n"
              << "\t-i,--iterations\t\tNumber of least squares iterations, default=200\n"
              << "\t-c,--coarse\t\tNumber of coarse allignment iterations, default=200000\n"
              << std::endl;
}


int main(int argc, char* argv[])
{
    std::string path1("void"), path2("void");
    double max_distance = 0.008f;
    double voxelSize = 0.02f;
    int n_it_max = 200;
    int coarse_it = 200000;
    float radiusSearchUS = 0.02f;
    bool USampling = false;
    bool VoxelGrid = false;
    bool moveCloud = false;


    if (argc < 3) {
        show_usage(argv[0]);
        return 1;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if(argc < 5){
            std::cerr << "You have to provide minimum a downsampling flag and values "
                    "and the paths for the scene and the object in order"<< std::endl;
            return  0;
        } else if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        } else if ((arg == "-v") || (arg == "--voxel")) {
            if (i + 2 < argc) { // Make sure we aren't at the end of argv!
                std::string value = argv[++i];
                voxelSize = std::stod(value); // Increment 'i' so we don't get the argument as the next argv[i].
                VoxelGrid = true;
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
        } else if ((arg == "-c") || (arg == "--coarse")) {
            if (i + 2 < argc) { // Make sure we aren't at the end of argv!
                std::string value = argv[++i];
                coarse_it = std::stoi(value); // Increment 'i' so we don't get the argument as the next argv[i].
            } else { // Uh-oh, there was no argument to the destination option.
                std::cerr << "There are no sources" << std::endl;
                return 1;
            }
        } else if ((arg == "-r") || (arg == "--radius")) {
            if (i + 2 < argc) { // Make sure we aren't at the end of argv!
                std::string value = argv[++i];
                radiusSearchUS = std::stof(value); // Increment 'i' so we don't get the argument as the next argv[i].
                USampling = true;
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
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

    std::cout << "\nReference cloud number of points before filtering:  " << reference_cloud->width * reference_cloud->height <<
              "\nCloud to allign number of points before filtering:  " << second_cloud->width * second_cloud->height << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    if(VoxelGrid) {
        //downsampling both clouds using voxelGrid
        pcl::VoxelGrid<pcl::PointXYZ> filter;
        filter.setLeafSize(voxelSize, voxelSize, voxelSize);
        filter.setInputCloud(reference_cloud);
        filter.filter(*reference_cloud_filtered);
        filter.setInputCloud(second_cloud);
        filter.filter(*second_cloud_filtered);
    }

    //prova uskeypoint
    if(USampling) {
        pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
        exctractUSkeyPoints(reference_cloud, reference_cloud_filtered, radiusSearchUS);
        exctractUSkeyPoints(second_cloud, second_cloud_filtered, radiusSearchUS);
    }

    //building KDtree
    double leaf_range = 0.02;
    VectorXdVector points(reference_cloud_filtered->width * reference_cloud_filtered->height);
    fromPclToKD(reference_cloud_filtered, points);
    BaseTreeNode* rootTree = buildTree(points, leaf_range);


    //converting PointCloudXYZ to 4*N Eigen Matrix needed by ICP
    MatrixXd eigen_reference_cloud_filtered;
    MatrixXd eigen_second_cloud_filtered;

    eigen_reference_cloud_filtered = fromPclToEigenM(reference_cloud_filtered);
    eigen_second_cloud_filtered = fromPclToEigenM(second_cloud_filtered);


    Matrix4d guess, identity;

    //move the cloud
    if(moveCloud) {

        Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();

        // Define a translation of 2.5 meters on the x axis.
        transform_2.translation() << 0.2, 0.0, 0.0;

        // The same rotation matrix as before; theta radians arround Z axis
        transform_2.rotate(Eigen::AngleAxisd(-0.10*M_PI, Eigen::Vector3d::UnitZ()));

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
    double last_error(0);
    double error_threshold(0.0000001);
    double inliers(0), total_points(0);
    matrix_container correspondences;
    icpResults leastSquaresResults;
    std::cout << "\nReference cloud number of points after filtering:  " << eigen_reference_cloud_filtered.cols() <<
              "\nCloud to allign number of points after filtering:  " << eigen_second_cloud_filtered.cols() <<std::endl;

    std::cout << "\n###### Coarse allignment ######\n" << std::endl;
    //performing coarse registration
    guess = coarseAllignment(eigen_reference_cloud_filtered, eigen_second_cloud_filtered, coarse_it, rootTree, max_distance);
    eigen_second_cloud_filtered = guess * eigen_second_cloud_filtered;

    std::cout << "\n###### Coarse registration matrix #######\n\n" << guess <<
              "\n\n###### ICP ######\n" << std::endl;

    //ICP loop with viewer
    //try to move a little if ICP stuck in a local minima
    while (!viewer->wasStopped() && n_it < n_it_max)
    {
        correspondences = findAllNeighbors(eigen_second_cloud_filtered, max_distance, rootTree);

        leastSquaresResults = allignClouds(correspondences.correspondences1,
                                                       correspondences.correspondences2,
                                                       identity, kernel_threshold);



        if(n_it%5 == 0) {
            if(eigen_reference_cloud_filtered.cols() > eigen_second_cloud_filtered.cols()) {
                inliers = correspondences.correspondences1.cols();
                total_points = eigen_second_cloud_filtered.cols();
            }
            else {
                inliers = correspondences.correspondences1.cols();
                total_points = eigen_reference_cloud_filtered.cols();
            }

            std::cout << "inliers: " << 100*inliers/total_points << " %  "
                      " error:  " << float(leastSquaresResults.chi) << std::endl;
        }

        eigen_second_cloud_filtered = leastSquaresResults.newGuess * eigen_second_cloud_filtered;
        guess = leastSquaresResults.newGuess * guess;

        if(n_it%5 == 0) {
            pcl::transformPointCloud(*second_cloud_filtered, *transformed_cloud, guess);
            viewer->removeAllPointClouds();
            viewer->addPointCloud(reference_cloud_filtered, red, "reference_cloud");
            viewer->addPointCloud(transformed_cloud, green, "second_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5,
                                                     "reference_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.5,
                                                     "second_cloud");

            viewer->spinOnce(1000);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }

        if(std::abs(leastSquaresResults.chi - last_error) < error_threshold) break;
        last_error = leastSquaresResults.chi;

        ++n_it;
    }

    std::cout << "\n########### Best registration matrix ###########\n\n" << guess <<
                 "\n\n##############################################\n" << std::endl;

    return 0;
}
