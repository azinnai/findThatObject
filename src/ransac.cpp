#include "icp.h"
icp::ransacResults icp::ransac(const Ref<const MatrixXd> cloud1, const Ref<const MatrixXd> cloud2,
                    int n_it, double sigmaThreshold, int minNumberOfCorrespondences){

    Matrix4d bestTransformation;
    MatrixXd randomSamples1, randomSamples2;
    double chi(DBL_MAX);
    double alignmentError;
    inliers in; //inliers struct
    icpResults resultsIcp;
    ransacResults resultsRansac;
    MatrixXd newCloud1, newCloud2;
    Vector3d rgb1, rgb2;
                    
    int minNumData(3);
    int lastInliers, bestCorrespondences(0);
    std::vector<int> pointIdxSearch;
    std::vector<double> pointRadiusSquaredDistance;
    std::map<int,std::pair<int, double>> correspondences;

    bestTransformation.setIdentity();
    int rnd=0;
    double squaredDistanceThreshold = 1.0;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree = kdTreeBuild(cloud1);

    float radiusSearch = 0.02;
    pcl::PointCloud<pcl::PointXYZ>::Ptr first_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    exctractUSkeyPoints(cloud1, first_keypoints, radiusSearch);
    exctractUSkeyPoints(cloud2, second_keypoints, radiusSearch);

    std::cout << first_keypoints->size() << std::endl;
    std::cout << second_keypoints->size() << std::endl;
    rgb1 << 255,0,0;
    rgb2 << 0,255,0;
    int j=0;
    /*randomSamples1.resize(3,first_keypoints->size());
    randomSamples2.resize(3,second_keypoints->size());
    for(j=0;  j<first_keypoints->size(); ++j){
        randomSamples1.col(j) << first_keypoints->points[j].x, first_keypoints->points[j].y, first_keypoints->points[j].z;
    }
    for(j=0;  j<first_keypoints->size(); ++j){
        randomSamples2.col(j) << second_keypoints->points[j].x, second_keypoints->points[j].y, second_keypoints->points[j].z;
    }
    pointCloudVis(cloud1, randomSamples1, MatrixXd::Identity(4,4), rgb1, rgb2);
    pointCloudVis(cloud2, randomSamples2, MatrixXd::Identity(4,4), rgb1, rgb2);
*/

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist1(0.0, std::nextafter(first_keypoints->size(), DBL_MAX));
    std::uniform_real_distribution<double> dist2(0.0, std::nextafter(second_keypoints->size(), DBL_MAX));
    
    randomSamples1.resize(3,minNumData);
    randomSamples2.resize(3,minNumData);

    
    for(int it=0; it<n_it; ++it){
        if(it%250==0) std::cout << "ransac iteration :    " << it << std::endl;

        for (int i=0; i<minNumData; ++i){
            rnd = (int)(dist1(mt));
            randomSamples1.col(i) << first_keypoints->points[rnd].x, first_keypoints->points[rnd].y, first_keypoints->points[rnd].z;
            //randomSamples1.insert(std::pair<int, Vector3d>(rnd, cloud1.col(rnd)));
            rnd = (int)(dist2(mt));
            randomSamples2.col(i) << second_keypoints->points[rnd].x, second_keypoints->points[rnd].y, second_keypoints->points[rnd].z;
            //randomSamples2.insert(std::pair<int, Vector3d>(rnd, cloud2.col(rnd)));
        }

        resultsIcp = relaxAllignClouds(randomSamples1,
                                    randomSamples2);

        //if (resultsIcp.sigma.norm() < sigmaThreshold){//if the trasformation is too distorted discard it
        if(1==1){
            MatrixXd tmpCloud(4, cloud2.cols());
            tmpCloud.bottomRows(1).setOnes();
            tmpCloud.topRows(3) = cloud2;

            correspondences = findCorrespondences(resultsIcp.newGuess*tmpCloud, squaredDistanceThreshold, kdtree); //to write, return two equal size clouds

            if(correspondences.size() > minNumberOfCorrespondences){
                in = resizeClouds(cloud1, cloud2, correspondences);

                int leastSquaresIteration(10);
                int kernel_threshold(10^1);
                //resultsIcp = allignClouds(in.inliers1, in.inliers2,
                //                     resultsIcp.newGuess, leastSquaresIteration,
                //                      kernel_threshold);
                resultsIcp = relaxAllignClouds(in.inliers1, in.inliers2);
                alignmentError = resultsIcp.chi + (1.0 - ((double)in.inliers1.cols())/cloud1.cols());
                if(alignmentError<chi){
                    std::cout << "ransac iteration :    " << it << std::endl;
                    std::cout << "number of inliers: "<< correspondences.size() << std::endl;
                    //chi = resultsIcp.chi[resultsIcp.chi.size()-1];
                    chi=alignmentError;
                    std::cout << "chi: "<< chi << std::endl;
                    bestTransformation = resultsIcp.newGuess;
                    newCloud1 = in.inliers1;
                    newCloud2 = in.inliers2;
                    rgb1 << 255,0,0;
                    rgb2 << 0,255,0;
                    //pointCloudVis(cloud1, cloud2, resultsIcp.newGuess, rgb1, rgb2);
                                    
                }
            }
        }
    }
    resultsRansac.bestCloud1 = newCloud1;
    resultsRansac.bestCloud2 = newCloud2;
    resultsRansac.bestTransformation = bestTransformation;
    
    return resultsRansac;
    
}



icp::inliers icp::resizeClouds(const Ref<const MatrixXd>& cloud1, const Ref<const MatrixXd>& cloud2, 
                        std::map<int, std::pair<int, double>>& correspondences){
    MatrixXd newCloud1(3,correspondences.size()), newCloud2(3,correspondences.size());
    inliers in;
    int i = 0;
    for(auto const& ent1: correspondences){
        newCloud1.col(i) = cloud1.col(ent1.first);
        newCloud2.col(i) = cloud2.col(ent1.second.first); 
        ++i;
    }

    in.inliers1 = newCloud1;
    in.inliers2 = newCloud2;

    return in;
}


std::map<int,std::pair<int, double>> icp::findCorrespondences(const Ref<const MatrixXd>& cloud, 
                                    double squaredDistanceThreshold, 
                                    pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree){
    
    pcl::PointXYZ point;
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::map<int,std::pair<int, double>> correspondences;
    std::pair<int,double> newPoint;

    for (int i=0; i<cloud.cols(); ++i){
        point.x = cloud.col(i)(0);
        point.y = cloud.col(i)(1);
        point.z = cloud.col(i)(2);
        


        
        if(kdtree.nearestKSearch(point, K, pointIdxNKNSearch, 
                                pointNKNSquaredDistance) > 0){
        
            if(pointNKNSquaredDistance[0] < squaredDistanceThreshold){
        
                newPoint.first = i;
                newPoint.second = pointNKNSquaredDistance[0];
        
                if (correspondences.find(pointIdxNKNSearch[0])==correspondences.end()){
                    correspondences.insert(std::pair<int,std::pair<int, double>>(pointIdxNKNSearch[0], newPoint));
                } else if(correspondences[pointIdxNKNSearch[0]].second > pointNKNSquaredDistance[0]){
                    correspondences[pointIdxNKNSearch[0]].first = newPoint.first;
                    correspondences[pointIdxNKNSearch[0]].second = newPoint.second;

                }
            }
        }
        
    }

    return correspondences;
}


MatrixXd icp::mapToMatrix(const std::map<int, Vector3d>& map){
    MatrixXd samples(3, map.size());
    int i=0;
    for ( auto const& it: map){
        samples.col(i) = it.second;
        ++i;
    }
    return samples;
}

pcl::KdTreeFLANN<pcl::PointXYZ> icp::kdTreeBuild(const Ref<const MatrixXd>& cloudEigen){
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointXYZ point;

    for(int i=0; i<cloudEigen.cols(); ++i){
        point.x = cloudEigen.col(i)(0);
        point.y = cloudEigen.col(i)(1);
        point.z = cloudEigen.col(i)(2);
        cloud->push_back(point);

    }
    kdtree.setInputCloud(cloud);

    std::cout << "######### kdTree completed!! #######" << std::endl;

    return kdtree;
}

void icp::exctractUSkeyPoints(const Ref<const MatrixXd>& cloud,   pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints,
                                        float radiusSearch){

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);

    // creating pcl point cloud form data
    for (int i=0; i<cloud.cols(); ++i) {
      pcl::PointXYZ point;
      point.x = cloud.col(i)(0);
      point.y = cloud.col(i)(1);
      point.z = cloud.col(i)(2);
      pointCloud->points.push_back(point);
    }
    
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud(pointCloud);
    uniform_sampling.setRadiusSearch(radiusSearch);
    uniform_sampling.filter(*model_keypoints);

}
pcl::RangeImage icp::exctractNARFkeyPoints(const Ref<const MatrixXd>& cloud) {
    // --------------------
    // -----Parameters-----
    // --------------------
    float angular_resolution = 0.5f;
    float support_size = 0.2f;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = false;
    pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
    Affine3f scene_sensor_pose (Affine3f::Identity ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> pointCloud;

    // creating pcl point cloud form data
    for (int i=0; i<cloud.cols(); ++i) {
      pcl::PointXYZ point;
      point.x = cloud.col(i)(0);
      point.y = cloud.col(i)(1);
      point.z = cloud.col(i)(2);
      pointCloud.points.push_back(point);
    }
    pointCloud.width = (uint32_t) pointCloud.points.size();
    pointCloud.height = 1;
    // -----------------------------------------------
    // -----Create RangeImage from the PointCloud-----
    // -----------------------------------------------
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;   
    range_image.createFromPointCloud (pointCloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
                               scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.integrateFarRanges (far_ranges);
    if (setUnseenToMaxRange)
        range_image.setUnseenToMaxRange ();
    // --------------------------------
    // -----Extract NARF keypoints-----
    // --------------------------------
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage (&range_image);
    narf_keypoint_detector.getParameters ().support_size = support_size;
    //narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
    //narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute (keypoint_indices);
    std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";



   
    return range_image;
}


