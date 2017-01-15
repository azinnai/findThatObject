#include "icp.h"
icp::ransacResults icp::ransac(const Ref<const MatrixXd> cloud1, const Ref<const MatrixXd> cloud2,
                    int n_it, double sigmaThreshold, int minNumberOfCorrespondences){

    Matrix4d bestTransformation;
    std::map<int, Vector3d> randomSamples1, randomSamples2;
    double chi(DBL_MAX);
    double alignmentError;
    inliers in; //inliers struct
    icpResults resultsIcp;
    ransacResults resultsRansac;
    MatrixXd newCloud1, newCloud2;

    int samples1(cloud1.cols());
    int samples2(cloud2.cols());
    int minNumData(3);
    int lastInliers, bestCorrespondences(0);
    std::vector<int> pointIdxSearch;
    std::vector<double> pointRadiusSquaredDistance;
    std::map<int,std::pair<int, double>> correspondences;

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_real_distribution<double> dist1(0.0, std::nextafter(samples1, DBL_MAX));
    std::uniform_real_distribution<double> dist2(0.0, std::nextafter(samples2, DBL_MAX));
    bestTransformation.setIdentity();
    int rnd=0;
    double squaredDistanceThreshold = 1.0;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree = kdTreeBuild(cloud1);

    for(int it=0; it<n_it; ++it){
        if(it%250==0) std::cout << "ransac iteration :    " << it << std::endl;

        for (int i=0; i<minNumData; ++i){
            rnd = (int)(dist1(mt));
            randomSamples1.insert(std::pair<int, Vector3d>(rnd, cloud1.col(rnd)));
            rnd = (int)(dist2(mt));
            randomSamples2.insert(std::pair<int, Vector3d>(rnd, cloud2.col(rnd)));
        }

        resultsIcp = relaxAllignClouds(mapToMatrix(randomSamples1),
                                    mapToMatrix(randomSamples2));


        //if (resultsIcp.sigma.norm() < sigmaThreshold){//if the trasformation is too distorted discard it
        if(1==1){
            MatrixXd tmpCloud(4, cloud2.cols());
            tmpCloud.bottomRows(1).setOnes();
            tmpCloud.topRows(3) = cloud2;

            correspondences = findCorrespondences(resultsIcp.newGuess*tmpCloud, squaredDistanceThreshold, kdtree); //to write, return two equal size clouds
            if(correspondences.size() > minNumberOfCorrespondences){
                in = resizeClouds(cloud1, cloud2, correspondences);

                int leastSquaresIteration(10);
                int kernel_threshold(10^2);
                //resultsIcp = allignClouds(in.inliers1, in.inliers2,
                //                     resultsIcp.newGuess, leastSquaresIteration,
                //                      kernel_threshold);
                resultsIcp = relaxAllignClouds(in.inliers1, in.inliers2);
                alignmentError = resultsIcp.totalError + (1.0 - ((double)in.inliers1.cols())/cloud1.cols());
                if(alignmentError<chi){
                    std::cout << "ransac iteration :    " << it << std::endl;
                    std::cout << "number of inliers: "<< correspondences.size() << std::endl;
                    //chi = resultsIcp.chi[resultsIcp.chi.size()-1];
                    chi=alignmentError;
                    std::cout << "chi: "<< chi << std::endl;
                    bestTransformation = resultsIcp.newGuess;
                    //MatrixXd tmpCloud1, tmpCloud2;
                    //tmpCloud1.resize(in.inliers1.rows(), in.inliers1.cols());
                    //tmpCloud2.resize(in.inliers2.rows(), in.inliers2.cols());
                    //tmpCloud1 = in.inliers1;
                    //tmpCloud2 = in.inliers2;
                    newCloud1 = in.inliers1;
                    newCloud2 = in.inliers2;
                    /*Vector3d rgb1, rgb2;
                    rgb1 << 255,0,0;
                    rgb2 << 0,255,0;
                    pointCloudVis(cloud1, cloud2, resultsIcp.newGuess, rgb1, rgb2);
                    */                
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
                                    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree){
    
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


