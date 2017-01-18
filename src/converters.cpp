#include "converters.h"

void fromPclToKD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                 VectorXdVector& points){
    for(int i=0; i < pcl_cloud->width; ++i){
        int dimension = 3;
        VectorXd point(dimension);

        point(0) = pcl_cloud->points[i].x;
        point(1) = pcl_cloud->points[i].y;
        point(2) = pcl_cloud->points[i].z;

        points[i] = point;
    }
}
MatrixXd fromPclToEigenM(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud){
    MatrixXd eigen_cloud(4,pcl_cloud->width);

    for(int i=0; i < pcl_cloud->width; ++i){
        int dimension = 4;
        VectorXd point(dimension);

        point(0) = pcl_cloud->points[i].x;
        point(1) = pcl_cloud->points[i].y;
        point(2) = pcl_cloud->points[i].z;
        point(3) = 1.0;
        eigen_cloud.col(i) = point;
    }
    return  eigen_cloud;
}

template<typename M>
M load_csv (const std::string & path) {

    std::ifstream indata;
    std::string line;
    std::vector<double> values;
    M md;
    int cols=0;
    indata.open(path);

    if(!indata.is_open()){
        std::cout << "Failed to open " << path << std::endl;
        throw -1;
    }
    while (std::getline(indata, line)) {


        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ' ')) {
            values.push_back(std::stof(cell));
        }

        ++cols;
    }

    md = MatrixXd::Map(&values[0], 3, cols); // THE ORDER OF EIGEN MATRIX IS COLUMN MAJOR!

    return md;
}

void fromCsvToPcd(std::string path, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr){

    MatrixXd cloud;
    cloud = load_csv<MatrixXd>(path);

    for(int i = 0; i < cloud.cols(); ++i){
        pcl::PointXYZ _point;
        _point.x = (float)cloud.col(i)(0);
        _point.y = (float)cloud.col(i)(1);
        _point.z = (float)cloud.col(i)(2);
        cloud_ptr->points.push_back (_point);
    }
    cloud_ptr->width = cloud.cols();
    cloud_ptr->height = 1;

}

matrix_container findAllNeighbors(const Ref<const MatrixXd>& queryM, double max_distance, BaseTreeNode* rootTree){

    MatrixXd correspondences1(3,1), correspondences2(3,1);

    for (int i=0; i < queryM.cols(); ++i){
        VectorXd answer(3), query(queryM.col(i).head(3));

        if (rootTree->findNeighbor(answer, query, max_distance) > 0){

            correspondences1.conservativeResize(correspondences1.rows(), correspondences1.cols() +1);
            correspondences2.conservativeResize(correspondences2.rows(), correspondences2.cols() +1);

            correspondences1.rightCols(1) = answer;
            correspondences2.rightCols(1) = query;
        }
    }

    correspondences1.conservativeResize(correspondences1.rows()+1, correspondences1.cols());
    correspondences2.conservativeResize(correspondences2.rows()+1, correspondences2.cols());
    correspondences1.bottomRows(1).setOnes();
    correspondences2.bottomRows(1).setOnes();
    matrix_container correspondences;
    correspondences.correspondences1 = correspondences1;
    correspondences.correspondences2 = correspondences2;
    return correspondences;
}