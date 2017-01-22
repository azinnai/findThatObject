#include "converters.h"

void fromPclToKD(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud,
                 VectorXdVector& points){
    for(int i=0; i < pcl_cloud->width * pcl_cloud->height; ++i){
        int dimension = 3;
        VectorXd point(dimension);

        point(0) = pcl_cloud->points[i].x;
        point(1) = pcl_cloud->points[i].y;
        point(2) = pcl_cloud->points[i].z;

        points[i] = point;
    }
}

MatrixXd fromPclToEigenM(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud){
    MatrixXd eigen_cloud(4, pcl_cloud->width);

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

    md = MatrixXd::Map(&values[0], 3, cols);

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

