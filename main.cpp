#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <istream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


//using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


using namespace Eigen;

#include "icp.h"

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    std::string line;
    std::vector<double> values;
    MatrixXd mf;
    int rows=0;
    indata.open(path);

    if(!indata.is_open()){
        std::cout << "Failed to open " << path << std::endl;
        throw -1;
    }
    while (std::getline(indata, line)) {


        std::stringstream lineStream(line);
        std::string cell;

        while (std::getline(lineStream, cell, ' ')) {
            values.push_back(std::stod(cell));
        }

        ++rows;
    }

    mf = MatrixXd::Map(&values[0], 3, rows); // THE ORDER OF EIGEN MATRIX IS COLUMN MAJOR!

    return mf;
}

MatrixXd rotateSamples(const Ref<const MatrixXd>& samples){

    Matrix3d R;

    R = AngleAxisd(double(0.0), Vector3d::UnitX())
        * AngleAxisd(double(0.0),  Vector3d::UnitY())
        * AngleAxisd(double(0.01), Vector3d::UnitZ());


    return R * samples;

}



int main()
{
    MatrixXd globe, globeScene, initialGuess(4,4);
    globe = load_csv<MatrixXd>("globe.txt");
    globeScene = load_csv<MatrixXd>("globe.txt");
    std::cout << "inner " << globe.size() << " outer " << globeScene.size() <<std::endl;
    icp prova(globe, rotateSamples(globeScene));
    prova.setInitialGuess(initialGuess.setIdentity());
    icp::icpRes results = prova.allignClouds(20);

    std::cout << " trasformation matrix : \n\n" << results.newGuess <<
                " \n\n chi \n\n" << results.chi << std::endl;
    return 0;
}
