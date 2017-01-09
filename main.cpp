#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "icp.h"
#include "geometry.h"

#include <Eigen/Dense>
#include <Eigen/Core>

template<typename M>
M load_csv (const std::string & path) {
    std::ifstream indata;
    std::string line;
    std::vector<double> values;
    MatrixXd md;
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
            values.push_back(std::stod(cell));
        }

        ++cols;
    }

    md = MatrixXd::Map(&values[0], 3, cols); // THE ORDER OF EIGEN MATRIX IS COLUMN MAJOR!

    return md;
}


int main()
{
    MatrixXd globe, globeR(3,3), globeR_hom(3,3), X_Guess(4,4);

    Matrix4d X_true;
    VectorXd x_true(6);

    globe = load_csv<MatrixXd>("globe.txt");
    //globeScene = load_csv<MatrixXd>("globe.txt");
    //globe = MatrixXd::Random(3,10);
    std::cout << "inner " << globe.cols() << " outer " << globe.rows() <<std::endl;

    globeR.resize(globe.rows(), globe.cols());
    globeR_hom.resize(globe.rows()+1, globe.cols());
    globeR_hom.setIdentity();

    x_true << 0.0,0.0,0.0, M_PI/2, M_PI/3, M_PI/6;
    X_true = v2t(x_true);

    globeR_hom.topRows(3) = globe;
    globeR_hom= X_true * globeR_hom;

    globeR = globeR_hom.topRows(3);

    //try to insert some noise.
    VectorXd movement(6);
    movement << 0.0,0.0,0.0,0.0,0.0,0.0;
    X_Guess = v2t(x_true + movement);

    double kernel_treshold(pow(10,1));
    icp test(globe, globeR);
    test.setInitialGuess(X_Guess);
    icp::icpResults results = test.allignClouds(20, kernel_treshold); //number of iterations;

    std::cout << " trasformation matrix : \n\n" << results.newGuess <<
                " \n\n chi \n\n" << results.chi <<
                "\n\n matrix difference \n\n"<< X_true-results.newGuess<<
                std::endl;



    return 0;
}
