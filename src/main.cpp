#include <iostream>
#include <string>
#include <fstream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "icp.h"
#include "geometry.h"


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
    MatrixXd globe, globeR(3,3), globeR_hom(3,3), X_Guess(4,4), globeScene;

    Matrix4d X_true;
    VectorXd x_true(6);

    globe = load_csv<MatrixXd>("../globe.txt");
    //globeScene = load_csv<MatrixXd>("globe-scene.txt");


    globeR.resize(globe.rows(), globe.cols());
    globeR_hom.resize(globe.rows()+1, globe.cols());
    globeR_hom.row(3).setOnes();

    x_true << 0.10, 0.10,0.10, 0.7, 0.7, 0.7;
    X_true = v2t(x_true);

    std::cout << "X_true  \n\n" << X_true << std::endl;


    globeR_hom.topRows(3) = globe;
    globeR_hom = X_true * globeR_hom;

    globeR = globeR_hom.topRows(3);

    double kernel_treshold(pow(10,1));
    int n_it = 10; //number of iterations of icp
    X_Guess.setIdentity();

    /*icp test(globe, globeScene);
    test.setInitialGuess(X_Guess);
    icp::icpResults results = test.allignClouds(n_it, kernel_treshold); //number of iterations;

    std::cout << " trasformation matrix : \n\n" << results.newGuess <<
                " \n\n chi \n\n" << results.chi <<
                "\n\n matrix difference \n\n"<< X_true-results.newGuess<<
                std::endl;
    */

    icp test(globe, globeR);
    test.setInitialGuess(X_Guess);
    icp::icpResults icpResults = test.allignClouds(n_it, kernel_treshold);

    icp::icpResults results = test.relaxAllignClouds(globe, globeR);


    std::cout << " trasformation matrix : \n\n" << icpResults.newGuess <<
            "\n\n matrix difference \n\n"<< X_true*icpResults.newGuess<<
            "\n\n chi matrix: \n\n " << icpResults.chi<<
            std::endl;

    /*std::cout << " trasformation matrix : \n\n" << results.newGuess <<
            "\n\n matrix difference \n\n"<< X_true-results.newGuess<<
            "\n\n sigma matrix: \n\n " << results.sigma <<
            std::endl;
*/
    return 0;
}
