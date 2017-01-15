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
    globeScene = load_csv<MatrixXd>("../globe-scene.txt");


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
    int n_it = 20; //number of iterations of icp
    X_Guess.setIdentity();

    /*icp test(globe, globeScene);
    test.setInitialGuess(X_Guess);
    icp::icpResults results = test.allignClouds(n_it, kernel_treshold); //number of iterations;

    std::cout << " trasformation matrix : \n\n" << results.newGuess <<
                " \n\n chi \n\n" << results.chi <<
                "\n\n matrix difference \n\n"<< X_true-results.newGuess<<
                std::endl;
    */

    icp test;
    
    //icp::icpResults icpResults = test.allignClouds(globe, globeR, X_Guess, n_it, kernel_treshold);

    //icp::icpResults relaxResults = test.relaxAllignClouds(globe, globeR);
    
    icp::ransacResults resultRansac = test.ransac(globe, globeR, 10000, 100, 100);

   /* std::cout << "Relax Results \n##################\n";
    std::cout << " trasformation matrix : \n\n" << relaxResults.newGuess.inverse() <<
            "\n\n matrix difference \n\n"<< X_true*relaxResults.newGuess<<
            "\n\n sigma values: \n\n " << relaxResults.sigma<<
            std::endl;
    std::cout << "icp Results \n##################\n";
    std::cout << " trasformation matrix : \n\n" << icpResults.newGuess.inverse() <<
            "\n\n matrix difference \n\n"<< X_true*icpResults.newGuess<<
            "\n\n chi matrix: \n\n " << icpResults.chi<<
            std::endl;
    */
    std::cout << "Ransac Results \n##################\n";
    std::cout << "true transformation matrix:\n\n" << X_true << 
            "\n\ntrasformation matrix : \n\n" << resultRansac.bestTransformation.inverse() <<
            "\n\n matrix difference \n\n"<< X_true*resultRansac.bestTransformation<<
            std::endl;
    
    //X_Guess = resultRansac.bestTransformation;

    
    Vector3d rgb1, rgb2;
    rgb1 << 255,0,0;
    rgb2 << 0,255,0;
    pointCloudVis(globe, globeR, resultRansac.bestTransformation, rgb1, rgb2);

    return 0;
}
