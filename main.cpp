#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <istream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <set>

//using namespace std;

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

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





int main()
{
    MatrixXd globe, globeScene;
    globe = load_csv<MatrixXd>("globe-scene.txt");
    globeScene = (load_csv<MatrixXd>("globe.txt"));


    return 0;
}
