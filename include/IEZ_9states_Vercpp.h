#ifndef IEZ_9STATES_VERCPP_   /* Include guard */
#define IEZ_9STATES_VERCPP_

#include <iostream>
#include <vector>
#include <unistd.h>
#include <fstream>
#include <thread>
#include <cmath>
#include <numeric>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include <unsupported/Eigen/MatrixFunctions>
#include <chrono>
#include "csv.h"
#include "json.hpp"

struct IEZ_ {
    double diff_endup;
    double travelled_distance;
};

struct IEZ_ iez_9states();
bool still_detection(int data_length, Eigen::MatrixXd acc_input, Eigen::MatrixXd gyro_input);
//test
std::vector<bool> still_detection_test();

#endif // IEZ_9STATES_VERCPP_