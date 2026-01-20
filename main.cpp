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
#include "include/IEZ_9states_Vercpp.h"
#include "include/ZUPT_test.h"
//#include "include/csv.h"

int main(){
    //定时器，测试用
    //5分钟之后开始矫正
    //std::this_thread::sleep_for(std::chrono::seconds(300));

    //原算法-cpp复现
    /*struct IEZ_ res = iez_9states();
    std::cout << "Difference between the initial and final position = " << res.diff_endup << "\n";
    std::cout << "stimated total travelled distance = " << res.travelled_distance << "\n";*/

    //静止检测-测试
    /*std::vector<bool> stat = still_detection_test();
    for(int i = 0; i < stat.size(); i++)
        if(stat[i] ==1 )
            std::cout << "存在动作：第" << i << "组；   动作指标：" << stat[i] << "\n";*/

    Eigen::MatrixXd acc_input(5, 3);
    Eigen::MatrixXd gyro_input(5, 3);
    acc_input <<    11.7187,-1.51155,1.91878,
                    11.9942,-1.29356,1.91399,
                    11.7858,-0.85279,1.28877,
                    12.0876,-0.94861,0.869559,
                    12.1906,-1.69839,0.596474;
    gyro_input <<   1.70182,-1.49377,-0.700451,
                    1.84617,-1.92413,-0.871958,
                    2.06987,-2.04717,-0.736137,
                    2.36921,-2.05569,-0.459169,
                    2.75217,-2.55956,-0.443722;
    //正式静止检测
    bool stat = still_detection(5, acc_input, gyro_input);
    std::cout << "动作指标：" << stat << "\n";

    return 0;
}