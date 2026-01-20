#include <iostream>
#include <vector>
#include <fstream>
#include <thread>
#include <cmath>
#include <numeric>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include <unsupported/Eigen/MatrixFunctions>
#include "iez_pdr.h"
#include "utils.h"

#ifdef _WIN32
#include <windows.h>
#include <stdlib.h>
#endif

int main() {
#ifdef _WIN32
    // 设置控制台代码页为 UTF-8，解决中文乱码问题
    SetConsoleOutputCP(65001);
    SetConsoleCP(65001);
    
    // 重定向标准错误句柄以抑制 gnuplot 的警告输出
    // 这样所有子进程（包括 gnuplot）都会继承这个设置
    HANDLE hOriginalStderr = GetStdHandle(STD_ERROR_HANDLE);
    HANDLE hNull = CreateFileA("nul", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hNull != INVALID_HANDLE_VALUE) {
        SetStdHandle(STD_ERROR_HANDLE, hNull);
    }
    
    // 将 gnuplot 路径添加到 PATH 环境变量（如果系统 PATH 中没有）
    // 尝试多个可能的路径
    std::vector<std::string> possible_paths = {
        "D:\\codes\\3rdParty\\gnuplot\\bin"
    };
    
    std::string gnuplot_path;
    for (const auto& path : possible_paths) {
        // 检查路径是否存在（通过检查 gnuplot.exe）
        std::string exe_path = path + "\\gnuplot.exe";
        DWORD dwAttrib = GetFileAttributesA(exe_path.c_str());
        if (dwAttrib != INVALID_FILE_ATTRIBUTES && 
            !(dwAttrib & FILE_ATTRIBUTE_DIRECTORY)) {
            gnuplot_path = path;
            break;
        }
    }
    
    // 如果找到 gnuplot 路径，添加到 PATH
    if (!gnuplot_path.empty()) {
        std::string current_path;
        size_t required_size = 0;
        getenv_s(&required_size, nullptr, 0, "PATH");
        if (required_size > 0) {
            current_path.resize(required_size);
            getenv_s(&required_size, current_path.data(), required_size, "PATH");
            // 移除末尾的空字符
            if (!current_path.empty() && current_path.back() == '\0') {
                current_path.pop_back();
            }
        }
        
        // 检查 PATH 中是否已包含 gnuplot 路径
        if (current_path.find(gnuplot_path) == std::string::npos) {
            // 添加到 PATH
            std::string new_path = current_path + ";" + gnuplot_path;
            _putenv_s("PATH", new_path.c_str());
        }
    }
#endif
    // 定时器，测试用
    // 5分钟之后开始矫正
    // std::this_thread::sleep_for(std::chrono::seconds(300));

    // 原算法复现
    {
        IEZ9StatesProcessor processor;
        IEZ_ res = processor.process();
        std::cout << "Difference between the initial and final position = " << res.diff_endup << "\n";
        std::cout << "Estimated total travelled distance = " << res.travelled_distance << "\n";
        
        // 可视化轨迹和旋转
        processor.visualize();
    }

    // 静止检测函数
    std::vector<bool> stat = still_detection_test();
    for (size_t i = 0; i < stat.size(); i++) {
        if (stat[i]) {
            std::cout << "存在动作: 第" << i << "组, 动作指标: " << stat[i] << std::endl;
        }
    }

    // 静止检测
    Eigen::MatrixXd acc_input(5, 3);
    Eigen::MatrixXd gyro_input(5, 3);
    acc_input << 11.7187, -1.51155, 1.91878,
                 11.9942, -1.29356, 1.91399,
                 11.7858, -0.85279, 1.28877,
                 12.0876, -0.94861, 0.869559,
                 12.1906, -1.69839, 0.596474;
    gyro_input << 1.70182, -1.49377, -0.700451,
                  1.84617, -1.92413, -0.871958,
                  2.06987, -2.04717, -0.736137,
                  2.36921, -2.05569, -0.459169,
                  2.75217, -2.55956, -0.443722;

    {
        StillDetectionProcessor detector(9.8, 9.4, 0.6, 1);
        bool is_stationary = detector.detect(5, acc_input, gyro_input);
        std::cout << "当前检测到动作: " << (is_stationary ? "静止中..." : "运动中...") << "\n";
    }

    // 测试矩阵工具类
    {
        std::vector<std::vector<double>> mat1 = {{1, 2}, {3, 4}};
        std::vector<std::vector<double>> mat2 = {{5, 6}, {7, 8}};
        
        try {
            auto result = MatrixUtils::multiply(mat1, mat2);
            std::cout << "矩阵乘法结果: [" << result[0][0] << ", " << result[0][1] 
                      << "; " << result[1][0] << ", " << result[1][1] << "]\n";
        } catch (const std::exception& e) {
            std::cerr << "矩阵乘法错误: " << e.what() << "\n";
        }
    }

    // 恢复标准错误句柄
    #ifdef _WIN32
    if (hNull != INVALID_HANDLE_VALUE) {
        SetStdHandle(STD_ERROR_HANDLE, hOriginalStderr);
        CloseHandle(hNull);
    }
    #endif

    return 0;
}