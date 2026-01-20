#ifndef IEZ_PDR_H_
#define IEZ_PDR_H_

// 在 Windows MSVC 上启用 M_PI 等数学常量（必须在包含 <cmath> 之前定义）
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <iostream>
#include <vector>
#include <fstream>
#include <thread>
#include <cmath>
#include <numeric>
#include <string>
#include <memory>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include <unsupported/Eigen/MatrixFunctions>
#include <chrono>
#include "csv.h"
#include "json.hpp"

// 结果
struct IEZ_ {
    double diff_endup = 0.0;
    double travelled_distance = 0.0;
};

// IEZ算法类
class IEZ9StatesProcessor {
public:

    explicit IEZ9StatesProcessor(
        const std::string& dataset_file = "../dataset/conf-3333-coleta04-02-06-21-5ds_03_test.csv",
        int acc_scale_range = 3,
        int gyro_scale_range = 3,
        int used_IMUsensor = 2
    );
    

    ~IEZ9StatesProcessor() = default;
    
    // 禁止拷贝构造和赋值
    IEZ9StatesProcessor(const IEZ9StatesProcessor&) = delete;
    IEZ9StatesProcessor& operator=(const IEZ9StatesProcessor&) = delete;
    
    // 允许移动构造和赋值
    IEZ9StatesProcessor(IEZ9StatesProcessor&&) = default;
    IEZ9StatesProcessor& operator=(IEZ9StatesProcessor&&) = default;
    
    IEZ_ process();
    
    // 可视化轨迹和旋转
    void visualize() const;

private:
    // 配置参数
    std::string dataset_file_name_;
    int acc_scale_range_;
    int gyro_scale_range_;
    int used_IMUsensor_;
    
    // 辅助方法
    void initializeMatrices(int file_length);
    void loadData(int file_length);
    void calculateInitialValues();
    void detectStationary(int file_length);
    void processMainLoop(int file_length);
    IEZ_ calculateResults(int file_length);
    
    // 数据成员
    int file_length_;
    Eigen::MatrixXd acc_s_;
    Eigen::MatrixXd gyro_s_;
    Eigen::MatrixXd acc_n_;
    Eigen::MatrixXd vel_n_;
    Eigen::MatrixXd pos_n_;
    std::vector<bool> stationary_;
    
    // 旋转矩阵历史（用于可视化）
    std::vector<Eigen::Matrix3d> rotation_history_;
    
    // 初始值和状态
    Eigen::Vector3d gyro_bias_;
    Eigen::Vector3d acc_bias_;
    Eigen::Matrix3d C_init_;
    
    // 常量参数
    static constexpr double g_ = 9.81;
    static constexpr double dt_ = 0.01;
    static constexpr int W_ = 10;
    static constexpr double sigma_omega_ = 0.01;
    static constexpr double sigma_a_ = 0.1;
    static constexpr double acc_stationary_threshold_H_ = 11.0;
    static constexpr double acc_stationary_threshold_L_ = 9.0;
    static constexpr double gyro_stationary_threshold_ = 0.6;
};

// 静止检测类
class StillDetectionProcessor {
public:

    StillDetectionProcessor(
        double acc_threshold_H = 9.8,
        double acc_threshold_L = 9.4,
        double gyro_threshold = 0.6,
        int window_size = 1
    );

    ~StillDetectionProcessor() = default;
    
    StillDetectionProcessor(const StillDetectionProcessor&) = delete;
    StillDetectionProcessor& operator=(const StillDetectionProcessor&) = delete;
    
    StillDetectionProcessor(StillDetectionProcessor&&) = default;
    StillDetectionProcessor& operator=(StillDetectionProcessor&&) = default;
    
    // 检测静止状态
    bool detect(int data_length, const Eigen::MatrixXd& acc_input, const Eigen::MatrixXd& gyro_input);

private:
    double acc_threshold_H_;
    double acc_threshold_L_;
    double gyro_threshold_;
    int window_size_;
};

// 测试函数：从文件读取数据并检测静止
std::vector<bool> still_detection_test();

#endif
