#include "iez_pdr.h"

// 如果定义了 USE_MATPLOT，包含 Matplot++ 头文件
#ifdef USE_MATPLOT
#include <matplot/matplot.h>
#endif

// 静态辅助函数：四元数转欧拉角
namespace {
    void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
        // roll X轴旋转
        double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        roll = atan2(sinr_cosp, cosr_cosp);

        // pitch Y轴旋转
        double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
        if (fabs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // 超过范围直接赋值90度
        else
            pitch = asin(sinp);

        // yaw Z轴旋转
        double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        yaw = atan2(siny_cosp, cosy_cosp);
    }
}

// IEZ9StatesProcessor 类实现
IEZ9StatesProcessor::IEZ9StatesProcessor(
    const std::string& dataset_file,
    int acc_scale_range,
    int gyro_scale_range,
    int used_IMUsensor)
    : dataset_file_name_(dataset_file)
    , acc_scale_range_(acc_scale_range)
    , gyro_scale_range_(gyro_scale_range)
    , used_IMUsensor_(used_IMUsensor)
    , file_length_(0)
    , gyro_bias_(Eigen::Vector3d::Zero())
    , acc_bias_(Eigen::Vector3d::Zero())
    , C_init_(Eigen::Matrix3d::Identity()) {
}

IEZ_ IEZ9StatesProcessor::process() {
    // 获取文件数据长度
    {
        io::LineReader in_line(dataset_file_name_);
        file_length_ = 0;
        while (char* line = in_line.next_line()) {
            file_length_++;
        }
        file_length_ -= 1; // 第一行不是数据，要去掉
        std::cout << "file length:" << file_length_ << "\n";
    }

    // 初始化矩阵
    initializeMatrices(file_length_);
    
    // 加载数据
    loadData(file_length_);
    
    // 计算初始值
    calculateInitialValues();
    
    // 检测静止状态
    detectStationary(file_length_);
    
    // 主循环处理
    processMainLoop(file_length_);
    
    // 计算结果
    return calculateResults(file_length_);
}

void IEZ9StatesProcessor::initializeMatrices(int file_length) {
    acc_s_ = Eigen::MatrixXd::Zero(file_length, 3);
    gyro_s_ = Eigen::MatrixXd::Zero(file_length, 3);
    acc_n_ = Eigen::MatrixXd::Zero(file_length, 3);
    vel_n_ = Eigen::MatrixXd::Zero(file_length, 3);
    pos_n_ = Eigen::MatrixXd::Zero(file_length, 3);
    stationary_.assign(file_length, false);
}

void IEZ9StatesProcessor::loadData(int file_length) {
    (void)file_length; // 抑制未使用参数警告
    // 计算缩放因子
    double acc_factor = 0.0;
    double gyro_factor = 0.0;

    switch (acc_scale_range_) {
        case 0: acc_factor = 16384.0; break;
        case 1: acc_factor = 8192.0; break;
        case 2: acc_factor = 4096.0; break;
        case 3: acc_factor = 2048.0; break;
        default: acc_factor = 2048.0; break;
    }

    switch (gyro_scale_range_) {
        case 0: gyro_factor = 131.0; break;
        case 1: gyro_factor = 65.5; break;
        case 2: gyro_factor = 32.8; break;
        case 3: gyro_factor = 16.4; break;
        default: gyro_factor = 16.4; break;
    }

    io::CSVReader<13> in(dataset_file_name_);
    in.read_header(io::ignore_extra_column, "timestamps", "ax1", "ay1", "az1", 
                   "gx1", "gy1", "gz1", "ax2", "ay2", "az2", "gx2", "gy2", "gz2");
    
    int timestamps, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2;

    // 双IMU，选择其中一个
    int num_4DataVec = 0;
    if (used_IMUsensor_ == 1) {
        while (in.read_row(timestamps, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2)) {
            acc_s_(num_4DataVec, 0) = (ax1 / acc_factor) * g_;
            acc_s_(num_4DataVec, 1) = (ay1 / acc_factor) * g_;
            acc_s_(num_4DataVec, 2) = (az1 / acc_factor) * g_;
            gyro_s_(num_4DataVec, 0) = (gx1 / gyro_factor) * (M_PI / 180.0);
            gyro_s_(num_4DataVec, 1) = (gy1 / gyro_factor) * (M_PI / 180.0);
            gyro_s_(num_4DataVec, 2) = (gz1 / gyro_factor) * (M_PI / 180.0);
            num_4DataVec++;
        }
    } else if (used_IMUsensor_ == 2) {
        while (in.read_row(timestamps, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2)) {
            acc_s_(num_4DataVec, 0) = (ax2 / acc_factor) * g_;
            acc_s_(num_4DataVec, 1) = (ay2 / acc_factor) * g_;
            acc_s_(num_4DataVec, 2) = (az2 / acc_factor) * g_;
            gyro_s_(num_4DataVec, 0) = (gx2 / gyro_factor) * (M_PI / 180.0);
            gyro_s_(num_4DataVec, 1) = (gy2 / gyro_factor) * (M_PI / 180.0);
            gyro_s_(num_4DataVec, 2) = (gz2 / gyro_factor) * (M_PI / 180.0);
            num_4DataVec++;
        }
    }
}

void IEZ9StatesProcessor::calculateInitialValues() {
    // 采样200个数据，求均值和初始
    Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();
    
    const int init_samples = 200;
    const int actual_samples = (init_samples < file_length_) ? init_samples : file_length_;
    
    for (int num_data = 0; num_data < actual_samples; num_data++) {
        acc_sum += acc_s_.row(num_data).transpose();
        gyro_sum += gyro_s_.row(num_data).transpose();
    }

    Eigen::Vector3d acc_mean = acc_sum / actual_samples;
    Eigen::Vector3d gyro_mean = gyro_sum / actual_samples;
    Eigen::Vector3d init_a = acc_mean;
    
    // 保存为成员变量
    gyro_bias_ = gyro_mean;
    acc_bias_ = Eigen::Vector3d::Zero(); // 初始加速度偏差为0

    for (int i = 0; i < 3; i++) {
        std::cout << "acc_mean:" << acc_mean(i) << "  gyro_mean:" << gyro_mean(i) 
                  << "  init_a:" << init_a(i) << "\n";
    }

    // 初始化导航坐标系
    double Pitch = -asin(init_a(0) / g_);
    double Roll = atan(init_a(1) / init_a(2));
    double Yaw = 0.0;

    // 欧拉角转旋转矩阵
    C_init_ << cos(Pitch)*cos(Yaw), sin(Roll)*sin(Pitch)*cos(Yaw)-cos(Roll)*sin(Yaw), 
               cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw),
               cos(Pitch)*sin(Yaw), sin(Roll)*sin(Pitch)*sin(Yaw) + cos(Roll)*cos(Yaw), 
               cos(Roll)*sin(Pitch)*sin(Yaw) - sin(Roll)*cos(Yaw),
               -sin(Pitch), sin(Roll)*cos(Pitch), cos(Roll)*cos(Pitch);

    // 预分配内存：导航坐标系加速度
    acc_n_.row(0) = (C_init_ * (acc_s_.row(0).transpose() - acc_bias_)).transpose();
}

void IEZ9StatesProcessor::detectStationary(int file_length) {
    // 计算加速度计和陀螺仪大小
    Eigen::VectorXd acc_mag = (acc_s_.array().square().rowwise().sum()).sqrt();
    Eigen::VectorXd gyro_mag = (gyro_s_.array().square().rowwise().sum()).sqrt();

    // 判断站立（静止）状态
    std::vector<bool> stationary_acc_H(file_length, false);
    std::vector<bool> stationary_acc_L(file_length, false);
    std::vector<bool> stationary_acc(file_length, false);
    std::vector<bool> stationary_gyro(file_length, false);

    for (int stat = 0; stat < file_length; stat++) {
        stationary_acc_H[stat] = (acc_mag(stat) < acc_stationary_threshold_H_);
        stationary_acc_L[stat] = (acc_mag(stat) > acc_stationary_threshold_L_);
        stationary_acc[stat] = (stationary_acc_H[stat] && stationary_acc_L[stat]);
        stationary_gyro[stat] = (gyro_mag(stat) < gyro_stationary_threshold_);
        stationary_[stat] = (stationary_acc[stat] && stationary_gyro[stat]);
    }

    // 消除假的站立（静止）状态 - 使用滑动窗口
    for (int k = 0; k < file_length - W_ + 1; k++) {
        if (stationary_[k] && stationary_[k + W_ - 1]) {
            for (int i = k; i < k + W_ && i < file_length; i++) {
                stationary_[i] = true;
            }
        }
    }

    for (int k = 0; k < file_length - W_ + 1; k++) {
        if (!stationary_[k] && !stationary_[k + W_ - 1]) {
            for (int i = k; i < k + W_ && i < file_length; i++) {
                stationary_[i] = false;
            }
        }
    }
}

void IEZ9StatesProcessor::processMainLoop(int file_length) {
    // 初始化矩阵
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.0001;
    
    Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
    H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
    
    // 使用保存的初始值
    Eigen::Matrix3d C = C_init_;
    Eigen::Matrix3d C_prev = C_init_;
    
    // 清空旋转历史并保存初始旋转矩阵
    rotation_history_.clear();
    rotation_history_.reserve(file_length);
    rotation_history_.push_back(C_init_); // 保存初始旋转

    // 误差协方差矩阵
    Eigen::Matrix<double, 9, 9> P = Eigen::Matrix<double, 9, 9>::Identity();

    // 算法主循环
    for (int t = 1; t < file_length; t++) {
        // INS惯导（变换，二重积分）
        Eigen::Vector3d gyro_s1 = gyro_s_.row(t).transpose() - gyro_bias_;

        // 角速率的斜对称矩阵
        Eigen::Matrix3d ang_rate_mat_P, ang_rate_mat_N;
        ang_rate_mat_P << 2.0, -gyro_s1(2)*dt_, gyro_s1(1)*dt_,
                          gyro_s1(2)*dt_, 2.0, -gyro_s1(0)*dt_,
                          -gyro_s1(1)*dt_, gyro_s1(0)*dt_, 2.0;
        ang_rate_mat_N << 2.0, gyro_s1(2)*dt_, -gyro_s1(1)*dt_,
                          -gyro_s1(2)*dt_, 2.0, gyro_s1(0)*dt_,
                          gyro_s1(1)*dt_, -gyro_s1(0)*dt_, 2.0;

        // 更新旋转估计
        C = C_prev * ang_rate_mat_P * ang_rate_mat_N.inverse();

        // 将加速度从传感器坐标系转换到导航坐标系
        Eigen::Vector3d acc_s_temp = acc_s_.row(t).transpose() - acc_bias_;
        acc_n_.row(t) = (0.5 * (C + C_prev) * acc_s_temp).transpose();

        // 第n帧加速度组成斜对称交叉积矩阵
        Eigen::Matrix3d S;
        S << 0, -acc_n_(t, 2), acc_n_(t, 1),
             acc_n_(t, 2), 0, -acc_n_(t, 0),
             -acc_n_(t, 1), acc_n_(t, 0), 0;

        Eigen::Vector3d acc_n_tP1 = acc_n_.row(t).transpose();
        Eigen::Vector3d acc_n_tM1 = acc_n_.row(t-1).transpose();
        Eigen::Vector3d g_vec(0.0, 0.0, g_);

        // 梯形积分计算速度和位置估计
        vel_n_.row(t) = vel_n_.row(t-1) + 
                       ((acc_n_tP1 - g_vec) + (acc_n_tM1 - g_vec)).transpose() * dt_ * 0.5;
        pos_n_.row(t) = pos_n_.row(t-1) + 
                       (vel_n_.row(t) + vel_n_.row(t-1)) * dt_ * 0.5;

        // 状态变化矩阵
        Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
        F.block<3, 3>(6, 0) = -dt_ * S;
        F.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt_;

        // 计算噪声方差Q
        Eigen::Matrix<double, 9, 9> tempQ = Eigen::Matrix<double, 9, 9>::Zero();
        tempQ.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * sigma_omega_;
        tempQ.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * sigma_a_;
        Eigen::Matrix<double, 9, 9> Q = (tempQ * dt_).array().square().matrix();

        // 协方差预测
        P = F * P * F.transpose() + Q;

        // 扩展卡尔曼滤波器 ---> 零速更新
        if (stationary_[t]) {
            // 计算卡尔曼增益
            Eigen::Matrix<double, 9, 3> K = P * H.transpose() * 
                                          (H * P * H.transpose() + R).inverse();

            // 计算误差状态
            Eigen::Vector3d tempVel_n = vel_n_.row(t).transpose();
            Eigen::Matrix<double, 9, 1> delta_x = K * tempVel_n;

            // 更新误差协方差矩阵
            Eigen::Matrix<double, 9, 9> iden_mat99 = Eigen::Matrix<double, 9, 9>::Identity();
            P = (iden_mat99 - K * H) * P;

            // 从KF状态获取误差
            Eigen::Vector3d attitude_error = delta_x.head<3>();
            Eigen::Vector3d pos_error = delta_x.segment<3>(3);
            Eigen::Vector3d vel_error = delta_x.tail<3>();

            // 纠正旋转的小角度斜对称矩阵
            Eigen::Matrix3d tempAng_matrix;
            tempAng_matrix << 0, -attitude_error(2), attitude_error(1),
                              attitude_error(2), 0, -attitude_error(0),
                              -attitude_error(1), attitude_error(0), 0;
            Eigen::Matrix3d ang_matrix = -tempAng_matrix;

            // 纠正旋转估计
            Eigen::Matrix3d iden_mat33 = Eigen::Matrix3d::Identity();
            C = (2 * iden_mat33 + ang_matrix) * 
                (2 * iden_mat33 - ang_matrix).inverse() * C;

            // 基于卡尔曼误差估计纠正位置和速度
            vel_n_.row(t) -= vel_error.transpose();
            pos_n_.row(t) -= pos_error.transpose();
        }

        // 保存旋转估计
        C_prev = C;
        
        // 保存旋转矩阵历史用于可视化
        rotation_history_.push_back(C);
    }
}

IEZ_ IEZ9StatesProcessor::calculateResults(int file_length) {
    IEZ_ result;
    
    // 估计发生旋转的地方（位置），可以用来画图可视化
    double angle = 0.0;
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << cos(angle), -sin(angle), sin(angle), cos(angle);

    Eigen::MatrixXd pos_r(2, file_length);
    for (int idx = 0; idx < file_length; idx++) {
        Eigen::Vector2d temp_Pos_n(pos_n_(idx, 0), pos_n_(idx, 1));
        pos_r.col(idx) = rotation_matrix * temp_Pos_n;
    }

    // 起点与终点的差距
    double diff_endup = (pos_r.col(0) - pos_r.col(file_length-1)).norm();
    result.diff_endup = diff_endup;

    // 计算总行驶距离
    double soma = 0.0;
    for (int t = 1; t < file_length; t++) {
        soma += (pos_r.col(t-1) - pos_r.col(t)).norm();
    }
    result.travelled_distance = soma;

    return result;
}

void IEZ9StatesProcessor::visualize() const {
    // 提取位置数据
    std::vector<double> x, y, z;
    std::vector<double> roll, pitch, yaw;
    std::vector<double> time;
    
    for (int t = 0; t < file_length_; t++) {
        x.push_back(pos_n_(t, 0));
        y.push_back(pos_n_(t, 1));
        z.push_back(pos_n_(t, 2));
        time.push_back(t * dt_);
        
        // 从旋转矩阵提取欧拉角
        if (t < static_cast<int>(rotation_history_.size())) {
            Eigen::Matrix3d C = rotation_history_[t];
            // 提取欧拉角 (ZYX顺序)
            double pitch_val = -asin(C(2, 0));
            double roll_val, yaw_val;
            if (cos(pitch_val) > 1e-6) {
                roll_val = atan2(C(2, 1), C(2, 2));
                yaw_val = atan2(C(1, 0), C(0, 0));
            } else {
                roll_val = atan2(-C(0, 1), C(1, 1));
                yaw_val = 0.0;
            }
            roll.push_back(roll_val * 180.0 / M_PI);
            pitch.push_back(pitch_val * 180.0 / M_PI);
            yaw.push_back(yaw_val * 180.0 / M_PI);
        } else {
            roll.push_back(0.0);
            pitch.push_back(0.0);
            yaw.push_back(0.0);
        }
    }
    
#ifdef USE_MATPLOT
    // 如果 Matplot++ 可用，尝试进行可视化
    try {
        using namespace matplot;
        
        // 使用 gnuplot 后端（默认后端）
        // 注意：需要 gnuplot 可执行文件在系统 PATH 中
        // 为了避免 multiplot 警告，使用单独的图形窗口而不是 subplot
        
        // 图形1: 3D轨迹
        {
            auto f1 = figure();
            plot3(x, y, z);
            title("3D轨迹");
            xlabel("X (m)");
            ylabel("Y (m)");
            zlabel("Z (m)");
            grid(on);
            // 保存图像而不是显示，避免 multiplot 问题
            save("trajectory_3d.png");
        }
        
        // 图形2: XY平面轨迹
        {
            auto f2 = figure();
            plot(x, y);
            title("XY平面轨迹");
            xlabel("X (m)");
            ylabel("Y (m)");
            grid(on);
            axis(equal);
            save("trajectory_xy.png");
        }
        
        // 图形3: 位置随时间变化
        {
            auto f3 = figure();
            // 使用矩阵方式绘制多条线，避免 hold 导致的 multiplot 问题
            std::vector<std::vector<double>> Y = {x, y, z};
            std::vector<std::string> labels = {"X", "Y", "Z"};
            plot(time, Y);
            title("位置随时间变化");
            xlabel("时间 (s)");
            ylabel("位置 (m)");
            legend(labels);
            grid(on);
            save("position_vs_time.png");
        }
        
        // 图形4: 欧拉角随时间变化
        {
            auto f4 = figure();
            // 使用矩阵方式绘制多条线，避免 hold 导致的 multiplot 问题
            std::vector<std::vector<double>> angles = {roll, pitch, yaw};
            std::vector<std::string> labels = {"Roll", "Pitch", "Yaw"};
            plot(time, angles);
            title("欧拉角随时间变化");
            xlabel("时间 (s)");
            ylabel("角度 (度)");
            legend(labels);
            grid(on);
            save("euler_angles.png");
        }
        
        std::cout << "\n可视化图像已保存:" << std::endl;
        std::cout << "  - trajectory_3d.png (3D轨迹)" << std::endl;
        std::cout << "  - trajectory_xy.png (XY平面轨迹)" << std::endl;
        std::cout << "  - position_vs_time.png (位置随时间变化)" << std::endl;
        std::cout << "  - euler_angles.png (欧拉角随时间变化)" << std::endl;
    } catch (const std::exception& e) {

        std::cout << "\n=== 轨迹可视化数据 ===" << std::endl;
        std::cout << "总数据点数: " << file_length_ << std::endl;
        std::cout << "\n前10个位置点:" << std::endl;
        std::cout << "时间(s)\tX(m)\tY(m)\tZ(m)\tRoll(度)\tPitch(度)\tYaw(度)" << std::endl;
        for (size_t i = 0; i < std::min(static_cast<size_t>(10), x.size()); i++) {
            std::cout << time[i] << "\t" << x[i] << "\t" << y[i] << "\t" << z[i] 
                      << "\t" << roll[i] << "\t" << pitch[i] << "\t" << yaw[i] << std::endl;
        }
    }
#else
    // 如果 Matplot++ 不可用，输出数据到控制台
    std::cout << "\n=== 轨迹可视化数据 ===" << std::endl;
    std::cout << "总数据点数: " << file_length_ << std::endl;
    std::cout << "\n前10个位置点:" << std::endl;
    std::cout << "时间(s)\tX(m)\tY(m)\tZ(m)\tRoll(度)\tPitch(度)\tYaw(度)" << std::endl;
    for (size_t i = 0; i < std::min(static_cast<size_t>(10), x.size()); i++) {
        std::cout << time[i] << "\t" << x[i] << "\t" << y[i] << "\t" << z[i] 
                  << "\t" << roll[i] << "\t" << pitch[i] << "\t" << yaw[i] << std::endl;
    }
    std::cout << "\n提示: 要启用图形可视化，请安装 Matplot++ 并在 CMakeLists.txt 中定义 USE_MATPLOT" << std::endl;
#endif
}

// StillDetectionProcessor 类实现
StillDetectionProcessor::StillDetectionProcessor(
    double acc_threshold_H,
    double acc_threshold_L,
    double gyro_threshold,
    int window_size)
    : acc_threshold_H_(acc_threshold_H)
    , acc_threshold_L_(acc_threshold_L)
    , gyro_threshold_(gyro_threshold)
    , window_size_(window_size) {
}

bool StillDetectionProcessor::detect(int data_length, 
                                     const Eigen::MatrixXd& acc_input, 
                                     const Eigen::MatrixXd& gyro_input) {
    Eigen::MatrixXd acc_s = acc_input;
    Eigen::MatrixXd gyro_s = gyro_input;

    // 计算加速度计和陀螺仪大小
    Eigen::VectorXd acc_mag = (acc_s.array().square().rowwise().sum()).sqrt();
    Eigen::VectorXd gyro_mag = (gyro_s.array().square().rowwise().sum()).sqrt();

    // 判断静止状态
    std::vector<bool> stationary(data_length, false);
    for (int stat = 0; stat < data_length; stat++) {
        bool stationary_acc = (acc_mag(stat) < acc_threshold_H_) && 
                              (acc_mag(stat) > acc_threshold_L_);
        bool stationary_gyro = (gyro_mag(stat) < gyro_threshold_);
        stationary[stat] = (stationary_acc && stationary_gyro);
    }

    // 消除假的站立（静止）状态
    for (int k = 0; k < data_length - window_size_ + 1; k++) {
        if (stationary[k] && stationary[k + window_size_ - 1]) {
            for (int i = k; i < k + window_size_ && i < data_length; i++) {
                stationary[i] = true;
            }
        }
    }

    for (int k = 0; k < data_length - window_size_ + 1; k++) {
        if (!stationary[k] && !stationary[k + window_size_ - 1]) {
            for (int i = k; i < k + window_size_ && i < data_length; i++) {
                stationary[i] = false;
            }
        }
    }

    // 判断决定是否是静止帧
    int still_Win = data_length;
    double station = stationary[0] ? 1.0 : 0.0;
    return (station / static_cast<double>(still_Win)) >= 0.99;
}

// 测试函数：从文件读取数据并检测静止
std::vector<bool> still_detection_test() {
    std::string acc_file_name = "../dataset/acc_data_2024-03-19_160030.csv";
    std::string gyro_file_name = "../dataset/gyro_data_2024-03-19_160030.csv";

    // 获取文件数据长度
    int data_length = 0;
    {
        io::LineReader acc_line(acc_file_name);
        while (char* line = acc_line.next_line()) {
            data_length++;
        }
        data_length -= 1; // 第一行不是数据，要去掉
    }
    std::cout << "acc length:" << data_length << "\n";

    // 读取原始数据
    Eigen::MatrixXd acc_s(data_length, 3);
    Eigen::MatrixXd gyro_s(data_length, 3);
    
    {
        io::CSVReader<3> ina(acc_file_name);
        ina.read_header(io::ignore_extra_column, "ax1", "ay1", "az1");
        double ax1, ay1, az1;
        int num_4DataVec = 0;
        while (ina.read_row(ax1, ay1, az1)) {
            acc_s(num_4DataVec, 0) = ax1;
            acc_s(num_4DataVec, 1) = ay1;
            acc_s(num_4DataVec, 2) = az1;
            num_4DataVec++;
        }
    }

    {
        io::CSVReader<3> ing(gyro_file_name);
        ing.read_header(io::ignore_extra_column, "gx1", "gy1", "gz1");
        double gx1, gy1, gz1;
        int num_4DataVec = 0;
        while (ing.read_row(gx1, gy1, gz1)) {
            gyro_s(num_4DataVec, 0) = gx1;
            gyro_s(num_4DataVec, 1) = gy1;
            gyro_s(num_4DataVec, 2) = gz1;
            num_4DataVec++;
        }
    }

    // 使用StillDetectionProcessor进行检测
    StillDetectionProcessor detector(9.8, 9.4, 0.6, 10);
    
    // 计算加速度计和陀螺仪大小
    Eigen::VectorXd acc_mag = (acc_s.array().square().rowwise().sum()).sqrt();
    Eigen::VectorXd gyro_mag = (gyro_s.array().square().rowwise().sum()).sqrt();

    std::vector<bool> stationary(data_length, false);
    for (int stat = 0; stat < data_length; stat++) {
        bool stationary_acc = (acc_mag(stat) < 9.8) && (acc_mag(stat) > 9.4);
        bool stationary_gyro = (gyro_mag(stat) < 0.6);
        stationary[stat] = (stationary_acc && stationary_gyro);
    }

    // 消除假的站立（静止）状态
    const int W = 10;
    for (int k = 0; k < data_length - W + 1; k++) {
        if (stationary[k] && stationary[k + W - 1]) {
            for (int i = k; i < k + W && i < data_length; i++) {
                stationary[i] = true;
            }
        }
    }

    for (int k = 0; k < data_length - W + 1; k++) {
        if (!stationary[k] && !stationary[k + W - 1]) {
            for (int i = k; i < k + W && i < data_length; i++) {
                stationary[i] = false;
            }
        }
    }

    // 判断决定是否是静止帧
    const int still_Win = 5;
    std::vector<bool> motion_test;
    double station = 0.0;
    int tt = 0;

    for (int k = 0; k < data_length - still_Win + 1; k += still_Win) {
        for (int j = k; j < k + still_Win && j < data_length; j++) {
            if (stationary[j]) {
                station++;
            }
        }
        motion_test.push_back((station / static_cast<double>(still_Win)) >= 0.99);
        tt++;
        station = 0;
    }

    return motion_test;
}
