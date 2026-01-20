#include "./include/IEZ_9states_Vercpp.h"

//静态方法只在本文件使用，防止重名函数
static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);

//四元数转欧拉角
static void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    // roll X轴旋转
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch Y轴旋转
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // 超过范围直接赋值90度
    else
    pitch = asin(sinp);

    // yaw Z轴旋转
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

//原始算法
struct IEZ_ iez_9states(){

    std::string dataset_file_name = "../dataset/conf-3333-coleta04-02-06-21-5ds_03_test.csv";
    struct IEZ_ iez_result;
    //获取文件数据长度
    io::LineReader in_line(dataset_file_name);
    int file_length = 0;
    while(char*line = in_line.next_line()){
        file_length++;
    }
    file_length -= 1;//第一行不是数据，要去掉
    std::cout << "file length:" << file_length << "\n";
    io::CSVReader<13> in(dataset_file_name);
    in.read_header(io::ignore_extra_column, "timestamps", "ax1", "ay1", "az1", "gx1", "gy1", "gz1", "ax2", "ay2", "az2", "gx2", "gy2", "gz2");
    int timestamps; int ax1; int ay1; int az1; int gx1; int gy1; int gz1; int ax2; int ay2; int az2; int gx2; int gy2; int gz2;

    //std::vector<std::vector<int>> Data_Vec( file_length,std::vector<int>(13, 0));

    //Eigen矩阵输出格式
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    Eigen::IOFormat HeavyFmt(Eigen::FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");

    //prepare data
    double g = 9.81; //Gravity
    int acc_scale_range = 3;
    int gyro_scale_range = 3;
    int used_IMUsensor = 2;
    double acc_factor = 0.0;
    double gyro_factor = 0.0;

    Eigen::MatrixXd acc_s(file_length, 3);
    Eigen::MatrixXd gyro_s(file_length, 3);
    //三轴加速度
    Eigen::MatrixXd acc_n(file_length, 3);
    //三轴速度
    Eigen::MatrixXd vel_n(file_length, 3);
    //三轴坐标
    Eigen::MatrixXd pos_n(file_length, 3);
    //ZUPT 的噪声协方差矩阵
    Eigen::MatrixXd R(3, 3);
    R(0, 0) = 0.0001;R(1, 1) = 0.0001;R(2, 2) = 0.0001;
    //ZUPT 测量矩阵
    Eigen::MatrixXd H(3, 9);
    H << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    Eigen::MatrixXd K(3, 9);

    std::vector<bool> stationary_acc_H(file_length, 0);
    std::vector<bool> stationary_acc_L(file_length, 0);
    std::vector<bool> stationary_acc(file_length, 0);
    std::vector<bool> stationary_gyro(file_length, 0);
    std::vector<bool> stationary(file_length, 0);
    /*Eigen::MatrixXi stationary_acc_H(file_length, 1);
    Eigen::MatrixXi stationary_acc_L(file_length, 1);
    Eigen::MatrixXi stationary_acc(file_length, 1);
    Eigen::MatrixXi stationary_gyro(file_length, 1);
    Eigen::MatrixXi stationary(file_length, 1);*/

    //加速度计和陀螺仪大小
    Eigen::MatrixXd acc_mag(file_length, 1);
    Eigen::MatrixXd gyro_mag(file_length, 1);
    //行驶（运动）距离
    Eigen::MatrixXd distance(file_length, 1);

    Eigen::MatrixXd C(3, 3);
    Eigen::MatrixXd C_prev(3, 3);
    Eigen::MatrixXd ang_rate_mat_N(3, 3);
    Eigen::MatrixXd ang_rate_mat_P(3, 3);

    Eigen::MatrixXd g_array(1, 3);g_array(0, 2) = g;
    Eigen::MatrixXd acc_mean(3, 1);
    Eigen::MatrixXd gyro_mean(3, 1);
    Eigen::MatrixXd acc_bias(1, 3);
    acc_bias << 0.0, 0.0, 0.0;
    Eigen::MatrixXd gyro_bias(3, 1);
    Eigen::MatrixXd gyro_s1(3, 1);
    Eigen::MatrixXd init_a(3, 1);
    Eigen::MatrixXd heading(file_length, 1);
    Eigen::MatrixXd S(3, 3);
    Eigen::MatrixXd F(9, 9);
    Eigen::MatrixXd Q(9, 9);
    //误差协方差矩阵
    Eigen::MatrixXd P(9, 9);


    int W = 10;//窗口

    double Pitch = 0.0;
    double Roll = 0.0;
    double Yaw = 0.0;
    //定义噪声系数，陀螺仪和加速度计
    double sigma_omega = 0.01;
    double sigma_a = 0.1;
    //double sigma_v = 0.01;
    //double sigma_v2 = 0.0001;
    double acc_stationary_threshold_H = 11.0;
    double acc_stationary_threshold_L = 9.0;
    double gyro_stationary_threshold = 0.6;
    double dt = 0.01;


    if(acc_scale_range == 0)
        acc_factor = 16384.0;
    if(acc_scale_range == 1)
        acc_factor = 8192.0;
    if(acc_scale_range == 2)
        acc_factor = 4096.0;
    if(acc_scale_range == 3)
        acc_factor = 2048.0;

    if(gyro_scale_range == 0)
        gyro_factor = 131.0;
    if(gyro_scale_range == 1)
        gyro_factor = 65.5;
    if(gyro_scale_range == 2)
        gyro_factor = 32.8;
    if(gyro_scale_range == 3)
        gyro_factor = 16.4;

    //双IMU，选择其中一个
    if (used_IMUsensor == 1){
        int num_4DataVec = 0;
        while(in.read_row(timestamps, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2)){
            acc_s(num_4DataVec, 0) = (ax1 / acc_factor)*g;
            acc_s(num_4DataVec, 1) = (ay1 / acc_factor)*g;
            acc_s(num_4DataVec, 2) = (az1 / acc_factor)*g;
            gyro_s(num_4DataVec, 0) = (gx1 / gyro_factor)* (M_PI / 180.0);
            gyro_s(num_4DataVec, 1) = (gy1 / gyro_factor)* (M_PI / 180.0);
            gyro_s(num_4DataVec, 2) = (gz1 / gyro_factor)* (M_PI / 180.0);
            num_4DataVec++;
        }
        num_4DataVec = 0;
    }
    if (used_IMUsensor == 2){
        int num_4DataVec = 0;
        while(in.read_row(timestamps, ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2)){
            acc_s(num_4DataVec, 0) = (ax2 / acc_factor)*g;
            acc_s(num_4DataVec, 1) = (ay2 / acc_factor)*g;
            acc_s(num_4DataVec, 2) = (az2 / acc_factor)*g;
            gyro_s(num_4DataVec, 0) = (gx2 / gyro_factor)* (M_PI / 180.0);
            gyro_s(num_4DataVec, 1) = (gy2 / gyro_factor)* (M_PI / 180.0);
            gyro_s(num_4DataVec, 2) = (gz2 / gyro_factor)* (M_PI / 180.0);
            num_4DataVec++;
        }
        num_4DataVec = 0;
    }
    //采样200个数据，求均值和初始
    double acc_sum_[3] = {0.0, 0.0, 0.0};
    double gyro_sum_[3] = {0.0, 0.0, 0.0};
    for(int num_data = 0; num_data < 200; num_data++){
        acc_sum_[0] = acc_sum_[0] + acc_s(num_data, 0);
        acc_sum_[1] = acc_sum_[1] + acc_s(num_data, 1);
        acc_sum_[2] = acc_sum_[2] + acc_s(num_data, 2);
        gyro_sum_[0] = gyro_sum_[0] + gyro_s(num_data, 0);
        gyro_sum_[1] = gyro_sum_[1] + gyro_s(num_data, 1);
        gyro_sum_[2] = gyro_sum_[2] + gyro_s(num_data, 2);
    }

    for(int i = 0; i < 3; i++){
        acc_mean(i,0) = acc_sum_[i] / 200;
        init_a(i,0) = acc_sum_[i] / 200;
        gyro_mean(i,0) = gyro_sum_[i] / 200;
        std::cout << "acc_mean:" << acc_mean(i,0) << "  gyro_mean:" << gyro_mean(i,0) << "  init_a:" << init_a(i,0) << "\n";
    }

    gyro_bias(0,0) = gyro_mean(0,0);
    gyro_bias(1,0) = gyro_mean(1,0);
    gyro_bias(2,0) = gyro_mean(2,0);

    //初始化导航坐标系, 
    //传感器的坐标轴Z轴垂直于地面，导航坐标系的坐标轴Z轴则是垂直向前，上述坐标轴转换就直接rpy反过来就行了，也就是ypr
    //加速度计的旋转假设是稳定的
    //假设前进方向与X轴重合
    Pitch = -asin(init_a(0,0)/g);
    Roll = atan(init_a(1,0)/init_a(2,0));
    Yaw = 0.0;
    //yaw = -0.3 #it is possible to set your own yaw if you know it by any means
    //std::cout << Pitch << " " << Roll << " " << Yaw << "\n";

    //欧拉角转旋转矩阵
    C << cos(Pitch)*cos(Yaw), sin(Roll)*sin(Pitch)*cos(Yaw)-cos(Roll)*sin(Yaw), cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw),
        cos(Pitch)*sin(Yaw), sin(Roll)*sin(Pitch)*sin(Yaw) + cos(Roll)*cos(Yaw), cos(Roll)*sin(Pitch)*sin(Yaw) - sin(Roll)*cos(Yaw),
        -sin(Pitch), sin(Roll)*cos(Pitch), cos(Roll)*cos(Pitch);

    C_prev << cos(Pitch)*cos(Yaw), sin(Roll)*sin(Pitch)*cos(Yaw)-cos(Roll)*sin(Yaw), cos(Roll)*sin(Pitch)*cos(Yaw) + sin(Roll)*sin(Yaw),
        cos(Pitch)*sin(Yaw), sin(Roll)*sin(Pitch)*sin(Yaw) + cos(Roll)*cos(Yaw), cos(Roll)*sin(Pitch)*sin(Yaw) - sin(Roll)*cos(Yaw), 
        -sin(Pitch), sin(Roll)*cos(Pitch), cos(Roll)*cos(Pitch);

    //航向轴角度，初始化为0
    heading(0,0) = Yaw;
    //Preallocate预分配内存 storage for accelerations in navigation frame导航坐标系.
    acc_n(0,0) = C(0,0)*acc_s(0,0)+C(0,1)*acc_s(0,1)+C(0,2)*acc_s(0,2);
    acc_n(0,1) = C(1,0)*acc_s(0,0)+C(1,1)*acc_s(0,1)+C(1,2)*acc_s(0,2);
    acc_n(0,2) = C(2,0)*acc_s(0,0)+C(2,1)*acc_s(0,1)+C(2,2)*acc_s(0,2);

    /*  站立检测（静止检测）  */
    //计算加速度计和陀螺仪大小
    for(int num_mag = 0; num_mag < file_length; num_mag++){
        acc_mag(num_mag, 0) = sqrt(pow(acc_s(num_mag, 0),2) + pow(acc_s(num_mag, 1),2) + pow(acc_s(num_mag, 2),2));
        gyro_mag(num_mag, 0) = sqrt(pow(gyro_s(num_mag, 0),2) + pow(gyro_s(num_mag, 1),2) + pow(gyro_s(num_mag, 2),2));
    }
    //判断站立（静止）状态
    for(int stat = 0; stat < file_length; stat++){
        stationary_acc_H[stat] = (acc_mag(stat, 0) < acc_stationary_threshold_H);
        stationary_acc_L[stat] = (acc_mag(stat, 0) > acc_stationary_threshold_L);
        stationary_acc[stat] = (stationary_acc_H[stat] && stationary_acc_L[stat]);//C1
        stationary_gyro[stat] = (gyro_mag(stat, 0) < gyro_stationary_threshold);//C2
        stationary[stat] = (stationary_acc[stat] && stationary_gyro[stat]);
    }
    //消除假的站立（静止）状态 //W是窗口大小
    for(int k = 0; k < file_length-W+1; k++){
        if ((stationary[k] == true) && (stationary[k+W-1] == true))
            for(int i = k; i < k+W; i++)
                stationary[i] = 1;
    }

    for(int k = 0; k < file_length-W+1; k++){
        if((stationary[k] == false) && (stationary[k+W-1] == false))
            for(int i = k; i < k+W; i++)
                stationary[i] = 0;
    }
    /*  站立检测（静止检测）*/

    //准备数据
    Eigen::MatrixXd acc_s_tempVec_(3,1);
    //t-1
    Eigen::MatrixXd acc_n_tempVec_tM1(1,3);
    //t+1
    Eigen::MatrixXd acc_n_tempVec_tP1(1,3);
    //算法主循环
    for(int t=1; t < file_length; t++){
        //开始 INS惯导（变换，二重积分）
        //减去gyro的偏差
        gyro_s1(0, 0) = gyro_s(t, 0) - gyro_bias(0, 0);
        gyro_s1(1, 0) = gyro_s(t, 1) - gyro_bias(1, 0);
        gyro_s1(2, 0) = gyro_s(t, 2) - gyro_bias(2, 0);


        //角速率的斜对称矩阵顺便微分
        ang_rate_mat_P <<               2.0, -gyro_s1(2, 0)*dt,  gyro_s1(1, 0)*dt,
                           gyro_s1(2, 0)*dt,               2.0, -gyro_s1(0, 0)*dt,
                          -gyro_s1(1, 0)*dt,  gyro_s1(0, 0)*dt,               2.0;
        ang_rate_mat_N <<               2.0,  gyro_s1(2, 0)*dt, -gyro_s1(1, 0)*dt,
                          -gyro_s1(2, 0)*dt,               2.0,  gyro_s1(0, 0)*dt,
                           gyro_s1(1, 0)*dt, -gyro_s1(0, 0)*dt,               2.0;

        //C = mutil_mat(C_prev, ang_rate_mat_P);
        //C = mat_inverse(ang_rate_mat_N);
        //C = mutil_mat(mutil_mat(C_prev, ang_rate_mat_P), mat_inverse(ang_rate_mat_N));

        //更新旋转估计
        //角速率的斜对称矩阵的微分（旋转矩阵R(t)的微分可以用角速度的斜对称矩阵）
        C = C_prev*ang_rate_mat_P*(ang_rate_mat_N.inverse());

        //将加速度从传感器坐标系转换到导航坐标系.
        acc_s_tempVec_(0, 0) = acc_s(t, 0) - acc_bias(0, 0);
        acc_s_tempVec_(1, 0) = acc_s(t, 1) - acc_bias(0, 1);
        acc_s_tempVec_(2, 0) = acc_s(t, 2) - acc_bias(0, 2);

        acc_n(t, 0) = (0.5*(C + C_prev)*acc_s_tempVec_)(0,0);
        acc_n(t, 1) = (0.5*(C + C_prev)*acc_s_tempVec_)(1,0);
        acc_n(t, 2) = (0.5*(C + C_prev)*acc_s_tempVec_)(2,0);

        //第n帧加速度组成斜对称交叉积矩阵
        S <<           0, -acc_n(t, 2),   acc_n(t, 1),
             acc_n(t, 2),            0,  -acc_n(t, 0),
            -acc_n(t, 1),  acc_n(t, 0),             0;

        acc_n_tempVec_tP1 << acc_n(t, 0), acc_n(t, 1), acc_n(t, 2);
        acc_n_tempVec_tM1 << acc_n(t-1, 0), acc_n(t-1, 1), acc_n(t-1, 2);

        //梯形积分计算速度和位置估计
        vel_n(t, 0) = vel_n(t-1, 0) + (((acc_n_tempVec_tP1 - g_array)+(acc_n_tempVec_tM1 - g_array))*dt*0.5)(0, 0);
        vel_n(t, 1) = vel_n(t-1, 1) + (((acc_n_tempVec_tP1 - g_array)+(acc_n_tempVec_tM1 - g_array))*dt*0.5)(0, 1);
        vel_n(t, 2) = vel_n(t-1, 2) + (((acc_n_tempVec_tP1 - g_array)+(acc_n_tempVec_tM1 - g_array))*dt*0.5)(0, 2);

        pos_n(t, 0) = (((pos_n(t-1, 0)) + (vel_n(t, 0) + vel_n(t-1,0))*dt*0.5));
        pos_n(t, 1) = (((pos_n(t-1, 1)) + (vel_n(t, 1) + vel_n(t-1,1))*dt*0.5));
        pos_n(t, 2) = (((pos_n(t-1, 2)) + (vel_n(t, 2) + vel_n(t-1,2))*dt*0.5));

        //状态变化矩阵（基础矩阵）
        F(0,0) = 1.0;F(1,1) = 1.0;F(2,2) = 1.0;F(3,3) = 1.0;
        F(4,4) = 1.0;F(5,5) = 1.0;F(6,6) = 1.0;F(7,7) = 1.0;F(8,8) = 1.0;
        F(6,1) = (-dt*S)(0,1);F(6,2) = (-dt*S)(0,2);F(7,0) = (-dt*S)(1,0);
        F(7,2) = (-dt*S)(1,2);F(8,0) = (-dt*S)(2,0);F(8,1) = (-dt*S)(2,1);
        F(3,6) = dt;F(4,7) = dt;F(5,8) = dt;


        Eigen::MatrixXd tempQ(9, 9);
        tempQ <<    sigma_omega, 0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0,
                    0.0, sigma_omega , 0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0 , 0.0,
                    0.0, 0.0 , sigma_omega, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0,
                    0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0,
                    0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0, 0.0,
                    0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , sigma_a, 0.0, 0.0,
                    0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , 0.0, sigma_a, 0.0,
                    0.0, 0.0 ,0.0, 0.0 , 0.0, 0.0 , 0.0, 0.0, sigma_a;

        //计算噪声方差Q
        Q = (tempQ*dt).array().square();
        //协方差预测
        //误差协方差矩阵正向传播
        P = F * P * F.transpose() + Q;

        //INS 惯性导航结束

        //扩展卡尔曼滤波器 ---> 零速更新
        Eigen::MatrixXd tempVel_n(3, 1);
        Eigen::MatrixXd delta_x(9, 1);
        if(stationary[t]){
            //卡尔曼滤波
            //计算卡尔曼增益
            K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

            //计算误差状态
            //更新滤波器状态
            tempVel_n << vel_n(t, 0), vel_n(t, 1), vel_n(t, 2);
            delta_x = K * tempVel_n;

            //单位矩阵
            Eigen::MatrixXd iden_mat99(9, 9);
            iden_mat99.setIdentity(9,9);

            //更新误差协方差矩阵
            P = (iden_mat99 - K * H) * P;

            //从KF状态总获取误差
            Eigen::MatrixXd attitude_error(3, 1);
            attitude_error << delta_x(0, 0), delta_x(1, 0), delta_x(2, 0);
            Eigen::MatrixXd pos_error(3, 1);
            pos_error << delta_x(3, 0), delta_x(4, 0), delta_x(5, 0);
            Eigen::MatrixXd vel_error(3, 1);
            vel_error << delta_x(6, 0), delta_x(7, 0), delta_x(8, 0);
            //卡尔曼滤波器零速更新

            
            //纠正惯导估计
            //# Skew-symmetric matrix for small angles to correct orientation.
            //纠正旋转的小角度斜对称矩阵
            Eigen::MatrixXd ang_matrix(3, 3);
            Eigen::MatrixXd all_zero(3, 3);
            Eigen::MatrixXd tempAng_matrix(3, 3);
            all_zero.setZero(3, 3);
            tempAng_matrix << 0, -attitude_error(2, 0), attitude_error(1, 0),
                          attitude_error(2, 0), 0, -attitude_error(0, 0),
                          -attitude_error(1, 0), attitude_error(0, 0), 0;
            ang_matrix = all_zero - tempAng_matrix;
            
            //纠正旋转估计
            Eigen::MatrixXd iden_mat33(3, 3);iden_mat33.setIdentity(3, 3);
            C = (2 * iden_mat33 + ang_matrix)*((2 * iden_mat33 - ang_matrix).inverse())*C;

            //基于卡尔曼误差估计纠正位置和速度
            vel_n(t, 0) = vel_n(t, 0) - vel_error(0, 0);
            vel_n(t, 1) = vel_n(t, 1) - vel_error(1, 0);
            vel_n(t, 2) = vel_n(t, 2) - vel_error(2, 0);
            pos_n(t, 0) = pos_n(t, 0) - pos_error(0, 0);
            pos_n(t, 1) = pos_n(t, 1) - pos_error(1, 0);
            pos_n(t, 2) = pos_n(t, 2) - pos_error(2, 0);

        }
        //估计并保存传感器的yaw（这里没有用到）
        //# Estimate and save the yaw of the sensor (different from the direction of travel). Unused here but potentially useful for orienting a GUI correctly.
        heading(t, 0) = atan2(C(1, 0), C(0, 0));

        //保存旋转估计，在循环开始的时候需要
        C_prev = C;

        distance(t, 0) = distance(t-1,0) + sqrt(pow(pos_n(t, 0) - pos_n(t-1, 0),2) + pow(pos_n(t, 1) - pos_n(t-1, 1),2));

        //refresh matrice
        /*acc_n_tempVec_tP1 << 0.0, 0.0, 0.0;
        acc_n_tempVec_tM1 << 0.0, 0.0, 0.0;
        acc_s_tempVec_ << 0.0, 0.0, 0.0;*/

    }

    //估计发生旋转的地方（位置），可以用来画图可视化
    double angle = 0.0;// np.deg2rad(0)

    Eigen::MatrixXd rotation_matrix(2, 2);
    rotation_matrix << cos(angle), -sin(angle), sin(angle), cos(angle);

    Eigen::MatrixXd pos_r(2, file_length);
    //pos_r.setZero(2,file_length);


    //pos_r(0, 0),pos_r(1, 0)起始位置
    //中间的值都是
    //pos_r(0, file_length),pos_r(1, file_length)终点位置
    Eigen::MatrixXd temp_Pos_n(2, 1);
    for(int idx = 0; idx < file_length; idx++){
        temp_Pos_n << pos_n(idx, 0), pos_n(idx, 1);
        pos_r(0, idx) = (rotation_matrix * temp_Pos_n)(0, 0);
        pos_r(1, idx) = (rotation_matrix * temp_Pos_n)(1, 0);
    }

    //起点与终点的差距
    //c
    double diff_endup = 0.0;

    diff_endup = sqrt(pow(pos_r(0, 0) - pos_r(0, file_length-1), 2) + pow(pos_r(1, 0) - pos_r(1, file_length-1), 2));
    //std::cout << "Difference between the initial and final position = " << diff_endup<< "\n";
    iez_result.diff_endup = diff_endup;

    double soma = 0.0;
    Eigen::MatrixXd diff_endup_array(1, file_length);

    for(int t = 1; t < file_length; t++){
        diff_endup_array(0, t) = sqrt(pow(pos_r(0, t-1) - pos_r(0, t), 2) + pow(pos_r(1, t-1) - pos_r(1, t), 2));
        soma += diff_endup_array(0, t);

    }
    //std::cout << "stimated total travelled distance = " << soma << "\n";
    iez_result.travelled_distance = soma;

    return iez_result;
}

/*
1.取数据（采样）：
	--初始化角度直接使用QVR数据（初始采样200帧IMU）
	--2帧图像共22帧IMU数据用于分析
	--gyro_bias读标定文件，路径：/persist/sensors/registry/registry/icm4x6xx_0_platform.gyro.fac_cal.bias
	--acc_bias读标定文件，路径：/persist/sensors/registry/registry/icm4x6xx_0_platform.accel.fac_cal.bias
	
2.算法过程：
	--检测静止，采样静止前的一段轨迹（固定采样窗口or动态采样窗口）
		--判断静止的阈值：1、threhold_acc_high\threhold_acc_low、threhold_gyro
	--检测运动，纠正旋转估计
*/

//检测静止-测试：输入5个IMU数据
//bool still_detection(int data_length, Eigen::MatrixXd acc_input, Eigen::MatrixXd gyro_input){
std::vector<bool> still_detection_test(){

    /*1、读取qvr前200帧数据，求取均值，作为IMU的加速度计和陀螺仪的偏差；*/

    std::string acc_file_name = "../dataset/acc_data_2024-03-19_160030.csv";
    std::string gyro_file_name = "../dataset/gyro_data_2024-03-19_160030.csv";

    //获取文件数据长度
    io::LineReader acc_line(acc_file_name);
    int data_length = 0;
    while(char*line = acc_line.next_line()){
        data_length++;
    }
    data_length -= 1;//第一行不是数据，要去掉
    std::cout << "acc length:" << data_length << "\n";
    io::CSVReader<3> ina(acc_file_name);
    ina.read_header(io::ignore_extra_column, "ax1", "ay1", "az1");
    double ax1; double ay1; double az1;

    //获取文件数据长度
    io::LineReader gyro_line(gyro_file_name);
    std::cout << "gyro length:" << data_length << "\n";
    io::CSVReader<3> ing(gyro_file_name);
    ing.read_header(io::ignore_extra_column, "gx1", "gy1", "gz1");
    double gx1; double gy1; double gz1;

    //读取原始数据
    Eigen::MatrixXd acc_s(data_length, 3);
    Eigen::MatrixXd gyro_s(data_length, 3);

    int num_4DataVec = 0;
    while(ina.read_row(ax1, ay1, az1)){
        acc_s(num_4DataVec, 0) = ax1;
        acc_s(num_4DataVec, 1) = ay1;
        acc_s(num_4DataVec, 2) = az1;
        num_4DataVec++;
    }
    num_4DataVec = 0;
    while(ing.read_row(gx1, gy1, gz1)){
        gyro_s(num_4DataVec, 0) = gx1;
        gyro_s(num_4DataVec, 1) = gy1;
        gyro_s(num_4DataVec, 2) = gz1;
        num_4DataVec++;
    }

    /*2、设计加速度计和陀螺仪的静止阈值；*/

    //加速度计和陀螺仪大小
    Eigen::MatrixXd acc_mag(data_length, 1);
    Eigen::MatrixXd gyro_mag(data_length, 1);
    int W = 10;//滑动窗口大小

    //
    double acc_stationary_threshold_H = 9.8;//9.8
    double acc_stationary_threshold_L = 9.4;
    double gyro_stationary_threshold = 0.6;

    std::vector<bool> stationary_acc_H(data_length, 0);
    std::vector<bool> stationary_acc_L(data_length, 0);
    std::vector<bool> stationary_acc(data_length, 0);
    std::vector<bool> stationary_gyro(data_length, 0);
    std::vector<bool> stationary(data_length, 0);

    /*3、静止检测，返回是否运动；*/
    for(int num_mag = 0; num_mag < data_length; num_mag++){
        acc_mag(num_mag, 0) = sqrt(pow(acc_s(num_mag, 0),2) + pow(acc_s(num_mag, 1),2) + pow(acc_s(num_mag, 2),2));
        gyro_mag(num_mag, 0) = sqrt(pow(gyro_s(num_mag, 0),2) + pow(gyro_s(num_mag, 1),2) + pow(gyro_s(num_mag, 2),2));
    }
    //判断站立（静止）状态
    for(int stat = 0; stat < data_length; stat++){
        stationary_acc_H[stat] = (acc_mag(stat, 0) < acc_stationary_threshold_H);
        stationary_acc_L[stat] = (acc_mag(stat, 0) > acc_stationary_threshold_L);
        stationary_acc[stat] = (stationary_acc_H[stat] && stationary_acc_L[stat]);//C1
        stationary_gyro[stat] = (gyro_mag(stat, 0) < gyro_stationary_threshold);//C2
        stationary[stat] = (stationary_acc[stat] && stationary_gyro[stat]);
    }
    //消除假的站立（静止）状态 //W是窗口大小
    for(int k = 0; k < data_length-W+1; k++){
        if ((stationary[k] == true) && (stationary[k+W-1] == true))
            for(int i = k; i < k+W; i++)
                stationary[i] = 1;
    }

    for(int k = 0; k < data_length-W+1; k++){
        if((stationary[k] == false) && (stationary[k+W-1] == false))
            for(int i = k; i < k+W; i++)
                stationary[i] = 0;
    }
    
    //判断决定是否是静止帧//still_Win是窗口大小
    int still_Win = 5;
    double station = 0.0;//静止的IMU帧
    int tt = 0;
    std::vector<bool> motion_test(data_length, 0);

    //整理轨迹中的静止帧
    for(int k = 0; k < data_length-still_Win+1; k+=still_Win){
        for(int j = k; j < k+still_Win; j++)
            if (stationary[k] == true)
                station++;
        if ((station / double(still_Win)) >= 0.99 ){
            motion_test[tt] = true;
        }
        else
            motion_test[tt] = false;
        tt++;
        station = 0;
    }

    return motion_test;
}

//检测静止：输入5个IMU数据
bool still_detection(int data_length, Eigen::MatrixXd acc_input, Eigen::MatrixXd gyro_input){

    /*1、读取qvr前200帧数据，求取均值，作为IMU的加速度计和陀螺仪的偏差；*/

    //读取原始数据
    Eigen::MatrixXd acc_s(data_length, 3);
    Eigen::MatrixXd gyro_s(data_length, 3);

    int num_4DataVec = 0;
    while(num_4DataVec < data_length){
        acc_s(num_4DataVec, 0) = acc_input(num_4DataVec, 0);
        acc_s(num_4DataVec, 1) = acc_input(num_4DataVec, 1);
        acc_s(num_4DataVec, 2) = acc_input(num_4DataVec, 2);
        num_4DataVec++;
    }
    num_4DataVec = 0;
    while(num_4DataVec < data_length){
        gyro_s(num_4DataVec, 0) = gyro_input(num_4DataVec, 0);
        gyro_s(num_4DataVec, 1) = gyro_input(num_4DataVec, 1);
        gyro_s(num_4DataVec, 2) = gyro_input(num_4DataVec, 2);
        num_4DataVec++;
    }

    /*2、设计加速度计和陀螺仪的静止阈值；*/

    //加速度计和陀螺仪大小
    Eigen::MatrixXd acc_mag(data_length, 1);
    Eigen::MatrixXd gyro_mag(data_length, 1);
    int W = 1;//滑动窗口大小//10

    //
    double acc_stationary_threshold_H = 9.8;//9.8
    double acc_stationary_threshold_L = 9.4;
    double gyro_stationary_threshold = 0.6;

    std::vector<bool> stationary_acc_H(data_length, 0);
    std::vector<bool> stationary_acc_L(data_length, 0);
    std::vector<bool> stationary_acc(data_length, 0);
    std::vector<bool> stationary_gyro(data_length, 0);
    std::vector<bool> stationary(data_length, 0);

    /*3、静止检测，返回是否运动；*/
    for(int num_mag = 0; num_mag < data_length; num_mag++){
        acc_mag(num_mag, 0) = sqrt(pow(acc_s(num_mag, 0),2) + pow(acc_s(num_mag, 1),2) + pow(acc_s(num_mag, 2),2));
        gyro_mag(num_mag, 0) = sqrt(pow(gyro_s(num_mag, 0),2) + pow(gyro_s(num_mag, 1),2) + pow(gyro_s(num_mag, 2),2));
    }
    //判断站立（静止）状态
    for(int stat = 0; stat < data_length; stat++){
        stationary_acc_H[stat] = (acc_mag(stat, 0) < acc_stationary_threshold_H);
        stationary_acc_L[stat] = (acc_mag(stat, 0) > acc_stationary_threshold_L);
        stationary_acc[stat] = (stationary_acc_H[stat] && stationary_acc_L[stat]);//C1
        stationary_gyro[stat] = (gyro_mag(stat, 0) < gyro_stationary_threshold);//C2
        stationary[stat] = (stationary_acc[stat] && stationary_gyro[stat]);
    }
    for(int i = 0; i < stationary.size(); i++)
        std::cout << stationary[i] << "\n";
    //消除假的站立（静止）状态 //W是窗口大小
    for(int k = 0; k < data_length-W+1; k++){
        if ((stationary[k] == true) && (stationary[k+W-1] == true))
            for(int i = k; i < k+W; i++)
                stationary[i] = 1;
    }

    for(int k = 0; k < data_length-W+1; k++){
        if((stationary[k] == false) && (stationary[k+W-1] == false))
            for(int i = k; i < k+W; i++)
                stationary[i] = 0;
    }

    //判断决定是否是静止帧 //still_Win是窗口大小
    int still_Win = data_length;
    double station = stationary[0];//静止的IMU帧
    bool motion_test;
    if ((station / double(still_Win)) >= 0.99 ){
        motion_test = true;
    }
    else
        motion_test = false;

    return motion_test;
}