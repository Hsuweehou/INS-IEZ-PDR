#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include "iez_pdr.h"
#include "Eigen/Eigen"
#include "Eigen/Dense"

// 前向声明
struct GLFWwindow;

class VisualizationWindow {
public:
    VisualizationWindow() : window_(nullptr), current_frame_(0), should_close_(false),
                            camera_distance_(10.0f), camera_rotation_x_(0.0f), camera_rotation_y_(0.0f),
                            camera_pan_x_(0.0f), camera_pan_y_(0.0f),
                            is_rotating_(false), is_panning_(false), last_mouse_x_(0.0f), last_mouse_y_(0.0f) {}
    ~VisualizationWindow() { shutdown(); }
    
    // 禁止拷贝
    VisualizationWindow(const VisualizationWindow&) = delete;
    VisualizationWindow& operator=(const VisualizationWindow&) = delete;
    
    // 初始化窗口
    bool initialize();
    
    // 渲染一帧
    void render(const IEZ_& result);
    
    // 关闭窗口
    void shutdown();
    
    // 检查是否应该关闭
    bool shouldClose() const;
    
private:
    GLFWwindow* window_;
    int current_frame_;
    bool should_close_;
    
    // 3D可视化参数
    float camera_distance_;
    float camera_rotation_x_;
    float camera_rotation_y_;
    float camera_pan_x_;  // 平移偏移X
    float camera_pan_y_;  // 平移偏移Y
    
    // 鼠标交互状态
    bool is_rotating_;      // 是否正在旋转（右键拖拽）
    bool is_panning_;       // 是否正在平移（左键拖拽）
    float last_mouse_x_;    // 上次鼠标X位置
    float last_mouse_y_;    // 上次鼠标Y位置
    
    // 辅助函数
    void showRotationMatrix(const Eigen::Matrix3d& R, int frame_idx);
    void showEulerAngles(double roll, double pitch, double yaw);
    void drawTrajectory2D(const Eigen::MatrixXd& positions);
    void drawTrajectory3D(const IEZ_& result);
    void drawStillDetection(const IEZ_& result);
    void loadChineseFont();
};

#endif // VISUALIZATION_H_
