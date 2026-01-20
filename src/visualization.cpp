#ifdef _WIN32
#define NOMINMAX  // 禁用 Windows.h 中的 min/max 宏定义
#include <windows.h>
#include <shlwapi.h>
#pragma comment(lib, "shlwapi.lib")
#endif
#include "visualization.h"
#include <iostream>
#include <cmath>
#include <cstdio>
#include <algorithm>

// ImGui 和 GLFW 头文件
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>

// 辅助函数：将旋转矩阵转换为欧拉角（已从矩阵中提取，这里仅用于显示）
void VisualizationWindow::showRotationMatrix(const Eigen::Matrix3d& R, int frame_idx) {
    ImGui::Text("帧 %d 旋转矩阵:", frame_idx);
    ImGui::Text("[%.4f, %.4f, %.4f]", R(0,0), R(0,1), R(0,2));
    ImGui::Text("[%.4f, %.4f, %.4f]", R(1,0), R(1,1), R(1,2));
    ImGui::Text("[%.4f, %.4f, %.4f]", R(2,0), R(2,1), R(2,2));
}

void VisualizationWindow::showEulerAngles(double roll, double pitch, double yaw) {
    ImGui::Text("欧拉角 (度):");
    ImGui::Text("Roll (X):  %.2f°", roll * 180.0 / M_PI);
    ImGui::Text("Pitch (Y): %.2f°", pitch * 180.0 / M_PI);
    ImGui::Text("Yaw (Z):   %.2f°", yaw * 180.0 / M_PI);
}

void VisualizationWindow::drawTrajectory2D(const Eigen::MatrixXd& positions) {
    if (positions.rows() == 0) return;
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    
    if (canvas_size.x < 50.0f || canvas_size.y < 50.0f) return;
    
    // 计算轨迹的边界
    double min_x = positions.col(0).minCoeff();
    double max_x = positions.col(0).maxCoeff();
    double min_y = positions.col(1).minCoeff();
    double max_y = positions.col(1).maxCoeff();
    
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    
    if (range_x < 1e-6) range_x = 1.0;
    if (range_y < 1e-6) range_y = 1.0;
    
    // 添加边距
    float margin = 20.0f;
    float plot_width = canvas_size.x - 2 * margin;
    float plot_height = canvas_size.y - 2 * margin;
    
    // 绘制坐标轴
    ImVec2 origin(canvas_pos.x + margin, canvas_pos.y + canvas_size.y - margin);
    draw_list->AddLine(
        ImVec2(canvas_pos.x + margin, origin.y),
        ImVec2(canvas_pos.x + canvas_size.x - margin, origin.y),
        IM_COL32(200, 200, 200, 255), 1.0f
    );
    draw_list->AddLine(
        ImVec2(origin.x, canvas_pos.y + margin),
        ImVec2(origin.x, canvas_pos.y + canvas_size.y - margin),
        IM_COL32(200, 200, 200, 255), 1.0f
    );
    
    // 绘制轨迹
    if (positions.rows() > 1) {
        for (int i = 0; i < positions.rows() - 1; i++) {
            float x1 = origin.x + (positions(i, 0) - min_x) / range_x * plot_width;
            float y1 = origin.y - (positions(i, 1) - min_y) / range_y * plot_height;
            float x2 = origin.x + (positions(i+1, 0) - min_x) / range_x * plot_width;
            float y2 = origin.y - (positions(i+1, 1) - min_y) / range_y * plot_height;
            
            draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), IM_COL32(0, 255, 0, 255), 2.0f);
        }
    }
    
    // 绘制起点和终点
    if (positions.rows() > 0) {
        float start_x = origin.x + (positions(0, 0) - min_x) / range_x * plot_width;
        float start_y = origin.y - (positions(0, 1) - min_y) / range_y * plot_height;
        draw_list->AddCircleFilled(ImVec2(start_x, start_y), 5.0f, IM_COL32(0, 255, 0, 255));
        
        if (positions.rows() > 1) {
            float end_x = origin.x + (positions(positions.rows()-1, 0) - min_x) / range_x * plot_width;
            float end_y = origin.y - (positions(positions.rows()-1, 1) - min_y) / range_y * plot_height;
            draw_list->AddCircleFilled(ImVec2(end_x, end_y), 5.0f, IM_COL32(255, 0, 0, 255));
        }
    }
    
    // 绘制当前帧的位置点
    if (positions.rows() > 0 && current_frame_ >= 0 && current_frame_ < positions.rows()) {
        float current_x = origin.x + (positions(current_frame_, 0) - min_x) / range_x * plot_width;
        float current_y = origin.y - (positions(current_frame_, 1) - min_y) / range_y * plot_height;
        
        // 绘制一个较大的黄色圆点表示当前帧
        draw_list->AddCircleFilled(ImVec2(current_x, current_y), 8.0f, IM_COL32(255, 255, 0, 255));
        // 绘制外圈
        draw_list->AddCircle(ImVec2(current_x, current_y), 10.0f, IM_COL32(255, 255, 0, 255), 0, 2.0f);
    }
    
    ImGui::InvisibleButton("canvas", canvas_size);
}

void VisualizationWindow::drawTrajectory3D(const IEZ_& result) {
    if (result.position_history_.rows() == 0) return;
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    
    if (canvas_size.x < 50.0f || canvas_size.y < 50.0f) return;
    
    // 获取ImGui IO用于鼠标输入
    ImGuiIO& io = ImGui::GetIO();
    
    // 检测鼠标是否在画布区域内
    ImVec2 mouse_pos = io.MousePos;
    bool is_mouse_over_canvas = (mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
                                  mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y);
    
    // 处理鼠标滚轮缩放
    if (is_mouse_over_canvas && io.MouseWheel != 0.0f) {
        float zoom_speed = 0.1f;
        camera_distance_ *= (1.0f - io.MouseWheel * zoom_speed);
        // 限制距离范围
        if (camera_distance_ < 0.1f) camera_distance_ = 0.1f;
        if (camera_distance_ > 100.0f) camera_distance_ = 100.0f;
    }
    
    // 处理鼠标拖拽
    bool right_button_down = io.MouseDown[1];  // 右键
    bool left_button_down = io.MouseDown[0];   // 左键
    
    if (is_mouse_over_canvas) {
        // 右键拖拽：旋转视角
        if (right_button_down) {
            if (!is_rotating_) {
                // 开始旋转
                is_rotating_ = true;
                last_mouse_x_ = mouse_pos.x;
                last_mouse_y_ = mouse_pos.y;
            } else {
                // 继续旋转
                float delta_x = mouse_pos.x - last_mouse_x_;
                float delta_y = mouse_pos.y - last_mouse_y_;
                float rotation_speed = 0.5f;
                camera_rotation_y_ -= delta_x * rotation_speed;  // 左右旋转方向反转
                camera_rotation_x_ += delta_y * rotation_speed;
                
                // 限制旋转角度
                if (camera_rotation_x_ > 90.0f) camera_rotation_x_ = 90.0f;
                if (camera_rotation_x_ < -90.0f) camera_rotation_x_ = -90.0f;
                
                last_mouse_x_ = mouse_pos.x;
                last_mouse_y_ = mouse_pos.y;
            }
        } else {
            is_rotating_ = false;
        }
        
        // 左键拖拽：平移视图
        if (left_button_down) {
            if (!is_panning_) {
                // 开始平移
                is_panning_ = true;
                last_mouse_x_ = mouse_pos.x;
                last_mouse_y_ = mouse_pos.y;
            } else {
                // 继续平移
                float delta_x = mouse_pos.x - last_mouse_x_;
                float delta_y = mouse_pos.y - last_mouse_y_;
                float pan_speed = 0.01f;
                camera_pan_x_ += delta_x * pan_speed;
                camera_pan_y_ += delta_y * pan_speed;  // 上下移动方向修正
                
                last_mouse_x_ = mouse_pos.x;
                last_mouse_y_ = mouse_pos.y;
            }
        } else {
            is_panning_ = false;
        }
    } else {
        // 鼠标不在画布上，停止拖拽
        is_rotating_ = false;
        is_panning_ = false;
    }
    
    // 计算3D边界
    double min_x = result.position_history_.col(0).minCoeff();
    double max_x = result.position_history_.col(0).maxCoeff();
    double min_y = result.position_history_.col(1).minCoeff();
    double max_y = result.position_history_.col(1).maxCoeff();
    double min_z = result.position_history_.col(2).minCoeff();
    double max_z = result.position_history_.col(2).maxCoeff();
    
    double range_x = max_x - min_x;
    double range_y = max_y - min_y;
    double range_z = max_z - min_z;
    
    if (range_x < 1e-6) range_x = 1.0;
    if (range_y < 1e-6) range_y = 1.0;
    if (range_z < 1e-6) range_z = 1.0;
    
    // 计算中心点
    double center_x = (min_x + max_x) * 0.5;
    double center_y = (min_y + max_y) * 0.5;
    double center_z = (min_z + max_z) * 0.5;
    
    // 3D到2D投影函数
    auto project3D = [&](double x, double y, double z) -> ImVec2 {
        // 将坐标归一化到中心
        x -= center_x;
        y -= center_y;
        z -= center_z;
        
        // 应用旋转（绕X和Y轴）
        double rad_x = camera_rotation_x_ * M_PI / 180.0;
        double rad_y = camera_rotation_y_ * M_PI / 180.0;
        
        // 绕Y轴旋转
        double x1 = x * cos(rad_y) - z * sin(rad_y);
        double z1 = x * sin(rad_y) + z * cos(rad_y);
        
        // 绕X轴旋转
        double y1 = y * cos(rad_x) - z1 * sin(rad_x);
        double z2 = y * sin(rad_x) + z1 * cos(rad_x);
        
        // 应用距离缩放
        double scale = camera_distance_;
        x1 /= scale;
        y1 /= scale;
        z2 /= scale;
        
        // 投影到2D（正交投影）
        float margin = 20.0f;
        float plot_width = canvas_size.x - 2 * margin;
        float plot_height = canvas_size.y - 2 * margin;
        
        float screen_x = canvas_pos.x + margin + plot_width * 0.5f + 
                         (float)(x1 / range_x * plot_width * 0.8) + camera_pan_x_ * plot_width;
        float screen_y = canvas_pos.y + margin + plot_height * 0.5f - 
                         (float)(y1 / range_y * plot_height * 0.8) + camera_pan_y_ * plot_height;
        
        return ImVec2(screen_x, screen_y);
    };
    
    // 绘制坐标轴
    // 坐标轴的原点应该是轨迹的3D中心点投影后的位置
    ImVec2 origin = project3D(center_x, center_y, center_z);
    
    // 计算轴的长度（使用最大范围的一部分）
    double axis_length = (std::max)((std::max)(range_x, range_y), range_z) * 0.3;
    
    // X轴（红色）
    ImVec2 x_end = project3D(center_x + axis_length, center_y, center_z);
    draw_list->AddLine(origin, x_end, IM_COL32(255, 0, 0, 255), 2.0f);
    draw_list->AddText(ImVec2(x_end.x + 5, x_end.y), IM_COL32(255, 0, 0, 255), "X");
    
    // Y轴（绿色）
    ImVec2 y_end = project3D(center_x, center_y + axis_length, center_z);
    draw_list->AddLine(origin, y_end, IM_COL32(0, 255, 0, 255), 2.0f);
    draw_list->AddText(ImVec2(y_end.x + 5, y_end.y), IM_COL32(0, 255, 0, 255), "Y");
    
    // Z轴（蓝色）
    ImVec2 z_end = project3D(center_x, center_y, center_z + axis_length);
    draw_list->AddLine(origin, z_end, IM_COL32(0, 0, 255, 255), 2.0f);
    draw_list->AddText(ImVec2(z_end.x + 5, z_end.y), IM_COL32(0, 0, 255, 255), "Z");
    
    // 绘制轨迹
    if (result.position_history_.rows() > 1) {
        for (int i = 0; i < result.position_history_.rows() - 1; i++) {
            ImVec2 p1 = project3D(
                result.position_history_(i, 0),
                result.position_history_(i, 1),
                result.position_history_(i, 2)
            );
            ImVec2 p2 = project3D(
                result.position_history_(i + 1, 0),
                result.position_history_(i + 1, 1),
                result.position_history_(i + 1, 2)
            );
            
            // 根据帧索引改变颜色（从绿色到黄色到红色）
            float t = static_cast<float>(i) / static_cast<float>(result.position_history_.rows() - 1);
            ImU32 color = IM_COL32(
                static_cast<int>(t * 255),
                255 - static_cast<int>(t * 128),
                0,
                255
            );
            
            draw_list->AddLine(p1, p2, color, 2.0f);
        }
    }
    
    // 绘制起点（绿色）
    if (result.position_history_.rows() > 0) {
        ImVec2 start = project3D(
            result.position_history_(0, 0),
            result.position_history_(0, 1),
            result.position_history_(0, 2)
        );
        draw_list->AddCircleFilled(start, 6.0f, IM_COL32(0, 255, 0, 255));
    }
    
    // 绘制终点（红色）
    if (result.position_history_.rows() > 1) {
        int last_idx = result.position_history_.rows() - 1;
        ImVec2 end = project3D(
            result.position_history_(last_idx, 0),
            result.position_history_(last_idx, 1),
            result.position_history_(last_idx, 2)
        );
        draw_list->AddCircleFilled(end, 6.0f, IM_COL32(255, 0, 0, 255));
    }
    
    // 绘制当前帧的位置和姿态
    if (current_frame_ < result.position_history_.rows() && 
        current_frame_ < result.rotation_history_.rows()) {
        // 当前位置
        ImVec2 current_pos = project3D(
            result.position_history_(current_frame_, 0),
            result.position_history_(current_frame_, 1),
            result.position_history_(current_frame_, 2)
        );
        draw_list->AddCircleFilled(current_pos, 8.0f, IM_COL32(255, 255, 0, 255));
        draw_list->AddCircle(current_pos, 10.0f, IM_COL32(255, 255, 0, 255), 0, 2.0f);
        
        // 绘制当前姿态的坐标系（旋转矩阵的三个轴）
        Eigen::Matrix3d R;
        R << result.rotation_history_(current_frame_, 0), 
             result.rotation_history_(current_frame_, 1), 
             result.rotation_history_(current_frame_, 2),
             result.rotation_history_(current_frame_, 3), 
             result.rotation_history_(current_frame_, 4), 
             result.rotation_history_(current_frame_, 5),
             result.rotation_history_(current_frame_, 6), 
             result.rotation_history_(current_frame_, 7), 
             result.rotation_history_(current_frame_, 8);
        
        // 坐标系轴的长度（米）
        double axis_length = (std::max)((std::max)(range_x, range_y), range_z) * 0.1;
        
        // X轴（红色）
        Eigen::Vector3d x_axis = R.col(0) * axis_length;
        ImVec2 x_axis_end = project3D(
            result.position_history_(current_frame_, 0) + x_axis(0),
            result.position_history_(current_frame_, 1) + x_axis(1),
            result.position_history_(current_frame_, 2) + x_axis(2)
        );
        draw_list->AddLine(current_pos, x_axis_end, IM_COL32(255, 100, 100, 255), 3.0f);
        
        // Y轴（绿色）
        Eigen::Vector3d y_axis = R.col(1) * axis_length;
        ImVec2 y_axis_end = project3D(
            result.position_history_(current_frame_, 0) + y_axis(0),
            result.position_history_(current_frame_, 1) + y_axis(1),
            result.position_history_(current_frame_, 2) + y_axis(2)
        );
        draw_list->AddLine(current_pos, y_axis_end, IM_COL32(100, 255, 100, 255), 3.0f);
        
        // Z轴（蓝色）
        Eigen::Vector3d z_axis = R.col(2) * axis_length;
        ImVec2 z_axis_end = project3D(
            result.position_history_(current_frame_, 0) + z_axis(0),
            result.position_history_(current_frame_, 1) + z_axis(1),
            result.position_history_(current_frame_, 2) + z_axis(2)
        );
        draw_list->AddLine(current_pos, z_axis_end, IM_COL32(100, 100, 255, 255), 3.0f);
    }
    
    // 创建不可见按钮用于捕获鼠标事件
    ImGui::InvisibleButton("canvas3d", canvas_size);
    
    // 如果鼠标在画布上且正在拖拽，设置鼠标光标
    if (is_mouse_over_canvas) {
        if (is_rotating_) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
        } else if (is_panning_) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
        }
    }
}

bool VisualizationWindow::initialize() {
    // 初始化 GLFW
    if (!glfwInit()) {
        std::cerr << "无法初始化 GLFW" << std::endl;
        return false;
    }
    
    // 设置 OpenGL 版本
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
    
    // 创建窗口
    window_ = glfwCreateWindow(1280, 720, "旋转和平移可视化", nullptr, nullptr);
    if (!window_) {
        std::cerr << "无法创建 GLFW 窗口" << std::endl;
        glfwTerminate();
        return false;
    }
    
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // 启用垂直同步
    
    // 在 Windows 上，OpenGL 函数通过 glfwGetProcAddress 加载
    // 不需要 GLAD，直接使用系统 OpenGL
    
    // 设置 ImGui 上下文
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    
    // 设置 ImGui 样式
    ImGui::StyleColorsDark();
    
    // 加载支持中文的字体（在初始化渲染器之前）
    loadChineseFont();
    
    // 初始化 ImGui 平台/渲染器绑定
    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    
    // 创建字体纹理（在渲染器初始化后）
    ImGui_ImplOpenGL3_CreateFontsTexture();
    
    return true;
}

void VisualizationWindow::render(const IEZ_& result) {
    if (!window_) return;
    
    glfwPollEvents();
    
    // 开始 ImGui 帧
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    // 设置主窗口
    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
    ImGui::Begin("旋转和平移可视化", nullptr, 
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | 
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_MenuBar);
    
    // 菜单栏
    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("文件")) {
            if (ImGui::MenuItem("退出", "Esc")) {
                should_close_ = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }
    
    // 左侧面板：信息显示
    ImGui::BeginChild("LeftPanel", ImVec2(300, 0), true);
    
    ImGui::Text("计算结果");
    ImGui::Separator();
    ImGui::Text("起点与终点差距: %.4f m", result.diff_endup);
    ImGui::Text("总行驶距离: %.4f m", result.travelled_distance);
    
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();
    
    // 当前帧选择
    if (result.position_history_.rows() > 0) {
        int max_frame = static_cast<int>(result.position_history_.rows()) - 1;
        ImGui::Text("当前帧: %d / %d", current_frame_, max_frame);
        ImGui::SliderInt("帧索引", &current_frame_, 0, max_frame);
        
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        
        // 显示当前位置
        if (current_frame_ < result.position_history_.rows()) {
            ImGui::Text("位置 (m):");
            ImGui::Text("X: %.4f", result.position_history_(current_frame_, 0));
            ImGui::Text("Y: %.4f", result.position_history_(current_frame_, 1));
            ImGui::Text("Z: %.4f", result.position_history_(current_frame_, 2));
            
            ImGui::Spacing();
            
            // 显示当前旋转矩阵
            if (current_frame_ < result.rotation_history_.rows()) {
                Eigen::Matrix3d R;
                R << result.rotation_history_(current_frame_, 0), 
                     result.rotation_history_(current_frame_, 1), 
                     result.rotation_history_(current_frame_, 2),
                     result.rotation_history_(current_frame_, 3), 
                     result.rotation_history_(current_frame_, 4), 
                     result.rotation_history_(current_frame_, 5),
                     result.rotation_history_(current_frame_, 6), 
                     result.rotation_history_(current_frame_, 7), 
                     result.rotation_history_(current_frame_, 8);
                
                showRotationMatrix(R, current_frame_);
                
                ImGui::Spacing();
                
                // 显示欧拉角
                if (current_frame_ < result.euler_angles_.rows()) {
                    showEulerAngles(
                        result.euler_angles_(current_frame_, 0),
                        result.euler_angles_(current_frame_, 1),
                        result.euler_angles_(current_frame_, 2)
                    );
                }
            }
        }
    }
    
    ImGui::EndChild();
    
    ImGui::SameLine();
    
    // 右侧面板：标签页
    ImGui::BeginChild("RightPanel", ImVec2(0, 0), true);
    
    // 创建标签页
    if (ImGui::BeginTabBar("VisualizationTabs")) {
        // 2D 轨迹标签页
        if (ImGui::BeginTabItem("2D 轨迹")) {
            ImGui::Text("2D 轨迹 (XY平面)");
            ImGui::Separator();
            
            if (result.position_history_.rows() > 0) {
                drawTrajectory2D(result.position_history_);
            } else {
                ImGui::Text("无数据可显示");
            }
            
            ImGui::EndTabItem();
        }
        
        // 3D 可视化标签页
        if (ImGui::BeginTabItem("3D 可视化")) {
            ImGui::Text("3D 旋转和平移可视化");
            ImGui::Separator();
            
            // 3D视图控制
            ImGui::Text("视图控制:");
            ImGui::SliderFloat("距离", &camera_distance_, 1.0f, 50.0f);
            ImGui::SliderFloat("旋转 X", &camera_rotation_x_, -180.0f, 180.0f);
            ImGui::SliderFloat("旋转 Y", &camera_rotation_y_, -180.0f, 180.0f);
            
            if (ImGui::Button("重置视图")) {
                camera_distance_ = 10.0f;
                camera_rotation_x_ = 0.0f;
                camera_rotation_y_ = 0.0f;
                camera_pan_x_ = 0.0f;
                camera_pan_y_ = 0.0f;
            }
            
            ImGui::Text("鼠标控制:");
            ImGui::BulletText("右键拖拽: 旋转视角");
            ImGui::BulletText("左键拖拽: 平移视图");
            ImGui::BulletText("滚轮: 缩放");
            
            ImGui::Separator();
            
            if (result.position_history_.rows() > 0) {
                drawTrajectory3D(result);
            } else {
                ImGui::Text("无数据可显示");
            }
            
            ImGui::EndTabItem();
        }
        
        // 静止检测可视化标签页
        if (ImGui::BeginTabItem("静止检测")) {
            ImGui::Text("静止检测可视化");
            ImGui::Separator();
            
            if (result.stationary_.size() > 0) {
                drawStillDetection(result);
            } else {
                ImGui::Text("无数据可显示");
            }
            
            ImGui::EndTabItem();
        }
        
        ImGui::EndTabBar();
    }
    
    ImGui::EndChild();
    
    ImGui::End();
    
    // 渲染
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window_, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
    glfwSwapBuffers(window_);
}

void VisualizationWindow::shutdown() {
    if (window_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        
        glfwDestroyWindow(window_);
        glfwTerminate();
        window_ = nullptr;
    }
}

bool VisualizationWindow::shouldClose() const {
    return should_close_ || (window_ && glfwWindowShouldClose(window_));
}

void VisualizationWindow::loadChineseFont() {
    ImGuiIO& io = ImGui::GetIO();
    
    // 清除默认字体
    io.Fonts->Clear();
    
#ifdef _WIN32
    // Windows 系统字体路径
    const char* font_paths[] = {
        "C:/Windows/Fonts/msyh.ttc",      // 微软雅黑
        "C:/Windows/Fonts/simsun.ttc",    // 宋体
        "C:/Windows/Fonts/simhei.ttf",    // 黑体
        "C:/Windows/Fonts/simkai.ttf",    // 楷体
    };
    
    ImFont* font = nullptr;
    for (const char* path : font_paths) {
        if (PathFileExistsA(path)) {
            // 加载字体，大小为 16，包含中文字符范围
            font = io.Fonts->AddFontFromFileTTF(path, 16.0f, nullptr, 
                                                io.Fonts->GetGlyphRangesChineseFull());
            if (font) {
                std::cout << "成功加载字体: " << path << std::endl;
                break;
            }
        }
    }
    
    // 如果系统字体都加载失败，使用默认字体并添加中文字符范围
    if (!font) {
        font = io.Fonts->AddFontDefault();
        if (font) {
            // 尝试合并中文字符
            ImFontConfig config;
            config.MergeMode = true;
            config.GlyphMinAdvanceX = 13.0f;
            
            // 尝试加载任意一个可用的中文字体
            for (const char* path : font_paths) {
                if (PathFileExistsA(path)) {
                    io.Fonts->AddFontFromFileTTF(path, 16.0f, &config, 
                                                io.Fonts->GetGlyphRangesChineseFull());
                    break;
                }
            }
        }
        std::cout << "使用默认字体并尝试合并中文字符" << std::endl;
    }
#else
    // Linux/Mac 系统字体
    const char* font_paths[] = {
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/arphic/uming.ttc",
        "/System/Library/Fonts/PingFang.ttc",  // macOS
    };
    
    ImFont* font = nullptr;
    for (const char* path : font_paths) {
        FILE* f = fopen(path, "rb");
        if (f) {
            fclose(f);
            font = io.Fonts->AddFontFromFileTTF(path, 16.0f, nullptr, 
                                                io.Fonts->GetGlyphRangesChineseFull());
            if (font) {
                std::cout << "成功加载字体: " << path << std::endl;
                break;
            }
        }
    }
    
    if (!font) {
        font = io.Fonts->AddFontDefault();
        std::cout << "使用默认字体" << std::endl;
    }
#endif
    
    // 重建字体纹理
    io.Fonts->Build();
    
    // 注意：字体纹理的上传将在 initialize() 函数中，在 ImGui_ImplOpenGL3_Init 之后进行
}

void VisualizationWindow::drawStillDetection(const IEZ_& result) {
    if (result.stationary_.size() == 0) return;
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    
    if (canvas_size.x < 50.0f || canvas_size.y < 50.0f) return;
    
    int data_length = static_cast<int>(result.stationary_.size());
    
    // 计算统计信息
    int stationary_count = 0;
    for (bool stat : result.stationary_) {
        if (stat) stationary_count++;
    }
    float stationary_ratio = data_length > 0 ? (float)stationary_count / (float)data_length : 0.0f;
    
    // 显示统计信息
    ImGui::Text("总帧数: %d", data_length);
    ImGui::Text("静止帧数: %d (%.2f%%)", stationary_count, stationary_ratio * 100.0f);
    ImGui::Text("运动帧数: %d (%.2f%%)", data_length - stationary_count, (1.0f - stationary_ratio) * 100.0f);
    
    ImGui::Separator();
    ImGui::Spacing();
    
    // 绘制阈值信息
    ImGui::Text("检测阈值:");
    ImGui::BulletText("加速度阈值: 9.0 - 11.0 m/s²");
    ImGui::BulletText("陀螺仪阈值: < 0.6 rad/s");
    
    ImGui::Separator();
    ImGui::Spacing();
    
    // 绘制加速度和陀螺仪大小的时间序列图
    if (result.acc_magnitude_.size() > 0 && result.gyro_magnitude_.size() > 0) {
        float margin = 20.0f;
        float plot_width = canvas_size.x - 2 * margin;
        float plot_height = (canvas_size.y - 2 * margin) * 0.4f;  // 每个图占40%高度
        
        // 计算加速度范围
        double acc_min = result.acc_magnitude_.minCoeff();
        double acc_max = result.acc_magnitude_.maxCoeff();
        double acc_range = acc_max - acc_min;
        if (acc_range < 1e-6) acc_range = 1.0;
        
        // 计算陀螺仪范围
        double gyro_min = result.gyro_magnitude_.minCoeff();
        double gyro_max = result.gyro_magnitude_.maxCoeff();
        double gyro_range = gyro_max - gyro_min;
        if (gyro_range < 1e-6) gyro_range = 1.0;
        
        // 绘制加速度大小图
        ImGui::Text("加速度大小 (m/s²)");
        ImVec2 acc_plot_pos(canvas_pos.x + margin, ImGui::GetCursorScreenPos().y);
        ImVec2 acc_plot_size(plot_width, plot_height);
        
        // 绘制背景和阈值线
        draw_list->AddRectFilled(acc_plot_pos, 
                                ImVec2(acc_plot_pos.x + acc_plot_size.x, acc_plot_pos.y + acc_plot_size.y),
                                IM_COL32(20, 20, 20, 255));
        
        // 绘制阈值线
        float acc_threshold_H_y = acc_plot_pos.y + acc_plot_size.y - 
                                  (11.0 - acc_min) / acc_range * acc_plot_size.y;
        float acc_threshold_L_y = acc_plot_pos.y + acc_plot_size.y - 
                                  (9.0 - acc_min) / acc_range * acc_plot_size.y;
        draw_list->AddLine(ImVec2(acc_plot_pos.x, acc_threshold_H_y),
                          ImVec2(acc_plot_pos.x + acc_plot_size.x, acc_threshold_H_y),
                          IM_COL32(255, 255, 0, 128), 1.0f);
        draw_list->AddLine(ImVec2(acc_plot_pos.x, acc_threshold_L_y),
                          ImVec2(acc_plot_pos.x + acc_plot_size.x, acc_threshold_L_y),
                          IM_COL32(255, 255, 0, 128), 1.0f);
        
        // 绘制加速度曲线
        if (data_length > 1) {
            float x_scale = data_length > 1 ? 1.0f / (float)(data_length - 1) : 1.0f;
            for (int i = 0; i < data_length - 1; i++) {
                float x1 = acc_plot_pos.x + (float)i * x_scale * acc_plot_size.x;
                float y1 = acc_plot_pos.y + acc_plot_size.y - 
                          (result.acc_magnitude_(i) - acc_min) / acc_range * acc_plot_size.y;
                float x2 = acc_plot_pos.x + (float)(i + 1) * x_scale * acc_plot_size.x;
                float y2 = acc_plot_pos.y + acc_plot_size.y - 
                          (result.acc_magnitude_(i + 1) - acc_min) / acc_range * acc_plot_size.y;
                
                // 根据静止状态改变颜色
                ImU32 color = result.stationary_[i] ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
                draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), color, 2.0f);
            }
        }
        
        // 绘制当前帧标记
        if (current_frame_ >= 0 && current_frame_ < data_length && data_length > 1) {
            float x_scale = 1.0f / (float)(data_length - 1);
            float current_x = acc_plot_pos.x + (float)current_frame_ * x_scale * acc_plot_size.x;
            draw_list->AddLine(ImVec2(current_x, acc_plot_pos.y),
                              ImVec2(current_x, acc_plot_pos.y + acc_plot_size.y),
                              IM_COL32(255, 255, 0, 255), 2.0f);
        }
        
        ImGui::SetCursorScreenPos(ImVec2(canvas_pos.x, acc_plot_pos.y + acc_plot_size.y + 10));
        ImGui::Spacing();
        
        // 绘制陀螺仪大小图
        ImGui::Text("陀螺仪大小 (rad/s)");
        ImVec2 gyro_plot_pos(canvas_pos.x + margin, ImGui::GetCursorScreenPos().y);
        ImVec2 gyro_plot_size(plot_width, plot_height);
        
        // 绘制背景和阈值线
        draw_list->AddRectFilled(gyro_plot_pos, 
                                ImVec2(gyro_plot_pos.x + gyro_plot_size.x, gyro_plot_pos.y + gyro_plot_size.y),
                                IM_COL32(20, 20, 20, 255));
        
        // 绘制阈值线
        float gyro_threshold_y = gyro_plot_pos.y + gyro_plot_size.y - 
                                 (0.6 - gyro_min) / gyro_range * gyro_plot_size.y;
        draw_list->AddLine(ImVec2(gyro_plot_pos.x, gyro_threshold_y),
                          ImVec2(gyro_plot_pos.x + gyro_plot_size.x, gyro_threshold_y),
                          IM_COL32(255, 255, 0, 128), 1.0f);
        
        // 绘制陀螺仪曲线
        if (data_length > 1) {
            float x_scale = data_length > 1 ? 1.0f / (float)(data_length - 1) : 1.0f;
            for (int i = 0; i < data_length - 1; i++) {
                float x1 = gyro_plot_pos.x + (float)i * x_scale * gyro_plot_size.x;
                float y1 = gyro_plot_pos.y + gyro_plot_size.y - 
                          (result.gyro_magnitude_(i) - gyro_min) / gyro_range * gyro_plot_size.y;
                float x2 = gyro_plot_pos.x + (float)(i + 1) * x_scale * gyro_plot_size.x;
                float y2 = gyro_plot_pos.y + gyro_plot_size.y - 
                          (result.gyro_magnitude_(i + 1) - gyro_min) / gyro_range * gyro_plot_size.y;
                
                // 根据静止状态改变颜色
                ImU32 color = result.stationary_[i] ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
                draw_list->AddLine(ImVec2(x1, y1), ImVec2(x2, y2), color, 2.0f);
            }
        }
        
        // 绘制当前帧标记
        if (current_frame_ >= 0 && current_frame_ < data_length && data_length > 1) {
            float x_scale = 1.0f / (float)(data_length - 1);
            float current_x = gyro_plot_pos.x + (float)current_frame_ * x_scale * gyro_plot_size.x;
            draw_list->AddLine(ImVec2(current_x, gyro_plot_pos.y),
                              ImVec2(current_x, gyro_plot_pos.y + gyro_plot_size.y),
                              IM_COL32(255, 255, 0, 255), 2.0f);
        }
        
        ImGui::SetCursorScreenPos(ImVec2(canvas_pos.x, gyro_plot_pos.y + gyro_plot_size.y + 10));
        
        // 绘制静止状态时间线
        ImGui::Spacing();
        ImGui::Text("静止状态时间线");
        ImVec2 timeline_pos(canvas_pos.x + margin, ImGui::GetCursorScreenPos().y);
        ImVec2 timeline_size(plot_width, 30.0f);
        
        // 绘制背景
        draw_list->AddRectFilled(timeline_pos, 
                                ImVec2(timeline_pos.x + timeline_size.x, timeline_pos.y + timeline_size.y),
                                IM_COL32(40, 40, 40, 255));
        
        // 绘制静止状态条
        for (int i = 0; i < data_length; i++) {
            float x = timeline_pos.x + (float)i / (float)data_length * timeline_size.x;
            float width = timeline_size.x / (float)data_length;
            ImU32 color = result.stationary_[i] ? IM_COL32(0, 255, 0, 255) : IM_COL32(255, 0, 0, 255);
            draw_list->AddRectFilled(ImVec2(x, timeline_pos.y),
                                    ImVec2(x + width, timeline_pos.y + timeline_size.y),
                                    color);
        }
        
        // 绘制当前帧标记
        if (current_frame_ >= 0 && current_frame_ < data_length) {
            float current_x = timeline_pos.x + (float)current_frame_ / (float)data_length * timeline_size.x;
            draw_list->AddLine(ImVec2(current_x, timeline_pos.y),
                              ImVec2(current_x, timeline_pos.y + timeline_size.y),
                              IM_COL32(255, 255, 0, 255), 2.0f);
        }
        
        ImGui::SetCursorScreenPos(ImVec2(canvas_pos.x, timeline_pos.y + timeline_size.y + 10));
        
        // 图例
        ImGui::Spacing();
        ImGui::Text("图例:");
        ImVec2 legend_pos = ImGui::GetCursorScreenPos();
        draw_list->AddRectFilled(ImVec2(legend_pos.x, legend_pos.y),
                                ImVec2(legend_pos.x + 20, legend_pos.y + 15),
                                IM_COL32(0, 255, 0, 255));
        ImGui::SameLine();
        ImGui::Text("静止");
        ImGui::SameLine(100);
        draw_list->AddRectFilled(ImVec2(legend_pos.x + 100, legend_pos.y),
                                ImVec2(legend_pos.x + 120, legend_pos.y + 15),
                                IM_COL32(255, 0, 0, 255));
        ImGui::SameLine();
        ImGui::Text("运动");
        ImGui::SameLine(200);
        draw_list->AddLine(ImVec2(legend_pos.x + 200, legend_pos.y + 7),
                          ImVec2(legend_pos.x + 220, legend_pos.y + 7),
                          IM_COL32(255, 255, 0, 255), 2.0f);
        ImGui::SameLine();
        ImGui::Text("当前帧");
    }
    
    ImGui::InvisibleButton("still_detection_canvas", canvas_size);
}
