#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <vector>
#include <memory>

// 矩阵工具类
class MatrixUtils {
public:

    MatrixUtils() = default;
    
    ~MatrixUtils() = default;
    
    // 禁止拷贝构造和赋值
    MatrixUtils(const MatrixUtils&) = delete;
    MatrixUtils& operator=(const MatrixUtils&) = delete;
    
    // 允许移动构造和赋值
    MatrixUtils(MatrixUtils&&) = default;
    MatrixUtils& operator=(MatrixUtils&&) = default;
    
    // 矩阵乘法
    static std::vector<std::vector<double>> multiply(
        const std::vector<std::vector<double>>& matrix1,
        const std::vector<std::vector<double>>& matrix2);
    
    // 矩阵求逆
    static std::vector<std::vector<double>> inverse(
        const std::vector<std::vector<double>>& matrix);

private:
    // 创建零矩阵
    static std::vector<std::vector<double>> makeZeroMatrix(int m, int n);
    
    // 计算行列式
    static double getDeterminant(const std::vector<std::vector<double>>& arcs, int n);
    
    // 计算伴随矩阵
    static void getAdjoint(const std::vector<std::vector<double>>& arcs, int n,
                          std::vector<std::vector<double>>& ans);
    
    // 矩阵求逆的内部实现
    static bool getMatrixInverse(const std::vector<std::vector<double>>& src, int n,
                                 std::vector<std::vector<double>>& des);
};

// 初始化数据函数（保持向后兼容）
void init_data();

// 向后兼容的函数接口
inline std::vector<std::vector<double>> mutil_mat(
    const std::vector<std::vector<double>>& matrix1,
    const std::vector<std::vector<double>>& matrix2) {
    return MatrixUtils::multiply(matrix1, matrix2);
}

inline std::vector<std::vector<double>> mat_inverse(
    const std::vector<std::vector<double>>& matrix_before) {
    return MatrixUtils::inverse(matrix_before);
}

#endif
