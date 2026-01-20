// 防止 Windows 头文件中的宏定义冲突
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <iostream>
#include <vector>
#include <stdexcept>
#include "utils.h"

// 初始化数据函数
void init_data() {
    constexpr double g = 9.81; // Gravity
    std::cout << g << "\n";
}

// MatrixUtils 类
std::vector<std::vector<double>> MatrixUtils::makeZeroMatrix(int m, int n) {

    std::vector<std::vector<double>> array(m, std::vector<double>(n, 0.0));
    return array;
}

std::vector<std::vector<double>> MatrixUtils::multiply(
    const std::vector<std::vector<double>>& matrix1,
    const std::vector<std::vector<double>>& matrix2) {
    // 输入验证
    if (matrix1.empty() || matrix2.empty() || matrix1[0].empty() || matrix2[0].empty()) {
        throw std::invalid_argument("矩阵不能为空");
    }
    
    int m = static_cast<int>(matrix1.size());
    int n = static_cast<int>(matrix1[0].size());
    int p = static_cast<int>(matrix2[0].size());
    
    if (static_cast<int>(matrix2.size()) != n) {
        throw std::invalid_argument("矩阵维度不匹配，无法相乘");
    }

    std::vector<std::vector<double>> result(m, std::vector<double>(p, 0.0));
    
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            double sum = 0.0;
            for (int k = 0; k < n; k++) {
                sum += matrix1[i][k] * matrix2[k][j];
            }
            result[i][j] = sum;
        }
    }
    
    return result;
}

double MatrixUtils::getDeterminant(const std::vector<std::vector<double>>& arcs, int n) {
    if (n == 1) {
        return arcs[0][0];
    }
    
    double ans = 0.0;

    std::vector<std::vector<double>> temp = makeZeroMatrix(n - 1, n - 1);
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n - 1; j++) {
            for (int k = 0; k < n - 1; k++) {
                temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];
            }
        }
        
        double t = getDeterminant(temp, n - 1);
        if (i % 2 == 0) {
            ans += arcs[0][i] * t;
        } else {
            ans -= arcs[0][i] * t;
        }
    }
    
    return ans;
}

void MatrixUtils::getAdjoint(const std::vector<std::vector<double>>& arcs, int n,
                            std::vector<std::vector<double>>& ans) {
    if (n == 1) {
        ans[0][0] = 1.0;
        return;
    }

    std::vector<std::vector<double>> temp = makeZeroMatrix(n, n);
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n - 1; k++) {
                for (int t = 0; t < n - 1; t++) {
                    temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
                }
            }
            ans[j][i] = getDeterminant(temp, n - 1);  // 此处顺便进行了转置
            if ((i + j) % 2 == 1) {
                ans[j][i] = -ans[j][i];
            }
        }
    }
}

bool MatrixUtils::getMatrixInverse(const std::vector<std::vector<double>>& src, int n,
                                   std::vector<std::vector<double>>& des) {
    double determinant = getDeterminant(src, n);
    
    if (std::abs(determinant) < 1e-10) {  // 使用小的阈值而不是精确的0
        std::cout << "原矩阵行列式为0，无法求逆。请重新运行" << "\n";
        return false;
    }

    std::vector<std::vector<double>> adjoint = makeZeroMatrix(n, n);
    getAdjoint(src, n, adjoint);
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            des[i][j] = adjoint[i][j] / determinant;
        }
    }
    
    return true;
}

std::vector<std::vector<double>> MatrixUtils::inverse(
    const std::vector<std::vector<double>>& matrix) {
    // 输入验证
    if (matrix.empty() || matrix[0].empty()) {
        throw std::invalid_argument("矩阵不能为空");
    }
    
    int n = static_cast<int>(matrix.size());
    if (static_cast<int>(matrix[0].size()) != n) {
        throw std::invalid_argument("矩阵必须是方阵");
    }

    std::vector<std::vector<double>> result = makeZeroMatrix(n, n);
    
    bool success = getMatrixInverse(matrix, n, result);
    if (!success) {
        throw std::runtime_error("矩阵求逆失败：行列式为0");
    }
    
    return result;
}
