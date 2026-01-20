#include <iostream>
#include <vector>
#include "./include/ZUPT_test.h"

void init_data(){
    double g = 9.81; //Gravity 
    std::cout << g << "\n";
}
//矩阵乘法
std::vector<std::vector<double>> mutil_mat(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2) {
    //两矩阵相乘
    int m = m1.size();
    int n = m1[0].size();
    int p = m2[0].size();
    std::vector<std::vector<double>> array;
    std::vector<double> temparay;
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            double sum = 0;
            for (int k = 0; k < n; k++) {
                sum += m1[i][k] * m2[k][j];
            }
            temparay.push_back(sum);
        }
        array.push_back(temparay);
        temparay.erase(temparay.begin(), temparay.end());
    }
    return array;
}


//矩阵求逆
std::vector<std::vector<double>> make_zero_martix(int m, int n) {
    //创建0矩阵
    std::vector<std::vector<double>> array;
    std::vector<double> temparay;
    for (int i = 0; i < m; ++i)// m*n 维数组
    {
        for (int j = 0; j < n; ++j)
            temparay.push_back(i * j);
        array.push_back(temparay);
        temparay.erase(temparay.begin(), temparay.end());
    }
    return array;
}

//按第一行展开计算|A|
double getA(std::vector<std::vector<double>> arcs, int n)
{
    if (n == 1)
    {
        return arcs[0][0];
    }
    double ans = 0;
    std::vector<std::vector<double>> temp = make_zero_martix(arcs.size(), arcs.size());
    int i, j, k;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n - 1; j++)
        {
            for (k = 0; k < n - 1; k++)
            {
                temp[j][k] = arcs[j + 1][(k >= i) ? k + 1 : k];
            }
        }
        double t = getA(temp, n - 1);
        if (i % 2 == 0)
        {
            ans += arcs[0][i] * t;
        }
        else
        {
            ans -= arcs[0][i] * t;
        }
    }
    return ans;
}

//计算每一行每一列的每个元素所对应的余子式，组成A*
void  getAStart(std::vector<std::vector<double>> arcs, int n, std::vector<std::vector<double>>& ans)
{
    if (n == 1)
    {
        ans[0][0] = 1;
        return;
    }
    int i, j, k, t;
    std::vector<std::vector<double>> temp = make_zero_martix(n, n);
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            for (k = 0; k < n - 1; k++)
            {
                for (t = 0; t < n - 1; t++)
                {
                    //cout << arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t] << endl;
                    temp[k][t] = arcs[k >= i ? k + 1 : k][t >= j ? t + 1 : t];
                }
            }
            ans[j][i] = getA(temp, n - 1);  //此处顺便进行了转置
            if ((i + j) % 2 == 1)
            {
                ans[j][i] = -ans[j][i];
            }
        }
    }
}

//得到给定矩阵src的逆矩阵保存到des中。
bool GetMatrixInverse(std::vector<std::vector<double>> src, int n, std::vector<std::vector<double>>& des)
{
    double flag = getA(src, n);
    std::vector<std::vector<double>> t = make_zero_martix(n, n);
    if (0 == flag)
    {
        std::cout << "原矩阵行列式为0，无法求逆。请重新运行" << "\n";
        return false;//如果算出矩阵的行列式为0，则不往下进行
    }
    else
    {
        getAStart(src, n, t);
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                des[i][j] = t[i][j] / flag;
            }
        }
    }
    return true;
}

std::vector<std::vector<double>> mat_inverse(std::vector<std::vector<double>> matrix_before) {
    //矩阵求逆
    bool flag;
    std::vector<std::vector<double>> matrix_after = make_zero_martix(matrix_before.size(), matrix_before.size());
    flag = GetMatrixInverse(matrix_before, matrix_before.size(), matrix_after);
    return matrix_after;
}