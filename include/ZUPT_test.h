#ifndef ZUPT_TEST_H_   /* Include guard */
#define ZUPT_TEST_H_

#include <iostream>
#include <vector>

void init_data();
std::vector<std::vector<double>> mutil_mat(std::vector<std::vector<double>> m1, std::vector<std::vector<double>> m2);
std::vector<std::vector<double>> mat_inverse(std::vector<std::vector<double>> matrix_before);
bool GetMatrixInverse(std::vector<std::vector<double>> src, int n, std::vector<std::vector<double>>& des);
void  getAStart(std::vector<std::vector<double>> arcs, int n, std::vector<std::vector<double>>& ans);
double getA(std::vector<std::vector<double>> arcs, int n);
std::vector<std::vector<double>> make_zero_martix(int m, int n);

#endif // FOO_H