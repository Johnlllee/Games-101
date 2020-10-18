#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

void print_point(Vector3f p) {
    printf("(%f, %f)\n", p(0) / p(2), p(1) / p(2));
}

int main() {
    // P = (2, 1)
    Vector3f P(2, 1, 1);
    print_point(P);
    // anti-clock-rotate 45 degree
    Matrix3f R1;
    double theta = 45.0 / 180 * M_PI;
    R1 << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
    P = R1 * P;
    print_point(P);
    // Trans move (1, 2)
    Matrix3f M1;
    M1 << 1, 0, 1, 0, 1, 2, 0, 0, 1;
    P = M1 * P;
    print_point(P);
    return 0;
}