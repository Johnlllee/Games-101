#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

int main() {
    // Math
    cout << "sqrt(2.0)=" << sqrt(2.0) << endl;
    cout << "acos(-1)=" << acos(-1) << endl;
    cout << "cos(M_PI)=" << cos(M_PI) << endl;
    cout << "sin(M_PI/2.0)=" << sin(M_PI/2.0) << endl;
    cout << "sin(M_PI)=" << sin(M_PI) << endl;
    cout << "asin(0)=" << asin(0) << endl;

    // Vector
    Vector3f v(1, 2, 3);
    Vector3f w(1, 0, 0);
    cout << "v = " << v.transpose() << endl;
    cout << "w = " << w.transpose() << endl;
    cout << "v norm = " << v.norm() << endl;
    cout << "w norm = " << w.norm() << endl;
    cout << "v + w = " << (v + w).transpose() << endl;
    cout << "v / 2 = " << (v / 2).transpose() << endl;
    cout << "v / 3 = " << (v * 3).transpose() << endl;
    cout << "v .dot w = " << v.dot(w) << endl;
    cout << "w .dot v = " << w.dot(v) << endl;
    cout << "v x w = " << v.cross(w).transpose() << endl;
    cout << "w x v = " << w.cross(v).transpose() << endl;
    cout << "w * v.trans = " << endl << w * v.transpose() << endl;
    cout << "v * w.trans = " << endl << v * w.transpose() << endl;
    cout << "Cos<v, w> = " << v.dot(w) / (v.norm() * w.norm()) << endl;
    cout << "ACos<v, w> = " << acos(v.dot(w) / (v.norm() * w.norm())) << endl;

    Vector3f v2(0, 1, 0);
    Vector3f w2(1, 0, 0);
    cout << "v2 x w2 = " << v2.cross(w2).transpose() << endl;
    cout << "w2 x v2 = " << w2.cross(v2).transpose() << endl;

    // Matrix
    // MatrixXf i(3,3), j(3,3);
    Matrix3f i, j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    cout << "Matrix i:" << endl << i << endl;
    cout << "Matrix j:" << endl << j << endl;
    cout << "i + j:" << endl << i + j << endl;
    cout << "i + j:" << endl << i + j << endl;
    cout << "i norm = " << i.norm() << endl;
    cout << "j norm = " << j.norm() << endl;
    cout << "i * j = " << endl << i * j << endl;
    cout << "i * v = " << endl << (i * v) << endl;
    return 0;
}