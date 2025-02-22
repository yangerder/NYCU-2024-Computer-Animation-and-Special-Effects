#include "acclaim/motion.h"
#include <fstream>
#include <iostream>
#include <utility>

#include "simulation/kinematics.h"
#include "..\..\include\acclaim\maths.h"

#define M_PI 3.14159265
const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 / M_PI;

namespace acclaim {

Eigen::Matrix3f EulerAngle2Matrix(const Eigen::Vector3f& m) {
    Eigen::AngleAxisf rollAngle(m[0], Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(m[1], Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(m[2], Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf quaternion = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3f rotationMatrix = quaternion.matrix();

    return rotationMatrix;
}

Eigen::Quaternionf EulerAngle2Quater(float roll, float pitch, float yaw) {  // Convert Euler angles to quarternion
    Eigen::Quaternionf q;
    q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

    return q;
}
}  // namespace acclaim
