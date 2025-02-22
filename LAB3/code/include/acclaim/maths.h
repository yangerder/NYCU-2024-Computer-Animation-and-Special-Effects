#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "posture.h"
#include "skeleton.h"
#include "util/filesystem.h"

#define M_PI 3.14159265
const double deg2rad = M_PI / 180.0;
const double rad2deg = 180.0 / M_PI;

#include <Eigen/Dense>
#include "Eigen/Core"
#include "Eigen/StdVector"

namespace acclaim {	
    Eigen::Matrix3f EulerAngle2Matrix(const Eigen::Vector3f& m);
    Eigen::Quaternionf EulerAngle2Quater(float roll, float pitch, float yaw);
}  // namespace acclaim
