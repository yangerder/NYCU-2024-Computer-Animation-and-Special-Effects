#include "acclaim/posture.h"
#include "util/helper.h"
#include "Eigen/Core"
#include <iostream>

#include <utility>

namespace acclaim {
Posture::Posture() noexcept {}

Posture::Posture(const std::size_t size) noexcept
    : bone_rotations(size, Eigen::Vector4d::Zero()), bone_translations(size, Eigen::Vector4d::Zero()) {}

Posture::Posture(const Posture &other) noexcept
    : bone_rotations(other.bone_rotations), bone_translations(other.bone_translations) {}

Posture::Posture(Posture &&other) noexcept
    : bone_rotations(std::move(other.bone_rotations)), bone_translations(std::move(other.bone_translations)) {}

Posture &Posture::operator=(const Posture &other) noexcept {
    if (this != &other) {
        bone_rotations = other.bone_rotations;
        bone_translations = other.bone_translations;
    }
    return *this;
}
Posture &Posture::operator=(Posture &&other) noexcept {
    if (this != &other) {
        bone_rotations = std::move(other.bone_rotations);
        bone_translations = std::move(other.bone_translations);
    }
    return *this;
}

double AngularDifference(double angle1, double angle2) {
    double diff = angle2 - angle1;
    diff = fmod(diff + 180.0, 360.0);
    if (diff < 0) diff += 360.0;
    return diff - 180.0;
}

double Posture::PoseDist(const Posture &p2, int numBones, const std::vector<double> &jntWeight) {
    double dist = 0.0;

    for (int i = 1; i < numBones; i++)  // ignore root orientation
    {
        const Eigen::Vector4d &rot1 = bone_rotations[i];
        const Eigen::Vector4d &rot2 = p2.bone_rotations[i];

        // Calculate distance for each angle component
        double angleDiffX = AngularDifference(rot1.x(), rot2.x());
        double angleDiffY = AngularDifference(rot1.y(), rot2.y());
        double angleDiffZ = AngularDifference(rot1.z(), rot2.z());

        Eigen::Vector3d diff(angleDiffX, angleDiffY, angleDiffZ);
        dist += diff.norm() * jntWeight[i];
    }

    return dist;
}

double Posture::getFacingAngle() {
    return bone_rotations[0].head<3>()[1];

}
}  // namespace acclaim
