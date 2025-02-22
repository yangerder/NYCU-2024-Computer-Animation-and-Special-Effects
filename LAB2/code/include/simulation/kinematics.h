#pragma once
#include <vector>
#include <Eigen/Geometry>
#include "acclaim/posture.h"

namespace acclaim {
struct Bone;
}
namespace kinematics {
// Apply forward kinematics to skeleton
void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone);

Eigen::VectorXd pseudoInverseLinearSolver(const Eigen::Matrix4Xd& Jacobian, const Eigen::Vector4d& target);

bool inverseJacobianIKSolver(std::vector<Eigen::Vector4d> targets, acclaim::Bone* end_bone,
                             acclaim::Posture& posture, std::vector<std::vector<Eigen::Vector4d*>> &jointChains,
                             std::vector < std::vector<acclaim::Bone*>> &boneChains, Eigen::Vector4d currentBasePos);
}  // namespace kinematics
