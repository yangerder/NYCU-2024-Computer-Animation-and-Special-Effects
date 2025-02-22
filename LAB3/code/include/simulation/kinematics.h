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
}  // namespace kinematics
