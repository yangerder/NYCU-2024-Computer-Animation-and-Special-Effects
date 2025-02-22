#include "simulation/kinematics.h"

#include "Eigen/Dense"
#include "acclaim/bone.h"
#include "util/helper.h"

namespace kinematics {

void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    if (bone->parent != NULL) {
        bone->start_position = bone->parent->end_position;
    } else {
        bone->start_position = posture.bone_translations[bone->idx];
    }

    if (bone->parent != NULL) {
        bone->rotation = bone->parent->rotation * bone->rot_parent_current *
                         util::rotateDegreeZYX(posture.bone_rotations[bone->idx]);

    } else {
        bone->rotation = bone->rot_parent_current;
        bone->rotation = util::rotateDegreeZYX(posture.bone_rotations[bone->idx]);
    }

    bone->end_position = bone->start_position;
    bone->end_position += bone->rotation * bone->dir * bone->length;

    if (bone->sibling != NULL) {
        forwardSolver(posture, bone->sibling);
    }

    if (bone->child != NULL) {
        forwardSolver(posture, bone->child);
    }
}
}  // namespace kinematics
