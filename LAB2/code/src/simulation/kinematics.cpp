#include "simulation/kinematics.h"

#include <iostream>
#include "Eigen/Dense"
#include "acclaim/bone.h"
#include "util/helper.h"

void DFS(const acclaim::Posture& posture, acclaim::Bone* bone, bool visited[]) {
    visited[bone->idx] = true;

    if (bone->parent != nullptr) {
        bone->start_position = bone->parent->end_position;
        bone->rotation = bone->parent->rotation * bone->rot_parent_current;
        bone->rotation *= util::rotateDegreeZYX(posture.bone_rotations[bone->idx]);
        bone->end_position = bone->start_position + bone->rotation * (bone->dir.normalized() * bone->length);
    }

    if (bone->child != nullptr && !visited[bone->child->idx]) {
        DFS(posture, bone->child, visited);
    }

    acclaim::Bone* tmp = bone->sibling;
    while (tmp != nullptr) {
        if (!visited[tmp->idx]) {
            DFS(posture, tmp, visited);
        }
        tmp = tmp->sibling;
    }
}


namespace kinematics {

void forwardSolver(const acclaim::Posture& posture, acclaim::Bone* bone) {
    // TODO (FK)
    // You should set these variables:
    //     bone->start_position = Eigen::Vector4d::Zero();
    //     bone->end_position = Eigen::Vector4d::Zero();
    //     bone->rotation = Eigen::Matrix4d::Zero();
    // The sample above just set everything to zero
    // Hint:
    //   1. posture.bone_translations, posture.bone_rotations
    // Note:
    //   1. This function will be called with bone == root bone of the skeleton
    //   2. we use 4D vector to represent 3D vector, so keep the last dimension as "0"
    //   3. util::rotate{Degree | Radian} {XYZ | ZYX}
    //      e.g. rotateDegreeXYZ(x, y, z) means:
    //      x, y and z are presented in degree rotate z degrees along z - axis first, then y degrees along y - axis, and
    //      then x degrees along x - axis
    bool visited[31] = {};
    bone->start_position = posture.bone_translations[0];
    bone->rotation = bone->rot_parent_current;
    bone->rotation = util::rotateDegreeZYX(posture.bone_rotations[bone->idx]);
    bone->end_position = bone->start_position;
    bone->end_position += bone->rotation * (bone->dir.normalized() * bone->length);
    visited[0] = true;

    DFS(posture, bone->child, visited);


}

Eigen::VectorXd pseudoInverseLinearSolver(const Eigen::Matrix4Xd& Jacobian, const Eigen::Vector4d& target) {
    // TODO (find x which min(| jacobian * x - target |))
    // Hint:
    //   1. Linear algebra - least squares solution
    //   2. https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse#Construction
    // Note:
    //   1. SVD or other pseudo-inverse method is useful
    //   2. Some of them have some limitation, if you use that method you should check it.
    Eigen::VectorXd deltatheta(Jacobian.cols());
    Eigen::JacobiSVD<Eigen::Matrix4Xd> svdSolver(Jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    deltatheta = svdSolver.solve(target);



    return deltatheta;
}

/**
 * @brief Perform inverse kinematics (IK)
 *
 * @param target_pos The position where `end_bone` (first joint in the chain) will move to.
 * @param start_bone This bone is the last bone you can move while doing IK
 * @param posture The original AMC motion's reference, you need to modify this
 * @param jointChains A 2D vector containing multiple 1D vectors, each of which holds pointers to Eigen::Vector4d
 * constituting a chain.
 * @param boneChains A 2D vector containing multiple 1D vectors, each of which holds pointers to acclaim::Bone
 * constituting a chain.
 * @param currentBasePos The base of the current chain.
 */


bool inverseJacobianIKSolver(std::vector<Eigen::Vector4d> target_pos, acclaim::Bone* end_bone,
                             acclaim::Posture& posture, std::vector<std::vector<Eigen::Vector4d*>>& jointChains,
                             std::vector<std::vector<acclaim::Bone*>>& boneChains, Eigen::Vector4d currentBasePos) {
    constexpr int max_iteration = 1000;
    constexpr double epsilon = 1E-3;
    constexpr double step = 0.1;
    // Since bone stores in bones[i] that i == bone->idx, we can use bone - bone->idx to find bones[0] which is the
    // root.
    acclaim::Bone* root_bone = end_bone - end_bone->idx;
    // TODO
    // Perform inverse kinematics (IK)
    // HINTs will tell you what should do in that area.
    // Of course you can ignore it (Any code below this line) and write your own code.
    acclaim::Posture original_posture(posture);

    size_t bone_num = 0;

    // Traverse each chain
    for (int chainIdx = 0; chainIdx < boneChains.size(); ++chainIdx) {
        bone_num = boneChains[chainIdx].size();
        Eigen::Matrix4Xd Jacobian(4, 3 * bone_num);
        Jacobian.setZero();

        for (int iter = 0; iter < max_iteration; ++iter) {
            Eigen::Vector4d desiredVector = target_pos[chainIdx] - *jointChains[chainIdx][0];
            if (desiredVector.norm() < epsilon) {
                break;
            }
            // TODO (compute jacobian)
            //   1. Compute arm vectors
            //   2. Compute jacobian columns, store in `Jacobian`
            // Hint:
            //   1. You should not put rotation in jacobian if it doesn't have that DoF.
            //   2. jacobian.col(/* some column index */) = /* jacobian column */

            for (long long i = 0; i < bone_num; i++) {
                Eigen::Vector4d armVector = *jointChains[chainIdx][0] - *jointChains[chainIdx][i + 1];
                auto rotationMatrix = boneChains[chainIdx][i]->rotation;
                for (int dofIndex = 0; dofIndex < 3; dofIndex++) {
                    if ((dofIndex == 0 && boneChains[chainIdx][i]->dofrx) ||
                        (dofIndex == 1 && boneChains[chainIdx][i]->dofry) ||
                        (dofIndex == 2 && boneChains[chainIdx][i]->dofrz)) {
                        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
                        axis[dofIndex] = 1; 

                        Eigen::Vector3d rotatedAxis = rotationMatrix.rotation() * axis;
                        Eigen::Vector4d dofCol(rotatedAxis.x(), rotatedAxis.y(), rotatedAxis.z(), 0.0);
                        dofCol.normalize();
                        Eigen::Vector3d jacobianCol = rotatedAxis.cross(armVector.head<3>());
                        Jacobian.block<3, 1>(0, 3 * i + dofIndex) = jacobianCol;
                    }
                }
            }


            Eigen::VectorXd deltatheta = step * pseudoInverseLinearSolver(Jacobian, desiredVector);

            // TODO (update rotation)
            //   Update `posture.bone_rotation` (in euler angle / degrees) using deltaTheta
            // Hint:
            //   1. You can ignore rotation limit of the bone.
            // Bonus:
            //   1. You cannot ignore rotation limit of the bone.
            double PI = 3.14159265358979323846;
            for (long long i = 0; i < bone_num; i++) {
                acclaim::Bone* currentBone = boneChains[chainIdx][i];
                for (int j = 0; j < 3; j++) {
                    bool isDofActive = false;  
                if ((j == 0 && currentBone->dofrx) ||
                    (j == 1 && currentBone->dofry) ||
                    (j == 2 && currentBone->dofrz)) {
                    isDofActive = true;  
                }

                if (isDofActive) {
                    posture.bone_rotations[currentBone->idx][j] += ((deltatheta[i * 3 + j]*180)/PI);
                }
                    
                }
                posture.bone_rotations[currentBone->idx][0] =
                    std::clamp(posture.bone_rotations[currentBone->idx][0], (double)currentBone->rxmin,
                               (double)currentBone->rxmax);
                posture.bone_rotations[currentBone->idx][1] =
                    std::clamp(posture.bone_rotations[currentBone->idx][1], (double)currentBone->rymin,
                               (double)currentBone->rymax);
                posture.bone_rotations[currentBone->idx][2] =
                    std::clamp(posture.bone_rotations[currentBone->idx][2], (double)currentBone->rzmin,
                               (double)currentBone->rzmax);

            }

            forwardSolver(posture, root_bone);
            // Deal with root translation
            if (chainIdx == 0) {
                posture.bone_translations[0] =
                    posture.bone_translations[0] + (currentBasePos - *jointChains[chainIdx][bone_num]);
            }
        }
    }

    // Return whether IK is stable (i.e. whether the ball is reachable) and let the skeleton not swing its hand in the
    // air
    bool stable = true;
    for (int i = 0; i < boneChains.size(); ++i) {
        if ((target_pos[i] - *jointChains[i][0]).norm() > epsilon) {
            stable = false;
        }
    }
    // You can replace "!stable" with "false" to see unstable results, but this may lead to some unexpected outcomes.
    if (!stable) {
        posture = original_posture;
        forwardSolver(posture, root_bone);
        return false;
    } else {
        original_posture = posture;
        return true;
    }
}
}  // namespace kinematics