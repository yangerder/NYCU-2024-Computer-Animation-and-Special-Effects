#include "acclaim/motion.h"
#include <fstream>
#include <iostream>
#include <utility>
#include <vector>

#include "simulation/kinematics.h"
#include "util/helper.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace acclaim {
Motion::Motion() {}
Motion::Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&_skeleton) noexcept
    : skeleton(std::move(_skeleton)) {
    postures.reserve(1024);
    if (!this->readAMCFile(amc_file)) {
        std::cerr << "Error in reading AMC file, this object is not initialized!" << std::endl;
        std::cerr << "You can call readAMCFile() to initialize again" << std::endl;
        postures.resize(0);
    }
}

const std::unique_ptr<Skeleton> &Motion::getSkeleton() const { return skeleton; }

Motion::Motion(const Motion &other) noexcept
    : skeleton(std::make_unique<Skeleton>(*other.skeleton)), postures(other.postures) {}

Motion::Motion(Motion &&other) noexcept : skeleton(std::move(other.skeleton)), postures(std::move(other.postures)) {}

Motion::Motion(const Motion &other, int startIdx, int endIdx) : skeleton(std::make_unique<Skeleton>(*other.skeleton)) {
    if (startIdx < 0 || startIdx >= other.postures.size() || endIdx <= startIdx || endIdx > other.postures.size()) {
        throw std::out_of_range("Invalid start or end index");
    }

    postures.assign(other.postures.begin() + startIdx, other.postures.begin() + endIdx);
}

Motion &Motion::operator=(const Motion &other) noexcept {
    if (this != &other) {
        skeleton.reset();
        skeleton = std::make_unique<Skeleton>(*other.skeleton);
        postures = other.postures;
    }
    return *this;
}

Motion &Motion::operator=(Motion &&other) noexcept {
    if (this != &other) {
        skeleton = std::move(other.skeleton);
        postures = std::move(other.postures);
    }
    return *this;
}

void Motion::remove(int begin, int end) {
    assert(end >= begin);
    postures.erase(postures.begin() + begin, postures.begin() + end);
}

void Motion::concatenate(Motion &m2) {
    for (int i = 0; i < m2.getFrameNum(); i++)
        postures.insert(postures.end(), m2.postures[i]);
}

Posture& Motion::getPosture(int FrameNum) { 
    return postures[FrameNum]; 
}

std::vector<Posture> Motion::getPostures() { 
    return postures; 
}

void Motion::setPosture(int FrameNum, const Posture &InPosture) {
    postures[FrameNum] = InPosture; 
}

int Motion::getFrameNum() const { return static_cast<int>(postures.size()); }

void Motion::forwardkinematics(int frame_idx) {
    kinematics::forwardSolver(postures[frame_idx], skeleton->getBonePointer(0));
    skeleton->setModelMatrices();
}

void Motion::transform(Eigen::Vector4d &newFacing, const Eigen::Vector4d &newPosition) {
    // **TODO**
    // Task: Transform the whole motion segment so that the root bone of the first posture(first frame) 
    //       of the motion is located at newPosition, and its facing be newFacing.
    //       The whole motion segment must remain continuous.
    if (postures.empty()) return;
    std::cout << "Transform called\n";

    Eigen::Vector4d initialFacing = postures[0].bone_rotations[0];
    Eigen::Vector4d initialPosition = postures[0].bone_translations[0];

    Eigen::Vector4d D = newPosition - initialPosition;

    Eigen::Matrix3d initialRotation = util::rotateDegreeZYX(initialFacing).toRotationMatrix();
    Eigen::Matrix3d newRotation = util::rotateDegreeZYX(newFacing).toRotationMatrix();


    Eigen::Vector3d initialDir = initialRotation * Eigen::Vector3d::UnitY();
    Eigen::Vector3d newDir = newRotation * Eigen::Vector3d::UnitY();
    double theta_y = atan2(newDir[0], newDir[2]) - atan2(initialDir[0], initialDir[2]);

    Eigen::Quaterniond rotation_y(Eigen::AngleAxisd(theta_y, Eigen::Vector3d::UnitY()));

    for (Posture &posture : postures) {

        Eigen::Vector4d current_pos = posture.bone_translations[0];
        Eigen::Vector4d current_rot = posture.bone_rotations[0];


        Eigen::Quaterniond currentRotQuat = util::EulerAngle2Quater(current_rot.head<3>());
        Eigen::Matrix3d currentRotMatrix = currentRotQuat.toRotationMatrix();
        Eigen::Quaterniond updatedRotQuat = rotation_y * currentRotQuat;
        Eigen::Vector3d updatedRot = util::Quater2EulerAngle(updatedRotQuat);

        postures[0].bone_rotations[0].head<3>()[0] = updatedRot[0];
        postures[0].bone_rotations[0].head<3>()[1] = updatedRot[1];
        postures[0].bone_rotations[0].head<3>()[2] = updatedRot[2];

        Eigen::Vector3d posDiff = (rotation_y * (current_pos.head<3>() - initialPosition.head<3>())) +
                                  (initialPosition.head<3>() + D.head<3>());
        Eigen::Vector4d updatedPos;
        updatedPos.head<3>() = posDiff;
        postures[0].bone_translations[0] = updatedPos;
    }
}

Motion blend(Motion bm1, Motion bm2, const std::vector<double> &weight) {
    // **TODO**
    // Task: Return a motion segment that blends bm1 and bm2.  
    //       bm1: tail of m1, bm2: head of m2
    //       You can assume that m2's root position and orientation is aleady aligned with m1 before blending.
    //       In other words, m2.transform(...) will be called before m1.blending(m2, blendWeight, blendWindowSize) is called
    Motion blendedMotion = bm1;
    int numFrames = bm1.getFrameNum();
    int numBones = bm1.getSkeleton()->getBoneNum();


    for (int frame = 0; frame < numFrames; frame++) {
        Posture blendedPosture(numBones);

        const Posture &posture1 = bm1.getPosture(frame);
        const Posture &posture2 = bm2.getPosture(frame);

        for (int bone = 0; bone < numBones; bone++) {
            Eigen::Quaterniond quat1 = util::EulerAngle2Quater(posture1.bone_rotations[bone].head<3>() * M_PI / 180.0);
            Eigen::Quaterniond quat2 = util::EulerAngle2Quater(posture2.bone_rotations[bone].head<3>() * M_PI / 180.0);

            Eigen::Quaterniond blendedQuat = quat1.slerp(weight[frame], quat2);
            Eigen::Vector3d blendedEuler = util::Quater2EulerAngle(blendedQuat) * 180.0 / M_PI;


            blendedPosture.bone_rotations[bone].head<3>()[1] = blendedEuler[1];
            blendedPosture.bone_rotations[bone].head<3>()[2] = blendedEuler[2];
            blendedPosture.bone_rotations[bone].head<3>()[0] = blendedEuler[0] ;

            blendedPosture.bone_translations[bone] = posture1.bone_translations[bone] * (1.0 - weight[frame]) +
                                                     posture2.bone_translations[bone] * weight[frame];
        }

        blendedMotion.setPosture(frame, blendedPosture);
    }

    return blendedMotion;
}

Motion Motion::blending(Motion &m2, const std::vector<double> &blendWeight, int blendWindowSize) {
    // do blending
    Motion bm1(*this, this->getFrameNum() - blendWindowSize, this->getFrameNum());
    Motion bm2(m2, 0, blendWindowSize);
    Motion bm = blend(bm1, bm2, blendWeight);
    return bm;
}


bool Motion::readAMCFile(const util::fs::path &file_name) {
    // Open AMC file
    std::ifstream input_stream(file_name);
    // Check if file successfully opened
    if (!input_stream) {
        std::cerr << "Failed to open " << file_name << std::endl;
        return false;
    }
    // There are (NUM_BONES_IN_ASF_FILE - 2) moving bones and 2 dummy bones (lhipjoint and rhipjoint)
    int movable_bones = skeleton->getMovableBoneNum();
    // Ignore header
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    input_stream.ignore(1024, '\n');
    int frame_num;
    std::string bone_name;
    while (input_stream >> frame_num) {
        auto &&current_posture = postures.emplace_back(skeleton->getBoneNum());
        for (int i = 0; i < movable_bones; ++i) {
            input_stream >> bone_name;
            const Bone &bone = *skeleton->getBonePointer(bone_name);
            int bone_idx = bone.idx;
            Eigen::Vector4d bone_rotation = Eigen::Vector4d::Zero();
            
            Eigen::Vector4d bone_translation = Eigen::Vector4d::Zero();
            if (bone.doftx) {
                input_stream >> bone_translation[0];
            }
            if (bone.dofty) {
                input_stream >> bone_translation[1];
            }
            if (bone.doftz) {
                input_stream >> bone_translation[2];
            }
            if (bone.dofrx) {
                input_stream >> bone_rotation[0];
            }
            if (bone.dofry) {
                input_stream >> bone_rotation[1];
                
            }
            if (bone.dofrz) {
                input_stream >> bone_rotation[2];
            }
            current_posture.bone_rotations[bone_idx] = std::move(bone_rotation);
            current_posture.bone_translations[bone_idx] = std::move(bone_translation);
            if (bone_idx == 0) {
                current_posture.bone_translations[bone_idx] *= skeleton->getScale();
            }
        }
    }
    input_stream.close();
    std::cout << frame_num << " samples in " << file_name.string() << " are read" << std::endl;
    return true;
}
void Motion::render(graphics::Program *program) const { skeleton->render(program); }

}  // namespace acclaim
