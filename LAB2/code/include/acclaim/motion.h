#pragma once

#include <memory>
#include <vector>

#include "Eigen/Core"

#include "posture.h"
#include "skeleton.h"
#include "util/filesystem.h"

namespace graphics {
class Program;
}

namespace acclaim {

class Motion final {
 public:
    Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&skeleton) noexcept;
    Motion(const Motion &) noexcept;
    Motion(Motion &&) noexcept;

    Motion &operator=(const Motion &) noexcept;
    Motion &operator=(Motion &&) noexcept;
    // You need this for alignment otherwise it may crash
    // Ref: https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // get the underlying skeleton
    const std::unique_ptr<Skeleton> &getSkeleton() const;
    // get total frame of the motion
    int getFrameNum() const;
    // Forward kinematics
    void forwardkinematics(int frame_idx);
    // Inverse kinematics
    bool inverseKinematics(std::vector<Eigen::Vector4d> targets, int end, int frame_idx);
    // render the underlying skeleton
    void render(graphics::Program *Program) const;

    // init skeleton
    void initSkeleton(int frame);

 private:
    // read motion data from file
    bool readAMCFile(const util::fs::path &file_name);
    std::unique_ptr<Skeleton> skeleton;
    std::vector<Posture> postures;
};
}  // namespace acclaim
