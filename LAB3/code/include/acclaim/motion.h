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
    Motion();
    Motion(const util::fs::path &amc_file, std::unique_ptr<Skeleton> &&skeleton) noexcept;
    Motion(const Motion &) noexcept;
    Motion(Motion &&) noexcept;
    Motion(const Motion &, int start, int end);

    Motion &operator=(const Motion &) noexcept;
    Motion &operator=(Motion &&) noexcept;
    // You need this for alignment otherwise it may crash
    // Ref: https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // get the underlying skeleton
    const std::unique_ptr<Skeleton> &getSkeleton() const;

    // remove frame 'begin' to 'end-1'
    void remove(int begin, int end);

    // concatenate current motion(m1) and input motion(m2)
    void concatenate(Motion &m2);

    Posture& getPosture(int FrameNum);
    std::vector<Posture> getPostures();

    // set posture of a certain frame
    void setPosture(int nFrameNum, const Posture &InPosture);

    // get total frame count of the motion
    int getFrameNum() const;

    // Forward kinematics
    void forwardkinematics(int frame_idx);

    // render the underlying skeleton
    void render(graphics::Program *Program) const;

    void transform(Eigen::Vector4d &newFacing, const Eigen::Vector4d &newPosition);
    Motion blending(Motion &m2, const std::vector<double> &blendWeight, int blendWindowSize);

 private:
    // read motion data from file
    bool readAMCFile(const util::fs::path &file_name);
    std::unique_ptr<Skeleton> skeleton;
    std::vector<Posture> postures;
};
}  // namespace acclaim
