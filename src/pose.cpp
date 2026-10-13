#include "rtvm/pose.hpp"

namespace rtvm {

Pose poseFromTum(double tx, double ty, double tz,
                 double qx, double qy, double qz, double qw) {
    // Rotation: TUM gives a quaternion (w last); Eigen wants w first.
    Eigen::Quaterniond q(qw, qx, qy, qz);
    q.normalize();  // file values are rounded; a rotation quaternion must have length 1
    Eigen::Matrix3d R = q.toRotationMatrix();

    // Position (translation) as a column vector.
    Eigen::Vector3d t(tx, ty, tz);

    // Combine into the 4x4 pose [R t; 0 0 0 1].
    Pose pose = Pose::Identity();
    pose.linear() = R;
    pose.translation() = t;
    return pose;
}

Pose relativePose(const Pose& a, const Pose& b) {
    // Undo a, then apply b: the motion from a to b, seen from a.
    return a.inverse() * b;
}

}  // namespace rtvm