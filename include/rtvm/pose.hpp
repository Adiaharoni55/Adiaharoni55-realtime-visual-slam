#pragma once

#include <Eigen/Geometry>

namespace rtvm {

// A pose = rotation + translation (where the camera is and which way it faces).
// Eigen::Isometry3d stores it as a 4x4 transform matrix.
using Pose = Eigen::Isometry3d;

// Build a pose from the TUM dataset format: translation (tx, ty, tz)
// and quaternion (qx, qy, qz, qw) -- note: TUM puts w LAST.
Pose poseFromTum(double tx, double ty, double tz,
                 double qx, double qy, double qz, double qw);

// How the camera moved from pose a to pose b, seen from a: a^-1 * b
Pose relativePose(const Pose& a, const Pose& b);

}  // namespace rtvm