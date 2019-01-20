#pragma once

#include <Eigen/Dense>

namespace hebi {
    struct daisyFbkLeg {
        // base -> shoulder -> knee
        Vector3d joint_ang;
        Vector3d joint_vel;
        Vector3d joint_tau;
    };

    struct daisyIMU {
        Vector3d acc;
        Vector3d gyro;
    };

}