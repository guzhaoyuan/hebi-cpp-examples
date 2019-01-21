#pragma once

#include <Eigen/Dense>

namespace hebi {
    struct daisyFbkLeg {
        // base -> shoulder -> knee
        Vector3d joint_ang;
        Vector3d joint_vel;
        Vector3d joint_tau;
    };
    // frame convention:  w: world frame, b: body frame, s: sensor frame
    struct daisyIMU {
        Vector3d acc_s;   // in sensor frame 
        Vector3d acc_b;     // in body frame 
        Vector3d gyro_s;
        Vector3d gyro_b;
    };

}