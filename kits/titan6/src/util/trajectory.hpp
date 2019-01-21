#pragma once

#include <cmath>
#include <Eigen/Dense>

namespace hebi {

    double QuinticTimeScaling(double Tf, double t)
    {
        double s;
        s = 10 * pow(t / Tf,  3) - 15 * pow(t / Tf,  4) + 6 * pow(t / Tf,  5);
        return s;
    }

    Eigen::MatrixXd JointTrajectory(Eigen::VectorXd& thetastart, Eigen::VectorXd& thetaend, double Tf, int N, int method)
    {
        double timegap = Tf/(N-1);
        Eigen::MatrixXd traj(thetastart.size(), N);
        for (int i = 0; i < N; i++)
        {
            double s = QuinticTimeScaling(Tf, timegap *i);
            traj.col(i) = thetastart + s *(thetaend - thetastart);    
        }
        return traj;
    }
}