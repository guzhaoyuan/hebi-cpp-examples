#pragma once

#include <memory>
#include <set>
#include <chrono>

#include <Eigen/Dense>

#include "group.hpp"
#include "group_command.hpp"

#include "quadruped_parameters.hpp"
#include "quadruped_leg.hpp"

/* 
  This class defines a quadruped robot using Hebi's daisy robot (or Mat6)
  The idea is to use leg 1 2 5 6 to run and use 3 4 to grab objects
*/

// Leg Numbering / Chassis Coordinate convention
// This should match ROS wheeled vehicle convention
//
//  1 \   / 2         +x
//     \ /            ^
//  3 ----- 4         |
//     / \      +y <--o
//  5 /   \ 6          +z
// different from hexapod, we use 3 4 to manipulate objects

using namespace Eigen;

namespace hebi {

class Quadruped 
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Allow Eigen member variables

    // learn from hebi source code to do this fancy construction method
    static std::unique_ptr<Quadruped> create(const QuadrupedParameters& params);
    virtual ~Quadruped() noexcept;

    Eigen::Vector3d getGravityDirection();
    Eigen::VectorXd getLegJointAngles(int index);

  private:
    // private constructor, it make sense because before construct must make sure group is successfully created
    Quadruped(std::shared_ptr<Group> group, const QuadrupedParameters& params);
    // hebi middleware to communicate with real hardware
    std::shared_ptr<Group> group_;
    GroupCommand cmd_;

    // leg info
    std::vector<std::unique_ptr<QuadLeg> > legs_;
    Eigen::VectorXd joint_angles; //get joint angles from fbk and put them into leg

    std::chrono::time_point<std::chrono::steady_clock> latest_fbk_time;
    QuadrupedParameters params_;

    // feedback physical quantities
    Eigen::Vector3d gravity_direction_;

    // two locks to get feedback
    std::mutex fbk_lock_;
    std::mutex grav_lock_;

    // control constants
    const float fbk_frq_hz_ = 200.0f;
    const float fbk_frq_ms_ = 5.0f;
    const float ctrl_frq_hz_ = 200.0f;
    // structural constants
    static const int num_legs_ = 6;
    const int num_locomote_legs_ = 4;
    const int num_manipulate_legs_ = 2;
    const int num_joints_per_leg_ = 3;
    const int num_joints_ = num_legs_ * num_joints_per_leg_;
    const int num_locomote_joints_ = num_locomote_legs_ * num_joints_per_leg_;
    const int num_manipulate_joints_ = num_manipulate_legs_ * num_joints_per_leg_;
    static constexpr double weight_ = 9.8f * 21.0f; // mass = 21 kg
};

} // namespace hebi
