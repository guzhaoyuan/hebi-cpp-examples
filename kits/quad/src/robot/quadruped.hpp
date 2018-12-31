#pragma once

#include <memory>
#include <set>
#include <chrono>

#include <Eigen/Dense>

#include "lookup.hpp"
#include "group.hpp"
#include "trajectory.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "group_info.hpp"

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
    // 1 2 5 6 are locomote legs, 3 4 are manipulate legs
    enum struct CtrlLegType { all, locomote, manipulate };
    enum struct SwingMode {swing_mode_virtualLeg1, swing_mode_virtualLeg2};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Allow Eigen member variables

    // learn from hebi source code to do this fancy construction method
    static std::unique_ptr<Quadruped> create(const QuadrupedParameters& params);
    virtual ~Quadruped() noexcept;

    Eigen::Vector3d getGravityDirection();
    Eigen::VectorXd getLegJointAngles(int index);

    void computeFootForces(Eigen::MatrixXd& foot_forces);

    // these functions are called in outside state machine, they must be called every iteration
    // and, in all these modes they will call sendCommand. Which is not good because we may want to combine 
    // functions in a state, so the cmd will be refreshed twice.
    // I think a better design methdology would either be 
    // 1. make sure outside state machine always only call one and only one
    //    function to enter quadruped, and combine state logics all in this function
    // 2. at state machine, explicitly call sendCommand after call functions that set cmd 
    //  It is just different levels of abstraction, need more thinking 
    bool planStandUpTraj(double duration_time);
    bool execStandUpTraj(double curr_time);
    bool spreadAllLegs();
    bool pushAllLegs(double curr_time, double total_time);
    bool prepareQuadMode();
    void runTest(SwingMode mode, double curr_time, double total_time);
    void prepareTrajectories(SwingMode mode, double leg_swing_time);
    bool reOrient(Matrix3d target_body_R);
    bool rePos(int move_id, double tgt_x, double tgt_y, double curr_time, double total_time);

    bool moveSingleLegTraj(int move_leg_id, double x_distance, double y_distance, double leg_swing_time);
    bool moveSingleLeg(int move_leg_id, double curr_time, double total_time);
    void saveFootPose();
    Eigen::Vector2d getFootPose(int id);
    Eigen::Matrix3d getBodyR() {return body_R;}

    void startBodyRUpdate() {updateBodyR = true;}
    void setStartGait() {startGait = true;}
    void lifeManipulatorLegs();
    void gentleLiftRecover(double curr_time, double total_time);  

    bool isExecution() {return is_exec_traj;}

    void setCommand(int index, const VectorXd* angles, const VectorXd* vels, const VectorXd* torques);
    void saveCommand();
    void loadCommand();
    void sendCommand();
    bool setGains();

    // Public Function below is added by Zhaoyuan, use at your own risk
    void moveLegs(double dx, double dy, double dz = 0);
    void moveBody(double dx, double dy, double dz = 0); // move body in cm
    void moveFootRel(int footIndex, double dx = 0, double dy = 0, double dz = 0);
    void moveFootAbs(int footIndex, double dx, double dy, double dz); // TODO: make clear about the com coordinate and leg coordinate
    
    void planFootTraj(int footIndex, double dx, double dy, double dz, double swingTime); // plan a triangle path, dz is height, dx,dx is distance reletive to original foot
    void followFootTraj(int trajFootIndex, double timeSpent);
    void planBodyTraj(double dx, double dy, double body_move_time);
    void followBodyTraj(double timeSpent);
    void freeze(); // stay where the robot was commanded

    // function for wave gait by zhaoyuan
    void followWaveGait(double timeSpent);
    void planWaveGait();
    double getTotalTime(){return totalTime;}
    void planBodyFootTraj(int legIndex, int timeStep, Eigen::MatrixXd &positions);
    void saveStanceCommand();
    void loadStanceCommand();

  private:
    // private constructor, it make sense because before construct must make sure group is successfully created
    Quadruped(std::shared_ptr<Group> group, const QuadrupedParameters& params);

    // private functions
    Eigen::Quaterniond average_quat(Eigen::Quaterniond average_q_, std::vector<Eigen::Quaterniond> q_list_);
    Eigen::Vector3d quat_log(Eigen::Quaterniond q);
    Eigen::Quaterniond quat_exp(Eigen::Vector3d qv);

    // hebi middleware to communicate with real hardware
    std::shared_ptr<Group> group_;
    GroupCommand cmd_;
    GroupCommand saved_cmd_;
    GroupCommand saved_stance_cmd_;

    // leg info
    std::vector<std::unique_ptr<QuadLeg> > legs_;
    Eigen::VectorXd joint_angles; //get joint angles from fbk and put them into leg

    std::chrono::time_point<std::chrono::steady_clock> latest_fbk_time;
    QuadrupedParameters params_;

    // feedback physical quantities
    Eigen::Vector3d gravity_direction_;
    Eigen::Matrix3d body_R;

    bool updateBodyR = false;
    bool startGait = false; // signal for first time enter Gait loop

    // two locks to get feedback
    std::mutex fbk_lock_;
    std::mutex grav_lock_;

    // planner trajectories
    std::vector<std::shared_ptr<trajectory::Trajectory>> startup_trajectories;
    std::vector<std::shared_ptr<trajectory::Trajectory>> stance_trajectories;  // used in runTest
    std::vector<std::shared_ptr<trajectory::Trajectory>> swing_trajectories;   // used in runTest and followFootTraj
    std::vector<std::shared_ptr<trajectory::Trajectory>> body_move_trajectories;  // used in runTest
    std::vector<std::shared_ptr<trajectory::Trajectory>> wave_gait_trajectories;  // used in runTest

    int trajFootIndex; // used to indicate which foot does the trajectory corresponding to
    bool is_exec_traj; // flag to show that it is still running trajectories 

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

    // structural and stance constants to make system stand
    // these bars need to be tuned
    const double bar_y = 0.1187; // distance fromt com of the robot to motor 0, y direction
    const double bar_x = 0.2057; // distance fromt com of the robot to motor 0, x direction
    double foot_bar_y = 0.1187 + 0.24; // distance fromt com of the robot to foot 0, y direction 
    double foot_bar_x = 0.2057 + 0.34; // distance fromt com of the robot to foot 0, x direction
    double foot_bar_y_list[4];          // (see function saveFootPose) 
    double foot_bar_x_list[4];          // (see function saveFootPose) 
    double foot_force_ratio[4];
    double nominal_height_z = 0.31;

    // leg Definition
    const std::vector<int> walkingLegs = {0, 1, 4, 5};
    const std::vector<int> manipulateLegs = {2, 3};

    // wave gait params
    const double totalTime = 10;
    const double stepSize = 16;

    Eigen::Vector4d base_stance_ee_xyz; // expressed in base motor's frame
    Eigen::Vector4d base_stance_ee_xyz_offset; // expressed in base motor's frame
    Eigen::Vector3d stance_ee_xyz_fk_offset;  // HEBI's bug: they does not consider the offset of the last link.
    Eigen::Vector3d com_stance_ee_xyz;  // expressed in com of the robot's frame
};

} // namespace hebi
