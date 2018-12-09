#pragma once

#include <memory>
#include "robot_model.hpp"
#include "quadruped_parameters.hpp"

namespace hebi {

// I would rather not create a new leg. But I think leg should just be leg...
class QuadLeg
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum struct LegConfiguration { Left, Right };
 
    QuadLeg(double angle_rad, 
            double distance, 
            const Eigen::VectorXd& current_angles, 
            const QuadrupedParameters& params, 
            int index, LegConfiguration configuration);

    static constexpr int getNumJoints() { return num_joints_; }
    int getIndex() { return index_; }

    void setJointAngles(Eigen::VectorXd& current_angles);
    Eigen::VectorXd getJointAngle();

    hebi::robot_model::RobotModel& getKinematics() { return *kin_; }
    const hebi::robot_model::RobotModel& getKinematics() const { return *kin_; }

  private:
    Eigen::VectorXd current_angles_;
    // to calcuate IK
    Eigen::VectorXd seed_angles_;
    int index_;  
    static constexpr int num_joints_ = 3;
    std::unique_ptr<hebi::robot_model::RobotModel> kin_;
    
    // Note -- one mass element for each COM frame in the kinematics!
    // do not quite understand the purpose of this variable yet, but keep it anyway
    Eigen::VectorXd masses_;

};


} // namespace hebi
