#include "quadruped_leg.hpp"
#include <iostream>

namespace hebi {

  using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
  using LinkType = hebi::robot_model::RobotModel::LinkType;

  QuadLeg::QuadLeg(double angle_rad, 
                   double distance, 
                   const Eigen::VectorXd& current_angles, 
                   const QuadrupedParameters& params, 
                   int index, LegConfiguration configuration)
  : index_(index)
  {
    // deep copy?
    current_angles_ = current_angles;
    kin_ = (configuration == LegConfiguration::Left) ?
      hebi::robot_model::RobotModel::loadHRDF("left.hrdf") :
      hebi::robot_model::RobotModel::loadHRDF("right.hrdf");
    if (!kin_)
    {
      // Could not find HRDF files!
      std::cerr << "Could not find or load HRDF file for leg!" << std::endl;
      assert("false");
      return;
    }

    kin_->getMasses(masses_);

    Matrix4d transform = Matrix4d::Identity();
    Matrix3d rotate = AngleAxisd(angle_rad, Eigen::Vector3d::UnitZ()).matrix();
    transform.topLeftCorner<3,3>() = rotate;
    Eigen::Vector3d tmp;
    tmp << distance, 0, 0;
    transform.topRightCorner<3,1>() = rotate * tmp;
    kin_->setBaseFrame(transform); // from center of the robot to the base joint of the leg

    seed_angles_.resize(num_joints_);
    if (configuration == LegConfiguration::Left)
      seed_angles_ << 0.2, -.3, -1.9;
    else
      seed_angles_ << 0.2, .3, 1.9;
  }

  void QuadLeg::setJointAngles(Eigen::VectorXd& new_angles)
  {
    current_angles_ = new_angles;
  }
  Eigen::VectorXd QuadLeg::getJointAngle()
  {
    return current_angles_;
  }

} // namespace hebi
