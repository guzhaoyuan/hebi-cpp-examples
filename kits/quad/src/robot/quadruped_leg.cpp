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
  : index_(index), spring_shift_(configuration == LegConfiguration::Right ? 3.75 : -3.75) //so hardcode
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

  bool QuadLeg::computeIK(Eigen::VectorXd& angles, Eigen::VectorXd& ee_pos)
  {
    auto res = kin_->solveIK(
        seed_angles_,
        angles,
        robot_model::EndEffectorPositionObjective(ee_pos));
    if (res.result != HebiStatusSuccess)
    {
      return false;
    }
    else
    {
      return true;
    }    
  }
  Eigen::VectorXd QuadLeg::computeCompensateTorques(const Eigen::VectorXd& angles, const Eigen::VectorXd& vels, const Eigen::Vector3d& gravity_vec, 
   const Eigen::Vector3d& foot_force)
  {
    // TODO: pull from XML?
    constexpr float drag_shift = 1.5; // Nm / (rad/sec)
    // Get the Jacobian
    Eigen::MatrixXd jacobian_ee;
    robot_model::MatrixXdVector jacobian_com;
    kin_->getJEndEffector(angles, jacobian_ee);
    kin_->getJ(HebiFrameTypeCenterOfMass, angles, jacobian_com);

    Eigen::VectorXd spring(num_joints_);
    spring << 0, spring_shift_ + drag_shift * vels(1), 0;
    Eigen::VectorXd stance(num_joints_);
    Eigen::VectorXd grav_comp(num_joints_);

    // test start up sequence so this foot force is hardcoded
    MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
    stance = jacobian_part.transpose() * (-foot_force);

    grav_comp.setZero();
    for (int i = 0; i < masses_.size(); ++i)
      grav_comp += - jacobian_com[i].topLeftCorner<3,num_joints_>().transpose() * (gravity_vec * masses_[i]);

    return grav_comp + stance + spring;
  }

  Eigen::MatrixXd QuadLeg::getBaseFrame()
  {
    return kin_->getBaseFrame();
  }
} // namespace hebi
