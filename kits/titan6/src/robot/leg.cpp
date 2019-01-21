#include "leg.hpp"
#include <iostream>


namespace hebi {

using ActuatorType = hebi::robot_model::RobotModel::ActuatorType;
using LinkType = hebi::robot_model::RobotModel::LinkType;

Leg::Leg(double angle_rad, double distance, const Eigen::VectorXd& current_angles,  const Eigen::Vector4d& _mount_point, const HexapodParameters& params, bool is_dummy, int index, LegConfiguration configuration_)
  : configuration(configuration_), index_(index), stance_radius_(params.stance_radius_), body_height_(params.default_body_height_), spring_shift_(configuration == LegConfiguration::Right ? 3.75 : -3.75) // Nm
{
  kin_ = configuration == LegConfiguration::Left ?
    hebi::robot_model::RobotModel::loadHRDF("left.hrdf") :
    hebi::robot_model::RobotModel::loadHRDF("right.hrdf");
  if (!kin_)
  {
    // Could not find HRDF files!
    // TODO: handle this better so we don't segfault later...probably a factory
    // for "leg"
    std::cerr << "Could not find or load HRDF file for leg!" << std::endl;
    assert("false");
    return;
  }

  mount_point = _mount_point;

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
 
  auto base_frame = kin_->getBaseFrame();
  Eigen::Vector4d tmp4(stance_radius_, 0, -body_height_, 0);
  home_stance_xyz_ = (base_frame * tmp4).topLeftCorner<3,1>();
  level_home_stance_xyz_ = home_stance_xyz_;

  // Set initial stance position
  computeFK(fbk_stance_xyz_, current_angles);
  computeFK(cmd_stance_xyz_, seed_angles_);

  // TODO: initialize better here? What did the MATLAB code do? (nevermind -- that fix wasn't
  cmd_stance_xyz_ = fbk_stance_xyz_;


  // build rbdl model 
  model = new RigidBodyDynamics::Model();

  model -> gravity = Vector3d(0, 0, -9.81); // use chassis body frame direction direction
  // all vectors below are represented in leg base frame!!!
  if (configuration == LegConfiguration::Left)
  {
    // joint float (at the center of the base) -> base -> joint1 -> link1 -> joint2 -> link2 -> joint3 -> link3
    joint_float = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFloatingBase);
    RigidBodyDynamics::Math::Matrix3d I_base;
    I_base << 533112*10e-9,	 85290  *10e-9,	    673     *10e-9,
               85290*10e-9,	 959826 *10e-9,	    808     *10e-9,
                 673*10e-9,	  808   *10e-9,	    1301756 *10e-9;   // this is wrong but tolerable
    body_base = RigidBodyDynamics::Body( 1.280,                                // mass
                      RigidBodyDynamics::Math::Vector3d(15*10e-3, 1.775*10e-3, 0),  // COM
                      I_base);                                  // inertia
    // 0 is the floating base
    body_base_id = model -> AddBody(0, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,0)), joint_float, body_base);

    joint_1 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,0,1));
    RigidBodyDynamics::Math::Matrix3d I1;
    I1 << 882225.81*10e-9,	 -44550.48*10e-9,	    138685.24*10e-9,
          -44550.48*10e-9,	 1688152.34*10e-9,	    -13138.04*10e-9,
          138685.24*10e-9,	 -13138.04*10e-9,	    1136465.63*10e-9;
    body_1 = RigidBodyDynamics::Body( 1.468,                                // mass
                      RigidBodyDynamics::Math::Vector3d(-12.56*10e-3, 9.09*10e-3, 48.47*10e-3),  // COM
                      I1);// inertia
    body_1_id = model -> AddBody(body_base_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,16*10e-3)), joint_1, body_1);
                                  
    

    joint_2 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,1,0));
    RigidBodyDynamics::Math::Matrix3d I2;
    I2 << 1025508.94*10e-9,	-1114524.54*10e-9,	    25297.09*10e-9,
	       -1114524.54*10e-9,	11305007.48*10e-9,	    18227.23*10e-9,
	          25297.09*10e-9,	    18227.23*10e-9,	    11190881.72*10e-9;
    body_2 = RigidBodyDynamics::Body( 1.742, 
                      RigidBodyDynamics::Math::Vector3d(238.31*10e-3, -7.17*10e-3, -1.31*10e-3), 
                      I2);
    body_2_id = model -> AddBody(body_1_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,-8.55*10e-3,55*10e-3)), joint_2, body_2);
     
    joint_3 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,-1,0));
    RigidBodyDynamics::Math::Matrix3d I3;
    I3 <<      93933*10e-9,	     -53812*10e-9,	    -1.03*10e-9,
	            -53812*10e-9,	    2680631*10e-9,	    -0.32*10e-9,
	             -1.03*10e-9,	      -0.32*10e-9,	  2660562*10e-9;
    body_3 = RigidBodyDynamics::Body( 0.27095, 
                      RigidBodyDynamics::Math::Vector3d(88.91*10e-3, -17.70*10e-3, 0*10e-3), 
                      I3);
    body_3_id = model -> AddBody(body_2_id, RigidBodyDynamics::Math::Xtrans(Vector3d(279.5*10e-3,-31.05*10e-3,0*10e-3)), joint_3, body_3);
  }
  else
  {
    // joint float (at the center of the base) -> base -> joint1 -> link1 -> joint2 -> link2 -> joint3 -> link3
    joint_float = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFloatingBase);
    RigidBodyDynamics::Math::Matrix3d I_base;
    I_base << 533112*10e-9,	 85290  *10e-9,	    673     *10e-9,
               85290*10e-9,	 959826 *10e-9,	    808     *10e-9,
                 673*10e-9,	  808   *10e-9,	    1301756 *10e-9;   // this is wrong but tolerable
    body_base = RigidBodyDynamics::Body( 1.280,                                // mass
                      RigidBodyDynamics::Math::Vector3d(15*10e-3, 1.775*10e-3, 0),  // COM
                      I_base);                                  // inertia
    // 0 is the floating base
    body_base_id = model -> AddBody(0, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,0)), joint_float, body_base);

    joint_1 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,0,1));
    RigidBodyDynamics::Math::Matrix3d I1;
    I1 << 1138475*10e-9,	 77725*10e-9,	    14289*10e-9,
          77725*10e-9,	 1975759*10e-9,	    -1151*10e-9,
          14289*10e-9,	 -1151*10e-9,	    1280182*10e-9;
    body_1 = RigidBodyDynamics::Body( 1.645,                                // mass
                      RigidBodyDynamics::Math::Vector3d(-10.71*10e-3, 10.14*10e-3, 43.11*10e-3),  // COM
                      I1);// inertia
    body_1_id = model -> AddBody(body_base_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,16*10e-3)), joint_1, body_1);
                                  
    

    joint_2 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,1,0));
    RigidBodyDynamics::Math::Matrix3d I2;
    I2 << 962949*10e-9,	794861*10e-9,	    -40378*10e-9,
	       794861*10e-9,	8860812*10e-9,	    16263*10e-9,
	          -40378*10e-9,	    16263*10e-9,	    8716509*10e-9;
    body_2 = RigidBodyDynamics::Body( 1.742, 
                      RigidBodyDynamics::Math::Vector3d(244.91*10e-3, 8.03*10e-3, 1.35*10e-3), 
                      I2);
    body_2_id = model -> AddBody(body_1_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,-8.55*10e-3,55*10e-3)), joint_2, body_2);
     
    joint_3 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,-1,0));
    RigidBodyDynamics::Math::Matrix3d I3;
    I3 <<      93933*10e-9,	     53812*10e-9,	    -1.03*10e-9,
	            53812*10e-9,	    2680631*10e-9,	    -0.32*10e-9,
	             -1.03*10e-9,	      -0.32*10e-9,	  2660562*10e-9;
    body_3 = RigidBodyDynamics::Body( 0.27095, 
                      RigidBodyDynamics::Math::Vector3d(103.91*10e-3, 17.06*10e-3, 0*10e-3), 
                      I3);
    body_3_id = model -> AddBody(body_2_id, RigidBodyDynamics::Math::Xtrans(Vector3d(279.5*10e-3,31.05*10e-3,0*10e-3)), joint_3, body_3);
  }

}

// Compute jacobian given position and velocities
bool Leg::computeJacobians(const Eigen::VectorXd& angles, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com)
{
  kin_->getJEndEffector(angles, jacobian_ee);
  kin_->getJ(HebiFrameTypeCenterOfMass, angles, jacobian_com);
}
 
bool Leg::computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com)
{
  // TODO: think about returning an error value, e.g., when IK fails?
  // TODO: add torque!
  angles.resize(num_joints_);
  vels.resize(num_joints_);
  if (step_) // Step
  {
    Eigen::VectorXd accels;
    step_->computeState(t, angles, vels, accels);
    computeJacobians(angles, jacobian_ee, jacobian_com);
  }
  else // Stance
  {
    auto res = kin_->solveIK(
      seed_angles_,
      angles,
      robot_model::EndEffectorPositionObjective(cmd_stance_xyz_));
    if (res.result != HebiStatusSuccess)
    {
      return false;
    }
    computeJacobians(angles, jacobian_ee, jacobian_com);
    // J(1:3,:) \ stance_vel_xyz)
    MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
    vels = jacobian_part.colPivHouseholderQr().solve(stance_vel_xyz_).eval();
    return true;
  }
}

Eigen::VectorXd Leg::computeTorques(const robot_model::MatrixXdVector& jacobian_com, const Eigen::MatrixXd& jacobian_ee, const Eigen::VectorXd& angles, const Eigen::VectorXd& vels, const Eigen::Vector3d& gravity_vec, const Eigen::Vector3d& foot_force)
{
  // TODO: pull from XML?
  constexpr float drag_shift = 1.5; // Nm / (rad/sec)
  
  Eigen::VectorXd spring(Leg::getNumJoints());
  spring << 0, spring_shift_ + drag_shift * vels(1), 0;
  Eigen::VectorXd stance(Leg::getNumJoints());
  Eigen::VectorXd grav_comp(Leg::getNumJoints());

  MatrixXd jacobian_part = jacobian_ee.topLeftCorner(3,jacobian_ee.cols());
  stance = jacobian_part.transpose() * (-foot_force);

  grav_comp.setZero();
  for (int i = 0; i < masses_.size(); ++i)
    grav_comp += - jacobian_com[i].topLeftCorner<3,Leg::getNumJoints()>().transpose() * (gravity_vec * masses_[i]);

  return grav_comp + stance + /*dyn_comp + */ spring;
}

void Leg::updateStance(const Eigen::Vector3d& trans_vel, const Eigen::Vector3d& rotate_vel, const Eigen::VectorXd& current_angles, double dt)
{
  // Get linear velocities of stance legs based on rotational/translational
  // velocities
  stance_vel_xyz_ = trans_vel + rotate_vel.cross(cmd_stance_xyz_);

  // Update position
  cmd_stance_xyz_ += trans_vel * dt;
  cmd_stance_xyz_ = (AngleAxisd(rotate_vel(2) * dt, Eigen::Vector3d::UnitZ()) *
                     AngleAxisd(rotate_vel(1) * dt, Eigen::Vector3d::UnitY()) *
                     AngleAxisd(rotate_vel(0) * dt, Eigen::Vector3d::UnitX()) *
                     cmd_stance_xyz_).eval();

  // Update from feedback
  // kin_->getEndEffector(current_angles, end_point_frame);
  computeFK(fbk_stance_xyz_, current_angles);

  // Update home stance to match the current z height
  level_home_stance_xyz_(2) += trans_vel(2) * dt;
  home_stance_xyz_ = AngleAxisd(0.2 * trans_vel(1), Eigen::Vector3d::UnitX()) *
                     AngleAxisd(-0.2 * trans_vel(0), Eigen::Vector3d::UnitY()) *
                     level_home_stance_xyz_;
}
void Leg::computeIK(Eigen::Vector3d& angles, const Eigen::VectorXd& ee_com_pos)
{
  bool isLeft = configuration == LegConfiguration::Left;
  const double t = configuration == LegConfiguration::Left ?(4.5057/100):(-4.5057/100);

  double dZ = ee_com_pos(2) - mount_point(2);
  double dX_com = ee_com_pos(0) - mount_point(0);
  double dY_com = ee_com_pos(1) - mount_point(1);
  
  // std::cout<<dX_com<<" "<<dY_com<<" "<<dZ<<" "<<std::endl;

  double dR_com = std::sqrt(dX_com*dX_com+dY_com*dY_com);
  double dX_leg = dX_com*cos(-mount_point(3)) - dY_com*sin(-mount_point(3));
  double dY_leg = dX_com*sin(-mount_point(3)) + dY_com*cos(-mount_point(3));

  // std::cout<<dX_leg<<" "<<dY_leg<<" "<<std::endl;

  double dPHI1T = std::atan2(dY_leg,dX_leg);
  double sinPHI1 = std::max(std::min(t/dR_com, 1.), -1.);

  if(abs(sinPHI1) == 1)
      std::cout<<"capped trig for leg"<<index_<<".\n";
  
  angles(0) = asin(sinPHI1) + dPHI1T;

  double dR = (dX_leg - sin(angles(0)) * t) / cos(angles(0));

  double theta2, theta3;
  try {
    // protected divide
    double cosPHI2 = std::max(std::min((L2*L2-dR*dR-dZ*dZ-L1*L1)/(-2*L1*std::sqrt(dR*dR+dZ*dZ)), 1.), -1.);
    double cosPHI3 = std::max(std::min((dR*dR+dZ*dZ-L1*L1-L2*L2)/(-2*L1*L2), 1.), -1.);

    if(std::abs(cosPHI2) == 1 || std::abs(cosPHI3) == 1)
      std::cout<<"capped trig for leg"<<index_<<".\n";

    theta2 = acos(cosPHI2); // theta2 always >= 0
    theta3 = acos(cosPHI3); // theta3 always >= 0
  } catch(...) {
    // handle any exception
    std::cerr << "Exception when calc theta2 theta3 @ Leg" << index_ << std::endl;
  }

  int solution = 0;
  if(solution == 0){
    angles(1) = atan2(dZ,dR) + theta2;
    angles(2) = theta3 - M_PI;
  }else if(solution == 1){
    angles(1) = atan2(dZ,dR) - theta2;
    angles(2) = M_PI - theta3;
  }

  // because the installation difference between Daisy and Titan
  if(isLeft)
    angles(1) = -angles(1);
  else
    angles(2) = -angles(2);
}

void Leg::computeFK(Eigen::Vector3d& ee_com_pos, Eigen::VectorXd angles)
{


  bool isLeft = configuration == LegConfiguration::Left;
  if(isLeft)
    angles(1) = -angles(1);
  else
    angles(2) = -angles(2);
  const double t = isLeft?(4.5057/100):(-4.5057/100);

  double dR_leg = cos(angles(1))*(L1+L2*cos(angles(2))) - sin(angles(1))*L2*sin(angles(2));
  double dX_leg = cos(angles(0))*dR_leg + sin(angles(0))*t;
  double dY_leg = sin(angles(0))*dR_leg - cos(angles(0))*t;
  double dZ_leg = sin(angles(1))*(L1+L2*cos(angles(2))) + cos(angles(1))*L2*sin(angles(2));

  ee_com_pos(0) = cos(mount_point(3))*dX_leg - sin(mount_point(3))*dY_leg + mount_point(0);
  ee_com_pos(1) = sin(mount_point(3))*dX_leg + cos(mount_point(3))*dY_leg + mount_point(1);
  ee_com_pos(2) = dZ_leg + mount_point(2);
}
  
void Leg::initStance(Eigen::VectorXd& current_angles)
{
  computeFK(cmd_stance_xyz_, current_angles);
}
void Leg::startStep(double t)
{
  step_.reset(new Step(t, this)); // TODO: why not use fbk stance here?
}

void Leg::updateStep(double t)
{
  assert(step_);
  if (!step_)
    return;
  // Update, marking as complete if we finish the step.
  if (step_->update(t, this))
  {
    cmd_stance_xyz_ = step_->getTouchDown();
    step_.reset(nullptr);
  }
}

double Leg::getStepTime(double t) const
{
  assert(step_);
  if (!step_)
    return 0;
  return t - step_->getStartTime();
}

double Leg::getStepPeriod() const
{
  assert(step_);
  if (!step_)
    return 0;
  return step_->period_;
}

} // namespace hebi
