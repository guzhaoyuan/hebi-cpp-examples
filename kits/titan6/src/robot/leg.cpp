#include "leg.hpp"
#include "../util/modern_robotics.hpp"
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
  cmd_stance_xyz_ = home_stance_xyz_;
  stance_vel_xyz_ = Vector3d(0,0,0);

  // build rbdl model 
  model = new RigidBodyDynamics::Model();

  model -> gravity = Vector3d(0, 0, -9.81); // use chassis body frame direction direction
  // all vectors below are represented in leg base frame!!!
  if (configuration == LegConfiguration::Left)
  {
    // joint float (at the center of the base) -> base -> joint1 -> link1 -> joint2 -> link2 -> joint3 -> link3
    joint_float = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);
    RigidBodyDynamics::Math::Matrix3d I_base;
    I_base << 208188*10e-9,	 33307  *10e-9,	    -262     *10e-9,
               33307*10e-9,	 374826 *10e-9,	    -315     *10e-9,
                -262*10e-9,	  -315   *10e-9,	    508355 *10e-9;   
    body_base = RigidBodyDynamics::Body( 0.5,                                // mass
                      RigidBodyDynamics::Math::Vector3d(-15*10e-3, -1.77*10e-3, 0),  // COM
                      I_base);                                  // inertia
    // 0 is the floating base
    body_base_id = model -> AddBody(0, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,0)), joint_float, body_base);

    joint_1 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,0,1));
    RigidBodyDynamics::Math::Matrix3d I1;
    I1 << 534850*10e-9,	 -37640*10e-9,	    -73232*10e-9,
          -37640*10e-9,	 866578*10e-9,	    3857*10e-9,
          -73232*10e-9,	 3857*10e-9,	    537262*10e-9;
    body_1 = RigidBodyDynamics::Body( 0.688,                                // mass
                      RigidBodyDynamics::Math::Vector3d(-9.79*10e-3, -10.89*10e-3, 44.72*10e-3),  // COM
                      I1);// inertia
    body_1_id = model -> AddBody(body_base_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,16*10e-3)), joint_1, body_1);
                                  
    

    joint_2 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,1,0));
    RigidBodyDynamics::Math::Matrix3d I2;
    I2 << 503535*10e-9,	-809250*10e-9,	    13544*10e-9,
	       -809250*10e-9,	10025568*10e-9,	    10176*10e-9,
	           13544*10e-9,	    10176*10e-9,	10027324*10e-9;
    body_2 = RigidBodyDynamics::Body( 0.812, 
                      RigidBodyDynamics::Math::Vector3d(258.70*10e-3, -3.52*10e-3, -1.11*10e-3), 
                      I2);
    body_2_id = model -> AddBody(body_1_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,-8.55*10e-3,55*10e-3)), joint_2, body_2);
     
    joint_3 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,-1,0));
    RigidBodyDynamics::Math::Matrix3d I3;
    I3 <<      75404*10e-9,	     3.31*10e-9,	    80356*10e-9,
	            3.31*10e-9,	    3483112*10e-9,	    0.97*10e-9,
	            80356*10e-9,	      0.97*10e-9,	  3492631*10e-9;
    body_3 = RigidBodyDynamics::Body( 0.23198, 
                      RigidBodyDynamics::Math::Vector3d(123.37*10e-3, 0*10e-3, 17.23*10e-3), 
                      I3);
    body_3_id = model -> AddBody(body_2_id, RigidBodyDynamics::Math::Xtrans(Vector3d(325.5*10e-3,-31.05*10e-3,0*10e-3)), joint_3, body_3);
  }
  else
  {
    // joint float (at the center of the base) -> base -> joint1 -> link1 -> joint2 -> link2 -> joint3 -> link3
    joint_float = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeFixed);
    RigidBodyDynamics::Math::Matrix3d I_base;
    I_base << 208188*10e-9,	 33307  *10e-9,	    -262     *10e-9,
               33307*10e-9,	 374826 *10e-9,	    -315     *10e-9,
                -262*10e-9,	  -315   *10e-9,	    508355 *10e-9;   
    body_base = RigidBodyDynamics::Body( 0.5,                                // mass
                      RigidBodyDynamics::Math::Vector3d(-15*10e-3, -1.77*10e-3, 0),  // COM
                      I_base);                                  // inertia
    // 0 is the floating base
    body_base_id = model -> AddBody(0, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,0)), joint_float, body_base);

    joint_1 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,0,1));
    RigidBodyDynamics::Math::Matrix3d I1;
    I1 << 511529*10e-9,	 38110*10e-9,	    1669*10e-9,
          38110*10e-9,	 843738*10e-9,	    -2817*10e-9,
          1669*10e-9,	 -2817*10e-9,	    537743*10e-9;
    body_1 = RigidBodyDynamics::Body( 0.688,                                // mass
                      RigidBodyDynamics::Math::Vector3d(-9.75*10e-3, 10.89*10e-3, 42.14*10e-3),  // COM
                      I1);// inertia
    body_1_id = model -> AddBody(body_base_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,0,16*10e-3)), joint_1, body_1);
                                  
    

    joint_2 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,-1,0));
    RigidBodyDynamics::Math::Matrix3d I2;
    I2 << 503535*10e-9,	809250*10e-9,	    13544*10e-9,
	       809250*10e-9,	10025568*10e-9,	    10176*10e-9,
	          13544*10e-9,	    10176*10e-9,	    10025568*10e-9;
    body_2 = RigidBodyDynamics::Body( 0.81278, 
                      RigidBodyDynamics::Math::Vector3d(258.70*10e-3, 3.52*10e-3, 1.13*10e-3), 
                      I2);
    body_2_id = model -> AddBody(body_1_id, RigidBodyDynamics::Math::Xtrans(Vector3d(0,-8.55*10e-3,55*10e-3)), joint_2, body_2);
     
    joint_3 = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, RigidBodyDynamics::Math::Vector3d(0,1,0));
    RigidBodyDynamics::Math::Matrix3d I3;
    I3 <<      75404*10e-9,	     -3.31*10e-9,	    -80356*10e-9,
	            -3.31*10e-9,	    3483112*10e-9,	    0.97*10e-9,
	            -80356*10e-9,	      0.97*10e-9,	  3492631*10e-9;
    body_3 = RigidBodyDynamics::Body( 0.23198, 
                      RigidBodyDynamics::Math::Vector3d(123.37*10e-3, 0*10e-3, -17.23*10e-3), 
                      I3);
    body_3_id = model -> AddBody(body_2_id, RigidBodyDynamics::Math::Xtrans(Vector3d(325.5*10e-3, 31.05*10e-3, 0*10e-3)), joint_3, body_3);
  }
  // construct twist list
  bool isLeft = configuration == LegConfiguration::Left;
  const double t = isLeft?(4.5057/100):(-4.5057/100);
  // get body twist list first
  int num_angles = current_angles.size();
  b_list = Eigen::MatrixXd(6, num_angles); // body twist list
  Eigen::Vector3d w(0,0,1);
  Eigen::Vector3d q(-L1-L2,t,0);
  Eigen::VectorXd twist(6);
  twist.segment<3>(0) = w;
  twist.segment<3>(3) = -w.cross(q);
  b_list.col(0) <<  twist;                 // joint1


  w = isLeft?Eigen::Vector3d(0,1,0):Eigen::Vector3d(0,-1,0);
  q = Eigen::Vector3d(-L1-L2,t,0);
  twist.segment<3>(0) = w;
  twist.segment<3>(3) = -w.cross(q);
  b_list.col(1) <<  twist;                //  joint2

  w = isLeft?Eigen::Vector3d(0,-1,0):Eigen::Vector3d(0,1,0);
  q = Eigen::Vector3d(-L2,t,0);
  twist.segment<3>(0) = w;
  twist.segment<3>(3) = -w.cross(q);
  b_list.col(2) <<  twist;                //  joint3

  // get spatial twist list 
  s_list = Eigen::MatrixXd(6, num_angles); // spatial twist list
  w = Eigen::Vector3d(0,0,1);
  q = Eigen::Vector3d(0,0,0);
  twist.segment<3>(0) = w;
  twist.segment<3>(3) = -w.cross(q);
  s_list.col(0) <<  twist;                 // joint1


  w = isLeft?Eigen::Vector3d(0,1,0):Eigen::Vector3d(0,-1,0);
  q = Eigen::Vector3d(0,0,d1Z);
  twist.segment<3>(0) = w;
  twist.segment<3>(3) = -w.cross(q);
  s_list.col(1) <<  twist;                //  joint2

  w = isLeft?Eigen::Vector3d(0,-1,0):Eigen::Vector3d(0,1,0);
  q = Eigen::Vector3d(L1,0,d1Z);
  twist.segment<3>(0) = w;
  twist.segment<3>(3) = -w.cross(q);
  s_list.col(2) <<  twist;                //  joint3

}

// Compute jacobian given position and velocities
bool Leg::computeJacobians(const Eigen::VectorXd& angles, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com)
{
  kin_->getJEndEffector(angles, jacobian_ee);
  kin_->getJ(HebiFrameTypeCenterOfMass, angles, jacobian_com);
}

// compute body jacobian use modern robotics lib
Eigen::MatrixXd Leg::computerJacobianBody(const Eigen::VectorXd& angles)
{   
 return mr::JacobianBody(b_list, angles);
}
  
Eigen::MatrixXd Leg::computerJacobianSpatial(const Eigen::VectorXd& angles)
{   
 return mr::JacobianSpace(s_list, angles);
}
  

// compute FK transformation matrix use modern robotics lib. 
// compare to computeFK function, this function gives rotation of the robot as well
Eigen::MatrixXd Leg::computerEEFKInSpace(const Eigen::VectorXd& angles)
{
  bool isLeft = configuration == LegConfiguration::Left;
  const double t = isLeft?(4.5057/100):(-4.5057/100);

  // std::cout <<"reach " << __FILE__ << __LINE__ << std::endl;
  // std::cout << s_list << std::endl;
  Eigen::MatrixXd g_s0(4,4); /// transformation between center of the base link and the end effector
  g_s0 << 1, 0, 0, L1+L2,
          0, 1, 0,    -t,
          0, 0, 1,   d1Z,
          0, 0, 0,     1;

  return mr::FKinSpace(g_s0, s_list, angles);
}
 
bool Leg::computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::VectorXd& accels, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com)
{
  // TODO: think about returning an error value, e.g., when IK fails?
  // TODO: add torque!
  angles.resize(num_joints_);
  vels.resize(num_joints_);
  if (step_) // Step
  {
    step_->computeState(t, angles, vels, accels);
    computeJacobians(angles, jacobian_ee, jacobian_com);
  }
  else // Stance
  {
    // auto res = kin_->solveIK(
    //   seed_angles_,
    //   angles,
    //   robot_model::EndEffectorPositionObjective(cmd_stance_xyz_));
    // if (res.result != HebiStatusSuccess)
    // {
    //   return false;
    // }
    Vector3d myangle;
    computeIK(myangle, cmd_stance_xyz_);
    angles = myangle;

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

void Leg::getInverseDynamics(Eigen::VectorXd& theta_d, Eigen::VectorXd& dtheta_d, Eigen::VectorXd& ddtheta_d, Eigen::VectorXd& tau)
{
  RigidBodyDynamics::InverseDynamics(*model, theta_d, dtheta_d, ddtheta_d, tau);
}

void Leg::setDynamicsGravity(Eigen::VectorXd& gravity_vec)
{
  model -> gravity << gravity_vec;
}

void Leg::initStance(Eigen::VectorXd& current_angles)
{
  cmd_stance_xyz_ = home_stance_xyz_;
  //computeFK(cmd_stance_xyz_, current_angles);
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
