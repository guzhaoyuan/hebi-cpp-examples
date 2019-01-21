#include <memory>
#include <rbdl/rbdl.h>
#include "robot_model.hpp"
#include "step.hpp"
#include "hexapod_parameters.hpp"


namespace hebi {

class Leg
{
public:
  enum Mode { Stance, Flight };
  enum class LegConfiguration { Left, Right };

  int index_;

  // Angle and distance from the center of the parent creature.
  Leg(double angle_rad, double distance, const Eigen::VectorXd& current_angles, const Eigen::Vector4d& _mount_point,  const HexapodParameters& params, bool is_dummy, int index, LegConfiguration _configuration);

  // Compute jacobian given position and velocities.  Usually, this is done internally
  // int `computeState`, but if the position/velocity is known (e.g., external
  // step control), this can be used to get these jacobians from the internal
  // kinematics object.
  bool computeJacobians(const Eigen::VectorXd& angles, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com);
  // TODO: return value?  What if IK fails?
  bool computeState(double t, Eigen::VectorXd& angles, Eigen::VectorXd& vels, Eigen::MatrixXd& jacobian_ee, robot_model::MatrixXdVector& jacobian_com);

  // TODO: combine with above computeState?
  // TODO: pass in torques as reference instead for consistency?
  Eigen::VectorXd computeTorques(const robot_model::MatrixXdVector& jacobian_com, const Eigen::MatrixXd& jacobian_ee, const Eigen::VectorXd& angles, const Eigen::VectorXd& vels, const Eigen::Vector3d& gravity_vec, const Eigen::Vector3d& foot_force);

  static constexpr int getNumJoints() { return num_joints_; };
  void initStance(Eigen::VectorXd& current_angles);
  void updateStance(const Eigen::Vector3d& trans_vel, const Eigen::Vector3d& rotate_vel, const Eigen::VectorXd& current_angles, double dt);

  void computeIK(Eigen::Vector3d& angles, const Eigen::VectorXd& ee_com_pos);
  void computeFK(Eigen::Vector3d& ee_com_pos, Eigen::VectorXd angles);

  // inverse dynamics 
  void getInverseDynamics(Eigen::VectorXd& theta_d, Eigen::VectorXd& dtheta_d, Eigen::VectorXd& ddtheta_d, Eigen::VectorXd& tau);

  const double getLevelHomeStanceZ() const { return level_home_stance_xyz_(2); }
  const Eigen::Vector3d& getHomeStanceXYZ() const { return home_stance_xyz_; }
  const Eigen::Vector3d& getCmdStanceXYZ() const { return cmd_stance_xyz_; }
  const Eigen::Vector3d& getFbkStanceXYZ() const { return fbk_stance_xyz_; }
  const Eigen::Vector3d& getStanceVelXYZ() const { return stance_vel_xyz_; }

  // NOTE: useful for jumping straight to home stance, e.g. for visualization
  // of stance parameters
  void setCmdStanceToHomeStance() { cmd_stance_xyz_ = home_stance_xyz_; }

  // TODO: think about where this should really be
  const Eigen::VectorXd& getSeedAngles() const { return seed_angles_; }

  // TODO: think about const for this, and other accessor functions for actually
  // getting info from inside
  hebi::robot_model::RobotModel& getKinematics() { return *kin_; }
  const hebi::robot_model::RobotModel& getKinematics() const { return *kin_; }

  // Am I actively stepping?
  Mode getMode() { return (step_) ? Mode::Flight : Mode::Stance; }

  void startStep(double t);
  void updateStep(double t);
  double getStepTime(double t) const;
  double getStepPeriod() const;

private:

  LegConfiguration configuration;
  static constexpr int num_joints_ = 3;
  float stance_radius_; // [m]
  float body_height_; // [m]
  const float spring_shift_; // [N*m] compensate for the spring torques
  Eigen::VectorXd seed_angles_;
  // TODO: leg should hold fbk angles 
  std::unique_ptr<Step> step_;

  Eigen::Vector3d home_stance_xyz_;
  Eigen::Vector3d level_home_stance_xyz_;
  Eigen::Vector3d fbk_stance_xyz_;
  Eigen::Vector3d cmd_stance_xyz_;
  Eigen::Vector3d stance_vel_xyz_;
  
  Eigen::Vector4d mount_point;
  const double L1 = 0.2795;
  const double L2 = 0.272;
  const double left_tran =  0.0425;
  const double left_d1Z = 0.07105;  // from base center to first joint, distance Z

  std::unique_ptr<hebi::robot_model::RobotModel> kin_;
  // Note -- one mass element for each COM frame in the kinematics!
  Eigen::VectorXd masses_;

  // rbdl element
  RigidBodyDynamics::Model* model;  
  unsigned int body_base_id, body_1_id, body_2_id, body_3_id;
  RigidBodyDynamics::Body body_base, body_1, body_2, body_3;
  RigidBodyDynamics::Joint joint_float, joint_1, joint_2, joint_3;

  // Allow Eigen member variables:
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace hebi
