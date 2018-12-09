#include <chrono>
#include <thread>
#include <ctime>
#include <fstream>

#include "quadruped.hpp"

namespace hebi {
  std::unique_ptr<Quadruped> Quadruped::create(const QuadrupedParameters& params)
  {
    hebi::Lookup lookup;
    std::vector<std::string> names;
    for (int i = 0; i < num_legs_; ++i)
    {
      names.push_back("base" + std::to_string(i + 1));
      names.push_back("shoulder" + std::to_string(i + 1));
      names.push_back("elbow" + std::to_string(i + 1));
    }

    // temporarily still use hexapod as name 
    std::vector<std::string> family = { "hexapod" };

    long timeout_ms = 4000; // use a 4 second timeout
    auto group = lookup.getGroupFromNames(family, names, timeout_ms);
    // omitted a bunch of error checkings which are in orginial code
    if (!group)
    {
      return nullptr;
    }
    group->setCommandLifetimeMs(100);

    return std::unique_ptr<Quadruped>(new Quadruped(group, params));
  }

  Quadruped::Quadruped(std::shared_ptr<Group> group, const QuadrupedParameters& params)
  : group_(group), params_(params), cmd_(group_ ? group_->size() : 1)
  {
    Eigen::Vector3d zero_vec = Eigen::Vector3d::Zero();
    legs_.emplace_back(new QuadLeg(30.0 * M_PI / 180.0, 0.2375, zero_vec, params, 0, QuadLeg::LegConfiguration::Left));
    legs_.emplace_back(new QuadLeg(-30.0 * M_PI / 180.0, 0.2375, zero_vec, params, 1, QuadLeg::LegConfiguration::Right));
    legs_.emplace_back(new QuadLeg(90.0 * M_PI / 180.0, 0.1875, zero_vec, params, 2, QuadLeg::LegConfiguration::Left));
    legs_.emplace_back(new QuadLeg(-90.0 * M_PI / 180.0, 0.1875, zero_vec, params, 3, QuadLeg::LegConfiguration::Right));
    legs_.emplace_back(new QuadLeg(150.0 * M_PI / 180.0, 0.2375, zero_vec, params, 4, QuadLeg::LegConfiguration::Left));
    legs_.emplace_back(new QuadLeg(-150.0 * M_PI / 180.0, 0.2375, zero_vec, params, 5, QuadLeg::LegConfiguration::Right));

    // This looks like black magic to me
    if (group_)
    {
      group_->addFeedbackHandler([this] (const GroupFeedback& fbk)
      {
        // FBK 1: get gravity direction
        // Some assistant variables calcuate needed physical quantities
        // A -z vector in a local frame.
        Eigen::Vector3d down(0, 0, -1);
        Eigen::Vector3d avg_grav;
        avg_grav.setZero();

        std::lock_guard<std::mutex> guard(fbk_lock_);
        latest_fbk_time = std::chrono::steady_clock::now();
        assert(fbk.size() == num_joints_);
        for (int i = 0; i < num_legs_; ++i)
        {
          // HEBI Quaternion
          auto mod_orientation = fbk[i * num_joints_per_leg_]
            .imu().orientation().get();
          // Eigen Quaternion
          Eigen::Quaterniond mod_orientation_eig(
            mod_orientation.getW(),
            mod_orientation.getX(),
            mod_orientation.getY(),
            mod_orientation.getZ());
          Eigen::Matrix3d mod_orientation_mat = mod_orientation_eig.toRotationMatrix();

          // Transform
          Eigen::Matrix4d trans = legs_[i]->getKinematics().getBaseFrame();
          Eigen::Vector3d my_grav = trans.topLeftCorner<3,3>() * mod_orientation_mat.transpose() * down;
          // If one of the modules isn't reporting valid feedback, ignore this:
          if (!std::isnan(my_grav[0]) && !std::isnan(my_grav[1]) && !std::isnan(my_grav[2]))
            avg_grav += my_grav;
        }

        // Average the feedback from various modules and normalize.
        avg_grav.normalize();
        {
          std::lock_guard<std::mutex> lg(grav_lock_);
          gravity_direction_ = avg_grav;
        }

        // FBK 2 read fbk positions to legs
        for (int i = 0; i < num_legs_; ++i)
        {
          Eigen::VectorXd pos_vec = Eigen::VectorXd::Zero(num_joints_per_leg_);
          for (int j = 0; j < num_joints_per_leg_; ++j)
          {
            auto& pos = fbk[i*num_joints_per_leg_+j].actuator().position();
            if (pos)
            {
              pos_vec(j) = pos.get();
            }
            else
            {
              pos_vec(j) = std::numeric_limits<double>::quiet_NaN();
            }
          }
          legs_[i]->setJointAngles(pos_vec);
        }

      });
      group_->setFeedbackFrequencyHz(fbk_frq_hz_); 
    }
  }

  Quadruped::~Quadruped()
  {
    if (group_)
    {
      group_->setFeedbackFrequencyHz(0);
      group_->clearFeedbackHandlers();
    }
  }

  Eigen::Vector3d Quadruped::getGravityDirection()
  {
    std::lock_guard<std::mutex> lg(grav_lock_);
    return gravity_direction_;
  }

  Eigen::VectorXd Quadruped::getLegJointAngles(int index)
  {
    return legs_[index]->getJointAngle();
  }

  bool Quadruped::planStandUpTraj(double duration_time)
  {
    // this is a hexapod movement ...
    for (int i = 0; i < num_legs_; ++i)
    {
      Eigen::VectorXd leg_start = getLegJointAngles(i);
      Eigen::VectorXd leg_end;
      

      auto base_frame = legs_[i] -> getBaseFrame();
      Eigen::Vector4d tmp4(0.55, 0, -0.21, 0); // hard code first
      Eigen::VectorXd home_stance_xyz = (base_frame * tmp4).topLeftCorner<3,1>();
      legs_[i]->computeIK(leg_end, home_stance_xyz);
      // TODO: fix! (quick and dirty -- leg mid is hardcoded as offset from leg end)
      Eigen::VectorXd leg_mid = leg_end;
      leg_mid(1) -= 0.3;
      leg_mid(2) -= 0.15;

      // Convert for trajectories
      int num_waypoints = 5;
      Eigen::MatrixXd positions(num_joints_per_leg_, num_waypoints);
      Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_joints_per_leg_, num_waypoints);
      Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_joints_per_leg_, num_waypoints);
      Eigen::VectorXd nan_column = Eigen::VectorXd::Constant(num_joints_per_leg_, std::numeric_limits<double>::quiet_NaN());
      // Is this one of the legs that takes a step first?
      bool step_first = (i == 0 || i == 3 || i == 4);

      // Set positions
      positions.col(0) = leg_start;
      positions.col(1) = step_first ? leg_mid : leg_start;
      positions.col(2) = step_first ? leg_end : leg_start;
      positions.col(3) = step_first ? leg_end : leg_mid;
      positions.col(4) = leg_end;

      velocities.col(1) = nan_column;
      velocities.col(3) = nan_column;
      accelerations.col(1) = nan_column;
      accelerations.col(3) = nan_column;

      Eigen::VectorXd times(num_waypoints);
      times << 0,
              0 + duration_time * 0.25,
              0 + duration_time * 0.5,
              0 + duration_time * 0.75,
              0 + duration_time;
      startup_trajectories.push_back(trajectory::Trajectory::createUnconstrainedQp(
        times, positions, &velocities, &accelerations));
    }
  }

  bool Quadruped::execStandUpTraj(double curr_time)
  {
    // Controls to send to the robot
    Eigen::VectorXd angles(num_joints_per_leg_);
    Eigen::VectorXd vels(num_joints_per_leg_);
    Eigen::VectorXd torques(num_joints_per_leg_); 
    for (int i = 0; i < num_legs_; ++i)
    {
      Eigen::VectorXd a(num_joints_per_leg_); // do not use acceleration
      startup_trajectories[i]->getState(curr_time, &angles, &vels, &a);

      Eigen::Vector3d gravity_vec = getGravityDirection() * 9.8f;
      Eigen::Vector3d foot_force = -weight_ * getGravityDirection() / 6.0f;
      torques = legs_[i]-> computeCompensateTorques(angles, vels, gravity_vec, foot_force); 

      setCommand(i, &angles, &vels, &torques);
    }
    sendCommand();
  }

  void Quadruped::setCommand(int index, const VectorXd* angles, const VectorXd* vels, const VectorXd* torques)
  {
    int leg_offset = index * num_joints_per_leg_;
    // I think do not need to check the size of the code
    if (angles != nullptr)
    {
      assert(angles->size() == num_joints_per_leg_);
      for (int i = 0; i < num_joints_per_leg_; ++i)
        cmd_[leg_offset + i].actuator().position().set((*angles)[i]);
    }
    if (vels != nullptr)
    {
      assert(vels->size() == num_joints_per_leg_);
      for (int i = 0; i < num_joints_per_leg_; ++i)
        cmd_[leg_offset + i].actuator().velocity().set((*vels)[i]);
    }
    if (torques != nullptr)
    {
      assert(torques->size() == num_joints_per_leg_);
      for (int i = 0; i < num_joints_per_leg_; ++i)
        cmd_[leg_offset + i].actuator().effort().set((*torques)[i]);
    }
  }

  void Quadruped::sendCommand()
  {
    if (group_)
      group_->sendCommand(cmd_);
  }

} // namespace hebi
