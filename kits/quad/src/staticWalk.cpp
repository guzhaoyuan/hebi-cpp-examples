#include <atomic>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <set>

#include "input/input_manager_mobile_io.hpp"
#include "robot/quadruped_parameters.hpp"
#include "robot/quadruped.hpp"

using namespace hebi;
using namespace Eigen;

/* state machine related needed variables */
// state definitions
enum ctrl_state_type {
  QUAD_CTRL_STAND_UP1,
  QUAD_CTRL_STAND_UP2,
  QUAD_CTRL_STAND_UP3,

  QUAD_STATIC_WALK,         // <- entry, move base and move four legs seperately
  
  CTRL_STATES_COUNT
};

enum static_walk_state {
  START,
  MOVE_BASE1,
  MOVE_LEG1,
  MOVE_BASE2,
  MOVE_LEG2,
  MOVE_BASE3,
  MOVE_LEG3,
  MOVE_BASE4,
  MOVE_LEG4,
  MOVE_BASE5,
  FINISH
};

// state transition variables
double startup_seconds = 1.9;
double re_pose_seconds = 3;

int main(int argc, char** argv)
{
  // INIT VARS
  bool is_quiet{}; // test where some steps go run or not

  // INIT STEP 1: init parameters (it is empty now, not used)
  QuadrupedParameters params;
  params.resetToDefaults();

  // INIT STEP 2: init input
  std::unique_ptr<input::InputManager> input(new input::InputManagerMobileIO());
  while (!input->isConnected())
  {
    static_cast<input::InputManagerMobileIO*>(input.get())->reset();
  }

  std::cout << "Found input joystick -- starting control program.\n";
  // INIT STEP 3: init robot planner
  std::unique_ptr<Quadruped> quadruped = Quadruped::create(params);
  quadruped -> setGains();

  // INIT STEP FINAL: start control state machine
  // input command from joystick (hebi's input manager use vector3f, i think use vector3d would be better)
  Eigen::Vector3f translation_velocity_cmd;
  translation_velocity_cmd.setZero();
  Eigen::Vector3f rotation_velocity_cmd;
  rotation_velocity_cmd.setZero();   


  // START CONTROL THREAD: this is so cool
  auto start_time = std::chrono::steady_clock::now();
  // some time for state use
  auto state_enter_time = std::chrono::steady_clock::now(); 
  auto state_curr_time = std::chrono::steady_clock::now(); 
  std::chrono::duration<double> state_run_time = std::chrono::seconds(0);
  long interval_ms = 5.0; // in milliseconds; e.g., 5 ms => 1000/5 = 200 Hz
  // http://stackoverflow.com/questions/30425772/c-11-calling-a-c-function-periodically
  std::atomic<bool> control_execute;
  control_execute.store(true, std::memory_order_release);
  int move_leg_id;

  // the main control thread
  ctrl_state_type cur_ctrl_state = QUAD_CTRL_STAND_UP1;
  static_walk_state cur_walk_state = START;
  // ctrl_state_type cur_ctrl_state = QUAD_CTRL_ORIENT;  // save some energy 
  auto prev_time = std::chrono::steady_clock::now();
  // Get dt (in seconds)
  std::chrono::duration<double> dt = std::chrono::seconds(0);


  // variables used in normal run test
  Eigen::Vector3d grav_vec;
  Eigen::VectorXd leg_angles;
  double leg_swing_time = 3;   // need to be tested
  double body_move_time = 2;
  bool isFinished;
  // some variables used in state passive_orient
  Eigen::Matrix3d bodyR, balance_body_R, diff_body_R;
  Eigen::Vector3d euler;
  Eigen::Quaterniond q_error;
  Eigen::Matrix3d control_R = Eigen::Matrix3d::Identity();
  Eigen::AngleAxisd tmp_aa;
  double new_angle;
  Eigen::Vector3d axis_aa;
  // used in rePose and extend
  double tgt_x, tgt_y;
  bool first_time_enter = false;

  while (control_execute.load(std::memory_order_acquire)) // I have no idea why this not working after remove Q application
  // while(true)
  {    
    // Wait!
    auto now_time = std::chrono::steady_clock::now();
    auto need_to_wait = std::max(0, (int)std::chrono::duration_cast<std::chrono::milliseconds>(prev_time + std::chrono::milliseconds(interval_ms) - now_time).count());
    std::this_thread::sleep_for(std::chrono::milliseconds(need_to_wait));

    // Get dt (in seconds)
    now_time = std::chrono::steady_clock::now();
    dt = std::chrono::duration_cast<std::chrono::duration<double>>(now_time - prev_time);
    prev_time = now_time;

    std::chrono::duration<double> elapsed_time(now_time - start_time);

    // Get joystick update, and update any relevant variables.
    input->update();
    if (input->getQuitButtonPushed())
    {
      std::cout << "program exit" << std::endl;
      std::terminate();
    }
    translation_velocity_cmd = input->getTranslationVelocityCmd();
    rotation_velocity_cmd = input->getRotationVelocityCmd();
    Eigen::Matrix3d target_body_R;
    
    // control state machine 
    // std::cout << "|Time: " << elapsed_time.count() <<  "| my current state is: " << cur_ctrl_state <<std::endl;
    switch (cur_ctrl_state)
    {

      // let me write a standup strategy myself
      // it takes three step, first spread legs to let belly touch the ground, then push legs to lift the body
      // finally lift the two arms 
      case QUAD_CTRL_STAND_UP1:
      {
        state_curr_time = std::chrono::steady_clock::now();
        state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

        isFinished = quadruped -> spreadAllLegs();
        if (state_run_time.count() >= startup_seconds)
        {
          cur_ctrl_state = QUAD_CTRL_STAND_UP2;
          state_enter_time = std::chrono::steady_clock::now(); 
        }
        break;
      }
      case QUAD_CTRL_STAND_UP2:
      {
        state_curr_time = std::chrono::steady_clock::now();
        state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

        isFinished = quadruped -> pushAllLegs(state_run_time.count(), startup_seconds);
        if (state_run_time.count() >= startup_seconds)
        {
          quadruped -> startBodyRUpdate();
          cur_ctrl_state = QUAD_STATIC_WALK;
          state_enter_time = std::chrono::steady_clock::now(); 
        }
        break;
      }

      case QUAD_CTRL_STAND_UP3:
      {
        state_curr_time = std::chrono::steady_clock::now();
        state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

        isFinished = quadruped -> prepareQuadMode();
        if (state_run_time.count() >= startup_seconds)
        {
          // the entry to some final state, either running or rotating
          cur_ctrl_state = QUAD_STATIC_WALK;
          balance_body_R = quadruped -> getBodyR();
          quadruped -> saveFootPose();
          quadruped -> setStartGait();
          state_enter_time = std::chrono::steady_clock::now(); 
          //quadruped -> prepareTrajectories(Quadruped::SwingMode::swing_mode_virtualLeg1, leg_swing_time);
        }
        break;
      }

      // move base and move four legs seperately
      case QUAD_STATIC_WALK:
      {
        bool ready = true;

        switch (cur_walk_state){
          case START:
            while(!ready){
              // std::cout << "the robot is not ready for start gait\n";
              quadruped -> freeze();
              // std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            quadruped -> freeze();
            cur_walk_state = MOVE_BASE1;
            first_time_enter = true;
            break;

          case MOVE_BASE1:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            if(first_time_enter){
              std::cout<< "enter MOVE_BASE1\n";
              quadruped -> planBodyTraj(10, 4, body_move_time);
              first_time_enter = false;
            }
            quadruped -> followBodyTraj(state_run_time.count());
            if (state_run_time.count() >= body_move_time)
            {
              cur_walk_state = MOVE_LEG1; 
              state_enter_time = std::chrono::steady_clock::now(); 
              first_time_enter = true;
            }
            break;

          case MOVE_LEG1:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            move_leg_id = 5;

            if(first_time_enter){
              std::cout<< "enter MOVE_LEG1\n";
              quadruped -> planFootTraj(move_leg_id, 20, 0, 15, leg_swing_time); // make sure only plan only, the plan is based on current position
              first_time_enter = false;
            }
            quadruped -> followFootTraj(move_leg_id, state_run_time.count());
            if (state_run_time.count() >= leg_swing_time)
            {
              cur_walk_state = MOVE_BASE2;
              state_enter_time = std::chrono::steady_clock::now();
              first_time_enter = true;
            }
            break;

          case MOVE_BASE2:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            if(first_time_enter){
              std::cout<< "enter MOVE_BASE2\n";
              quadruped -> planBodyTraj(-10, 0, body_move_time);
              first_time_enter = false;
            }
            quadruped -> followBodyTraj(state_run_time.count());
            if (state_run_time.count() >= body_move_time)
            {
              cur_walk_state = MOVE_LEG2; 
              state_enter_time = std::chrono::steady_clock::now(); 
              first_time_enter = true;
            }
            break;

          case MOVE_LEG2:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            move_leg_id = 1;

            if(first_time_enter){
              std::cout<< "enter MOVE_LEG2\n";
              quadruped -> planFootTraj(move_leg_id, 20, 0, 15, leg_swing_time); // make sure only plan only, the plan is based on current position
              first_time_enter = false;
            }
            quadruped -> followFootTraj(move_leg_id, state_run_time.count());
            if (state_run_time.count() >= leg_swing_time)
            {
              cur_walk_state = MOVE_BASE3;
              state_enter_time = std::chrono::steady_clock::now();
              first_time_enter = true;
            }
            break;

          case MOVE_BASE3:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            if(first_time_enter){
              std::cout<< "enter MOVE_BASE3\n";
              quadruped -> planBodyTraj(15, -8, body_move_time);
              first_time_enter = false;
            }
            quadruped -> followBodyTraj(state_run_time.count());
            if (state_run_time.count() >= body_move_time)
            {
              cur_walk_state = MOVE_LEG3; 
              state_enter_time = std::chrono::steady_clock::now(); 
              first_time_enter = true;
            }
            break;

          case MOVE_LEG3:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            move_leg_id = 4;

            if(first_time_enter){
              std::cout<< "enter MOVE_LEG3\n";
              quadruped -> planFootTraj(move_leg_id, 20, 0, 15, leg_swing_time); // make sure only plan only, the plan is based on current position
              first_time_enter = false;
            }
            quadruped -> followFootTraj(move_leg_id, state_run_time.count());
            if (state_run_time.count() >= leg_swing_time)
            {
              cur_walk_state = MOVE_BASE4;
              state_enter_time = std::chrono::steady_clock::now();
              first_time_enter = true;
            }
            break;

          case MOVE_BASE4:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            if(first_time_enter){
              std::cout<< "enter MOVE_BASE4\n";
              quadruped -> planBodyTraj(-5, -2, body_move_time);
              first_time_enter = false;
            }
            quadruped -> followBodyTraj(state_run_time.count());
            if (state_run_time.count() >= body_move_time)
            {
              cur_walk_state = MOVE_LEG4; 
              state_enter_time = std::chrono::steady_clock::now(); 
              first_time_enter = true;
            }
            break;

          case MOVE_LEG4:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            move_leg_id = 0;

            if(first_time_enter){
              std::cout<< "enter MOVE_LEG4\n";
              quadruped -> planFootTraj(move_leg_id, 20, 0, 15, leg_swing_time); // make sure only plan only, the plan is based on current position
              first_time_enter = false;
            }
            quadruped -> followFootTraj(move_leg_id, state_run_time.count());
            if (state_run_time.count() >= leg_swing_time)
            {
              cur_walk_state = MOVE_BASE5;
              state_enter_time = std::chrono::steady_clock::now();
              first_time_enter = true;
            }
            break;

          case MOVE_BASE5:
            state_curr_time = std::chrono::steady_clock::now();
            state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

            if(first_time_enter){
              std::cout<< "enter MOVE_BASE5\n";
              quadruped -> planBodyTraj(10, 6, body_move_time);
              first_time_enter = false;
            }
            quadruped -> followBodyTraj(state_run_time.count());
            if (state_run_time.count() >= body_move_time)
            {
              cur_walk_state = FINISH; 
              state_enter_time = std::chrono::steady_clock::now(); 
              first_time_enter = true;
            }
            break;

          case FINISH:
          default:
            quadruped -> freeze();
            break;
        }
          
        break;
      }


      case CTRL_STATES_COUNT:
      default:
        break;

    }
  }
  
  // control_execute.store(false, std::memory_order_release);
  // control_thread.join();
  return 0;
}