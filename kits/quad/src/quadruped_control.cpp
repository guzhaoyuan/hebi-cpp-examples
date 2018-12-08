#include <atomic>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <set>

#include <QtWidgets/QApplication>

#include "input/input_manager_mobile_io.hpp"
#include "robot/quadruped_parameters.hpp"

using namespace hebi;
using namespace Eigen;

/* state machine related needed variables */
// state definitions
enum ctrl_state_type {
  CTRL_START_UP,
  CTRL_NORMAL,
  CTRL_STATES_COUNT
};
// state transition variables
double startup_seconds = 4.5;

int main(int argc, char** argv)
{
  // hexapod initially start some Qt object, it seems it is just used as
  QApplication app(argc, argv);

  // INIT VARS
  bool is_quiet{}; // test where some steps go run or not

  // INIT STEP 1: init parameters (use default parameters)
  QuadrupedParameters params;
  params.resetToDefaults();

  // INIT STEP 2: init input
  std::unique_ptr<input::InputManager> input(new input::InputManagerMobileIO());
  if (!is_quiet && !input->isConnected())
  {
      std::cout << "Could not find input joystick." << std::endl;
      return 1;
  }
  // ------------ Retry a "reset" multiple times! Wait for this in a loop.
  while (is_quiet && !input->isConnected())
  {
      static_cast<input::InputManagerMobileIO*>(input.get())->reset();
  }

  std::cout << "Found input joystick -- starting control program.\n";

  // INIT STEP FINAL: start control state machine
  // input command from joystick (hebi's input manager use vector3f, i think use vector3d would be better)
  Eigen::Vector3f translation_velocity_cmd;
  translation_velocity_cmd.setZero();
  Eigen::Vector3f rotation_velocity_cmd;
  rotation_velocity_cmd.setZero();   
  // Controls to send to the robot
  // Eigen::VectorXd angles(Leg::getNumJoints());
  // Eigen::VectorXd vels(Leg::getNumJoints());
  // Eigen::VectorXd torques(Leg::getNumJoints()); 

  // START CONTROL THREAD: this is so cool
  auto start_time = std::chrono::steady_clock::now();
  long interval_ms = 5.0; // in milliseconds; e.g., 5 ms => 1000/5 = 200 Hz
  // http://stackoverflow.com/questions/30425772/c-11-calling-a-c-function-periodically
  std::atomic<bool> control_execute;
  control_execute.store(true, std::memory_order_release);
  std::thread control_thread([&]()
  {
    // the main control thread
    ctrl_state_type cur_ctrl_state = CTRL_START_UP;
    auto prev_time = std::chrono::steady_clock::now();
    // Get dt (in seconds)
    std::chrono::duration<double> dt = std::chrono::seconds(0);
    while (control_execute.load(std::memory_order_acquire))
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
        app.exit();
      }
      translation_velocity_cmd = input->getTranslationVelocityCmd();
      rotation_velocity_cmd = input->getRotationVelocityCmd();

      // control state machine (do not put actual execution logic here)
      std::cout << "|Time: " << elapsed_time.count() <<  "| my current state is: " << cur_ctrl_state <<std::endl;
      switch (cur_ctrl_state)
      {
        case CTRL_START_UP:
          // start up logic 

          if (elapsed_time.count() >= startup_seconds)
          {
            cur_ctrl_state = CTRL_NORMAL;
          }
          continue;

        case CTRL_NORMAL:
          // normal state logic 
          continue;

        default:
          continue;

      }
      
    }
  });

  bool res = app.exec();
  control_execute.store(false, std::memory_order_release);
  control_thread.join();
  return res;
}