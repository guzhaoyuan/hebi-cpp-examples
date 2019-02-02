#include <QGuiApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QMessageBox>

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QLabel>
#include <QtWidgets/QHBoxLayout>

#include "display/hexapod_view_2d.hpp"

#include "robot/hexapod.hpp"
#include "input/input_manager_mobile_io.hpp"
#include "util/trajectory.hpp"
#include <atomic>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <set>

using namespace hebi;
using namespace Eigen;
/* state machine related needed variables */
// state definitions
enum ctrl_state_type {
  TITAN6_CTRL_STAND_UP1,
  TITAN6_CTRL_STAND_UP2,

  TITAN6_CTRL_STAND_UP3,  // this state is used to record sensor bias and do initialization of estimation

  TITAN6_TRIPOD_GAIT,
  TITAN6_WAVE_GAIT,

  LEG_DYN_CTRL_TEST_PLAN,
  LEG_DYN_CTRL_TEST_RUN,

  CTRL_STATES_COUNT
};

bool parse_parameters(int argc, char** argv, bool& visualize, bool& dummy, bool& partial, bool& quiet, std::set<int>& partial_legs)
{
  visualize = false;
  dummy = false;
  partial = false;
  quiet = false;
  bool valid = true;
  int idx = 1; // Ignore program name!
  for (;idx < argc; ++idx)
  {
    std::string str_arg(argv[idx]);
    if (str_arg == "-h")
    {
      std::cout << "Hexapod control usage:\n" <<
      "    -d\n" <<
      "        Dummy hexapod; no modules are connected to.  Incompatible with \"-p\".\n\n" <<
      "    -p <list of integer leg indices>\n" <<
      "        Creates a partial dummy hexapod; only the legs with the specified indices\n" <<
      "        are connected to.\n\n" <<
      "    -v\n" <<
      "        Visualize -- show a simple rendering of the robot.\n\n" <<
      "    -q\n" <<
      "        Quiet mode (no dialog messages; waits and tries to continue on failure such as no modules on the network).\n\n" <<
      "    -h\n" <<
      "        Print this help and return." << std::endl;
      return false;
    }
    else if (str_arg == "-v")
    {
      visualize = true;
      continue;
    }
    else if (str_arg == "-d")
    {
      dummy = true;
      continue;
    }
    else if (str_arg == "-p" && idx + 1 < argc)
    {
      partial = true;
      for (++idx; idx < argc; ++idx)
      {
        try
        {
          int partial_leg = std::stoi(std::string(argv[idx]));
          partial_legs.insert(partial_leg);
        }
        catch (const std::invalid_argument&)
        {
          idx -= 2; // -2, because we will add one next start of the parent loop
          break;
        }
      }
      continue;
    }
    else if (str_arg == "-q")
    {
      quiet = true;
      continue;
    }
    else
    {
      valid = false;
      break;
    }
  }
  // Do all exclusive argument checks here
  if (!valid ||
      (dummy && partial) ||
      (partial && partial_legs.size() == 0))
  {
    std::cout << "Invalid combination of arguments! Use \"-h\" for usage." << std::endl;
    return false;
  }
  return true;
}

// Get the hexapod, handling errors as appropriate
std::unique_ptr<Hexapod> getHexapod(const HexapodParameters& params, bool is_dummy, bool is_partial, bool is_quiet, const std::set<int>& legs)
{
  std::unique_ptr<Hexapod> hexapod;

  HexapodErrors hex_errors;
  hex_errors.has_valid_initial_feedback = true;
  hex_errors.first_out_of_range_leg = -1;
  hex_errors.m_stop_pressed = false;

  if (is_dummy)
  {
    std::cout << "Creating dummy hexapod for testing\n";
    hexapod = Hexapod::createDummy(params);
  }
  else if (is_partial)
  {
    std::cout << "Creating partial hexapod for testing\n";
    hexapod = Hexapod::createPartial(params, legs, hex_errors);
  }
  else
  {
    std::cout << "Searching network for modules...\n";
    hexapod = Hexapod::create(params, hex_errors);
  }

  if (!hexapod)
  {
    if (!is_quiet)
    {
      QMessageBox param_error;
      param_error.setWindowTitle("Could not start up hexapod");
      param_error.setText("Could not find hexapod on network -- check that all modules are visible via the Scope GUI.");
      param_error.exec();
    }
    return nullptr;
  }

  // Clear any colors potentially set elsewhere
  hexapod->clearLegColors();

  if(!hex_errors.has_valid_initial_feedback)
  {
    if (!is_quiet)
    {
      QMessageBox param_error;
      param_error.setWindowTitle("Did not get initial feedback from modules!");
      param_error.setText("Could not get feedback from all modules -- this could indicate an intermittent network connection with the modules.");
      param_error.exec();
    }
    return nullptr;
  }

  // if(hex_errors.first_out_of_range_leg != -1)
  // {
  //   if (!is_quiet)
  //   {
  //     QMessageBox param_error;
  //     param_error.setWindowTitle("Base module out of range!");
  //     std::string message = "Base module out of range (leg " + std::to_string(hex_errors.first_out_of_range_leg + 1) + ")";
  //     param_error.setText(QString(message.c_str()));
  //     param_error.exec();
  //   }
  //   else
  //   {
  //     // Set to red!
  //     hexapod->setLegColor(hex_errors.first_out_of_range_leg, 255, 0, 0);
  //   }
  //   return nullptr;
  // }

  if (hex_errors.m_stop_pressed)
  {
    if (!is_quiet)
    {
      QMessageBox param_error;
      param_error.setWindowTitle("M-stop active!");
      param_error.setText("M-stop is active on at least one module! Reset before continuing.");
      param_error.exec();
    }
    return nullptr;
  }

  return hexapod;

}

int main(int argc, char** argv)
{
  // Construct initially so I can pop up error dialogs!
  QApplication app(argc, argv);

  bool do_visualize{};
  bool is_dummy{};
  bool is_partial{};
  bool is_quiet{};
  std::set<int> legs;
  if (!parse_parameters(argc, argv, do_visualize, is_dummy, is_partial, is_quiet, legs))
    return 1;

  HexapodParameters params;
  if (!params.loadFromFile("hex_config.xml"))
  {
    params.resetToDefaults();
    if (!is_quiet)
    {
      QMessageBox param_error;
      param_error.setWindowTitle("Could not find parameters");
      param_error.setText("Could not find hexapod parameters, or file corrupt! Using default parameters.");
      param_error.exec();
    }
  }

  std::unique_ptr<Hexapod> hexapod = getHexapod(params, is_dummy, is_partial, is_quiet, legs);
  while (is_quiet && !hexapod)
  {
    hexapod = getHexapod(params, is_dummy, is_partial, is_quiet, legs);
  }

  if(!hexapod && !is_quiet)
    return 1;

  if (!is_dummy)
  {
    // Try (twice) to set gains; short circuits after first success
    if (hexapod && !hexapod->setGains() && !hexapod->setGains())
    {
      if (!is_quiet)
      {
        QMessageBox param_error;
        param_error.setWindowTitle("Could not set gains!");
        param_error.setText("Could not set controller gains on connected modules -- this could indicate an intermittent network connection with the modules.");
        param_error.exec();
      }      
    }
    if (hexapod)
    {
      hexapod->startLogging();
    }
  }

  // Set LEDs blue to indicate looking for joystick
  for (int i = 0; i < 6; ++i)
    hexapod->setLegColor(i, 0, 0, 255);

  // connections
  double period = 5.0;
  bool last_connection_status = true;
  bool high_freq_logging = false;
  std::unique_ptr<input::InputManager> input(new input::InputManagerMobileIO());
  
  if (!is_quiet && !input->isConnected())
  {
    std::cout << "Could not find I/O board for joystick." << std::endl;
    return 1;
  }
  // Retry a "reset" multiple times! Wait for this in a loop.
  while (is_quiet && !input->isConnected())
  {
    static_cast<input::InputManagerMobileIO*>(input.get())->reset();
  }

  hexapod->clearLegColors();

  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Found robot -- starting control program.\n";

  double overall_width = do_visualize ? 1100 : 300;
  double overall_height = do_visualize ? 800 : 150;

  QWidget *widget = new QWidget;

  // These could be put into a struct -- basically, we just need to hold on to
  // these objects during application runtime, and they may be optional. Should
  // really refactor more completely here.
  QLabel* mode;
  QLabel* feedback_status;
  std::unique_ptr<QGraphicsScene> scene;/* state machine related needed variables */
  std::unique_ptr<HexapodView2D> hexapod_display;
  std::unique_ptr<QGraphicsView> view;

  if (do_visualize)
  {
    scene.reset(new QGraphicsScene(QRectF(0, 0, overall_width - 200, overall_height)));
    hexapod_display.reset(new HexapodView2D(scene.get()));

    view.reset(new QGraphicsView(scene.get()));
    view->setBackgroundBrush(Qt::black);
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->resize(overall_width + 10, overall_height + 10);

    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    vLayout->setAlignment(Qt::AlignTop);
    hLayout->addWidget(view.get(), 1);
    hLayout->addLayout(vLayout);

    ////////////////////////
    // Build the panel with buttons/labels/etc.
    // TODO: abstract out to a different class?
    QLabel* mode_label = new QLabel("Current Mode:");
    auto font = mode_label->font();
    font.setWeight(QFont::Bold);
    mode_label->setFont(font);
    mode = new QLabel("---");

    QLabel* feedback_status_label = new QLabel("Module Ethernet Connection:");
    font = feedback_status_label->font();
    font.setWeight(QFont::Bold);
    feedback_status_label->setFont(font);

    feedback_status = new QLabel("Good");
    QPalette palette = feedback_status->palette();
    palette.setColor(QPalette::Background, Qt::green);
    feedback_status->setAutoFillBackground(true);
    feedback_status->setPalette(palette);

    QLabel* frequency_label = new QLabel("Logging Frequencies (low/high):");
    font = frequency_label->font();
    font.setWeight(QFont::Bold);
    frequency_label->setFont(font);
    std::string frequency_text = std::to_string(params.low_log_frequency_hz_) + "/" + std::to_string(params.high_log_frequency_hz_);
    QLabel* frequency = new QLabel(frequency_text.c_str());

    vLayout->addWidget(mode_label);
    vLayout->addWidget(mode);
    vLayout->addWidget(feedback_status_label);
    vLayout->addWidget(feedback_status);
    vLayout->addWidget(frequency_label);
    vLayout->addWidget(frequency);
    ////////////////////////
  }
  else
  {
    QVBoxLayout *vLayout = new QVBoxLayout(widget);
    QHBoxLayout *hLayout1 = new QHBoxLayout();
    QHBoxLayout *hLayout2 = new QHBoxLayout();
    QHBoxLayout *hLayout3 = new QHBoxLayout();
    QLabel* mode_label = new QLabel("Current Mode:");
    auto font = mode_label->font();
    font.setWeight(QFont::Bold);
    mode_label->setFont(font);
    mode = new QLabel("---");

    QLabel* feedback_status_label = new QLabel("Module Ethernet Connection:");
    font = feedback_status_label->font();
    font.setWeight(QFont::Bold);
    feedback_status_label->setFont(font);
    feedback_status = new QLabel("Good");
    QPalette palette = feedback_status->palette();
    palette.setColor(QPalette::Background, Qt::green);
    feedback_status->setAutoFillBackground(true);
    feedback_status->setPalette(palette);

    QLabel* frequency_label = new QLabel("Logging Frequencies (low/high):");
    font = frequency_label->font();
    font.setWeight(QFont::Bold);
    frequency_label->setFont(font);
    std::string frequency_text = std::to_string(params.low_log_frequency_hz_) + "/" + std::to_string(params.high_log_frequency_hz_);
    QLabel* frequency = new QLabel(frequency_text.c_str());

    hLayout1->addWidget(mode_label);
    hLayout1->addWidget(mode);
    hLayout2->addWidget(feedback_status_label);
    hLayout2->addWidget(feedback_status);
    hLayout3->addWidget(frequency_label);
    hLayout3->addWidget(frequency);
    vLayout->addLayout(hLayout1);
    vLayout->addLayout(hLayout2);
    vLayout->addLayout(hLayout3);
  }

  widget->setWindowTitle(QStringLiteral("heXapod Control"));
  widget->show();
  widget->resize(overall_width, overall_height);

  Eigen::Vector3f translation_velocity_cmd;
  translation_velocity_cmd.setZero();
  Eigen::Vector3f rotation_velocity_cmd;
  rotation_velocity_cmd.setZero();

  // state transition related 
  ctrl_state_type curr_ctrl_state = TITAN6_CTRL_STAND_UP1;
  // ctrl_state_type curr_ctrl_state = LEG_DYN_CTRL_TEST_PLAN;
  //tripod_gait_state curr_gait_state = STANCE;
  // state transition parameters
  double startup_seconds = 5;
  double bias_record_seconds = 3;
  // some time for state use
  auto state_enter_time = std::chrono::steady_clock::now(); 
  auto state_curr_time = std::chrono::steady_clock::now(); 
  std::chrono::duration<double> state_run_time = std::chrono::seconds(0);

  // Controls to send to the robot
  std::vector<std::shared_ptr<trajectory::Trajectory>> startup_trajectories;
  Eigen::VectorXd angles(Leg::getNumJoints());
  Eigen::VectorXd vels(Leg::getNumJoints());
  Eigen::VectorXd accels(Leg::getNumJoints());
  Eigen::VectorXd torques(Leg::getNumJoints());
  Eigen::MatrixXd foot_forces(3,6); // 3 (xyz) by num legs
  foot_forces.setZero();

  auto start = std::chrono::steady_clock::now();
  long interval_ms = period;
  // http://stackoverflow.com/questions/30425772/c-11-calling-a-c-function-periodically
  std::atomic<bool> control_execute;
  control_execute.store(true, std::memory_order_release);
  std::thread control_thread([&]()
  {

    auto prev = std::chrono::steady_clock::now();
    // Get dt (in seconds)
    std::chrono::duration<double> dt = std::chrono::seconds(0);
    while (control_execute.load(std::memory_order_acquire))
    {

      // Wait!
      auto now = std::chrono::steady_clock::now();
      auto need_to_wait = std::max(0, (int)std::chrono::duration_cast<std::chrono::milliseconds>(prev + std::chrono::milliseconds(interval_ms) - now).count());
      std::this_thread::sleep_for(std::chrono::milliseconds(need_to_wait));

      // Get dt (in seconds)
      now = std::chrono::steady_clock::now();
      dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
      prev = now;

      // show connection status
      if (!is_dummy)
      {
        // More than 2x the feedback period?
        if ((now - hexapod->getLastFeedbackTime()) > std::chrono::milliseconds((long)(hexapod->getFeedbackPeriodMs() * 2.0)))
        {
          if (last_connection_status)
          {
            feedback_status->setText("Intermittent");
            QPalette palette = feedback_status->palette();
            palette.setColor(QPalette::Background, Qt::red);
            feedback_status->setPalette(palette);
          }
          last_connection_status = false;
        }
        else
        {
          if (!last_connection_status)
          {
            feedback_status->setText("Good");
            QPalette palette = feedback_status->palette();
            palette.setColor(QPalette::Background, Qt::green);
            feedback_status->setPalette(palette);
          }
          last_connection_status = true;
        }
      }

      // In seconds:
      std::chrono::duration<double> elapsed(now - start);

      // Get joystick update, and update any relevant variables.
      input->update();
      if (input->getQuitButtonPushed())
        app.exit();

      translation_velocity_cmd = input->getTranslationVelocityCmd();
      rotation_velocity_cmd = input->getRotationVelocityCmd();
      hexapod->updateMode(input->getAndResetModeToggleCount());

      // The first minute of every 30 minutes: record log?
      if (fmod(elapsed.count(), 1800) < 60)
      {
        if (!high_freq_logging)
        {
          high_freq_logging = true;
          if (hexapod->hasLogGroup())
            hexapod->setLoggingFrequency(params.high_log_frequency_hz_);
        }
      }
      else
      {
        if (high_freq_logging)
        {
          high_freq_logging = false;
          if (hexapod->hasLogGroup())
            hexapod->setLoggingFrequency(params.low_log_frequency_hz_);
        }
      }

      // control state machine 
      switch (curr_ctrl_state)
      {
        case TITAN6_CTRL_STAND_UP1:
        {
          mode->setText("Startup Plan");
          state_curr_time = std::chrono::steady_clock::now();
          state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

          for (int i = 0; i < 6; ++i)

          {
            // set blue
            hexapod->setLegColor(i, 0, 0, 255);
            Eigen::VectorXd leg_start = hexapod->getLegFeedback(i);

            Eigen::Vector3d goal(3);
            auto base_frame = hexapod->getLeg(i) -> getKinematics().getBaseFrame();
            Eigen::Vector4d tmp4(0.50, 0, -0.10, 0); // hard code first
            Eigen::VectorXd home_stance_xyz = (base_frame * tmp4).topLeftCorner<3,1>();
            
            
            hexapod->getLeg(i)->computeIK(goal, home_stance_xyz);

            Eigen::VectorXd leg_mid = goal;
            
            
            home_stance_xyz = hexapod->getLeg(i)->getHomeStanceXYZ(); 
            
            
            hexapod->getLeg(i)->computeIK(goal, home_stance_xyz);

            Eigen::VectorXd leg_end = goal;
            // Convert for trajectories
            int num_waypoints = 5;          // Controls to send to the robot
            int num_joints = Leg::getNumJoints();
            Eigen::MatrixXd positions(num_joints, num_waypoints);
            Eigen::MatrixXd velocities = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
            Eigen::MatrixXd accelerations = Eigen::MatrixXd::Zero(num_joints, num_waypoints);
            Eigen::VectorXd nan_column = Eigen::VectorXd::Constant(num_joints, std::numeric_limits<double>::quiet_NaN());

            // Set positions
            positions.col(0) = leg_start;
            positions.col(1) = leg_start ;
            positions.col(2) = leg_mid ;
            positions.col(3) = leg_mid;
            positions.col(4) = leg_end;

            velocities.col(1) = nan_column;
            velocities.col(3) = nan_column;
            accelerations.col(1) = nan_column;
            accelerations.col(3) = nan_column;

            Eigen::VectorXd times(num_waypoints);
            double local_start = 0;
            double total = startup_seconds;
            times << local_start,
                    local_start + total * 0.25,
                    local_start + total * 0.5,
                    local_start + total * 0.75,
                    local_start + total;
            startup_trajectories.push_back(trajectory::Trajectory::createUnconstrainedQp(
              times, positions, &velocities, &accelerations));

          }
          
          // transit after plan the trajectories
          curr_ctrl_state = TITAN6_CTRL_STAND_UP2;
          state_enter_time = std::chrono::steady_clock::now();
          
          break;

        }

        case TITAN6_CTRL_STAND_UP2:
        {
          // std::cout << "state 2" <<std::endl;
          state_curr_time = std::chrono::steady_clock::now();
          state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);
          mode->setText("Startup Exec");
          
          Eigen::VectorXd gravity_vec = hexapod->getGravityDirection() * 9.8;
          // Follow t_l:
          for (int i = 0; i < 6; ++i)
          {
            hexapod->setLegColor(i, 255, 0, 0);
            startup_trajectories[i]->getState(state_run_time.count(), &angles, &vels, &accels);
            
            hebi::Leg* curr_leg = hexapod->getLeg(i);

            // For rendering:
            if (hexapod_display)
              hexapod_display->updateLeg(curr_leg, i, angles);
            
            // TODO: add actual foot torque for startup?
            Eigen::VectorXd foot_force(6); foot_force << 0,0,0,21*9.8/6,0,0;
            hexapod -> computeDynamicTorques(i, angles, vels, accels, gravity_vec, torques, foot_force);

            hexapod->setCommand(i, &angles, &vels, &torques);

          }
          hexapod->sendCommand();

          if (state_run_time.count() >= startup_seconds)
          {
            curr_ctrl_state = TITAN6_CTRL_STAND_UP3;
            state_enter_time = std::chrono::steady_clock::now();
            
            hexapod -> startRecordBias();
          }
          break;
        }

        case TITAN6_CTRL_STAND_UP3:
        {
          state_curr_time = std::chrono::steady_clock::now();
          state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);
          mode->setText("Record bias");

          // send previous command;
          hexapod->sendCommand();
          // set some different color
          for (int i = 0; i < 6; ++i)
          {
            hexapod->setLegColor(i, 0, 255, 255);
          }
          

          if (state_run_time.count() >= bias_record_seconds)
          {
            curr_ctrl_state = TITAN6_TRIPOD_GAIT;
            state_enter_time = std::chrono::steady_clock::now();
            hexapod -> stopRecordBias();
            hexapod -> initStance(state_run_time.count()); 
          }
          break;
        }

        case TITAN6_TRIPOD_GAIT:
        {
          // stays in this state most of the time
          mode->setText("Tripod Gait Mode");
          for (int i = 0; i < 6; ++i)
          {
            hexapod->setLegColor(i, 0, 255, 0);
          }
          state_curr_time = std::chrono::steady_clock::now();
          state_run_time = std::chrono::duration_cast<std::chrono::duration<double>>(state_curr_time - state_enter_time);

          // Calculate how the weight is distributed
          //hexapod->computeFootForces(state_run_time.count(), foot_forces);
          hexapod->updateStance(
            0.3*translation_velocity_cmd.cast<double>(),
            rotation_velocity_cmd.cast<double>(),
            dt.count());
          if (hexapod->needToStep())
          {
            hexapod->startStep(state_run_time.count());
          }

          hexapod->updateSteps(state_run_time.count());

          Eigen::MatrixXd jacobian_ee;
          robot_model::MatrixXdVector jacobian_com;
          Eigen::VectorXd gravity_vec = hexapod->getGravityDirection() * 9.8;
          // std::cout << hexapod->getGravityDirection() * 9.8 << std::endl;
          for (int i = 0; i < 6; ++i)
          {
            Eigen::VectorXd a(3);
            hebi::Leg* curr_leg = hexapod->getLeg(i);
            curr_leg->computeState(state_run_time.count(), angles, vels, accels, jacobian_ee, jacobian_com);

            // For rendering:
            if (hexapod_display)
              hexapod_display->updateLeg(curr_leg, i, angles);

            Eigen::VectorXd body_force(6); body_force << 0,0,0,0,0,0;
            // std::cout << hexapod -> isStepping() << std::endl;
            if (hexapod -> isStepping())
            {
              if (curr_leg->getMode() == Leg::Mode::Flight)
              {
                body_force(3) = 0;
              }    
              else
              {
                body_force(3) = 21*9.8/3.0;
              }
                
            }
            else
            {
              body_force(3) = 21*9.8/6.0;
            }
            
            hexapod -> computeDynamicTorques(i, angles, vels, accels, gravity_vec, torques, body_force);
            

            hexapod->setCommand(i, &angles, &vels, &torques);
          }
          hexapod->sendCommand();
          break;
        }

        case CTRL_STATES_COUNT:
        default:
        {
         
          break;
        }
      }

    } 
  });
  bool res = app.exec();
  control_execute.store(false, std::memory_order_release);
  control_thread.join();
  return res;
}
