#pragma once

#include "SRUKF.hpp"
#include "utils.hpp"
#include <Eigen/Dense>

/*
 *  This is a state estimator 
 *  My final goal is to estimator many states, but now just start with orientation and velocity
 */

using namespace Eigen;

namespace hebi {

	void process_func_gyro(VectorXd& x_t, const VectorXd& x_t_1, const Vector3d w, const double dt);
	void measure_func_acc(VectorXd& z_t, const VectorXd& x_t);

	class Estimator
	{
		public:
			Estimator();
			Estimator(int state_size_, int measure_size_);
			~Estimator();

			void initialize(VectorXd& b_g, VectorXd& b_a);
			bool isInitialized(){return filter_initialized_;}

			bool getState(VectorXd & estimate_state);

			void update(const Vector3d& acc_b, const Vector3d& gyro_b, const double dt);

		private:
			SRUKF* ukf;
			// implementation in this version:
			// state size 10
			// 4 rotation
			// 3 gyro bias
			// 3 acc bias

			double state_size;
			double measure_size;
			double dt;

			bool filter_initialized_;

			Vector3d state_pos_xyz;
			Vector3d state_vel_xyz;
			Vector3d state_acc_xyz;
			Quaterniond state_rotation;
			Vector3d state_vel_angular;
			Vector3d state_bias_acc;
			Vector3d state_bias_gyro; 
			VectorXd state;

			MatrixXd R; // process noise
			MatrixXd Q; // measurement noise

			void (*f_func)(VectorXd& x_t, const VectorXd& x_t_1, const Vector3d w, const double dt);
			void (*h_func)(VectorXd& z_t, const VectorXd& x_t);
	};



} // namespace hebi