#include "Estimator.hpp"


namespace hebi {

  // in this, we assume x_t only has dim 4
  void process_func_gyro(VectorXd& x_t, const VectorXd& x_t_1, const Vector3d w, const double dt)
  {
    Quaterniond q_1(x_t_1.segment<4>(0));
    // Estimated gyro bias
    Vector3d b_g(x_t_1.segment<3>(0 + 4));
    // Esitmated accelerometer bias
    Vector3d b_a(x_t_1.segment<3>(0 + 4 + 3));

    // current body angular velocity
    Vector3d w_b = w - b_g;

    // this is q_eb
    Quaterniond q = Quaterniond((Matrix4d::Identity() + Omega(w_b * dt) / 2.0)*q_1.coeffs());
    q.normalize();

    x_t.segment<4>(0) = q.coeffs();
    x_t.segment<3>(0 + 4) = b_g;
    x_t.segment<3>(0 + 4 + 3) = b_a;
  }

  void measure_func_acc(VectorXd& z_t, const VectorXd& x_t)
  {
    Quaterniond q(x_t.segment<4>(0));
    Vector3d g_e(0,0,-9.8);
    z_t = q.inverse() * g_e;
  }

  Estimator::Estimator():
    filter_initialized_(false)
  {
    state_size = 10;
    measure_size = 3;

    f_func = &process_func_gyro;
    h_func = &measure_func_acc;

    state.resize(state_size);
    R = 0.1*MatrixXd::Identity(state_size, state_size);
    Q = 0.1*MatrixXd::Identity(measure_size, measure_size);

  }

	Estimator::Estimator(int state_size_, int measure_size_):
    filter_initialized_(false)
  {
    state_size = state_size_;
    measure_size = measure_size_;

    f_func = &process_func_gyro;
    h_func = &measure_func_acc;

    state.resize(state_size);
    // TODO: read noise from constructor input
    R = 0.1*MatrixXd::Identity(state_size, state_size);
    Q = 0.1*MatrixXd::Identity(measure_size, measure_size);
  }

  Estimator::~Estimator(){}

  // must provide initial bias
  void Estimator::initialize(VectorXd& _b_g, VectorXd& _b_a)
  {

    state.segment<4>(0) = Quaterniond(1, 0, 0, 0).coeffs();
    state.segment<3>(0 + 4) = _b_g;
    state.segment<3>(0 + 4 + 3) = _b_a;

    // // delete previous filter if exists
    // if (ukf != NULL)
    // {
    //   delete ukf;
    // }

    ukf = new SRUKF(state_size, measure_size, 0.01, 0.4, f_func, h_func);
  
    ukf -> setR(R);
    ukf -> setQ(Q);

    ukf -> state_pre = state;
    ukf -> state_post = state;

    ukf->S_pre = MatrixXd::Identity(state_size,measure_size);
		ukf->S_post = MatrixXd::Identity(state_size,measure_size);


		filter_initialized_ = true;
  }



  void Estimator::update(const Vector3d& acc_b, const Vector3d& gyro_b, const double dt)
  {
    // do nothing first
  }

  



}