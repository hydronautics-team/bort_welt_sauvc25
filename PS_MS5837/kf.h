#pragma once

#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <deque>



class KalmanFilter {
  using calc_model_func_t = std::function<Eigen::MatrixXd(const double)>;
  using calc_model_noise_func_t = std::function<Eigen::MatrixXd(const double)>;

 public:
  KalmanFilter(const size_t in_size, const size_t out_size, const Eigen::MatrixXd& H,
               const Eigen::MatrixXd& R, const Eigen::MatrixXd& P,
               const calc_model_func_t& calc_model, const calc_model_noise_func_t& calc_model_noise)
      : H_(H),
        R_(R),
        P_(P),
        I_(Eigen::MatrixXd::Identity(in_size, out_size)),
        state_(Eigen::VectorXd::Zero(out_size)),
        calc_model_(calc_model),
        calc_model_noise_(calc_model_noise) {
    assert(in_size == H_.rows());
    assert(out_size == H_.cols());
    assert(in_size == R_.rows());
    assert(in_size == R_.cols());
    assert(out_size == P_.rows());
    assert(out_size == P_.cols());
    Eigen::MatrixXd check_F = calc_model_(1.0);
    assert(out_size == check_F.rows());
    assert(out_size == check_F.cols());
    Eigen::MatrixXd check_Q = calc_model_noise_(1.0);
    assert(out_size == check_Q.rows());
    assert(out_size == check_Q.cols());
  }

  void update(const Eigen::VectorXd& z, const double dt) {
    predict(dt);
    correct(z);
  }

  const Eigen::VectorXd& get_state() const { return state_; };

 private:
  void predict(const double dt) {
    Eigen::MatrixXd F = calc_model_(dt);
    Eigen::MatrixXd Q = calc_model_noise_(dt);
    state_ = F * state_;
    P_ = F * P_ * F.transpose() + Q;
  }

  void correct(const Eigen::VectorXd& z) {
    assert(z.size() == H_.cols());
    Eigen::MatrixXd K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
    state_ += K * (z - H_ * state_);
    P_ = (I_ - K * H_) * P_;
  }

  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd P_;

  Eigen::MatrixXd I_;

  Eigen::VectorXd state_;

  calc_model_func_t calc_model_;
  calc_model_noise_func_t calc_model_noise_;
};

inline KalmanFilter build_depth_sensor_filter() {
  constexpr size_t in_size = 1;
  constexpr size_t out_size = 2;
//  static const Eigen::MatrixXd H{{1, 0}};
  Eigen::MatrixXd H(1,2);
  H << 1.0, 0;
//  static const Eigen::MatrixXd R{{std::sqrt(0.02)}};  // [m]
  Eigen::MatrixXd R(1,1);
  R << 0.02;

//  static const Eigen::MatrixXd P{{1.0, 0}, {0, 0.25}};
  Eigen::MatrixXd P(2,2);
  P << 1.0, 0,
       0, 0.25;


  return KalmanFilter(
      in_size, out_size, H, R, P,
      [](const double dt) {
        Eigen::MatrixXd F(2,2);
        F << 1.0, dt,
             0.0, 1.0;

        return F;
      },
      [](const double dt) {
        constexpr double koef = 1.0;
      Eigen::MatrixXd Q(2,2);
      Q << std::sqrt(dt), dt,
           dt, 1.0;

        return Q * koef;
      });
}

/* EXAMPLE
  // create
  auto depth_kf = build_depth_sensor_filter();
  /// callback
  depth_kf.update(Eigen::VectorXd{2.123}, 0.5);
  /// get state
  const auto current_state = depth_kf.get_state();
  const double depth = current_state(0);
  const double depth_velocity = current_state(1);
*/

