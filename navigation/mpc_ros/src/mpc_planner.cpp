#include <vector>
#include <map>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include "mpc_planner.h"
#include <ros/ros.h>
#include <cmath>

using namespace std;
using namespace casadi;

namespace mpc_ros {

MPC::MPC() {}

void MPC::LoadParams(const std::map<string, double> &params) {
    _dt = params.at("DT");
    _mpc_steps = params.at("STEPS");
    _ref_vel = params.at("REF_V");
    _ref_cte = params.at("REF_CTE");
    _ref_etheta = params.at("REF_ETHETA");
    _w_cte = params.at("W_CTE");
    _w_etheta = params.at("W_EPSI");
    _w_vel = params.at("W_V");
    _w_angvel = params.at("W_ANGVEL");
    _w_accel = params.at("W_A");
    _w_angvel_d = params.at("W_DANGVEL");
    _w_accel_d = params.at("W_DA");

    _max_angvel = params.at("ANGVEL");
    _max_throttle = params.at("MAXTHR");
    _bound_value  = params.at("BOUND");

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
    _v_start = _theta_start + _mpc_steps;
    _cte_start = _v_start + _mpc_steps;
    _etheta_start = _cte_start + _mpc_steps;
    _angvel_start = _etheta_start + _mpc_steps;
    _a_start = _angvel_start + _mpc_steps - 1;
}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    size_t n_states = 6;  // x, y, theta, v, cte, etheta
    size_t n_controls = 2;  // angvel, accel
    size_t total_vars = _mpc_steps * n_states + (_mpc_steps - 1) * n_controls;
    size_t total_constraints = _mpc_steps * n_states;

    // 심볼릭 변수 정의
    MX vars = MX::sym("vars", total_vars);
    MX fg = MX::zeros(total_constraints + 1, 1);

    // 비용 함수 정의
    MX cost = MX::zeros(1);
    for (int i = 0; i < _mpc_steps; ++i) {
        cost += _w_cte * MX::pow(vars(_cte_start + i) - _ref_cte, 2);
        cost += _w_etheta * MX::pow(vars(_etheta_start + i) - _ref_etheta, 2);
        cost += _w_vel * MX::pow(vars(_v_start + i) - _ref_vel, 2);
    }

    for (int i = 0; i < _mpc_steps - 1; ++i) {
        cost += _w_angvel * MX::pow(vars(_angvel_start + i), 2);
        cost += _w_accel * MX::pow(vars(_a_start + i), 2);
    }

    for (int i = 0; i < _mpc_steps - 2; ++i) {
        cost += _w_angvel_d * MX::pow(vars(_angvel_start + i + 1) - vars(_angvel_start + i), 2);
        cost += _w_accel_d * MX::pow(vars(_a_start + i + 1) - vars(_a_start + i), 2);
    }

    fg(0) = cost;

    // 초기 상태 제약 조건
    fg(1 + _x_start) = vars(_x_start) - state[0];
    fg(1 + _y_start) = vars(_y_start) - state[1];
    fg(1 + _theta_start) = vars(_theta_start) - state[2];
    fg(1 + _v_start) = vars(_v_start) - state[3];
    fg(1 + _cte_start) = vars(_cte_start) - state[4];
    fg(1 + _etheta_start) = vars(_etheta_start) - state[5];

    // 동역학 모델 제약 조건
    for (int i = 0; i < _mpc_steps - 1; ++i) {
        MX x1 = vars(_x_start + i + 1);
        MX y1 = vars(_y_start + i + 1);
        MX theta1 = vars(_theta_start + i + 1);
        MX v1 = vars(_v_start + i + 1);
        MX cte1 = vars(_cte_start + i + 1);
        MX etheta1 = vars(_etheta_start + i + 1);

        MX x0 = vars(_x_start + i);
        MX y0 = vars(_y_start + i);
        MX theta0 = vars(_theta_start + i);
        MX v0 = vars(_v_start + i);
        MX cte0 = vars(_cte_start + i);
        MX etheta0 = vars(_etheta_start + i);

        MX w0 = vars(_angvel_start + i);
        MX a0 = vars(_a_start + i);

        MX f0 = coeffs(0);
        for (int j = 1; j < coeffs.size(); ++j) {
            f0 += coeffs(j) * MX::pow(x0, j);
        }

        MX traj_grad0 = coeffs(1);
        for (int j = 2; j < coeffs.size(); ++j) {
            traj_grad0 += j * coeffs(j) * MX::pow(x0, j - 1);
        }
        traj_grad0 = MX::atan(traj_grad0);

        fg(2 + _x_start + i) = x1 - (x0 + v0 * MX::cos(theta0) * _dt);
        fg(2 + _y_start + i) = y1 - (y0 + v0 * MX::sin(theta0) * _dt);
        fg(2 + _theta_start + i) = theta1 - (theta0 + w0 * _dt);
        fg(2 + _v_start + i) = v1 - (v0 + a0 * _dt);
        fg(2 + _cte_start + i) = cte1 - ((f0 - y0) + v0 * MX::sin(etheta0) * _dt);
        fg(2 + _etheta_start + i) = etheta1 - (etheta0 + w0 * _dt);
    }

    // 최적화 문제 정의
    Function solver = nlpsol("solver", "ipopt", {{"x", vars}, {"f", fg(0)}, {"g", fg(Slice(1, fg.size1()))}});

    // 초기화 및 경계 설정
    DM x0 = DM::zeros(total_vars);
    DM lbx = DM::ones(total_vars);
    DM ubx = DM::ones(total_vars);
    DM lbg = DM::zeros(total_constraints);
    DM ubg = DM::zeros(total_constraints);

    for (int i = _angvel_start; i < _a_start; i++){
        lbx(i) = -_max_angvel;
        ubx(i) = _max_angvel;
    }

    for (int i = _a_start; i < total_vars; i++){
        lbx(i) = -_max_throttle;
        ubx(i) = _max_throttle;
    }

     for (int i = 0; i < total_constraints; i++){
        lbg(i) = 0;
        ubg(i) = 0;
     }

    x0(_x_start) = state[0];
    x0(_y_start) = state[1];
    x0(_theta_start) = state[2];
    x0(_v_start) = state[3];
    x0(_cte_start) = state[4];
    x0(_etheta_start) = state[5];

    std::map<std::string, DM> arg = {{"x0", x0}, {"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}};

    // 문제 해결
    std::map<std::string, DM> res = solver(arg);

    double cost_ = double(res["f"].scalar());
    ROS_WARN("Final cost: %f", cost_);

    ROS_WARN("Optimized linear velocity: %.6f, angular velocity: %.6f", 
         double(res["x"](_v_start).scalar()), 
         double(res["x"](_angvel_start).scalar()));

    // mpc_x, mpc_y, mpc_theta 업데이트
    mpc_x.clear();
    mpc_y.clear();
    mpc_theta.clear();
    for (size_t i = 0; i < _mpc_steps; i++) {
        mpc_x.push_back(double(res["x"](_x_start + i).scalar()));
        mpc_y.push_back(double(res["x"](_y_start + i).scalar()));
        mpc_theta.push_back(double(res["x"](_theta_start + i).scalar()));
    }

    vector<double> result;
    for (int i = 0; i < n_controls; ++i) {
        result.push_back(double(res["x"](i).scalar()));
    }
    return result;
}


} // namespace mpc_ros

