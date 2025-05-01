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

    for (int i = 0; i < state.size(); i++) {
        if (std::isnan(state[i]) || std::isinf(state[i])) {
            ROS_ERROR("state[%d] is NaN or Inf, cannot proceed!", i);
            return {};
        }
    }
    
    for (int i = 0; i < coeffs.size(); i++) {
        if (std::isnan(coeffs[i]) || std::isinf(coeffs[i])) {
            ROS_ERROR("coeffs[%d] is NaN or Inf, cannot proceed!", i);
            return {};
        }
    }

    size_t n_states = 6;  // x, y, theta, v, cte, etheta
    size_t n_controls = 2;  // angvel, accel
    size_t total_vars = _mpc_steps * n_states + (_mpc_steps - 1) * n_controls;
    size_t total_constraints = _mpc_steps * n_states;

    MX vars = MX::sym("vars", total_vars);
    MX fg = MX::zeros(total_constraints + 1, 1);

    MX cost = MX::zeros(1);
    for (int i = 0; i < _mpc_steps; ++i) {
        cost += _w_cte * MX::pow(vars(_cte_start + i) - _ref_cte, 2) + 0.5 * state(4) * state(4);
        cost += _w_etheta * MX::pow(vars(_etheta_start + i) - _ref_etheta, 2) + 0.5 * state(5) * state(5);
        cost += _w_vel * MX::pow(vars(_v_start + i) - _ref_vel, 2) + 0.5 * state(1) * state(1);
    }

    for (int i = 0; i < _mpc_steps - 1; ++i) {
        cost += _w_angvel * MX::pow(vars(_angvel_start + i), 2) + 0.5 * state(2) * state(2);    
        cost += _w_accel * MX::pow(vars(_a_start + i), 2);
     }

    for (int i = 0; i < _mpc_steps - 2; ++i) {
        cost += _w_angvel_d * MX::pow(vars(_angvel_start + i + 1) - vars(_angvel_start + i), 2);
        cost += _w_accel_d * MX::pow(vars(_a_start + i + 1) - vars(_a_start + i), 2);
     }

    fg(0) = cost;

    fg(1 + _x_start) = vars(_x_start);
    fg(1 + _y_start) = vars(_y_start);
    fg(1 + _theta_start) = vars(_theta_start);
    fg(1 + _v_start) = vars(_v_start);
    fg(1 + _cte_start) = vars(_cte_start);
    fg(1 + _etheta_start) = vars(_etheta_start);

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

        MX second_derivative = coeffs(2);
        for (int j = 3; j < coeffs.size(); ++j) {
            second_derivative += j * (j - 1) * coeffs(j) * MX::pow(x0, j - 2);
        }

        MX kappa = second_derivative / MX::pow(1 + traj_grad0 * traj_grad0, 1.5);

        fg(2 + _x_start + i) = x1 - (x0 + v0 * MX::cos(theta0) * _dt);
        fg(2 + _y_start + i) = y1 - (y0 + v0 * MX::sin(theta0) * _dt);
        fg(2 + _theta_start + i) = theta1 - (theta0 + w0 * _dt);
        fg(2 + _v_start + i) = v1 - (v0 + a0 * _dt);
        fg(2 + _cte_start + i) = cte1 - ((f0 - y0) + v0 * MX::sin(etheta0) * _dt);
        fg(2 + _etheta_start + i) = etheta1 - (theta0 - traj_grad0 - kappa * v0 * _dt);
    }

    DM x0 = DM::zeros(total_vars);
    DM lbx = DM::ones(total_vars);
    DM ubx = DM::ones(total_vars);
    DM lbg = DM::zeros(total_constraints);
    DM ubg = DM::zeros(total_constraints);

    x0(_x_start) = state[0];
    x0(_y_start) = state[1];
    x0(_theta_start) = state[2];
    x0(_v_start) = state[3];
    x0(_cte_start) = state[4];
    x0(_etheta_start) = state[5];

    for (int i = 0; i < _angvel_start; i++) {
        lbx(i) = -_bound_value;
        ubx(i) = _bound_value;
    }

    for (int i = _angvel_start; i < _a_start; i++){
        lbx(i) = -_max_angvel;
        ubx(i) = _max_angvel;
    }
    
    for (int i = _a_start; i < total_vars; i++) {
        lbx(i) = -_max_throttle;
        ubx(i) = _max_throttle;
    }

    for (int i = 0; i < total_constraints; i++) {
    if (std::isnan(double(lbg(i).scalar())) || std::isinf(double(lbg(i).scalar()))) {
        ROS_ERROR("lbg[%d] is NaN or Inf!", i);
        return {};
     }
    if (std::isnan(double(ubg(i).scalar())) || std::isinf(double(ubg(i).scalar()))) {
        ROS_ERROR("ubg[%d] is NaN or Inf!", i);
        return {};
     }
    }

    for (int i = 0; i < total_constraints; i++)
    {
        lbg(i) = 0;
        ubg(i) = 0;
    }

    lbg(_x_start) = state[0];
    lbg(_y_start) = state[1];
    lbg(_theta_start) = state[2];
    lbg(_v_start) = state[3];
    lbg(_cte_start) = state[4];
    lbg(_etheta_start) = state[5];

    ubg(_x_start) = state[0];
    ubg(_y_start) = state[1];
    ubg(_theta_start) = state[2];
    ubg(_v_start) = state[3];
    ubg(_cte_start) = state[4];
    ubg(_etheta_start) = state[5];

    ipopt_options["print_level"] = 0;       
    ipopt_options["max_cpu_time"] = 1.0;   
    ipopt_options["linear_solver"] = "mumps"; 
    casadi_options["ipopt"] = ipopt_options;

    size_t g_size = fg.size1();
    if (g_size <= 1) {
        ROS_ERROR("fg.size1() is too small: %zu", g_size);
        return {};
    }

    solver = nlpsol("solver", "ipopt", {{"x", vars}, {"f", fg(0)}, {"g", fg(Slice(1, fg.size1()))}}, casadi_options);

    std::map<std::string, DM> arg = {{"x0", x0}, {"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}};

    std::map<std::string, DM> res = solver(arg);

    for (size_t i = 0; i < _mpc_steps; i++) {
        double result_x = double(res["x"](_x_start + i).scalar());
        if (std::isnan(result_x) || std::isinf(result_x)) {
            ROS_ERROR("MPC result contains NaN at step %zu", i);
            return {};
        }
    }

    double cost_ = double(res["f"].scalar());
    ROS_WARN("Final cost: %f", cost_);

for (size_t i = 0; i < _mpc_steps; i++) {
    ROS_INFO("Step %zu: x = %.3f, y = %.3f, theta = %.3f, v = %.3f, cte = %.3f, etheta = %.3f", 
             i, 
             double(res["x"](_x_start + i).scalar()), 
             double(res["x"](_y_start + i).scalar()), 
             double(res["x"](_theta_start + i).scalar()), 
             double(res["x"](_v_start + i).scalar()), 
             double(res["x"](_cte_start + i).scalar()), 
             double(res["x"](_etheta_start + i).scalar()));
}

ROS_INFO("Initial State: x = %.3f, y = %.3f, theta = %.3f, v = %.3f, cte = %.3f, etheta = %.3f", 
         double(state[0]), 
         double(state[1]), 
         double(state[2]), 
         double(state[3]), 
         double(state[4]), 
         double(state[5]));

auto solver_status = solver.stats()["return_status"];
ROS_WARN("Solver Status: %s", solver_status.get_str().c_str());

for (int i = 0; i < coeffs.size(); ++i) {
    ROS_INFO("Polynomial coeff[%d]: %.3f", i, coeffs[i]);
}

    mpc_x.clear();
    mpc_y.clear();
    mpc_theta.clear();
    for (size_t i = 0; i < _mpc_steps; i++) {
        mpc_x.push_back(double(res["x"](_x_start + i).scalar()));
        mpc_y.push_back(double(res["x"](_y_start + i).scalar()));
        mpc_theta.push_back(double(res["x"](_theta_start + i).scalar()));
    }

    vector<double> result;
    result.push_back(double(res["x"](_angvel_start).scalar()));
    result.push_back(double(res["x"](_a_start).scalar()));
    
    return result;
}


} // namespace mpc_ros
