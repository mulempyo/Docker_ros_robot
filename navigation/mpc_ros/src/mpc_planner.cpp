#include <vector>
#include <map>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include "mpc_planner.h"

using namespace std;
using namespace casadi;

namespace mpc_ros {

MPC::MPC() {}

void MPC::LoadParams(const std::map<string, double> &params) {
    _params = params;
    _dt = _params["DT"];
    _mpc_steps = _params["STEPS"];
    _ref_vel = _params["REF_V"];
    _ref_cte = _params["REF_CTE"];
    _ref_etheta = _params["REF_ETHETA"];
    _w_cte = _params["W_CTE"];
    _w_etheta = _params["W_EPSI"];
    _w_vel = _params["W_V"];
    _w_angvel = _params["W_ANGVEL"];
    _w_accel = _params["W_A"];

    _x_start = 0;
    _y_start = _x_start + _mpc_steps;
    _theta_start = _y_start + _mpc_steps;
 }

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    size_t n_states = 6;  // x, y, theta, v, cte, etheta
    size_t n_controls = 2;  // angvel, accel

    // CasADi variables
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX theta = MX::sym("theta");
    MX v = MX::sym("v");
    MX cte = MX::sym("cte");
    MX etheta = MX::sym("etheta");

    MX angvel = MX::sym("angvel");
    MX accel = MX::sym("accel");

    // Define the grid
    std::vector<MX> states = {x, y, theta, v, cte, etheta};
    std::vector<MX> controls = {angvel, accel};

    // Define the cost
    MX cost = 0;
    for (size_t t = 0; t < _mpc_steps; ++t) {
        cost += _w_cte * MX::pow(cte, 2);
        cost += _w_etheta * MX::pow(etheta, 2);
        cost += _w_vel * MX::pow(v - _ref_vel, 2);
    }

    for (size_t t = 0; t < _mpc_steps - 1; ++t) {
        cost += _w_angvel * MX::pow(angvel, 2);
        cost += _w_accel * MX::pow(accel, 2);
    }

    // System dynamics constraints
    std::vector<MX> constraints;
    for (size_t t = 0; t < _mpc_steps - 1; ++t) {
        constraints.push_back(x - (x + v * MX::cos(theta) * _dt));
        constraints.push_back(y - (y + v * MX::sin(theta) * _dt));
        constraints.push_back(theta - (theta + angvel * _dt));
        constraints.push_back(v - (v + accel * _dt));
        constraints.push_back(cte - (coeffs[0] + coeffs[1] * x - y + v * MX::sin(etheta) * _dt));
        constraints.push_back(etheta - (etheta + angvel * _dt));
    }

    // Define the optimization problem
    MX nlp = MX::vertcat({
        MX::vertcat(states),
        MX::vertcat(controls),
        MX::vertcat(constraints)
    });

    Function solver = nlpsol("solver", "ipopt", {
        {"x", nlp},
        {"f", cost},
        {"g", MX::vertcat(constraints)}
    });

    // Solve the problem
    std::map<std::string, DM> arg, res;
    arg["x0"] = DM::zeros(n_states + n_controls);  // Initial guess
    arg["lbx"] = DM::ones(n_states + n_controls) * -1e20;
    arg["ubx"] = DM::ones(n_states + n_controls) * 1e20;

    res = solver(arg);

    // Extract the results
    vector<double> result;
    result.push_back(double(res["x"](n_states)));       // angvel
    result.push_back(double(res["x"](n_states + 1)));  // accel

    // Save predicted trajectory
    mpc_x.clear();
    mpc_y.clear();
    mpc_theta.clear();

    for (size_t i = 0; i < _mpc_steps; i++) {
        mpc_x.push_back(double(res["x"](_x_start + i)));
        mpc_y.push_back(double(res["x"](_y_start + i)));
        mpc_theta.push_back(double(res["x"](_theta_start + i)));
    }

    return result;
  }
} // namespace mpc_ros

