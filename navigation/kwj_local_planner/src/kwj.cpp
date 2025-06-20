#include <vector>
#include <map>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include "kwj.h"
#include <ros/ros.h>
#include <cmath>

using namespace std;
using namespace casadi;

namespace kwj_local_planner {

KWJ::KWJ() {}

void KWJ::LoadParams(const std::map<string, double> &params) {
    _dt = params.at("DT");
    _kwj_steps = params.at("STEPS");
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
    _T_max    = params.at("T_MAX");    
    _dt_min   = params.at("DT_MIN");   
    _dt_max   = params.at("DT_MAX");
    d_min_ = params.at("OBSTACLE_MIN");
    w_obs_ = params.at("OBSTACLE_WEIGHT");
    _w_dt_smooth = 0.5;

    _max_angvel = params.at("ANGVEL");
    _max_throttle = params.at("MAXTHR");
    _bound_value  = params.at("BOUND");

    _x_start = 0;
    _y_start = _x_start + _kwj_steps;
    _theta_start = _y_start + _kwj_steps;
    _v_start = _theta_start + _kwj_steps;
    _cte_start = _v_start + _kwj_steps;
    _etheta_start = _cte_start + _kwj_steps;
    _angvel_start = _etheta_start + _kwj_steps;
    _a_start = _angvel_start + _kwj_steps - 1;
    _dt_start = _a_start + (_kwj_steps - 1);
}

void KWJ::obstacle(std::vector<std::pair<double,double>> obstacles){
    obstacles_.clear();
    obstacles_ = obstacles;
}

void KWJ::currentPose(double x, double y, double th){
    current_x = x;
    current_y = y;
    current_th = th;
}

vector<double> KWJ::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {

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

    double R = 0.5;
    std::vector<std::pair<double,double>> local_obs;
    for (auto &ob : obstacles_) {
        double dx = ob.first  - current_x;  
        double dy = ob.second - current_y;
    if (std::hypot(dx,dy) < R)  
        local_obs.push_back(ob);
    } 

    double l = 0.2;
    double w = 0.155;

    std::vector<std::pair<double,double>> corners = {
        { l/2,  w/2},
        { l/2, -w/2},
        {-l/2,  w/2},
        {-l/2, -w/2},
    };

    size_t n_states = 6;  // x, y, theta, v, cte, etheta
    size_t n_controls = 2;  // angvel, accel
    size_t n_time = _kwj_steps - 1;
    size_t n_obs = local_obs.size();
    size_t N          = _kwj_steps;
    size_t total_vars = _kwj_steps * n_states + (_kwj_steps - 1) * n_controls + n_time;
    size_t total_constraints = _kwj_steps * n_states + _kwj_steps * n_obs * corners.size();

    MX vars = MX::sym("vars", total_vars);
    MX fg = MX::zeros(total_constraints + 1, 1);

    MX cost = MX::zeros(1);

    for (int i = 0; i < _kwj_steps; ++i) {
        cost += _w_cte * MX::pow(vars(_cte_start + i) - _ref_cte, 2) + 0.5 * state(4) * state(4);
        cost += _w_etheta * MX::pow(vars(_etheta_start + i) - _ref_etheta, 2);
        cost += _w_vel * MX::pow(vars(_v_start + i) - _ref_vel, 2) + 0.5 * state(3) * state(3);
    }

    for (int i = 1; i < _kwj_steps - 1; ++i) {
        cost += _w_dt_smooth * pow(vars(_dt_start + i) - vars(_dt_start + i - 1), 2);
    }

    for (int i = 0; i < _kwj_steps - 1; ++i) {
        cost += _w_angvel * MX::pow(vars(_angvel_start + i), 2) + 0.5 * state(2) * state(2);    
        cost += _w_accel * MX::pow(vars(_a_start + i), 2);
     }

    for (int i = 0; i < _kwj_steps - 2; ++i) {
        cost += _w_angvel_d * MX::pow(vars(_angvel_start + i + 1) - vars(_angvel_start + i), 2);
        cost += _w_accel_d * MX::pow(vars(_a_start + i + 1) - vars(_a_start + i), 2);
     }

    std::vector<MX> obs_x_f, obs_y_f;
    for (auto &ob : local_obs) {
        obs_x_f.emplace_back(MX(ob.first));
        obs_y_f.emplace_back(MX(ob.second));
    }

    DM obs_x_dm = DM(obs_x_f);
    DM obs_y_dm = DM(obs_y_f);
    MX obs_x = MX(obs_x_dm);
    MX obs_y = MX(obs_y_dm);

    for (int i = 0; i < _kwj_steps; ++i) {
         MX x_i = vars(_x_start + i);
         MX y_i = vars(_y_start + i);
         MX dx      = x_i - obs_x;                   
         MX dy      = y_i - obs_y;                   
         MX dists   = sqrt(dx*dx + dy*dy);          
         MX viols   = fmax(MX::zeros((int)n_obs,1), d_min_ - dists);
         MX term    = dot(viols, viols);            
         cost += w_obs_ * term;
    }

    fg(0) = cost;

    int idx = 1 + _kwj_steps * n_states;
    for(int i = 0; i < _kwj_steps; ++i) {
        MX xi = vars(_x_start + i);
        MX yi = vars(_y_start + i);
        MX thi = vars(_theta_start + i);
        for(auto &off : corners) {
          MX x_corner = xi + cos(thi)*off.first - sin(thi)*off.second;
          MX y_corner = yi + sin(thi)*off.first + cos(thi)*off.second;
          for(int j = 0; j < (int)n_obs; ++j) {
            MX ox = obs_x_f[j], oy = obs_y_f[j];
            fg(idx++) = sqrt( pow(x_corner-ox,2) + pow(y_corner-oy,2) ) - d_min_;
          }
        }
    } 

    fg(1 + _x_start) = vars(_x_start);
    fg(1 + _y_start) = vars(_y_start);
    fg(1 + _theta_start) = vars(_theta_start);
    fg(1 + _v_start) = vars(_v_start);
    fg(1 + _cte_start) = vars(_cte_start);
    fg(1 + _etheta_start) = vars(_etheta_start);


    for (int i = 0; i < _kwj_steps - 1; ++i) {
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
        MX dt_i = vars(_dt_start  + i);

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

        fg(2 + _x_start + i) = x1 - (x0 + v0 * MX::cos(theta0) * dt_i);
        fg(2 + _y_start + i) = y1 - (y0 + v0 * MX::sin(theta0) * dt_i);
        fg(2 + _theta_start + i) = theta1 - (theta0 + w0 * dt_i);
        fg(2 + _v_start + i) = v1 - (v0 + a0 * dt_i);
        fg(2 + _cte_start + i) = cte1 - ((f0 - y0) + v0 * MX::sin(etheta0) * dt_i);
        fg(2 + _etheta_start + i) = etheta1 - (theta0 - traj_grad0 - kappa * v0 * dt_i);
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
    
    for (int i = _a_start; i < _dt_start; i++) {
        lbx(i) = -_max_throttle;
        ubx(i) = _max_throttle;
    }

    for (int i = _dt_start; i < total_vars; ++i) {
        lbx(i) = _dt_min;
        ubx(i) = _dt_max;
    }

    int coll_start = 1 + _kwj_steps*n_states;  
    for (int k = coll_start; k < total_constraints; ++k) {
        lbg(k) = d_min_;
        ubg(k) = 1e6;
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

    lbg(total_constraints - 1) = 0;         
    ubg(total_constraints - 1) = _T_max;

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
    ipopt_options["max_cpu_time"] = 5.0;   
    ipopt_options["linear_solver"] = "mumps"; 
    ipopt_options["sb"] = "yes";
    casadi_options["ipopt"] = ipopt_options;

    size_t g_size = fg.size1();
    if (g_size <= 1) {
        ROS_ERROR("fg.size1() is too small: %zu", g_size);
        return {};
    }

    solver = nlpsol("solver", "ipopt", {{"x", vars}, {"f", fg(0)}, {"g", fg(Slice(1, fg.size1()))}}, casadi_options);

    std::map<std::string, DM> arg = {{"x0", x0}, {"lbx", lbx}, {"ubx", ubx}, {"lbg", lbg}, {"ubg", ubg}};

    std::map<std::string, DM> res = solver(arg);

    for (size_t i = 0; i < _kwj_steps; i++) {
        double result_x = double(res["x"](_x_start + i).scalar());
        if (std::isnan(result_x) || std::isinf(result_x)) {
            ROS_ERROR("KWJ result contains NaN at step %zu", i);
            return {};
        }
    }

    double cost_ = double(res["f"].scalar());
    /*ROS_WARN("Final cost: %f", cost_);

    for (size_t i = 0; i < _kwj_steps; i++) {
        ROS_INFO("Step %zu: x = %.3f, y = %.3f, theta = %.3f, v = %.3f, cte = %.3f, etheta = %.3f", 
                 i, 
                 double(res["x"](_x_start + i).scalar()), 
                 double(res["x"](_y_start + i).scalar()), 
                 double(res["x"](_theta_start + i).scalar()), 
                 double(res["x"](_v_start + i).scalar()), 
                 double(res["x"](_cte_start + i).scalar()), 
                 double(res["x"](_etheta_start + i).scalar()));
    }

    for (size_t i = 0; i < _kwj_steps -1 ; i++){
        ROS_INFO("time = %.3f", double(res["x"](_etheta_start + i).scalar()));
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
}*/

    kwj_x.clear();
    kwj_y.clear();
    kwj_theta.clear();
    kwj_time.clear();
    for (size_t i = 0; i < _kwj_steps; i++) {
        kwj_x.push_back(double(res["x"](_x_start + i).scalar()));
        kwj_y.push_back(double(res["x"](_y_start + i).scalar()));
        kwj_theta.push_back(double(res["x"](_theta_start + i).scalar()));
    }

    for (size_t i = 0; i < _kwj_steps - 1; i++) {
        kwj_time.push_back(double(res["x"](_dt_start + i).scalar()));
    }

    vector<double> result;
    result.push_back(double(res["x"](_angvel_start).scalar()));
    result.push_back(double(res["x"](_a_start).scalar()));
    
    return result;
 }


} // namespace kwj_local_planner
