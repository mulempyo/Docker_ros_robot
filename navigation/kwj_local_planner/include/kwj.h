#include <vector>
#include <map>
#include <Eigen/Core>
#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

namespace kwj_local_planner {

class KWJ {
public:
    KWJ();

    vector<double> kwj_x;
    vector<double> kwj_y;
    vector<double> kwj_theta;

    void LoadParams(const std::map<string, double> &params);
    void obstacle(std::vector<std::pair<double,double>> obstacles);
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
    std::map<string, double> _params;
    std::vector<std::pair<double,double>> obstacles_;
    double _dt, _ref_cte, _ref_etheta, _ref_vel, _dt_min, _dt_max, _T_max, d_min_, w_obs_;  //d_min_: distance to obstacle, w_obs: weight about avoid obstacle
    double _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
    double _max_speed, _max_angvel, _max_throttle, _bound_value;
    int _kwj_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start, _dt_start;
    Dict ipopt_options;
    Dict casadi_options;
    Function solver;
};

} // namespace kwj_local_planner
