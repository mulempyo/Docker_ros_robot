#include <vector>
#include <map>
#include <Eigen/Core>
#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

namespace mpc_ros {

class MPC {
public:
    MPC();

    vector<double> mpc_x;
    vector<double> mpc_y;
    vector<double> mpc_theta;

    void LoadParams(const std::map<string, double> &params);
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
    std::map<string, double> _params;
    double _dt, _ref_cte, _ref_etheta, _ref_vel;
    double _w_cte, _w_etheta, _w_vel, _w_angvel, _w_accel, _w_angvel_d, _w_accel_d;
    double _max_speed, _max_angvel, _max_throttle, _bound_value;
    int _mpc_steps, _x_start, _y_start, _theta_start, _v_start, _cte_start, _etheta_start, _angvel_start, _a_start;
    Dict ipopt_options;
    Dict casadi_options;
    Function solver;
};

} // namespace mpc_ros
