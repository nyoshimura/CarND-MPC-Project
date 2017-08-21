#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

size_t start_[8]; // x, y, psi, v, cte, epsi, delta, a
for(int i = 0; i < 8; i++)
{
  start_[i] = N*i;
  if(i==7) start_[i] = N*i - 1;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;
    // calculate cost
    for(int t = 0; t < N; t++)
    {
      fg[0] += CppAD::pow(vars[start_[3] + t] - ref_v, 2); // velocity control
      fg[0] += CppAD::pow(vars[start_[4] + t], 2); // position control
      fg[0] += CppAD::pow(vars[start_[5] + t], 2); // yaw rate control
    }
    for(int t = 0; t < N - 1; t++)
    {
      fg[0] += delta_gain * CppAD::pow(vars[start_[6] + t], 2); // minimize the use of actuators
    }
    // minimize the value gap between sequential actuations (smoothing)
    for(int t = 0; t < N - 2; t++)
    {
      fg[0] += delta_gap_gain * CppAD::pow(vars[start_[6] + t + 1] - vars[start_[6] + t], 2);
      fg[0] += CppAD::pow(vars[start_[7] + t + 1] - vars[start_[7] + t], 2);
    }

    // initial constraints
    for(int i = 0; i < 8; i++)
    {
      fg[start_[i] + 1] = vars[start_[i]];
    }
    // rest of constraints
    AD<double> state_now[8];
    AD<double> state_last[8];
    for(int t = 0; t < N - 1; t++)
    {
      for(int i = 0; i < 8; i++)
      {
        state_now[i] = vars[start_[i] + t + 1]; // state at time t+1
        state_last[i] = vars[start_[i] + t]; // state at time t
      }
      AD<double> f_ = coeffs[0] + coeffs[1] * state_last[0] + coeffs[2] * CppAD::pow(state_last[0], 2) + coeffs[3] * CppAD::pow(state_last[0], 3);
      AD<double> psides_ = CppAD::atan(3 * coeffs[3] * CppAD::pow(state_last[0], 2) + 2 * coeffs[2] * state_last[0] + coeffs[1]);
      // MPC
      fg[start_[0] + t + 2] = state_now[0] - (state_last[0] + state_last[3] * CppAD::cos(state_last[2]) * dt); // x
      fg[start_[1] + t + 2] = state_now[1] - (state_last[1] + state_last[3] * CppAD::sin(state_last[2]) * dt); // y
      fg[start_[2] + t + 2] = state_now[2] - (state_last[2] - state_last[3] * state_last[6] / Lf * dt); // psi
      fg[start_[3] + t + 2] = state_now[3] - (state_last[3] + state_last[7] * dt); // v
      fg[start_[4] + t + 2] = state_now[4] - (f_ - state_last[1] + state_last[3] * CppAD::sin(state_last[5]) * dt); // cte
      fg[start_[5] + t + 2] = state_now[5] - (state_last[2] - psides_ + state_last[3] * state_last[6] / Lf * dt); // epsi
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N - 1);
  // Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set lower and upper limits for variables.
  // for non-actuators values
  for(int i = 0; i < start_[6]; i++)
  {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // for actuators values (-25 & 25 degrees)
  for(int i = start_[6]; i < start_[7]; i++)
  {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // for acc/decc
  for(int i = start_[7]; i < n_vars; i++)
  {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  predicted_x.clear();
  predicted_y.clear();
  for(int i = 0; i < N; i++)
  {
    predicted_x.push_back(solution.x[start_[0] + i]);
    predicted_y.push_back(solution.x[start_[1] + i]);
  }
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return { solution.x[start_[6]], solution.x[start_[7]] };
}
