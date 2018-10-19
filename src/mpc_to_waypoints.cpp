#include "MPC.h"
#include <cmath>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/Core"
#include "Eigen/QR"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

using CppAD::AD;

// TODO: Set N and dt
size_t N = 15 ;
double dt = 0.15 ;
//20 , 0.15 is ok
// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// NOTE: feel free to play around with this
// or do something completely different
double ref_v = 40;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;



class FG_eval 
{
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
		
		// weigths matrix 
		Eigen::VectorXd weigths = Eigen::VectorXd(7);
		weigths << 1, 1, 1, 100, 1, 1, 1;
		
		// The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Reference State Cost
		// N points so N erros
		for (size_t k = 0 ; k < N; ++k)
		{
			// cte
			fg[0] += weigths(0) * CppAD::pow(vars[cte_start + k], 2);
			
			// orientation
			fg[0] += weigths(1) * CppAD::pow(vars[epsi_start + k], 2);
			
			// velocity 
			fg[0] += weigths(2) * CppAD::pow(vars[v_start + k] - ref_v, 2);
		}
		
		// minimize actuators input
		// N points so N - 1 actuators
		for (size_t k = 0 ; k < N - 1; ++k)
		{
			// steering 
			fg[0] += weigths(3) * CppAD::pow(vars[delta_start + k], 2);
			
			// Acceleration
			fg[0] += weigths(4) * CppAD::pow(vars[a_start + k], 2);						
		}
		
		// minimize actuators changes rate
		// N points so N - 2 actuators differences
		for (size_t k = 0 ; k < N - 2; ++k)
		{
			// steering 
			fg[0] += weigths(5) * CppAD::pow(vars[delta_start + k + 1] - vars[delta_start + k], 2);
			
			// Acceleration
			fg[0] += weigths(6) * CppAD::pow(vars[a_start + k + 1] - vars[a_start + k], 2);						
		}
		
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
		
		// The rest of the constraints
		// N points, x_start contains the initial position constraint, add N - 1 constraints for the dynamic motion model 
    for (size_t k = 0; k < N - 1; ++k)
		{
			// State at time t + 1
			AD<double> x1 = vars[x_start + k + 1];
			AD<double> y1 = vars[y_start + k + 1];
			AD<double> v1 = vars[v_start + k + 1];
			AD<double> psi1 = vars[psi_start + k + 1];
			AD<double> cte1 = vars[cte_start + k + 1];
			AD<double> epsi1 = vars[epsi_start + k + 1];
			
			// State at time t
			AD<double> x0 = vars[x_start + k];
			AD<double> y0 = vars[y_start + k];
			AD<double> v0 = vars[v_start + k];
			AD<double> psi0 = vars[psi_start + k];
			AD<double> epsi0 = vars[epsi_start + k];
			// actuators at time t
			
			AD<double> delta0 = vars[delta_start + k];	
			AD<double> a0 = vars[a_start + k];
			
			// equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
			
			AD<double> f0 = coeffs[0] + (coeffs[1] * x0) + (coeffs[2] * pow(x0,2)) + (coeffs[3] * pow(x0,3));
			
      AD<double> psides0 = CppAD::atan(coeffs[1] + (2 * coeffs[2] * x0) + (3 * coeffs[3]* pow(x0,2) ));
			
			fg[1 + x_start + k + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);	
			
			fg[1 + y_start + k + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			
			fg[1 + psi_start + k + 1] = psi1 - (psi0 + (v0 * delta0 * dt) / Lf );			
			
			fg[1 + v_start + k + 1] = v1 - (v0 + a0 * dt);
			
			fg[1 + cte_start + k + 1] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
					
			fg[1 + epsi_start + k + 1] = epsi1 - ((psi0 - psides0) + v0 * delta0 * dt / Lf );
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
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = N * 6 + (N - 1) * 2;
  // Number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }
  
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

	// Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (size_t i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
	
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

	constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  
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
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
	// Return next state
  return {solution.x[x_start + 1],
					solution.x[y_start + 1],
					solution.x[psi_start + 1],
					solution.x[v_start + 1],
					solution.x[cte_start + 1],
					solution.x[epsi_start + 1]};
}


inline bool read_lake_data(std::string filename, vector<double>& ptsx, vector<double>& ptsy) 
{

	// Get file of map:
	std::ifstream in_file_map(filename.c_str());
	// Return if we can't open the file.
	if (!in_file_map) {

		return false;
	}
	
	// Declare single line of map file:
	std::string line_map;
	// skip first line
	getline(in_file_map, line_map);
	// Run over each single line:
	while(getline(in_file_map, line_map)){

		std::istringstream iss_map(line_map);

		// Declare landmark values and ID:
		double x_, y_;
		char c;
		// Read data from current line to values::
		iss_map >> x_ >> c >> y_;		
		
		ptsx.push_back(x_);
		ptsy.push_back(y_);
	}
	return true;
}

//
// Helper functions to fit and evaluate polynomials.
//

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) 
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
	{
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) 
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


int main()
{
  
	
	MPC mpc;
  size_t iters = 50;

	vector<double> ptsx;
	vector<double> ptsy;
	bool test = read_lake_data("../lake_track_waypoints.csv", ptsx, ptsy);
	if (!test )
	{
		cout << "Error in reading lake data" << endl;
		return -1;
	}
	
	// The simulator provides the six lake_track_waypoints
	
	ptsx.resize(6);
	ptsy.resize(6);
	


  // NOTE: free feel to play around with these
	// Initial value close to way points 0
  double x = 100;
  double y = 80;
  double psi = 2;
  double v = 0;
	
	// The waypoints are given from the map space, 
	// convert to car space. This makes the car sitting at the center of the car space 
	

	for (size_t i = 0; i < ptsx.size(); ++i)
	{
		auto d_x = ptsx[i] - x;
		auto d_y = ptsy[i] - y;
		
		ptsx[i] = d_x * cos(psi) + d_y * sin(psi);
		ptsy[i] = -d_x * sin(psi) + d_y * cos(psi);
	}
	
	// TODO: fit a polynomial to the above x and y coordinates
	double* ptr_x = &ptsx[0];
	double* ptr_y = &ptsy[0];
	
	Eigen::Map<Eigen::VectorXd> ptsx_vec(ptr_x, ptsx.size());
	Eigen::Map<Eigen::VectorXd> ptsy_vec(ptr_y, ptsy.size());
	

  // TODO: fit a polynomial to the above x and y coordinates
  auto coeffs = polyfit(ptsx_vec, ptsy_vec, 3);

  // TODO: calculate the cross track error
  double cte = -polyeval(coeffs, 0);
  // TODO: calculate the orientation error
  double epsi = -atan(coeffs[1]);

	// Initial point is important in MPC
  Eigen::VectorXd state(6);
  state << 0, -80, 0, v, cte, epsi;
  cout << "state " << state << endl;

  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;

    auto vars = mpc.Solve(state, coeffs);

    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);

    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);

    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
  }



  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  plt::figure();
  plt::title("Waypoints");
  plt::plot(ptsx, ptsy, "ro");
  plt::plot(x_vals,y_vals,"go");

  plt::figure();
  plt::subplot(2, 2, 1);
  plt::title("CTE");
  plt::plot(cte_vals);
  plt::subplot(2, 2, 2);
  plt::title("EPSI");
  plt::plot(epsi_vals);
  plt::subplot(2, 2, 3);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(2, 2, 4);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::show();
}
