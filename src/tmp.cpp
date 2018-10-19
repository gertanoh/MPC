class FG_eval 
{
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
	
	
	
  FG_eval(Eigen::VectorXd coeffs_in) 
	{ 
		coeffs = coeffs_in;		
	}

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
		
		
		Eigen::VectorXd weigths = Eigen::VectorXd(7);
		weigths << 1, 1, 1, 1, 1, 1, 1;
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
		fg[0] = 0;

    // Reference State Cost
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
		for (size_t k = 1 ; k < N - 1; ++k)
		{
			// steering 
			fg[0] += weigths(3) * CppAD::pow(vars[delta_start + k], 2);
			
			// Acceleration
			fg[0] += weigths(4) * CppAD::pow(vars[a_start + k], 2);						
		}
		
		// minimize actuators changes rate
		for (size_t k = 2 ; k < N - 2; ++k)
		{
			// steering 
			fg[0] += weigths(5) * CppAD::pow(vars[delta_start + k + 1] - vars[delta_start + k], 2);

			// Acceleration
			fg[0] += weigths(6) * CppAD::pow(vars[a_start + k + 1] - vars[a_start + k], 2);						
		}
		
    //
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
    for (size_t t = 1; t < N - 1; t++)
		{
      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      
			AD<double> y1 = vars[y_start + t];
      AD<double> y0 = vars[y_start + t - 1];
			
			AD<double> v1 = vars[v_start + t];
      AD<double> v0 = vars[v_start + t - 1];
			
			AD<double> psi1 = vars[psi_start + t];
      AD<double> psi0 = vars[psi_start + t - 1];	
						
			AD<double> cte1 = vars[cte_start + t];
			
			AD<double> epsi1 = vars[epsi_start + t];
      AD<double> epsi0 = vars[epsi_start + t - 1];	
			
			
			AD<double> delta0 = vars[delta_start + t - 1];		
			AD<double> a0 = vars[a_start + t - 1];
			
			// trajectory is a third degree polynomial  
			AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * pow(x0, 2) + coeffs[3] * pow(x0, 3);
			AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * pow(x0, 2) );
			
      // TODO: Setup the rest of the model constraints
			// equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);			
			fg[2 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[2 + psi_start + t] = psi1 - (psi0 + (v0 / Lf) * delta0 * dt);			
			fg[2 + v_start + t] = v1 - (v0 + a0 * dt);
			fg[2 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[2 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * (delta0 / Lf ) * dt);
    }	
  }
};
