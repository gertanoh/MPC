# Model Predictive Control 

------------------------------------------------------------------------------------

The goal is to implement in model predictive control to drive a car around a path.
The implementation used data and code skeleton from Udacity Github 
at [https://github.com/udacity/CarND-MPC-Project).


## Kinematic Model

### State
* x 		position 
* y 		position
* psi 	orientation of the vehicle 
* v   	velocity
* cte		cross track error (distance from the ideal path)
* epsi 	orientation error 
* delta	steering angle
* a			acceleration (throttle or brake pedals)

### Model Equations
* x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
* y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
* psi_[t+1] = psi[t] + (v[t] / Lf) * delta[t] * dt
* v_[t+1] = v[t] + a[t] * dt
* cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
* epsi[t+1] = psi[t] - psides[t] + (v[t] / Lf) * delta[t] * dt 


## Constraints

* a 							: [-1, 1]
* steering angle	: [-25°, 25°]


## Prediction Horizon
* P is the prediction horizon, how far in the distance do MPC predict
* In this model, the control horizon is equal to the prediction horizon
* An improvement would be to to define a control horizon and use move blocking
* P = N * dt
* N is the number of timesteps the model predicts ahead. As N increases, the model predicts further ahead.
* dt is the length of each timestep. dt is also the sample time at which the actuators are applied.

## Cost function

* weights are used to balance the goals of the optimizer
* cost = sum_over_N(w_0 * cte^2) + sum_over_N(w_1 * epsi^2) + sum_over_N(w_2 * (v - vref)^2) + sum_over_N-1(w_3 * delta^2) + sum_over_N-1(w_4 * a^2) + sum_over_N-2(w_5 * (a_t+1 - a_t)^2) + sum_over_N-2(w_5 * (delta_t+1 - delta_t)^2)


## Preprocessing and Polynomial Fitting
The road path is fitted with a polynomial of degree 3.

## Hyper Parameters Tuning
A difficult part of the project.
* In driving, the required prediction horizon would be two seconds. However, in this project, 2 seconds was too much. So I chose 1 s.
* From there, N was set to 10 and dt to 0.1.
* The weights tuning was done manually and turned to take some time. The main idea was to drive around smoothly

## Latency
* 100ms of latency has to be taken into account. This reflects the time taken by the actuators command to arrive in the physical unit.

## Result
* The present weight values allow good driving on the first part of the road. However, I was not able to find the right weight values for smooth riding on the first and second part.
* An optimization would be to use an optimizer to find the correct weights values.

## Dependency


* All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
* Linux: make is installed by default on most Linux distros
* Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
* Linux: gcc / g++ is installed by default on most Linux distros
* Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
* Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
* Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
* If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
* Mac: `brew install ipopt`
* Linux
* You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
* Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
* Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
* Mac: `brew install cppad`
* Linux `sudo apt-get install cppad` or equivalent.
* Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.
