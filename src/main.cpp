#include "json.hpp"
#include "MPC.h"
#include <cmath>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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




int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
					
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
					double steer = j[1]["steering_angle"];
					double throttle = j[1]["throttle"];
					
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */					          
					// Transform the waypoints into vehicle coordinates. The car is always at the origin the car space, thus the substraction. It reduces calculation
					vector<double> waypoints_x(ptsx.size());
					vector<double> waypoints_y(ptsx.size());
					
					for (size_t k = 0 ; k < ptsx.size(); ++k)
					{
						auto d_x = ptsx[k] - px;
						auto d_y = ptsy[k] - py;
						
						waypoints_x[k] = d_x * cos(psi) + d_y * sin(psi);
						waypoints_y[k] = -d_x * sin(psi) + d_y * cos(psi);
					}
					
					
					Eigen::Map<Eigen::VectorXd> ptsx_vec(&waypoints_x[0], ptsx.size());
					Eigen::Map<Eigen::VectorXd> ptsy_vec(&waypoints_y[0], ptsy.size());
					
					// fit 3rd polynomial to pts 
					auto coeffs = polyfit(ptsx_vec, ptsy_vec, 3);	
					
					// Find cte and epsi 				
					// x =  y = 0
					double cte = polyeval(coeffs, 0);
					double epsi = -atan(coeffs[1]);
					
					Eigen::VectorXd state(6);
					
					// How far has the vehicle traveled during the latency
          
					// starting point is x = 0, y = 0, v = 0 , psi = 0
          static const double dt	=	0.1; // latency 100ms 
          static const double Lf	=	2.67;
					
          double next_x 					=	v * dt; // y stays 0 as sin (0) = 0
          double next_psi = -(v * steer * dt) / Lf;
					double next_v = v + throttle * dt;
					double next_cte = cte + v * sin(epsi) * dt;
					// espi is the diff btw psi and psides
					double next_epsi = epsi + next_psi;


          state << next_x, 0.0, next_psi, next_v, next_cte, next_epsi;					
					
					double steer_value;
          double throttle_value;
					vector<double> result = mpc.Solve(state, coeffs);
					
					steer_value = result[0];
					std::cout <<"steer value : " << steer_value << std::endl;
					throttle_value = result[1];
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
					// noticed in the simulator that positive value equals right turn
          msgJson["steering_angle"] = -steer_value / (deg2rad(25) * Lf);
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajector
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;					
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
					// mpc solve returns predicted path 
					for (size_t k = 2; k < result.size(); ++k)
					{
						if (k % 2 == 0)
						{
							mpc_x_vals.push_back(result[k]);
						}
						else 
						{
							mpc_y_vals.push_back(result[k]);
						}
					}
					
					
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
					static const int refer_points = 25;
					for (int k = 0 ; k < refer_points; ++k)
					{
						next_x_vals.push_back(2.5*k);
						next_y_vals.push_back(polyeval(coeffs, 2.5*k));
					}
					
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
