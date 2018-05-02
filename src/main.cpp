#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  /************************************************************
  * Initialize PID Controllers
  ***********************************************************/
  PID pid_steering, pid_throttle;

  /* PID Gains */
  // double Kp_s = 0.50; 
  // double Ki_s = 0.01; 
  // double Kd_s = 1.00; 
  double Kp_t = 0.5;
  double Ki_t = 0.0;
  double Kd_t = 2.0;

  /* Twiddle Parameters */
  // double p[3] = {0.05, 1.5, 0.005};
  // double p[3] = {0.076, 1.87, 0.006};
  // double p[3] = {0.04, 3.2, 0.008};
  double p[3] = {0.15, 2.2, 0.015};
  double dp[3] = {0.01, 0.2, 0.001};
  double error[3] = {1.0E50, 0.0, 0.0};
  unsigned int it = 0;
  unsigned int index = 0;

  /* Initalize Controllers */
  pid_steering.Init(p[0], p[2], p[1]);
  pid_throttle.Init(Kp_t, Ki_t, Kd_t);
  
  #define ITERATIONS   500
  //#define TWIDDLE_TUNE

  h.onMessage([&pid_steering, &pid_throttle, &p, &dp, &it, &index, &error](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;
          
          /************************************************************
           * Calculate Steering Angle
           ***********************************************************/
          pid_steering.UpdateError(cte);
          steer_value = pid_steering.TotalError();

          #ifdef TWIDDLE_TUNE
            /* Only Run Twiddle @ Speed */
            if (speed >= 15) {

              // Update error.
              error[1] += error[1] + (double)fabs(steer_value);
              
              if (it == (ITERATIONS - 1)) {

                /* Check Error Status */
                if (error[1] < error[0] && error[2] == 0.0) {
                  error[0] = error[1];        // Set new best error.
                  dp[index] *= 1.05;          // Update new error step.
                  p[index] += dp[index];      // Update gain.
                  error[2] = 1.0;
                }
                else if (error[1] < error[0] && error[2] == 1.0) {
                  error[0] = error[1];          // Set new best error.
                  p[index] -= 2.0 * dp[index];  // Move two steps in other direction.
                  error[2] = 1.0;               // Set last state.
                }
                else if (error[1] >= error[0]) {
                  p[index] += dp[index];      // Restore original gain value.
                  dp[index] *= 0.95;          // Update new error step.
                  error[2] = 0.0;             // Reset last state.
                  index = (index + 1) % 3;    // Move to next parameter.
                }
                p[index] -= 0.2 * dp[index];
                dp[index] *= 0.9;

                // Update parameters.
                pid_steering.SetGains(p);
                error[1] = 0.0;
                std::cout << "[Kp, Ki, Kd]: " << pid_steering.Kp << ", " << pid_steering.Ki << ", " << pid_steering.Kd << std::endl;  

              }
              it = (it + 1) % ITERATIONS;
            } else {
              // reset counter and error
              error[1] = 0.0;
              it = 0;
            }
          #endif

          // Map steering to range.
          if (steer_value < -1.0)
            steer_value = -1.0;
          else if (steer_value > 1.0)
            steer_value = 1.0;

          /************************************************************
           * Throttle Value
           ***********************************************************/
          pid_throttle.UpdateError(cte);
          throttle_value = pid_throttle.TotalError();

          // Map throttle.
          double max_speed = 0.45;
          if (throttle_value < -max_speed)
            throttle_value = -max_speed;
          if (throttle_value > max_speed)
            throttle_value = max_speed;
          throttle_value = max_speed - fabs(throttle_value) + 0.1;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
