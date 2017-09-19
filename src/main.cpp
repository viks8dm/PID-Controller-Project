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

  //create PID parameter
  PID pid;
  // TODO: Initialize the pid variable.
  // define control parameters
  // 0.35, 0.01, 0.004
  // 0.2, 0.004, 3.0
  double Kp_ = 0.12;    // 0.12
  double Ki_ = 0.006 ;  //0.006;
  double Kd_ = 40.0;    // 60.0

  // initialize PID control
  pid.Init(Kp_, Ki_, Kd_);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // change throttle angle and hence update speed
          double throttle_value = 0.7;
          // high error and slope, slow down
          if (fabs(cte)>0.35) {
            throttle_value = 0.3;
          }
          if (fabs(steer_value)>0.85) {
            if (speed > 25) {
              throttle_value = -0.1;
            }
            else {
              throttle_value = 0.1;
            }
          }
          else {
            // slope is high, car trying to go away, slow down
            if (fabs(pid.d_error)>0.1 and fabs(pid.d_error)<=0.2) {
              throttle_value = 0.0;
            }
            // car deviating a lot, apply break
            else if (fabs(pid.d_error)>0.2 and speed>30) {
                throttle_value = -0.2;
            }
          }

          // compute error parameters
          pid.UpdateError(cte);
          // compute steering angle value
          steer_value = pid.TotalError();

          // limit steering value between [-1, 1]
          if (steer_value>1) {steer_value=1;}
          if (steer_value<-1) {steer_value=-1;}

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          //std::cout << std::setprecision(4) << cte << '\t' << std::setprecision(4) << steer_value << '\t\t' << std::setprecision(2) << speed << std::endl;

          //std::cout << cte << '\t' << steer_value << '\t' << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; //default-value--> 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
