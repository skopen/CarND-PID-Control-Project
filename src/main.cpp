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

double FACTOR = 0.2;
double p[3] = {0.1, 0.004, 1};
//double dp[3] = {1, 1, 1};
double dp[3] = {p[0]*FACTOR, p[1]*FACTOR, p[2]*FACTOR};
bool PIDHyperparamTuning = true;
int paramTuned = -1;
int iter = 0;
int MAX_ITER = 100; // this is N in the Python version
double BEST_ERROR = 99999999;
double BEST_PARAMS[] = {0, 0, 0};
double ERROR = 0;
int ROUND = 1;
bool TUNING_COMPLETE = false;

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

  PID pid;
  // TODO: Initialize the pid variable.

  // pid.Init(0.4, 0.004, 4) --> very bumpy ride, swerves widly to left and right
  // pid.Init(0.4, 0.04, 4); --> less swervy ride
  // pid.Init(0.4, 0.4, 4); --> can't ride, so bad
  // pid.Init(0.4, 0.4, 0.4); --> really bad, can't drive
  // pid.Init(0.4, 0.004, 2); --> somewhat stable
  // pid.Init(0.1, 0.004, 1);  --> most stable so far
  // pid.Init(0.07, 0.004, 1); --> It's OK.

  pid.Init(p[0], p[1], p[2]);

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
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          if (!TUNING_COMPLETE)
          {
        	  //std::cout << "ITER: " << iter << std::endl;

        	  iter++;

			  if (iter == 1)
			  {
				  paramTuned = (paramTuned + 1)%3;

				  if ((dp[0] + dp[1] + dp[2]) < 0.2 && (paramTuned == 0))
				  {
					  std::cout << "=================================================================." << std::endl;
					  std::cout << "=================================================================." << std::endl;
					  std::cout << "TUNING COMPLETE. BEST PARAMS: " << BEST_PARAMS[0] << ", " << BEST_PARAMS[1] << ", " << BEST_PARAMS[2];
					  std::cout << "=================================================================." << std::endl;
					  std::cout << "=================================================================." << std::endl;

					  TUNING_COMPLETE = true;

					  pid.Init(BEST_PARAMS[0], BEST_PARAMS[1], BEST_PARAMS[2]);
				  }

				  p[paramTuned] += dp[paramTuned];
			  }

			  if (iter == MAX_ITER)
			  {
				  ERROR += pid.TotalError();
			  }

			  if (iter == MAX_ITER*2 && ROUND == 1)
			  {
				  if (ERROR < BEST_ERROR)
				  {
					  BEST_ERROR = ERROR;
					  BEST_PARAMS[0] = p[0]; BEST_PARAMS[1] = p[1]; BEST_PARAMS[2] = p[2];

					  dp[paramTuned] *= 1.1;

					  std::cout << "BEST_ERROR: " << BEST_ERROR << " PARAMS: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
				  }
				  else
				  {
					  p[paramTuned] -= 2*dp[paramTuned];
					  pid.Init(p[0], p[1], p[2]);
					  ROUND = 2;
				  }

				  iter = 0;
				  ERROR = 0;
			  }
			  else if (iter == MAX_ITER*2 && ROUND == 2)
			  {
				  if (ERROR < BEST_ERROR)
				  {
					  BEST_ERROR = ERROR;
					  BEST_PARAMS[0] = p[0]; BEST_PARAMS[1] = p[1]; BEST_PARAMS[2] = p[2];

					  dp[paramTuned] *= 1.1;

					  std::cout << "BEST_ERROR: " << BEST_ERROR << " PARAMS: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
				  }
				  else
				  {
					  p[paramTuned] += dp[paramTuned];
					  dp[paramTuned] *= 0.9;
					  pid.Init(p[0], p[1], p[2]);
					  ROUND = 1;
				  }

				  iter = 0;
				  ERROR = 0;
			  }
          }

          pid.UpdateError(cte);
          steer_value = pid.getControl();

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Speed: " << speed << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
