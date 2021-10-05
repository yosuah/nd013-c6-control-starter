/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 				Aaron Brown
 **********************************************/

/**
 * @file main.cpp
 **/

#include <string>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <iostream>
#include <fstream>
#include <typeinfo>

#include "json.hpp"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include "Eigen/QR"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"

#include <limits>
#include <iostream>
#include <fstream>
#include <uWS/uWS.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <time.h>
#include <string>

using namespace std;
using json = nlohmann::json;

#define _USE_MATH_DEFINES

string hasData(string s) {
  auto found_null = s.find("null");
    auto b1 = s.find_first_of("{");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos) {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}


template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double angle_between_points(double x1, double y1, double x2, double y2){
  return atan2(y2-y1, x2-x1);
}

BehaviorPlannerFSM behavior_planner(
      P_LOOKAHEAD_TIME, P_LOOKAHEAD_MIN, P_LOOKAHEAD_MAX, P_SPEED_LIMIT,
      P_STOP_THRESHOLD_SPEED, P_REQ_STOPPED_TIME, P_REACTION_TIME,
      P_MAX_ACCEL, P_STOP_LINE_BUFFER);

// Decalre and initialized the Motion Planner and all its class requirements
MotionPlanner motion_planner(P_NUM_PATHS, P_GOAL_OFFSET, P_ERR_TOLERANCE);

bool have_obst = false;
vector<State> obstacles;

void path_planner(vector<double>& x_points, vector<double>& y_points, vector<double>& v_points, double yaw, double velocity, State goal, bool is_junction, string tl_state, vector< vector<double> >& spirals_x, vector< vector<double> >& spirals_y, vector< vector<double> >& spirals_v, vector<int>& best_spirals){

  State ego_state;

  ego_state.location.x = x_points[x_points.size()-1];
  ego_state.location.y = y_points[y_points.size()-1];
  ego_state.velocity.x = velocity;

  if( x_points.size() > 1 ){
  	ego_state.rotation.yaw = angle_between_points(x_points[x_points.size()-2], y_points[y_points.size()-2], x_points[x_points.size()-1], y_points[y_points.size()-1]);
  	ego_state.velocity.x = v_points[v_points.size()-1];
  	if(velocity < 0.01)
  		ego_state.rotation.yaw = yaw;

  }

  Maneuver behavior = behavior_planner.get_active_maneuver();

  goal = behavior_planner.state_transition(ego_state, goal, is_junction, tl_state);

  if(behavior == STOPPED){

  	size_t max_points = 20;
  	double point_x = x_points[x_points.size()-1];
  	double point_y = y_points[x_points.size()-1];
  	while( x_points.size() < max_points ){
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);

  	}
  	return;
  }

  auto goal_set = motion_planner.generate_offset_goals(goal);

  auto spirals = motion_planner.generate_spirals(ego_state, goal_set);

  auto desired_speed = utils::magnitude(goal.velocity);

  State lead_car_state;  // = to the vehicle ahead...

  if(spirals.size() == 0){
  	cout << "Error: No spirals generated " << endl;
  	return;
  }

  for(size_t i = 0; i < spirals.size(); i++){

    auto trajectory = motion_planner._velocity_profile_generator.generate_trajectory( spirals[i], desired_speed, ego_state,
                                                                                    lead_car_state, behavior);

    vector<double> spiral_x;
    vector<double> spiral_y;
    vector<double> spiral_v;
    for(size_t j = 0; j < trajectory.size(); j++){
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }

    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);

  }
  
  best_spirals = motion_planner.get_best_spiral_idx(spirals, obstacles, goal);
  int best_spiral_idx = -1;
  
  if(best_spirals.size() > 0)
  	best_spiral_idx = best_spirals[best_spirals.size()-1];
  
  // cout << "Spirals: " << spirals.size() << ", best spirals : " << best_spirals.size() << ", idx: " << best_spiral_idx << endl;

  if (best_spiral_idx >= 0) {
    size_t index = 0;
    size_t max_points = 20;
    size_t add_points = spirals_x[best_spiral_idx].size();
    while( x_points.size() < max_points && index < add_points ){
      double point_x = spirals_x[best_spiral_idx][index];
      double point_y = spirals_y[best_spiral_idx][index];
      double velocity = spirals_v[best_spiral_idx][index];
      index++;
      x_points.push_back(point_x);
      y_points.push_back(point_y);
      v_points.push_back(velocity);
    }
  }
  
  // std::cout << "Output generated.." << std::endl;


}

void set_obst(vector<double> x_points, vector<double> y_points, vector<State>& obstacles, bool& obst_flag)
{
	for( size_t i = 0; i < x_points.size(); i++){
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}

int main (int argc, char* argv[])
{
  if (argc != 1 && argc != 9) {
    std::cout << "incorrect number of arguments passed - either 0 or 8 are accepted" << std::endl;
    return 1;
  }
  std::cout << "arguments: " << argv[1] << ", " << argv[2] << ", " << argv[3] << ", " << argv[4] << ", " << argv[5] << ", " << argv[6] << ", " <<  argv[7] << " " << argv[8] << std::endl;
  std::cout << "starting server" << std::endl;
  uWS::Hub h;

  double last_sim_time = -1.0; // NOTE: the simulation time of the last message received
  int i = 0;

  fstream file_steer;
  file_steer.open("steer_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_steer.close();
  fstream file_throttle;
  file_throttle.open("throttle_pid_data.txt", std::ofstream::out | std::ofstream::trunc);
  file_throttle.close();

  // initialize pid steer
  PID pid_steer = PID();
  double steer_kp = 30.0;
  double steer_ki = 0.0;
  double steer_kd = 0.0;
  if (argc >= 7) {
    steer_kp = std::stod(argv[1]);
    steer_ki = std::stod(argv[2]);
  	steer_kd = std::stod(argv[3]);
  }
  
  // NOTE: The task description required to set 1.2 as the steering limit, but according to the Carla 
  // documentation the valid range is [-1.0, 1.0]
  pid_steer.Init(steer_kp, steer_ki, steer_kd, /*output_lim_max*/ 1.0, /*output_lim_min*/ -1.0);

  // initialize pid throttle
  PID pid_throttle = PID();
  double throttle_kp = 0.02;
  double throttle_ki = 0.0;
  double throttle_kd = 0.0;
  if (argc >= 7) {
    throttle_kp = std::stod(argv[4]);
    throttle_ki = std::stod(argv[5]);
  	throttle_kd = std::stod(argv[6]);
  }
  pid_throttle.Init(throttle_kp, throttle_ki, throttle_kd, /*output_lim_max*/ 1, /*output_lim_min*/ -1);
  
  double cum_abs_steer_error = 0.0;
  double cum_abs_throttle_error = 0.0;
  double previous_x_position = 0.0;
  double previous_y_position = 0.0;
  double total_distance = 0.0;
  bool is_first = true;
  
  std::string throttle_op_mode = argv[7];
  std::string steer_op_mode = argv[8];

  h.onMessage([&pid_steer, &pid_throttle, &i, 
               &cum_abs_steer_error, &cum_abs_throttle_error, &previous_x_position, &previous_y_position,
               &total_distance, &is_first, &argv, &throttle_op_mode, &steer_op_mode, &last_sim_time](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t /*length*/, uWS::OpCode /*opCode*/)
  {
    // std::cout << "Message received in client" << std::endl;
    
        auto s = hasData(data);

        if (s != "") {

          auto data = json::parse(s);

          // create file to save values
          fstream file_steer;
          file_steer.open("steer_pid_data.txt");
          fstream file_throttle;
          file_throttle.open("throttle_pid_data.txt");

          vector<double> x_points_received = data["traj_x"];
          vector<double> y_points_received = data["traj_y"];
          vector<double> v_points_received = data["traj_v"];
          double yaw = data["yaw"]; // NOTE: This is NOT the same yaw that Carla calculates, which is VERY confusing. 
          							// I have additionally sent the yaw calculated by Carla, because that works well.
          double carla_yaw = data["carla_yaw"];
          double velocity = data["velocity"];
          double sim_time = data["time"];
          double waypoint_x = data["waypoint_x"];
          double waypoint_y = data["waypoint_y"];
          double waypoint_t = data["waypoint_t"];
          bool is_junction = data["waypoint_j"];
          string tl_state = data["tl_state"];

          double x_position = data["location_x"];
          double y_position = data["location_y"];
          // double z_position = data["location_z"];
          
          // NOTE: originally the simulator sent all previously calculated positions back, and the path planner only appended
          // new points at the end of that vector. This however meant that there was much less feedback, for example if the vehicle
          // behaved different than what the planner originally expected then it took a long time to correct that. I wanted better
          // control, so in every iteration I only pass the current state to the planner and force it to start from scratch.
          vector<double> x_points {x_position};
          vector<double> y_points {y_position};
          vector<double> v_points {velocity};

          // FIXME
//           vector<double> x_points {x_points_received};
//           vector<double> y_points {y_points_received};
//           vector<double> v_points {v_points_received};
          
          if (is_first) {
            is_first = false;
          } else {
            total_distance += std::sqrt(std::pow(x_position - previous_x_position, 2) + std::pow(y_position - previous_y_position, 2));
          }
          previous_x_position = x_position;
          previous_y_position = y_position;

          if(!have_obst){
          	vector<double> x_obst = data["obst_x"];
          	vector<double> y_obst = data["obst_y"];
          	set_obst(x_obst, y_obst, obstacles, have_obst);
          }

          State goal;
          goal.location.x = waypoint_x;
          goal.location.y = waypoint_y;
          goal.rotation.yaw = waypoint_t;

          vector< vector<double> > spirals_x;
          vector< vector<double> > spirals_y;
          vector< vector<double> > spirals_v;
          vector<int> best_spirals;
          
          path_planner(x_points, y_points, v_points, yaw, velocity, goal, is_junction, tl_state, spirals_x, spirals_y, spirals_v, best_spirals);
          
          // In case all spirals are invalid then send the points received (calculated in the previous round) as a fallback in order
          // to avoid breaking the simulation.
          // This was important because originally the final waypoint (goal) sent by the simulator could get really far ahead,
          // in which case it frequently happened that there were colliding obstacles in all spirals. Now this is fixed
          // by limiting the total lookahead distance in the simulator, which caused the problem to go away.
          if (x_points.size() == 1) {
            x_points = x_points_received;
            y_points = y_points_received;
            v_points = v_points_received;
          }
          
          // cout << "x_points received: " << x_points_received.size() << " sent " << x_points.size() << endl;
          
          // Save time and compute delta time
          // NOTE: this was the original code, which I believe is seriously flawed.
          // This code measures time difference in the client, as opposed to the simulation, which is what we are actually interested in.
          // On top of that this code measures time difference in integer seconds, so new_delta_time is very coarse grained and can
          // even be 0. I have changed this to measuring the actual simulation time difference.
          /*time(&timer);
          new_delta_time = difftime(timer, prev_timer);
          prev_timer = timer;*/
          double new_delta_time;
          if (last_sim_time < 0.0) {
            new_delta_time = 0.0;
          } else {
            new_delta_time = sim_time - last_sim_time;
          }
          last_sim_time = sim_time;

          ////////////////////////////////////////
          // Steering control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_steer.UpdateDeltaTime(new_delta_time);

          // Compute steer error

          double error_steer;
          double steer_output;
          if (steer_op_mode == "straight") {
            // positive is right, negative is left
          	steer_output = 0.0;
            error_steer = 0.0;
          }
          else {
            error_steer = utils::get_steer_error(x_position, y_position, carla_yaw, 
                                                        // x_points[0] + 10, 9
                                                        // x_points[5], y_points[5]
                                                        x_points[x_points.size()-1], y_points[y_points.size()-1]
                                                       );
            pid_steer.UpdateError(error_steer);    
            steer_output = pid_steer.TotalError();
          }
          cum_abs_steer_error += std::abs(error_steer);

          // Save data
          file_steer.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j) {
              file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
          }
          file_steer  << i ;
          file_steer  << " " << error_steer;
          file_steer  << " " << steer_output << endl;

          ////////////////////////////////////////
          // Throttle control
          ////////////////////////////////////////

          // Update the delta time with the previous command
          pid_throttle.UpdateDeltaTime(new_delta_time);

          // Compute error of speed
          double error_throttle;
          double throttle;
          // If the operation mode is constant speed then try to maintain it. Turns out that simply applying a constant throttle
          // results in an almost constant speed of 3, so I cheat here. In this case set error_throttle to 0, because we are not using
          // the throttle controller, so are only interested in the steer error.
          if (throttle_op_mode == "constant_speed") {
            error_throttle = 0.0;
            throttle = 0.35;
          }
          // When the operation mode is constant speed reference then submit a constant speed reference of 3m/s and a waypoint
          // 20m ahead, regardless of what output the path planner generated. This makes sense when the world is empty (no
          // obstacles to avoid), and only on the first street.
          else if (throttle_op_mode == "constant_speed_reference") {
            error_throttle = utils::get_throttle_error(x_position, y_position, /*v_points[0]*/ velocity,
                                                       x_position + 20, y_position, 3
                                                      );
            // Compute control to apply
            pid_throttle.UpdateError(error_throttle);
            throttle = pid_throttle.TotalError(true);
          } 
          // In all other cases submit the path planned by the planner to the controller.
          else {
            error_throttle = utils::get_throttle_error(x_position, y_position, /*v_points[0]*/ velocity, 
                                                       x_points[x_points.size()-1], y_points[y_points.size()-1], v_points[v_points.size()-1]
                                                       //x_points[2], y_points[2], v_points[2]
                                                      );
            // Compute control to apply
            pid_throttle.UpdateError(error_throttle);
            throttle = pid_throttle.TotalError();
          }
          cum_abs_throttle_error += std::abs(error_throttle);

          double brake_output;
		  double throttle_output;
          
          // Adapt the negative throttle to break
          if (throttle > 0.0) {
            throttle_output = throttle;
            brake_output = 0;
          } else {
            throttle_output = 0;
            brake_output = 0;
            if (throttle < -0.2)
            	brake_output = -throttle / 4.0;
          }
          
          //throttle_output = 0.3;
          //brake_output = 0;
          
          // std::cout << "Controller done.." << std::endl;
          
          // Save data
          file_throttle.seekg(std::ios::beg);
          for(int j=0; j < i - 1; ++j){
              file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
          }
          file_throttle  << i ;
          file_throttle  << " " << error_throttle;
          file_throttle  << " " << brake_output;
          file_throttle  << " " << throttle_output << endl;

          // Send control
          json msgJson;
          msgJson["brake"] = brake_output;
          msgJson["throttle"] = throttle_output;
          msgJson["steer"] = steer_output;

          msgJson["trajectory_x"] = x_points;
          msgJson["trajectory_y"] = y_points;
          msgJson["trajectory_v"] = v_points;
          msgJson["spirals_x"] = spirals_x;
          msgJson["spirals_y"] = spirals_y;
          msgJson["spirals_v"] = spirals_v;
          msgJson["spiral_idx"] = best_spirals;
          msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();
          std::vector<double> vec {};
          msgJson["steer_heading_vec"] = "";
          msgJson["steer_target"] = "";

          // min point threshold before doing the update
          // for high update rate use 19 for slow update rate use 4
          // NOTE: this setting was not respected due to a bug in simulatorAPI.py. I have fixed that.
          msgJson["update_point_thresh"] = 16;

          i = i + 1;
          file_steer.close();
          file_throttle.close();
          
          // When reaching a fixed number of steps or a hard-coded time, end the simulation, so all of the executions
          // are roughly comparable. This is useful for doing parameter optimization, as the code is executed many
          // times in that case
          if (i > 300 || sim_time > 40) {
            fstream file_cumulated;
			file_cumulated.open("cumulated_data.txt", std::ofstream::out | std::ofstream::app);
            
            // NOTE: The header is added by the external Python runner script, because this program is executed several times, but we only need one header
            // file_cumulated  << "steer_kp, steer_ki, steer_kd, throttle_kp, throttle_ki, throttle_kd, steer_mae, throttle_mae" << std::endl;
            file_cumulated  << argv[1] << "," << argv[2] << "," << argv[3] << "," << argv[4] << "," << argv[5] << "," << argv[6] << "," ;
            file_cumulated  << cum_abs_steer_error / total_distance << ",";
            file_cumulated  << cum_abs_throttle_error / total_distance << std::endl;
            file_cumulated.close();
            
            std::cout << "Sending closing message at time " << sim_time << ", iteration " << i << std::endl;
            msgJson["close"] = 1;
            
          }
          
          auto msg = msgJson.dump();

      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      // std::cout << "Message sent to server" << std::endl;

    }

  });


  h.onConnection([](uWS::WebSocket<uWS::SERVER> /*ws*/, uWS::HttpRequest /*req*/)
  {
      cout << "Connected!!!" << endl;
    });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int /*code*/, char * /*message*/, size_t /*length*/)
    {
      ws.close();
      cout << "Disconnected" << endl;
    });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
    {
      cout << "Listening to port " << port << endl;
      h.run();
    }
  else
    {
      cerr << "Failed to listen to port" << endl;
      return -1;
    }


}
