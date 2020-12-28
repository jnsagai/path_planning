#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

typedef enum e_StateType{
  RD,
  LK,
  LCL,
  LCR,
  PLCL,
  PLCR
}t_StateType;

typedef struct s_NextCarValues{
  vector<double> next_x_vals;
  vector<double> next_y_vals;
}t_NextCarValues;

typedef struct s_WayPoints{
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
}t_WayPoints;

typedef struct s_CarState{
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
}t_CarState;

typedef struct s_PreviousCarPath{
  vector<double> previous_path_x;
  vector<double> previous_path_y;
}t_PreviousCarPath;


#define saturation(x, y) (x > y ? y : x)
t_NextCarValues CalcNextCarValues(const int &lane, const double &ref_vel, const t_WayPoints &map_waypoints, const t_CarState &car_state, const t_PreviousCarPath &previous_path);
bool CheckLaneChange(const t_StateType &checkLane,const int &currentLane,const vector<vector<double>> sensor_fusion, const t_CarState &car_state);
bool IsSafeToChangeLane(vector<vector<double>> cars_info, const t_CarState &car_state, const double &offset_safe_s_sup, const double &offset_safe_s_inf);
t_StateType GetNewState(const t_CarState &car_state, const vector<vector<double>> sensor_fusion, const int &currentLane, vector<double> costFuncWeights);
double CarSpeedCost(double targetCarSpeed);
double CarDistanceCost(double targetCarDistance);
int GetCarLane(double d);

//Weights for the Cost Functions (Speed cost, distance cost and Change Lane cost)
vector<double> weights = {5, 10, 3};

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //start with lane 1
  int lane = 1;

  t_StateType CarState = RD;

  //Have a reference velocity to target
  double ref_vel = 0.0; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel, &CarState]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if(prev_size > 0){
            car_s = end_path_s;
          }

          t_WayPoints way_p = {map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy};
          t_CarState state_car = {car_x, car_y, car_s, car_d, car_yaw, car_speed};
          t_PreviousCarPath prev_path = {previous_path_x, previous_path_y};
          
          switch(CarState){
            case RD:
            {
                ref_vel += 0.224;
                CarState = LK;
              break;
            }
            case LK:
            {
              bool too_close = false;

              //find ref_v to use
              for(unsigned int i = 0; i < sensor_fusion.size(); ++i){
                //car is in my lane
                float d = sensor_fusion[i][6];
                if(d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2)){
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx + vy*vy);
                  double check_car_s = sensor_fusion[i][5];

                  //If  using previous points can project s value out
                  check_car_s += ((double)prev_size * 0.02 * check_speed);
                  //check s values greater than mine and s gap
                  if((check_car_s > car_s) && ((check_car_s - car_s) < 20)){
                    too_close = true;
                  }
                }
              }
              if(too_close){
                ref_vel -= 0.4;
              }
              else if(ref_vel < 49.5){
                ref_vel += 0.4;
              }

              //Get the new state
              t_StateType newState = GetNewState(state_car, sensor_fusion, lane, weights);
              if(newState == LCL){
                lane--;
                CarState = LCL;
              }
              else if(newState == LCR){
                lane++;
                CarState = LCR;
              }
              break;
            }
            case LCL:
            case LCR:
            {
              //Keep in this state until the d value is near the center of the lane
              //After change state to Keep Lane
              if(car_d <= (4 + (4 * lane)) && car_d >= (4 * lane)){
                CarState = LK;
              }
              if(ref_vel < 49.5){
                ref_vel += 0.4;
              }
              break;
            }            
            case PLCL:
            {
              break;
            }   
            case PLCR:
            {
              break;
            }   
          }

          json msgJson;
          t_NextCarValues nextValues = CalcNextCarValues(lane, ref_vel, way_p, state_car, prev_path);

          msgJson["next_x"] = nextValues.next_x_vals;
          msgJson["next_y"] = nextValues.next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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

t_NextCarValues CalcNextCarValues(const int &lane, const double &ref_vel, const t_WayPoints &map_waypoints, const t_CarState &car_state, const t_PreviousCarPath &previous_path){
  // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
  // Later we will interoplate these waypoints with a spline and fill it in with more points that control speed.
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states
  // either we will reference the starting point as where the car is or at the previous path end point
  double ref_x = car_state.car_x;
  double ref_y = car_state.car_y;
  double ref_yaw = deg2rad(car_state.car_yaw);

  int prev_size = previous_path.previous_path_x.size();

  // if previous size is almost empty, use the car as starting reference
  if(prev_size < 2) {
    // use two points that make the path tangent to the car
    double prev_car_x = car_state.car_x - cos(car_state.car_yaw);
    double prev_car_y = car_state.car_y - sin(car_state.car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_state.car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_state.car_y);
  }
  // use the previous path's end point as starting reference
  else {
    // Redefine reference state as previous path end point
    ref_x = previous_path.previous_path_x[prev_size - 1];
    ref_y = previous_path.previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path.previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path.previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  //In Frenet add evenly 30m spaced points ahead of the starting reference
  vector<double> next_wp0 = getXY(car_state.car_s + 30,(2 + 4 * lane), map_waypoints.map_waypoints_s, map_waypoints.map_waypoints_x, map_waypoints.map_waypoints_y);
  vector<double> next_wp1 = getXY(car_state.car_s + 60,(2 + 4 * lane), map_waypoints.map_waypoints_s, map_waypoints.map_waypoints_x, map_waypoints.map_waypoints_y);
  vector<double> next_wp2 = getXY(car_state.car_s + 90,(2 + 4 * lane), map_waypoints.map_waypoints_s, map_waypoints.map_waypoints_x, map_waypoints.map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for(unsigned int i = 0;i < ptsx.size(); ++i){
    //shift car reference angle to 0 degrees

    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  //create a spline

  tk::spline s;

  //set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  //Define the actual (x,y) points we will use for the planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  //Start with all of the previous path points from last time
  for (unsigned int i = 0; i < previous_path.previous_path_x.size(); ++i){
    next_x_vals.push_back(previous_path.previous_path_x[i]);
    next_y_vals.push_back(previous_path.previous_path_y[i]);
  }

  //Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

  double x_add_on = 0;

  //Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
  for(unsigned int i = 1;i <= 50 - previous_path.previous_path_x.size(); ++i){
    double N = (target_dist / (0.02 *  ref_vel / 2.24));
    double x_point = x_add_on + (target_x / N);
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    //rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return {next_x_vals, next_y_vals};
}

t_StateType GetNewState(const t_CarState &car_state, const vector<vector<double>> sensor_fusion, const int &currentLane, vector<double> costFuncWeights)
{
  //Get the nearest cars in front of ego car
  t_CarState left_car, right_car, center_car;

  //Lane Costs
  double leftLaneCost, rightLaneCost, centerLaneCost;

  //New State
  t_StateType newState = LK;
  
  //Initialize with high s and speed values
  left_car.car_s = 9999;
  right_car.car_s = 9999;
  center_car.car_s = 9999;

  left_car.car_speed = 50.0;
  right_car.car_speed = 50.0;
  center_car.car_speed = 50.0;

  //Declare some variables for the target car states
  double target_car_s, target_car_d, vx, vy, target_car_speed;

  //Get the nearest cars on each lane
  for(unsigned int i = 0; i < sensor_fusion.size(); ++i){
    target_car_s = sensor_fusion[i][5];
    target_car_d = sensor_fusion[i][6];
    vx = sensor_fusion[i][3];
    vy = sensor_fusion[i][4];
    target_car_speed = sqrt(vx*vx + vy*vy);

    if(GetCarLane(target_car_d) == 0 && target_car_s > car_state.car_s && target_car_s < (left_car.car_s + car_state.car_s) && (target_car_s - car_state.car_s) <= 75)
    {
      left_car.car_s = target_car_s - car_state.car_s;
      left_car.car_speed = target_car_speed;
    }
    else if(GetCarLane(target_car_d) == 1 && target_car_s > car_state.car_s && target_car_s < (center_car.car_s + car_state.car_s) && (target_car_s - car_state.car_s) <= 75)
    {
      center_car.car_s = target_car_s - car_state.car_s;
      center_car.car_speed = target_car_speed;
    }
    else if(GetCarLane(target_car_d) == 2 && target_car_s > car_state.car_s && target_car_s < (right_car.car_s + car_state.car_s) && (target_car_s - car_state.car_s) <= 75)
    {
      right_car.car_s = target_car_s - car_state.car_s;
      right_car.car_speed = target_car_speed;
    }
  }

  //Calculate the cost for each path
  //Left Lane Costs
  if(currentLane == 0){
    //Let's check the cost for keeping in this lane
    leftLaneCost = CarSpeedCost(left_car.car_speed) * costFuncWeights[0] + CarDistanceCost(left_car.car_s) * costFuncWeights[1];

    //Let's check the cost for lane change right    
    //Check whether it is possible to lane change right
    if(CheckLaneChange(LCR, currentLane, sensor_fusion, car_state)){
      centerLaneCost = CarSpeedCost(center_car.car_speed) * costFuncWeights[0] + CarDistanceCost(center_car.car_s) * costFuncWeights[1] + costFuncWeights[2];
      rightLaneCost = CarSpeedCost(right_car.car_speed) * costFuncWeights[0] + CarDistanceCost(right_car.car_s) * costFuncWeights[1] + costFuncWeights[2];
    }
    //If it is not possible to change lane, the cost is the highest
    else{
      centerLaneCost = 1 * costFuncWeights[0] + 1 * costFuncWeights[1] + costFuncWeights[2];
      rightLaneCost = 1 * costFuncWeights[0] + 1 * costFuncWeights[1] + costFuncWeights[2];
    }
  }
  
  //Center Lane Costs
  else if(currentLane == 1){
    //Let's check the cost for keeping in this lane
    centerLaneCost = CarSpeedCost(center_car.car_speed) * costFuncWeights[0] + CarDistanceCost(center_car.car_s) * costFuncWeights[1];

    //Let's check the cost for lane change right    
    //Check whether it is possible to lane change right
    if(CheckLaneChange(LCR, currentLane, sensor_fusion, car_state)){
      rightLaneCost = CarSpeedCost(right_car.car_speed) * costFuncWeights[0] + CarDistanceCost(right_car.car_s) * costFuncWeights[1] + costFuncWeights[2];
    }
    //If it is not possible to change lane, the cost is the highest
    else{
      rightLaneCost = 1 * costFuncWeights[0] + 1 * costFuncWeights[1] + costFuncWeights[2];
    }

    //Let's check the cost for lane change left    
    //Check whether it is possible to lane change left
    if(CheckLaneChange(LCL, currentLane, sensor_fusion, car_state)){
      leftLaneCost = CarSpeedCost(left_car.car_speed) * costFuncWeights[0] + CarDistanceCost(left_car.car_s) * costFuncWeights[1] + costFuncWeights[2];
    }
    //If it is not possible to change lane, the cost is the highest
    else{
      leftLaneCost = 1 * costFuncWeights[0] + 1 * costFuncWeights[1] + costFuncWeights[2];
    }
  }

  //Right Lane Costs
  if(currentLane == 2){
    //Let's check the cost for keeping in this lane
    rightLaneCost = CarSpeedCost(right_car.car_speed) * costFuncWeights[0] + CarDistanceCost(right_car.car_s) * costFuncWeights[1];

    //Let's check the cost for lane change left    
    //Check whether it is possible to lane change left
    if(CheckLaneChange(LCL, currentLane, sensor_fusion, car_state)){
      centerLaneCost = CarSpeedCost(center_car.car_speed) * costFuncWeights[0] + CarDistanceCost(center_car.car_s) * costFuncWeights[1] + costFuncWeights[2];
      leftLaneCost = CarSpeedCost(left_car.car_speed) * costFuncWeights[0] + CarDistanceCost(left_car.car_s) * costFuncWeights[1] + costFuncWeights[2];
    }
    //If it is not possible to change lane, the cost is the highest
    else{
      centerLaneCost = 1 * costFuncWeights[0] + 1 * costFuncWeights[1] + costFuncWeights[2];
      leftLaneCost = 1 * costFuncWeights[0] + 1 * costFuncWeights[1] + costFuncWeights[2];
    }
  }

  //Process the new state
  if(currentLane == 0){ leftLaneCost <= centerLaneCost ? newState = LK : newState = LCR; } //Current in Left Lane 
  else if(currentLane == 2){ rightLaneCost <= centerLaneCost ? newState = LK : newState = LCL; } //Current in Right Lane
  //Current in center lane
  else{
    if(centerLaneCost <= rightLaneCost && centerLaneCost <= leftLaneCost) { newState = LK; }
    else if (rightLaneCost < centerLaneCost && rightLaneCost <= leftLaneCost) { newState = LCR; }
    else { newState =  LCL; }
  }

  return newState;
}

bool CheckLaneChange(const t_StateType &checkLane,const int &currentLane,const vector<vector<double>> sensor_fusion, const t_CarState &car_state)
{
  vector<vector<double>> side_cars_info;

  if(checkLane == LCL){
    //If the ego car is already in the lane 0 is not possible to change lane left
    if(currentLane == 0) { return false; }

    //Get all cars in the left lane
    for(unsigned int i = 0; i < sensor_fusion.size(); ++i){
      double d = sensor_fusion[i][6];
      if(d < ((4 * (currentLane - 1)) + 4) && d > (4 * (currentLane - 1))){
        side_cars_info.push_back(sensor_fusion[i]);
      }
    }
  }
  else if(checkLane == LCR){
    //If the ego car is already in the lane 2 is not possible to change lane right
    if(currentLane == 2) { return false; }
    //Get all cars in the left lane
    for(unsigned int i = 0; i < sensor_fusion.size(); ++i){
      double d = sensor_fusion[i][6];
      if(d < ((4 * (currentLane + 1)) + 4) && d > (4 * (currentLane + 1))){
        side_cars_info.push_back(sensor_fusion[i]);
      }
    }
  }
  else{
    return false;
  }

  //If there is no car in the left / right lane then is safe to change lane
  if(side_cars_info.size() <= 0){
    return true;
  }

  //Check whether is safe to change lane
  return(IsSafeToChangeLane(side_cars_info, car_state, 15.0, 20.0));
}

bool IsSafeToChangeLane(vector<vector<double>> cars_info, const t_CarState &car_state, const double &offset_safe_s_sup, const double &offset_safe_s_inf)
{
  //First let's sort the sensor fusion info (cars_info) by the s value
  std::sort(cars_info.begin(), cars_info.end(),
            [](const std::vector<double>& a, const std::vector<double>& b){
              return a[5] < b[5];
            });

  //Declare some variables for the side car states
  double side_car_s, vx, vy, side_car_speed, side_rel_vel, side_rel_s, future_car_s;

  //Check whether there is any car inside the unsafe s zone
  for(unsigned int i = 0; i < cars_info.size(); ++i){
    side_car_s = cars_info[i][5];
    vx = cars_info[i][3];
    vy = cars_info[i][4];
    side_car_speed = sqrt(vx*vx + vy*vy);
    side_rel_vel = car_state.car_speed - side_car_speed; //Relative velocity of the side car related to the ego car
    side_rel_s = (side_car_s + (side_rel_vel * 0.5)); //Relative side car s value after 0.5s      
    future_car_s = (car_state.car_s + (car_state.car_speed * 0.5));    

    if(((side_car_s < (car_state.car_s + offset_safe_s_sup)) && (side_car_s > (car_state.car_s - offset_safe_s_inf)))
      || ((side_rel_s < (future_car_s + offset_safe_s_sup)) && (side_rel_s > (future_car_s - offset_safe_s_inf)))){
      //The side car is inside the unsafe zone!!!
      return false;
    }
  }
  
  return true;
}

double CarSpeedCost(double targetCarSpeed)
{
  double cost = 1 - (targetCarSpeed / 50);
  
  return cost;
}

double CarDistanceCost(double targetCarDistance)
{
  double cost = exp( -5 * (saturation(targetCarDistance, 75.0) / 75.0));
  
  return cost;
}

int GetCarLane(double d)
{
  int carLane = 0;

  if (d >= 0 && d < 4){carLane = 0;}
  else if (d >=4 && d < 8){carLane = 1;}
  else if (d >=8 && d <= 12){carLane = 2;}

  return carLane;
}