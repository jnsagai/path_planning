[image1]: ./images/fsm.png "FSM"
[image2]: ./images/spline.png "Path"
[video1]: ./images/video.mp4 "Video"

# Udacity Path Planning Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. It is provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Behavior Planner

The car behavior is controlled by a behavior planner based on a Finite State Machine with 4 distinct states, as shown in the image below:
* Ready: Initial state, used to first set-up the initial car speed and acceleration, then occurs the transition to Keep Lane State;
* Keep Lane State: In this state, the car tries to keep road speed whenever is possible by accelerating until reaches this limit. If a car appears in front of the vehicle in this state, the car disaccelerate until a safe distance is ensured. Another feature of this state is to evaluate the cost function to perform the transition to other states, or to keep in the same state. For each lane it is evaluated a cost and, according to this cost, the new state is computed.
* Change Lane Left / Right: During this state the car its lane to the immediate side lane. The car keeps in this state until it is ensured that the transition to a new lane is completed. After that, the FSM returns to Keep Lane State.

![alt text][image1]

## Cost Function

In order to decide which State shall be executed for the FSM, a set of cost functions was designed. The main goals of the cost function are to keep the car at the highest speed possible (obeying the road speed limit) and also avoid collision with other vehicles. For each cost function, a parametrized weight is defined and used to calibrate the sensitivity of each function. The following cost functions were implemented:
* Car Speed Cost: Penalize paths where the nearest car in the path has a speed slower than the road speed limit.
  - cost = 1 - (targetCarSpeed - 50)
* Car Distance Cost: Penalize paths where the nearest car in the path is close to the ego car. The maximum range of actuation is 75m.
  - cost = exp( -5 * (min(targetCarDistance, 75.0) / 75.0))
* Change Lane Cost: Try to keep the car in the current lane in order to avoid unnecessary lane change, for comfort reasons.
  - cost = Constant Value

## Path Generator

Based on the current lane as the starting point and the goal lane as the ending point, a path is created based on a list of widely spaced (x, y) waypoints, evenly spaced at 30m, then the waypoints are interpolated with a spline and filled in with more points that control speed.
Instead of always generating a new trajectory from scratch for each interaction, the last waypoints from the previous spline are used in order to always have a smooth transition between the last trajectory and the current one. Using this technique the jerk value stays under the safety range.
It was used the [spline C++ tool](http://kluge.in-chemnitz.de/opensource/spline/) for generating the spline trajectory. The next image demonstrates a path using this strategy:

![alt text][image2]

## Demonstration

The following video demonstrate the application of the behavior planning and the Trajectory generation where the ego car safely overtake a car.

![alt text][video1]


