# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
The goal of this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. Based on provided the car's localization, sensor fusion data and a sparse map list of waypoints around the highway, ego vehicle goes as close as possible to the 50 MPH (or ~80 KMH) speed limit, which means passes slower traffic when possible. The ego vehicle avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The ego vehicle is able to make one complete loop around the 6946m highway. Since ego vehicle is trying to go 50 MPH (or ~80 KMH), it should take a little over 5 minutes to complete 1 loop. Ego vehicle does not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

   
### Simulator.
This project requires simulator to be executed. Term3 Simulator which contains the Path Planning Project can be downloaded from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```


#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo with all the submodules: `git clone --recurse-submodules https://github.com/marcin-sielski/CarND-Path-Planning-Project.git`
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

## Model Documentation

The path for the ego vehicle is generated in three distinctive steps commented in details in the source code:

1. Prediction

In this step the algorithm is analysing the current and future positions of other vehicles in relation to the ego vehicle. The purpose of this step is to generate eligible behaviours for the ego vehicle. The steps are following:  

* Iterate through all the vehicles detected by data fusion,

* Get vehicle attributes such as speed, path distance in Frenet coordinates and current lane,

* Mark if vehicle is ahead of the ego vehicle whitin certain distance,

* Mark if vehicle is or will be on the left side of the ego vehicle within certain distance,

* Mark if vehicle is or will be on the right side of the ego vehicle within certain distance,   

2. Behaviour Planning

In this step the algoirithm is deciding what to do the best to keep the reference speed (brake, accelerate or change lane) based
on the prediction.

* If there is vehicle ahead and there is no vehicle on the left side change line to the left if possible,

* If there is vehicle ahead and there is no vehicle on the right side change line to the left if possible,

* If there is vehicle ahead and there is no possiblity to change lane deaccelerate the speed,

* If there is no vehicle ahead and ego vehicle is not on the default lane try to change lane to default,

* If there is no vehicle ahead accelerate to the desired speed of 80 kmh, 

3. Trajectory Generation

In this step the algorithm is generating the detailed trajectory for the ego vehicle based on the planned behaviour.

* If there are no previous points from the trajectory add current ego vehicle position to the trajectory plan,

* If there are previous points from the trajectory add two of them to the trajectory plan,

* In addition setup some future points acording to the selected lane and the map,

* Convert trajectory coordinates to the ego vehicle coordinates to simplify math in the trajectory plan,

* Create spline on the trajectory plan,

* Build the trajectory plan based on planned speed,

* Convert trajectory plan to word coordinates,

* Add previous path to the trajectory,

* Add trajectory plan to the trajectory.

