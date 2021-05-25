# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[img1]: ./output.png
[img2]: ./insert1.png
[img3]: ./insert2.png

### output display

![img1]

### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

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

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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

## Project Instructions and Rubric

### Overview
![img2]

In an autonomous vehicle, path planning requires the cooperation of different layers. The diagram above provides an overview of six components which are involved in a given self-driving system: `Motion Control`, `Sensor Fusion`, `Localization`, `Prediction`, `Behaviour`, `Trajectory`.

### Trajectory planning

There are many approaches to trajectory generation, and in this project we have opted for computing trajectories in a Frenet coordinate system.

*Trajectories in Frenet (left) and normal Cartesian (right) coordinate systems.*
![img3]

This is precisely what a Frenet coordinate system offers: in such a system we split our plane into a longitudinal and lateral axis, respectively denoted as S and D.

### Work steps

**1. Analyze the data from the sensor fusion and categorize other vehicles by lane.**

    // find the lanes of each car
    double findlane(double car_d) {
        int lane_number = -1;
        if (car_d > 0.0 && car_d < 4.0){
            lane_number = 0;
        } else if (car_d > 4.0 && car_d < 8.0) {
            lane_number = 1;
        } else if (car_d > 8.0 && car_d < 12.0) {
            lane_number = 2;
        }

        return lane_number;
    }

**2. Find the closest vehicle in each lane and update their longitudial and lateral distance and velocity.**

    // loop for each cars in sensor data
    for (int i = 0; i < sensor_fusion.size(); ++i) {
        auto fusion_data = sensor_fusion[i];
        double fusion_id = fusion_data[0];
        double fusion_vx = fusion_data[3];
        double fusion_vy = fusion_data[4];
        double fusion_s = fusion_data[5];
        int fusion_d = fusion_data[6];
        double fusion_speed = sqrt(fusion_vx * fusion_vx + fusion_vy * fusion_vy);
        
        // distance from the other car in s direction
        double dist = fusion_s - car_s;

        // find the other car's lane
        int others_lane = findlane(fusion_d);

        // consider three position
        if (others_lane == my_lane) {// in the same lane
            // keep a safe distance
            if (dist > 0 && dist < 30) {
                car_head = true;
                if (fusion_speed) {
                    closest_front_vel = ms_to_mph(fusion_speed);
                }
            }

            // check if it will collide
            if (dist > 0 && dist < 15) {
                car_crash = true;
            }

            // update the closest front distance
            if (dist > 0) {
                closest_front_dist = min(dist, closest_front_dist);
            }

        } else if (others_lane == my_lane - 1) {// in the left lane
            // mind your left
            if (dist > -30 && dist < 30) {
                car_left = true;
            }

            if (dist > 0) {
                if (dist < closest_leftfront_dist) {
                    // update the closest leftfront distance
                    closest_leftfront_dist = dist;
                    if (dist < 10) {
                        // distance from the other car in d direction
                        closest_left_d = car_d - fusion_d;
                    }
                }                                
            } else {
                // update the closest leftback distance
                closest_leftback_dist = min(abs(dist), closest_leftback_dist);
            }

        } else if (others_lane == my_lane + 1) {
            // mind your right
            if (dist > -30 && dist < 30) {
                car_right = true;
            }

            if (dist > 0) {
                if (dist < closest_rightfront_dist) {
                    // update the closest rightfront distance
                    closest_rightfront_dist = dist;
                    if (dist < 10) {
                        // distance from the other car in d direction
                        closest_right_d = fusion_d - car_d;
                    }
                }
            } else {
                // update the cloest rightback distance
                closest_rightback_dist = min(abs(dist), closest_rightback_dist);
            }

        }
    }

**3. Calculate cost of decisions and select the action with minimal cost.**

4 possible decisions were considered, and our car will choose fairly on their costs:

1. Stay in the lane with max velocity
2. Change to the right lane
3. Change to the left lane
4. Stay in the lane but slow down

**Cost function — Continue in the lane**

    double keep_lane_cost(bool car_head, int my_lane, double closest_front_dist, int cost_collision) {
        double cost = 0;
        if (car_head) {// check if there's a car within 30m
            cost = cost_collision;
        } else {
            if (my_lane == 1) {// prefer to stay in the middle lane
                cost = 50;
            } else {
                cost = 100;
            }
            if (closest_front_dist >= 150) {// check if there's a car within 150m
                cost += 0;
            } else {
                cost += 100;
            }
        }

        return cost;
    }

**Cost function — Change to right / left lane**

    double turn_right_cost(bool car_right, int my_lane, double closest_rightfront_dist, double closest_rightback_dist,
        int cost_collision, int right_turn_cost) {
        double cost = 0;
        if (car_right || my_lane == 2) { //Cars in lane 2 cannot change lanes to the right
            cost = cost_collision;
        } else {
            if (closest_rightfront_dist > 100 && closest_rightback_dist > 20) {// more safer lane changing
                cost = 0.5 * right_turn_cost;
            } else {
                cost = right_turn_cost;
            }
        }

        return cost;
    }

    double turn_left_cost(bool car_left, int my_lane, double closest_leftfront_dist, double closest_leftback_dist,
                          int cost_collision, int left_turn_cost) {
        double cost = 0;
        if (car_left || my_lane == 0) { //Cars in lane 0 cannot change lanes to the left
            cost = cost_collision;
        } else {
            if (closest_leftfront_dist > 100 && closest_leftback_dist > 20) {// more safer lane changing
                cost = 0.5 * left_turn_cost;
            } else {
                cost = left_turn_cost;
            }
        }

        return cost;
    }

**Cost Function — Slow down in the lane**

the cost of slow down is 200, right between the cost of collision and lane change, thus the action is less preferred than lane change but better than collision.

    double slow_down(double slow_down_cost) {
        double cost = 0;
        cost = slow_down_cost;
        return cost;
    }

**4. Make decisions based on cost-function.**

At every instance, the cost associated with all the decisions are calculated and the optimal decision is the one with the minimal cost.

    // make decision with the minimum cost.
    int decision = -1;
    double min_cost = 1000;

    for (int i = 0; i < costs.size(); ++i) {// find the minimum cost
        double the_cost = costs[i];
        if (the_cost < min_cost) {
            min_cost = the_cost;
            decision = i;
        }
    }

    cout << "min cost: " << min_cost << endl;

    if (decision == 0) {
        cout << "decision==0: Continue with max velocity "   << endl;
    } else if (decision == 1) {
        cout << "decision==1: Change to right lane  "        << endl;
    } else if (decision == 2) {
        cout << "decision=2: Change to left lane  "          << endl;
    } else if (decision == 3) {
        cout << "decision==3: Continue with lower velocity " << endl;
    }

**5. Once decision has been made, decide the target speed and lane for that decision.**

    // set some performance parameters of our car
    double speed_diff = 0; // speed difference for each step
    const double MAX_SPEED = 49.5; //mph
    const double MAX_ACC = .224; // on each timestep

    // take action
    if ((decision == 0) || (decision == -1)) {// keep going
        lane = my_lane;

        if (ref_vel < MAX_SPEED) {// accelerate
            speed_diff += MAX_ACC;
        }

        if (closest_left_d < 3.5 || closest_right_d < 3.5){
            // avoid other cars that might change the lane before us (car width ~ 3m)
            speed_diff -= MAX_ACC;
        }

    } else if (decision == 1) {// change right lane
        lane = my_lane + 1;

    } else if (decision == 2) {// change left lane
        lane = my_lane - 1;

    } else if (decision == 3) {// slow down
        lane = my_lane;

        if (car_crash) {// emergency collision avoidance
            speed_diff -= MAX_ACC;
        } else {// check if we are faster than the front car
            if (ref_vel - closest_front_vel > 0) {
                speed_diff -= MAX_ACC;
            } else {
                speed_diff -= 0.0;
            }
        }

    } else {
        cout << "Error" << endl;
    }

**6. Initialize the spline and output the trajectory of future path points.**

*Follow by the steps shown in Q&A video :*

* 2 previous points and 3 furture points is picked to initialize the spline.
* output path points from previous path for continuity.
* calculate target distance (y position) on 30 m ahead using spline.
* divide target distance into N segements, calculate corresponding y position using spline.
* transform our coordination system from Frenet to Cartesian and output furture point location to simulator.

## Following work:

1. Different sets of cost functions can give our vehicle different driving behaviors. After going through a lot of tests in the simulator, our current solution still may lead to collisions in some special cases.

2. How can our vehicle predict the behavior of vehicles in other lanes to better prevent collisions?
