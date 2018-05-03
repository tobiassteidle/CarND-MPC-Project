# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Video
[![MPC](http://img.youtube.com/vi/H5Z6NXRMo2s/0.jpg)](http://www.youtube.com/watch?v=H5Z6NXRMo2s "MPC")

## Rubic points

### The Model

A kinematic model is used in this project. The kinematic model is a simplification of a dynamic model and ignores tire forces, gravity and mass.  This reduces the accuracy of the models, but also makes them more comprehensible.
At low and medium speeds, the driving dynamics of kinetic models often correspond to the actual driving dynamics.

##### The kinematic model contains the following state:
x: X position of the vehicle  
y: Y position of the vehicle  
psi: orientation of the vehicle  
v: Speed of the vehicle

Additionally, the cte (cross-track error) and epsi (psi error) are added.

##### The actuators are:
delta: Steering angle  
a: Acceleration

##### The update equations are:
```
x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt  
y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt  
psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt  
v_[t] = v[t-1] + a[t-1] * dt  
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt  
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt  
```

### Timestep Length and Elapsed Duration (N & dt)

As final values for N and dt I have chosen 10 and 0.09. After several attempts I noticed that if the N value was too low, the vehicle did not plan far enough ahead and reacted too late to curves. If the value is too high, the vehicle can be planned relatively far into the future and requires more computing power. After a few tests, a value of 10 seemed to me to be a suitable average. If the dt is too low, the vehicle has already started to oscillate on the straight. If the dt is too high, the vehicle has oriented itself more towards the inside of the curve and has bent into the curve too early.

### Polynomial Fitting and MPC Preprocessing

The waypoints are transformed into the vehicle perspective in advance. This simplifies the adaptation of the polynomial to the waypoints, since x and y coordinates of the vehicle are at the origin (0, 0) and the orientation is also 0.

### Model Predictive Control with Latency

To simulate real conditions an artificial latency of 100ms was added. To compensate for the latency, the current state of the kinematic model was calculated with delay.

The following equation was used for this:
```
Eigen::VectorXd state(6);  
const double Lf = 2.67;  
const double dt = 0.1;  

const double delay_px = v * dt;  
const double delay_py = 0.0;  
const double delay_psi = v * (-delta) / Lf * dt;  
const double delay_v = v + a * dt;  
const double delay_cte = cte + v * sin(epsi) * dt;  
const double delay_epsi = epsi + v * (-delta) / Lf * dt;  

state << delay_px, delay_py, delay_psi, delay_v, delay_cte, delay_epsi;  
```

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).
