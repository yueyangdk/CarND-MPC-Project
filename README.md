# Udacity Self-Driving Car Engineer Nanodegree Program
# *Model Predictive Controller Project*

## Overview

This project implements a Model Predictive Controller to navigate a reference track in the Udacity simulator [Downloaded here](https://github.com/udacity/self-driving-car-sim/releases), which transmit telemetry and track waypoint data via uWebSocket while sending steering and acceleration value back to simulator. Model predictive control is an advanced metod that is used to control a process while satisfying a set of constraints. In this imlementation, we make use of Ipopt and Cppad libraries to calculate an optimal trajectory and associated acceleration to minimize error with reference waypoints from a third-order polynomial while taking care of those constraints due to model and dynamical limits. The optimization process only considers a short duration and produce optimized trajectory for that duration based on the vehicle's kinematics model and a cost function consists of cross-tract-error, orientation angle error and other penalty factor when we want to improve particular performance.

## Introduction

###The Model: 

The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi). Actuator outputs are acceleration (throttle value) and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:

![equations](./eqns.png)

###Timestep Length and Elapsed Duration (N & dt): 

The values chosen for N and dt are 10 and 0.1, respectively. Admittedly, this was at the suggestion of Udacity's MPC quiz. These values mean that the optimizer is considering a one-second duration in which to determine a corrective trajectory. Adjusting either N or dt (even by small amounts) often produced erratic behavior. Other values tried include 20 / 0.05, 8 / 0.125, 6 / 0.15, and many others. 

###Polynomial Fitting and MPC Preprocessing: 

The waypoints are preprocessed by transforming them to the vehicle's perspective. This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero. 

###Model Predictive Control with Latency: 

The approach to handling the latency is to not only limit the speed and orientation angle but also alter the time step. First, according to the kinematic model, the state of current timestep depends on the actuation from previous timestep. With a delay of 100ms, which happens to be the timestep interval, the actuations are applied an additional timestep later. That's the change we need to make in our code. Also, we can take additional element into account, by penalizing the product of velocity and delta and results will have a much smoother cornering performance.


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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Final Paramters
The parameters were chosen manually by try and error. A very large potion of the cost function is mude up of cte and orientational error and the reason is obvious because the mainly objective to stay with reference trajectery is to minimize both error. Other penalizing factors are chosen based on particular need for better performance. An example of a non-linear cost function for optimization is given by:

![cost function](.cost_function.png)



