# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Implementation

### The Model

The model that was used was a Global Kinematic Model that ignores forces such as tire forces and gravity. It sacrifices some accuracy, but is in turn much easier to compute.

The model is governed by the following variables and equations:

The state is governed by vehicle position x and y, orientation psi, and velocity v. Changes to the state are controlled by actuators  
δ for the steering angle and a for the accelation. The Cross Track Error cte and Orientation Error (epsi) are also tracked. 

The update equations are as follows:

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt

y[t+1] = y[t] + v[t] * sin(psi[t]) * dt

psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

v[t+1] = v[t] + a[t] * dt

cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

epsi[t+1] = psi[t] - psi_ref[t] + v[t] * delta[t] / Lf * dt

where Lf is the distance between the front of the vehicle and its center of gravity and dt is the timestep length between actuations.

### Timestep Length and Elapsed Duration (N & dt)

In this model, N represents the number of timesteps in the horizon. The product of N and dt is the elapsed duration. 

I originally chose values of 25 and 0.05 for N and dt respectively. I found that by reducing the number of timesteps from 25 to 10, the optimizer focused more on the near-term and achieved better performance. Decreasing the number of timesteps to 8 and 6 reduced the performance and so a value of 10 seemed optimal. 

I attempted both larger and smaller values of dt. Larger values of dt resulted in worse results. Reducing the value of dt did not change performance drastically, but was more computationally expensive and so did not have any benefit over a value of 0.05. 

### Polynomial Fitting and MPC Preprocessing

In order to model the waypoints, the coordinates were transformed from map coordinates into the vehicle's coordinates system so that they were in a similar frame of reference. A third order polynomial was then fit to these waypoints.

Since the vehicle's frame of reference was used, the vehicle is at the origin and the state is passed to the solver with values 0, 0, 0 for x, y, and psi respectively along with the current values for velocity, cte, and epsi. The cte is calculated from the fitted polynomial while the epsi is calculated from the arctangent of its derivative. 

### Model Predictive Control with Latency

The project requirements specify that the model must account for a 100 ms delay (latency) between actuation commands and their execution. To account for this, I used robust coefficients in front of the cost function and passed the reference state for x as being the reference state with the current velocity 100ms in the future (i.e x = v * latency). 

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.0.zip).
  * If you have MacOS and have [Homebrew](https://brew.sh/) installed you can just run the ./install-mac.sh script to install this.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt --with-openblas`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/CarND-MPC-Project/releases).



## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

