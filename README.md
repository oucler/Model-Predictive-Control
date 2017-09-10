# Model Predictive Controller(MPC) Project

## Table Content: ##
- [Objective](#objective)
- [Results](#results)
- [How to run](#howto)
- [Dependencies](#dependency)
- [Directory Structure](#structure)
- [Code Variables](#variables)
- [Kinematic Model](#model)
- [Variable Optimization](#optimization)
- [Polynomial Fitting and Preprocessing](#fitting)
- [Cost Function](#cost)
- [Latency Calculation](#latency)
- [Discussions](#discussions)


## Objective: <a name="objective"></a>

The goal is to drive a car in the simulator without getting off the track and using MPC controller method to calculate the steering and speed. 

## Results: <a name="results"></a>

Due to the size limitation of github only small size of the video is part of the github repository and the video clip is playing below.

![](videos/mpc.gif)

Entire video capture is uploaded to youtube link: [Full Video](https://www.youtube.com/watch?v=YZcmExkdCZw&feature=youtu.be)


## How to run: <a name="howto"></a>

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

	mkdir build
	cd build
	cmake ..
	make
	./mpc

## Dependencies <a name="dependency"></a>

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
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.
	
## Directory Structure <a name="structure"></a>
The directory structure of this repository is as follows:

```
root
|   CMakeLists.txt
|   cmakepatch.txt
|   README.md
|   *.sh
|
|___videos
|   |
|   |  mpc.gif
|
|
|___src
    |   main.cpp
    |   MPC.h
    |   MPC.cpp
    |   json.hpp
    |   Eigen-3.3
```

## Code Variables <a name="variables"></a>

	px: The current location in the x-axis of an arbitrary in global map coordinate system.

	py: The current location in the x-axis of an arbitrary in global map coordinate system.

	psi: The current orientation of the vehicle.

	v: The current velocity of the vehicle.

	cte: This is the cross track error which is the difference between our desired position and actual position. 

	epsi: This is the orientation error which is the difference between our desired orientation and actual orientation. Our desired orientation is the heading tangent to our road curve. 

## Kinematic Model <a name="model"></a>

The state variables get updated based elapse time dt, the current state, and our actuations steering_angle and throttle.

px` = px + v * cos(psi) * dt
py` = py + v * sin(psi) ( dt)
psi` = psi + v / Lf * (-delta) * dt
v` = v + throttle * dt

Lf - this is the length from front of vehicle to its Center-of-Gravity

We can also predict the next cte, and epsi based on our actuations.

cte` = cte - v * sin(epsi) * dt
epsi` = epsi +  v / Lf * (-steering_angle) * dt

Recall from the discussion:

cte = py_desired - py
epsi = psi - psi_desired
py_desired = f(px)
psi_desired = atan(f`(px))
where f is the road curve function
      f` is the derivative of f
      
## Variable Optimization <name="optimization"></a>

**Timestep Length (N):** Having bigger number for N it increase computation for predicting future states and also it makes it slower. I noticed that having larger values cause the car off the track. I tried values between 10 and 20 and 10 provided the best result.

**Duration (dt):** This is the time delta between each feature state calculation if this value picked smaller it will require more computations. I tried values between 0.01 and 0.1 seconds keeping the N constant and 0.1 has the optimal result. 

**N*dt**: Multiplication of N and dt will give future state predictions and among N between 10 and 20 and dt between 0,01 and 0.1. Multiplication of 10 and 0.1 gave the best result. 

## Polynomial Fitting and Preprocessing<a name="fitting"></a>

px and py points are rotated 90 degree to match with the car's orientation in the simulator. After that it is transfomed into Eigen Vector space. To make future state prediction ptsx and ptsy points are fitted into third degree polinomial and the equations is adjusted based on the new location. 

Using helper functions cte and epsi can be calculated easily (after rotation:px=0,py=0,v=0). 

	   // Rotate the car to make things simpler
          for (unsigned int i=0;i<ptsx.size();++i){
        	  dx = ptsx[i] - px;
        	  dy = ptsy[i] - py;
        	  ptsx[i] = dx*cos(-psi) - dy*sin(-psi);
        	  ptsy[i] = dx*sin(-psi) + dy*cos(-psi);
          }

          // Transfer vectors into Eigen Vector format
          double *ptrx = &ptsx[0];
          Eigen::Map<Eigen::VectorXd> ptsx_map(ptrx,6);

          double *ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsy_map(ptry,6);

          // Fit waypoints into a polynomial function
          auto coeffs = polyfit(ptsx_map, ptsy_map, 3);
          // Calculate the cross track error
          double cte = polyeval(coeffs, 0) ;
          // Calculate the orientation error
          double epsi = - atan(coeffs[1]);
          

## Cost Function <a name="cost"></a>

The cost function is named as fg and its 0^th ^ index is where all the computation results are added. The car's behavior in the simulator determined by the cost function value. Some of the parameters are penalized and wights for those paramters shown below. 
	
	// Weight declaration
	const double w_cte = 10000;
	const double w_epsi = 10000;
	const double w_delta = 5;
	const double w_a = 5;
	const double w_diff_delta = 20000;

	  // The part of the cost based on the reference state
	  for (int t = 0; t < N; t++) {
	    fg[0] += w_cte*CppAD::pow(vars[cte_start + t] - ref_cte, 2);
	    fg[0] += w_epsi*CppAD::pow(vars[epsi_start + t] -ref_espi, 2);
	    fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
	  }

	  // Minimize change-rate
	  for (int t = 0; t < N - 1; t++) {
	    fg[0] += w_delta*CppAD::pow(vars[delta_start + t], 2);
	    fg[0] += w_a*CppAD::pow(vars[a_start + t], 2);
	  }
	  // Minimize the value gap between sequential actuations.
	  for (int t = 0; t < N - 2; t++) {
	    fg[0] += w_diff_delta*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
	    fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
	  }

## Latency Calculation <a name="latency"></a>

To be more precise dt (100ms) latency value needs to be taken into account. Calculation of state and actuator parameters are shown below.  

          // Predicting state with latency
          px = 0.0 + v * dt;
          py = 0.0;
          psi = 0.0 + v * (-steering_angle) / Lf * dt;
          v = v + throttle * dt;
          cte = cte + v * sin(epsi) * dt;
          epsi = epsi + v * (-steering_angle) / Lf * dt;

## Discussions <a name="discussions"></a>

For improvements, time delta, number of steps, and parameters can be optimized. So it can reached to a maximum speed without getting off the track. 


