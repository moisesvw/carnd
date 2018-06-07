# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
### MPC Project
![Alt text](output.gif "MPC")

The goal of this project is to find the steering angle and
acceleration values of the autonomous car base on a reference
trajectory, to achieve that the next steps were needed:

find the coefficients that fits better to the trajectory
of reference using PolyFit Function.

Initialize the state vector and pass it over the mpc Solve function with the coefficients, this function will find out
the actuators values namely angle of steering and acceleration
that best fit to the reference trajectory. This means that in
this process took place an optimization that finds these values that minimize the costs that every factor adds to
the process. In this part were necessary to add weights
to every factor. As can see below the delta weights that
correspond to the steering angle has the highest weights 
since this brings more stability on predictions of steering
angles.

```
  double w_cte_state  = 30;
  double w_epsi_state = 500;
  double w_v_state    = 30;
  double w_delta_act  = 3000;
  double w_a_act      = 1000;
  double w_delta_seq  = 160000;
  double w_a_seq      = 1000;
```

##### N and dt values

The prediction points are defined by N and the time elapsed between every point are defined by dt, the values tested for for N ranges from 5 to 25; dt ranges from 0.005 to 0.1.

When N is large in the values between 18 - 25 the dt values tested does not change to much the result. The result was not good this number of points make the predictions to oscillate to much having the car drive unsafe. The max value of N that works better is N=15 and dt=0.05

##### Preprocessing and Delay

The reference trajectory points were transformed from its original coordinates to the car coordinates as show below:

```
  Eigen::VectorXd vxs = Eigen::VectorXd::Zero(ptsx.size());
  Eigen::VectorXd vys = Eigen::VectorXd::Zero(ptsy.size());
  double d_x, d_y, psi__;
  for(int i=0; i < ptsx.size(); i++){
    d_x = ptsx[i] - px;
    d_y = ptsy[i] - py;
    psi__ = 0.0 - psi;
    vxs(i) = d_x * cos(psi__) - d_y * sin(psi__);
    vys(i) = d_x * sin(psi__) + d_y * cos(psi__);
  }
```

The initial state takes into account the delay of 100 milliseconds as can be observed below

```
  int delay_ = 100;
  double delay = delay_/1000.0;
  double epsi_init = -atan(coeffs[1]);

  double x_    = v * cos(0) * delay;
  double y_    = v * sin(0) * delay;
  double psi_  = 0 - (v * delta * delay / mpc.Lf);
  double v_    = v + a * delay;
  double cte_  = coeffs[0] + (v * sin(epsi_init) * delay);
  double epsi_ = epsi_init - (v * atan(coeffs[1]) * delay / mpc.Lf);
```

Reducing the time horizon by using smaller dt values also helps
to deal with the delay.


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

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
