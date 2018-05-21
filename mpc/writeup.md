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