CarND-MPC-Project
June 2017

Carlos Arreaza

1) The Model: Student describes their model in detail. This includes the state, actuators and update equations.
The model used is a dynamical model of the vehicle using mathematical equations to predict the position of the vehicle given some inputs (force, which ends up in the vehicle mass accelerating). Predicting the cross track error and vehicle orientation error, one can account for this and then accomodate for the steering.
States: this model uses 4 states. The x and y position of the vehicle. Psi, the vehicle's orientation. Finally, the vehicle's velocity v.
Actuators: the vehicle's acceleration a, and the vehicles steering delta. 

The update equations give the states x,y,psi,v values for t+1 given their values at t. The errors used are the cross track error (cte) and the error in the orientation (epsi):
            // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
            // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
            // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
            // v_[t+1] = v[t] + a[t] * dt
            // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
            // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


2) Timestep Length and Elapsed Duration (N & dt): Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
-N determines the amount of steps ahead the model calculates. The more steps (bigger N) the more computation time the algorithm takes and slower the whole process. I started using 25 but obtained better results with 12 in the end.
-dt: determines the amount of time between steps. By choosing a very small dt, one would get a prediction horizon (T = N*dt) that is not too far away. We want to be able to look ahead, not too close, and not too far. Thus, by choosing dt=0.1 sec, we get a T = 10*0.1 = 1 second future prediction. This number worked well in the simulations. Others tried were 0.01, 0.05, and 0.5.

3) Polynomial Fitting and MPC Preprocessing: A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
-A polynomial is used to fit the waypoints. However, these waypoints are in the map's coordinate system and need to be changed to the vehicle coordinate system. using the following:
                        trans_ptsx(i) =   cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
                        trans_ptsy(i) =  -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);

4) Model Predictive Control with Latency: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
To deal with latency, a trial on error method was used to determine the best combination of cost functions to optimize the algorithm. For example, different weights were given to the cross track error, the orientation error, and both actuators the steering and the acceleration, see comments below:
            fg[0] += 10*CppAD::pow(vars[cte_start + i] - ref_cte, 2); //Here, the cte is given some importance
            fg[0] += 100*CppAD::pow(vars[epsi_start + i] - ref_epsi, 2); //Here, the error in orientation is given much more importance
            fg[0] += 4000*CppAD::pow(vars[delta_start + i], 2); //Here, the use of the steering angle is heavily penalized
            fg[0] += 5*CppAD::pow(vars[a_start + i], 2); //Here, the use of the acceleration pedal is not that much penalized

