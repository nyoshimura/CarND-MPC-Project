# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

### 6 Specifications
#### 1. My code should compile (done)
#### 2. The Model
The state is comprised of the following parameters:
- x position of the vehicle.
- y position of the vehicle.
- psi: vehicle orientation.
- v: vehicle speed.
- cte: cross-track error.
- epsi: orientation error.

The actuators which are used
- delta: steering angle.
- a: throttle value.

#### 3. Timestep Length and Elapsed Duration (N & dt)
The number of predicted timesteps(N) and the duration of each timestep(dt) was chosen to maximize the accuracy of the prediction.
It was found the higher limit(which corresponds to the latency) on the timestep duration worked well, allowing us to choose smaller values of N.
N values larger than 10 were found to take the processing time over the latency period, which induces erratic behaviour.  
#### 4. Polynomial Fitting and MPC Preprocessing
The reference path is first translated to coordinates relative to the car, then a 3rd degree polynomial is fitted to it. The coefficients for this polynomial are then used in the MPC calculation.  
#### 5. Model Predictive Control with Latency
A 100 ms latency between measurement and calculation is expected. To compensate for this, a kinematic vehicle model is used with 'dt'=0.1s.
#### 6. The vehicle must successfully drive a lap around the track (done)
