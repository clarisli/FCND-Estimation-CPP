# Estimation Project #

The goal is to develop the estimation portion of the controller used in the CPP simulator. The steps of this project are following:

- Collect some simulated noisy sensor data and estimate the standard deviation of the quad's sensor.
- Implement a better rate gyro attitude integration scheme to reduce the errors in the estimated attitude.
- Implement the prediction step.
- Add data from magnetometer to improve the filter's performance in estimating the vehicle's heading.
- Implement GPS update in measurement model.
- Combine the estimator with controller from last project.



[//]: # (Image References)
[image1]: ./images/gyro_r_matrix.png
[image2]: ./images/attitude-screenshot.png
[image3]: ./images/transition_function.png
[image4]: ./images/predict-slow-drift.png
[image5]: ./images/rbg.png
[image6]: ./images/jacobian.png
[image7]: ./images/rbg_prime.png
[image8]: ./images/prediction.png
[image9]: ./images/predict-good-cov.png
[image10]: ./images/magnetometer_z.png
[image11]: ./images/magnetometer_h.png
[image12]: ./images/magnetometer_h_prime.png
[image13]: ./images/gps_z.png
[image14]: ./images/gps_h.png
[image15]: ./images/gps_h_prime.png
[image16]: ./images/step6.gif
[image17]: ./images/step5.gif
[image18]: ./images/step4.gif
[image19]: ./images/step3_2.gif
[image20]: ./images/step3_1.gif
[image21]: ./images/cover.gif

![alt text][image21]

## Setup ##

Here are the setup and install instructions for each of the recommended IDEs for each different OS options:

#### Windows

1. Download and install [Visual Studio](https://www.visualstudio.com/vs/community/)
2. Select *Open Project / Solution* and open `<simulator>/Simulator.sln`
3. From the *Project* menu, select the *Retarget solution* option and select the Windows SDK that is installed on your computer (this should have been installed when installing Visual Studio or upon opening of the project).
4. To compile and run the project / simulator, simply click on the green play button at the top of the screen.  When you run the simulator, you should see a single quadcopter, falling down.


#### OS X

1. Download and install XCode from the App Store if you don't already have it installed.
2. Open the project from the `<simulator>/project` directory.
3. After opening project, you need to set the working directory:
  1. Go to *(Project Name)* | *Edit Scheme*
  2. In new window, under *Run/Debug* on left side, under the *Options* tab, set Working Directory to `$PROJECT_DIR` and check ‘use custom working directory’.
  3. Compile and run the project. You should see a single quadcopter, falling down.


#### Linux

1. Download and install QtCreator.
2. Open the project from the `<simulator>/project` directory.
3. Compile and run the project.  You should see a single quadcopter, falling down.


## Implement Estimator ##


### Task 1: Sensor Noise ###

I first ran scenario `06_NoisySensors` to collect the noisy sensor data:

- `config/log/backup/gps_x.txt` (GPS X data)
- `config/log/backup/imu_x.txt` (Accelerometer X data)

Then used `np.std()` to find out the standard deviation of these data:

```
GPS X Std: 0.7139549237774065
IMU X Std: 0.5095402073668808
```
I did this in cell 1 to 3 in `visualizations.ipynb`.

I plugged in my result into `config/6_Sensornoise.txt`:

```
MeasuredStdDev_GPSPosXY = 0.714
MeasuredStdDev_AccelXY = 0.510
```

My standard deviation correctly captured ~68% of the sensor measurements:

```
Simulation #3 (../config/06_SensorNoise.txt)
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time
```


### Task 2: Attitude Estimation ###

The goal was to improve a complementary filter-type attitude filter. It was originally implemented using a linear, small-angle approximation integration method. Find this in lines 113 to 119 of the funciton `UpdateFromIMU()` in `QuadEstimatorEKF.cpp`.

To reduce the error, I used a non-linear approach with the with Euler forward method - I first created rotation matrix based on the current Euler angles, then used this rotation matrix to integrate the body rate into new Euler angles:

![alt text][image1]

I did this in lines 93 to 110 of the funciton `UpdateFromIMU()` in `QuadEstimatorEKF.cpp`.

With this, I successfully reduced the attitude errors to get within 0.1 rad for each of the Euler angles, as shown in the screenshot below.

```
Simulation #2 (../config/07_AttitudeEstimation.txt)
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
```

The screenshot below shows a comparison between linear and non-linear errors:

![alt text][image2]

In the screenshot above the attitude estimation using linear scheme (left) and using the improved nonlinear scheme (right). Note that Y axis on error is much greater on left.


### Task 3: Prediction Step ###

Prediction can be break down into two parts: state mean and state covariance.

![alt text][image8]

#### 3.1: State Prediction

A state transition function is defined as following:

![alt text][image3]

A rotation matrix is required for the second part of transition function. This matrix rotates from body frame to global frame, and it is defined below:

![alt text][image5]

This rotation matrix was implemented in the original code, I used `attitude.Rotate_BtoI(<V3F>)` to rotate the acceleration from body frame to inertial(i.e. global) frame.

I did this in lines 182 to 190 of the funciton `PredictState()` in `QuadEstimatorEKF.cpp`. 

With this, in scenario `08_PredictState`, the estimator state track the actual state, with only reasonably slow drift, as shown in the figure below:

![alt text][image20]

#### 3.2: Covariance Prediction

I took the Jacobian of g to perform a linear approximation of the system:

![alt text][image6]

And defined the Rbg' as following:

![alt text][image7]

I implemented it in lines 218 to 223 of the function `GetRbgPrime()` in `QuadEstimatorEKF.cpp`.

I then used the Jacobian and Rbg' to predict the state covariance forward. I did this in lines 269 to 277 of `Predict()` in `QuadEstimatorEKF.cpp`.

To capture the magnitude of error more precisely, I also tuned the parameters in `QuadEstimatorEKF.txt`.

```
QPosXYStd = 0.03
QVelXYStd = 0.1
```

![alt text][image19]


### Task 4: Magnetometer Update ###

![alt text][image18]

The goal was to improve the filter's performance in estimating the vehicle's heading. I did this by including magnetometer update into the state.

I first tuned the parameter `QYawStd` in `QuadEstimatorEKF.txt` to **0.8** to make sure it approximately captures the magnitude of the drift.

I assume the megnetometer is reporting yaw in the global frame:

![alt text][image10]   ![alt text][image11]

Since it is linear, the derivative is a matrix of zeros and ones.

![alt text][image12]

I did this magnetometer update in lines 332 to 340 of the function `UpdateFromMag()` in `QuadEstimatorEKF.cpp`.

I successfully obtained an estimated standard deviation that accurately captures the error and maintain an error of less than 0.1rad in heading for at least 10 seconds of the simulation.


### Task 5: Closed Loop + GPS Update ###

![alt text][image17]

Run scenario `11_GPSUpdate`. 

I tuned the process noise model in `QuadEstimatorEKF.txt` to capture the error with estimated uncertainty of the filter.

I assume we get postion and velocity from GPS measurements:

![alt text][image13]   ![alt text][image14]

Then the partial derivative is the identity matrix:

![alt text][image15]

I did this in lines 302 to 311 of the function `UpdateFromGPS()` in `QuadEstimatorEKF.cpp`.

I successfully completed the entire simulation cycle with estimated position error of < 1m.*

```
Simulation #26 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```

### Task 6: Adding Own Controller ###

![alt text][image16]

In this step, I combined the estimator with the controller I implemented from last project. I replaced `QuadController.cpp` and `QuadControlParams.txt` with my own code, then ran the scenario `11_GPSUpdate` again.

I successfully completed the entire simulation cycle with estimated position error of < 1m.*

```
Simulation #102 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```