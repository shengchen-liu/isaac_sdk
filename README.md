# segway_rmp_lawson

[//]: # "Image References"

[image_lawson_iso]: ./images/lawson_iso.jpg "lawson_iso_view"
[image_rmp210]: ./images/rmp210.jpg "rmp210"
[image_kalman_filter]: ./images/Basic_concept_of_Kalman_filtering.svg.png "Kalman Filter Concept"
[image_imu_hookup_2]: ./images/IMU_hookup_2.jpg "LSM6DSS hookup"

[TOC]



## Hardware configuration

1. (iso view)

   ![alt text][image_lawson_iso]

2. (RMP 210)

   ![alt text][image_rmp210]

   (source: https://www.segway.com/robotics/commercial/rmp210/)

3. (Side view)

## IMU

An **inertial measurement unit** (**IMU**) is an electronic device that measures and reports a body's [specific force](https://en.wikipedia.org/wiki/Specific_force), angular rate, and sometimes the [orientation](https://en.wikipedia.org/wiki/Orientation_(geometry)) of the body, using a combination of [accelerometers](https://en.wikipedia.org/wiki/Accelerometer), [gyroscopes](https://en.wikipedia.org/wiki/Gyroscope), and sometimes [magnetometers](https://en.wikipedia.org/wiki/Magnetometers). IMUs are typically used to maneuver [aircraft](https://en.wikipedia.org/wiki/Aircraft) (an [attitude and heading reference system](https://en.wikipedia.org/wiki/Attitude_and_heading_reference_system)), including [unmanned aerial vehicles](https://en.wikipedia.org/wiki/Unmanned_aerial_vehicle) (UAVs), among many others, and [spacecraft](https://en.wikipedia.org/wiki/Spacecraft), including [satellites](https://en.wikipedia.org/wiki/Satellite) and [landers](https://en.wikipedia.org/wiki/Lander_(spacecraft)). 

(source: https://en.wikipedia.org/wiki/Inertial_measurement_unit)

### LSM6DS33

LSM6DS33 is a 6 DOF imu sensor that combines a digital 3-axis accelerometer and 3-axis gyroscope into a single package.  The sensor provides six independent acceleration and rotation rate readings whose sensitivities can be set in the ranges of ±2 g to ±16 g and ±125°/s to ±2000°/s, available through I²C and SPI interfaces.   Spec details can be found in this document: https://www.st.com/resource/en/datasheet/lsm6ds33.pdf.

Demo video:

<iframe width="560" height="315" src="https://www.youtube.com/embed/ZNAch-skd68" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
#### Wire hookup

| LSM6DSS33 | Jetson Xavier        |
| --------- | -------------------- |
| VDD       | 3.3 VDC (pin 1)      |
| GND       | GND (pin 6)          |
| SDA       | I2C_GP2_DAT (pin 27) |
| SCL       | I2C_GP2_CLK (pin 28) |
| SDO       | GND (pin 6)          |

![alt text][image_imu_hookup_2]

Each channel from IMU raw data is a 16-bit signal.  LSM6DS33 outputs a 'high' and a 'low' byte for each reading.  So in the code, we need to combine high and low bytes.

#### Gyro

The gyro measures the angular velocity ($\theta$) in three dimensions. 
$$
gyroRate=(gyroData[0] | gyroData[1] << 8)*gyroScale
$$
Unfortunately, the gyro drifts over time. That means it can not be trusted for a longer timespan, but it is very precise for a short time. This is when the accelerometer comes in handy. It does not have any drift, but it is too unstable for shorter timespan.

#### Accelerometer

The accelerometer measures the acceleration (g's) in three dimensions. 
$$
accel = accelData[0] | accelData[1] << 8
$$
The accelerometer  does not have any drift, but it is too unstable for shorter timespan. 

#### Fusion Algorithm

Gyro and accelerometer both have their own advantages and disadvantages.  Gyro is very precise, but tend to drift. The accelerometer is a bit unstable, but does not drift.  In order to obtain precise pose estimation, fusion algorithms are utilized. 

##### 1. Kalman Filter

**Kalman filtering**, also known as **linear quadratic estimation** (**LQE**), is an [algorithm](https://en.wikipedia.org/wiki/Algorithm) that uses a series of measurements observed over time,  in this context an accelerometer and a gyroscope.  These measurements containing [statistical noise](https://en.wikipedia.org/wiki/Statistical_noise) and other inaccuracies.  The Kalman filter estimates the state of the system, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone, by estimating a [joint probability distribution](https://en.wikipedia.org/wiki/Joint_probability_distribution) over the variables for each timeframe. 

(source: https://en.wikipedia.org/wiki/Kalman_filter)

In this context the problem is that the accelerometer is in general very noise when it is used to measure the gravitational acceleration since the robot is moving back and forth. The problem with the gyro is that it drifts over time – just like a spinning wheel-gyro will start to fall down when it is losing speed.
In short you can say that you can only trust the gyroscope on a short term while you can only trust the accelerometer on a long term.

![alt text][image_kalman_filter]

(source: https://en.wikipedia.org/wiki/Kalman_filter)

Kalman filtering is a recursive two-step process: *prediction*, and *update*. The **prediction step** produces estimates of the current variables along with their uncertainties. These estimates are based on the assumed model of how the estimates change over time. The **update step** is done when the next measurements *(subject to noise)* is observed. In this step, the estimates  are updated based on the weighted average of the predicted state and the state based on the current measurement. A lower weight is given to that with a higher uncertainty.

**The system state**

*Previous state*:
$$
\hat{x}_{k-1|k-1}
$$
Which is the previous estimated state based on the previous state and the estimates of the states before it.

*Prior state*:
$$
\hat{x}_{k|k-1}
$$
A priori means the estimate of the state matrix at the current time k based on the previous state of the system and the estimates of the states before it.

*Posterior state*:
$$
\hat{x}_{k|k}
$$
Is the estimated of the state at time k given observations up to and including at time k.

###### Prediction step

When designing the Kalman filter, we have to define the two linear functions: the state transition function and the measurement function.  The state transition function is

$$
x_k=F∗x_{k-1}+B\dot{u}_k+w_k
$$

Where $x_k$is the state matrix which is given by:

$$
x_k = \begin{pmatrix} \theta \\ \dot{\theta}_b \end{pmatrix}_k
$$

As you can see the output of the filter will be the angle ![\theta](http://s0.wp.com/latex.php?latex=%5Ctheta&bg=ffffff&fg=000000&s=0) but also the bias ![\dot{\theta}_b](http://s0.wp.com/latex.php?latex=%5Cdot%7B%5Ctheta%7D_b&bg=ffffff&fg=000000&s=0) based upon the measurements from the accelerometer and gyroscope. The bias is the amount the gyro has drifted. This means that one can get the true rate by subtracting the bias from the gyro measurement.

The next is the ![F](http://s0.wp.com/latex.php?latex=F&bg=ffffff&fg=000000&s=0) matrix, which is the state transition model which is applied to the previous state ![x_{k-1}](http://s0.wp.com/latex.php?latex=x_%7Bk-1%7D&bg=ffffff&fg=000000&s=0).

In this case ![F](http://s0.wp.com/latex.php?latex=F&bg=ffffff&fg=000000&s=0) is defined as:

![\boldsymbol{F} = \begin{bmatrix} 1 & -\Delta t \\ 0 & 1 \end{bmatrix}](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7BF%7D+%3D+%5Cbegin%7Bbmatrix%7D+1+%26+-%5CDelta+t+%5C%5C+0+%26+1+%5Cend%7Bbmatrix%7D&bg=ffffff&fg=000000&s=0)

The next is the control input ![u_k](http://s0.wp.com/latex.php?latex=u_k&bg=ffffff&fg=000000&s=0), in this case it is the gyroscope measurement in degrees per second (°/s) at time k, this is also called the rate ![\dot{\theta}](http://s0.wp.com/latex.php?latex=%5Cdot%7B%5Ctheta%7D&bg=ffffff&fg=000000&s=0). We will actually rewrite the state equation as:

![\boldsymbol{x}_k = \boldsymbol{F}x_{k-1} + \boldsymbol{B}{\dot{\theta}_k} + w_k](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7Bx%7D_k+%3D+%5Cboldsymbol%7BF%7Dx_%7Bk-1%7D+%2B+%5Cboldsymbol%7BB%7D%7B%5Cdot%7B%5Ctheta%7D_k%7D+%2B+w_k&bg=ffffff&fg=000000&s=0)

The next thing is the ![B](http://s0.wp.com/latex.php?latex=B&bg=ffffff&fg=000000&s=0) matrix. Which is called the control-input model, which is defined as:

![\boldsymbol{B} = \begin{bmatrix} \Delta t \\ 0 \end{bmatrix}](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7BB%7D+%3D+%5Cbegin%7Bbmatrix%7D+%5CDelta+t+%5C%5C+0+%5Cend%7Bbmatrix%7D&bg=ffffff&fg=000000&s=0)

This makes perfectly sense as you will get the angle ![\theta](http://s0.wp.com/latex.php?latex=%5Ctheta&bg=ffffff&fg=000000&s=0) when you multiply the rate ![\dot{\theta}](http://s0.wp.com/latex.php?latex=%5Cdot%7B%5Ctheta%7D&bg=ffffff&fg=000000&s=0) by the delta time ![\Delta t](http://s0.wp.com/latex.php?latex=%5CDelta+t&bg=ffffff&fg=000000&s=0) and since we can not calculate the bias directly based on the rate we will set the bottom of the matrix to 0.

![w_k](http://s0.wp.com/latex.php?latex=w_k&bg=ffffff&fg=000000&s=0) is process noise which is Gaussian distributed with a zero mean and with covariance ![Q](http://s0.wp.com/latex.php?latex=Q&bg=ffffff&fg=000000&s=0) to the time k:



![\boldsymbol{w}_k \sim N\left ( 0, \boldsymbol{Q}_k \right )](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7Bw%7D_k+%5Csim+N%5Cleft+%28+0%2C+%5Cboldsymbol%7BQ%7D_k+%5Cright+%29&bg=ffffff&fg=000000&s=0)

![Q_k](http://s0.wp.com/latex.php?latex=Q_k&bg=ffffff&fg=000000&s=0) is the process noise covariance matrix and in this case the covariance matrix of the state estimate of the accelerometer and bias. In this case we will consider the estimate of the bias and the accelerometer to be independent, so it’s actually just equal to the variance of the estimate of the accelerometer and bias.
The final matrix is defined as so:

![\boldsymbol{Q}_k = \begin{bmatrix} Q_\theta & 0 \\ 0 & Q_{\dot{\theta}_b} \end{bmatrix}\Delta t ](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7BQ%7D_k+%3D+%5Cbegin%7Bbmatrix%7D+Q_%5Ctheta+%26+0+%5C%5C+0+%26+Q_%7B%5Cdot%7B%5Ctheta%7D_b%7D+%5Cend%7Bbmatrix%7D%5CDelta+t+&bg=ffffff&fg=000000&s=0)

###### Update step

For the update step, we use the measurement function to map the state vector into the measurement space of the sensor. The observation ![z_k](http://s0.wp.com/latex.php?latex=z_k&bg=ffffff&fg=000000&s=0) is given by:



![\boldsymbol{z}_k = \boldsymbol{H}x_{k} + v_k](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7Bz%7D_k+%3D+%5Cboldsymbol%7BH%7Dx_%7Bk%7D+%2B+v_k&bg=ffffff&fg=000000&s=0)



As you can see the measurement ![z_k](http://s0.wp.com/latex.php?latex=z_k&bg=ffffff&fg=000000&s=0) is given by the current state ![x_k](http://s0.wp.com/latex.php?latex=x_k&bg=ffffff&fg=000000&s=0) multiplied by the ![H](http://s0.wp.com/latex.php?latex=H&bg=ffffff&fg=000000&s=0) matrix plus the measurement noise ![v_k](http://s0.wp.com/latex.php?latex=v_k&bg=ffffff&fg=000000&s=0).

![H](http://s0.wp.com/latex.php?latex=H&bg=ffffff&fg=000000&s=0) is called the observation model and is used to map the true state space into the observed space. The true state can not be observed. Since the measurement is just the measurement from the accelerometer, ![H](http://s0.wp.com/latex.php?latex=H&bg=ffffff&fg=000000&s=0) is given by:

![\boldsymbol{H} = \begin{bmatrix} 1 & 0 \end{bmatrix}](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7BH%7D+%3D+%5Cbegin%7Bbmatrix%7D+1+%26+0+%5Cend%7Bbmatrix%7D&bg=ffffff&fg=000000&s=0)



The noise of the measurement have to be Gaussian distributed as well with a zero mean and ![R](http://s0.wp.com/latex.php?latex=R&bg=ffffff&fg=000000&s=0) as the covariance:

![\boldsymbol{v}_k \sim N\left ( 0, \boldsymbol{R} \right )](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7Bv%7D_k+%5Csim+N%5Cleft+%28+0%2C+%5Cboldsymbol%7BR%7D+%5Cright+%29&bg=ffffff&fg=000000&s=0)



But as ![R](http://s0.wp.com/latex.php?latex=R&bg=ffffff&fg=000000&s=0) is not a matrix the measurement noise is just equal to the variance of the measurement, since the covariance of the same variable is equal to the variance. See this [page](http://en.wikipedia.org/wiki/Covariance_matrix#Generalization_of_the_variance) for more information.
Now we can define ![R](http://s0.wp.com/latex.php?latex=R&bg=ffffff&fg=000000&s=0) as so:



![\boldsymbol{R} = E \begin{bmatrix} v_k & {v_k}^T \end{bmatrix} = var(v_k) ](http://s0.wp.com/latex.php?latex=%5Cboldsymbol%7BR%7D+%3D+E+%5Cbegin%7Bbmatrix%7D+v_k+%26+%7Bv_k%7D%5ET+%5Cend%7Bbmatrix%7D+%3D+var%28v_k%29+&bg=ffffff&fg=000000&s=0)



More information about covariance can be found on [Wikipedia](http://en.wikipedia.org/wiki/Covariance_matrix#Conflicting_nomenclatures_and_notations) and in [my assignment](http://www.tkjelectronics.dk/uploads/Kalman_SRP.zip).

We will assume that the measurement noise is the same and does not depend on the time k:



![var(v_k) = var(v) ](http://s0.wp.com/latex.php?latex=var%28v_k%29+%3D+var%28v%29+&bg=ffffff&fg=000000&s=0)



Note that if you set the measurement noise variance ![var(v)](http://s0.wp.com/latex.php?latex=var%28v%29&bg=ffffff&fg=000000&s=0) too high the filter will respond really slowly as it is trusting new measurements less, but if it is too small the value might overshoot and be noisy since we trust the accelerometer measurements too much.

So to round up you have to find the the process noise variances ![Q_\theta](http://s0.wp.com/latex.php?latex=Q_%5Ctheta&bg=ffffff&fg=000000&s=0) and ![Q_{\dot{\theta}_b}](http://s0.wp.com/latex.php?latex=Q_%7B%5Cdot%7B%5Ctheta%7D_b%7D&bg=ffffff&fg=000000&s=0) and the measurement variance of the measurement noise ![var(v)](http://s0.wp.com/latex.php?latex=var%28v%29&bg=ffffff&fg=000000&s=0)





## lawson_sim

### lawson_sim_third_person

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/lawson_sim:lawson_sim -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
   ```

   

2. Start Unreal Engine

   ```bash
   shengchen@shengchen-HP:~/isaac/UnrealEngine-IsaacSim_1.2$ ./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P -vulkan -isaac_sim_config_json="/home/shengchen/isaac/isaac-sdk-2019.2-30e21124/apps/segway_rmp_lawson/lawson_sim/bridge_config/lawson_full.json"
   ```

3. (video here)

### lawson_sim_joystick

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/lawson_sim:lawson_sim -- --config="apps/assets/maps/carter_warehouse_p.config.json" --graph="apps/assets/maps/carter_warehouse_p.graph.json"
   ```

2. start Unreal Engine

   ```bash
shengchen@shengchen-HP:~/isaac/UnrealEngine-IsaacSim_1.2$ ./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P -vulkan -isaac_sim_config_json="/home/shengchen/isaac/isaac-sdk-2019.2-30e21124/apps/segway_rmp_lawson/lawson_sim/bridge_config/lawson_full.json"
   ```
   
3. (video here)

### lawson_sim_mapping

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson/lawson_sim:lawson_sim_mapping
   ```

   

2. start Unreal Engine

   ```bash
   shengchen@shengchen-HP:~/isaac/UnrealEngine-IsaacSim_1.2$ ./Engine/Binaries/Linux/UE4Editor IsaacSimProject CarterWarehouse_P -vulkan -isaac_sim_config_json="/home/shengchen/isaac/isaac-sdk-2019.2-30e21124/apps/segway_rmp_lawson/lawson_sim/bridge_config/lawson_full.json"
   ```

3. (video here)

## lawson

1. start application

   ```bash
   shengchen@shengchen-HP:~/isaac/isaac-sdk-2019.2-30e21124$ bazel run apps/segway_rmp_lawson:lawson
   ```

   