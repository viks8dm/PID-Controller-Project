# PID Controller Project

In this project-4 of term-2 of the self driving car nanodegree program by Udacity, a PID controller should be designed to minimize the cross-track-error (CTE) of the car in simulator so that it successfully drives for at least one lap under simulator conditions with reasonable driving behavior (considered relatively safe visually).

The project has been created using Udacity's [starter Code](https://github.com/udacity/CarND-PID-Control-Project)

## Implementation outline

The minimum requirement for successful implementation was to initialize the parameters in `PID.cpp` script; compute individual errors for proportional, derivative and integral terms; compute the total PID-error; and finally tune the PID parameters.

Tuning the PID parameters was the most difficult of the tasks listed above. For this purpose, one can potentially use some existing methods, like twiddle, as was outlined in one of the lectures. However, I leveraged some previous controls background and tuned the parameters by trial-and-error.

#### Vehicle speed control

After this basic implementation I ran the [simulator](https://github.com/udacity/self-driving-car-sim/releases/). However, it gave an uneasy feeling since the vehicle was running at the same speed on a straight road, as well as around bends. Hence, I followed on the suggestions and tweaked speed (`throttle-value`) based on CTE, CTE-change-rate, steering-angle and speed.

This is a very simple implementation where I follow following logic:

* `start with default throttle-value = 0.7`
* `IF CTE > 0.35, set throttle-value = 0.3`
* `IF steer-angle > 0.85 (indicates large correction)`
	* `IF speed > 25, apply brake: throttle-value = -0.1`
	* `ELSE slow-down, throttle-value = 0.1 `
* `ELSE `
	* `IF 0.1 < CTE-slope <= 0.2 (car trying sharp turn)`
		* `set throttle-value = 0.0 (release pedal)`
	* `ELSE CTE-slopE > 0.2 and speed>30 (high-speed, sharp turn)`
		* `apply harder brake, throttle-value = -0.2`

		
#### Limit steering-angle

As recommended, once the steering value is estimated based on total error update, a limit is enforced, so that the steering-value is in the range [-1, 1].


## Results & Discussion

[image1]: ./results/PID_final_CTE.jpg "CTE for final PID implementation"
[image2]: ./results/PID_final_speed.jpg "Speed for final PID implementation"
[image3]: ./results/PID_final_steering.jpg "Steering for final PID implementation"
[image4]: ./results/KP_KI_CTE.jpg "CTE for final-PID, KP-only, KP-KI implementation"
[image5]: ./results/KP_KI_speed.jpg "Speed for ffinal-PID, KP-only, KP-KI implementation"
[image6]: ./results/KP_KI_steering.jpg "Steering for final-PID, KP-only, KP-KI implementation"
[image7]: ./results/KD_CTE.jpg "CTE for low and high KD implementation"
[image8]: ./results/KD_speed.jpg "Speed for low and high KD implementation"
[image9]: ./results/car_stuck_KP_only.jpg "vehicle overshoots and gets stuck"

The final set of PID parameters I use for this implementation are: `KP = 0.12, KI = 0.006, KD = 40.0`. A video of the simulated car driving around the track with this final set of parameters, can be found [here](https://youtu.be/05zCk2-rL1U).`

Corresponding change in CTE, vehicle-speed and steering-angle over time are also shown in the images below. One can see some jitter in the CTE and steering-angle values, which I believe is because of the derivative parameter in the PID control. To correct this one can potentially further tune the `KD` value and/or reduce the time step (I have taken default time step here). In real-life situations, one should add a low-pass filter to the input data as well.

--

![alt text][image1]
![alt text][image2]
![alt text][image3]
--

### PID parameter tuning

Tuning PID controller for a given control problem is not an easy task, however, I have relied on some prior experience.

##### KP-tuning

The proportional term produces an output that is proportional to CTE. A high proportional gain will result in a large steering correction, which can will pull the vehicle close to center-line, however, the vehicle can potentially overshoot and become unstable (in one of the cases, it got stuck, as in shown in the image below). 

--

![alt text][image9]
--

On the other hand, a small KP can potentially lead to insufficient steering-angle correction which is problematic espetially when the vehicle is going around sharp turns. For this problem, some KP values were tried till it seemed that the vehicle was oscillating (at low-frequency) around the center line with approximately equal deviation on right and left side, or in other words, vehicle was showing a fixed-frequency sinusoidal behaviour, without significant overshoot beyond the lane boundaries on a straight section of the road. This can be seen in the plots of CTE and steering angle below, for `KP-only` curve.

##### KI-tuning

The integral term is sum of instantaneous error over time and gives correction estimate for accumulated offset. The integral term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure proportional controller. However, since the integral term responds to accumulated errors from the past, it can cause the present value to overshoot the setpoint value. This can be seen in the plots below, for `KP-KI` curve. The KI value for this plot is the final value I use. The tuning was done to the point where the similar sinusoidal behavior was observed in CTE value, as was mentioned for KP-tuning....one interesting point to note here is that the car actually started moving in a circle after some time, which can be seen from the steering-angle values in the 3rd plot below, as steering-angle becomes constant, while CTE shows sinusoidal behavior.

--

![alt text][image4]
![alt text][image5]
![alt text][image6]
--

##### KD-tuning

The derivative term changes with rate of change of CTE, and accounts for magnitude of the contribution of the derivative term to the overall control action. In effect, the derivative term predicts system behavior and helps in faster settling time and stability, if implemented properly. This is evident from the images shown above. If we compare `PID-final` curve, with `KP-only` or `KP-KI` curves, we can see that the `PID-final` does not lead to system overshoot.

A high value of `KD` will lead to overall smooth tracking, but it can cause system (vehicle) stall. As can be seen in the plots below, a high `KD` slows down the vehicle (due to overcorrection). On the other hand, a low `KD` value, will not correct the oscillations introduced by `KP` and `KI` terms and hence vehicle will keep oscillating around the road center with relatively higher magnitude than is desired (check CTE figure below).

Derivative term can potentially introduce high-frequency corrective response and is typically used with a low-pass filter.

For final tuning, once `KP` and `KI` were fixed, `KD` was tuned, starting with very low (1.0) and high (200) values were and then updated based on vehicle's performance, till a relatively smooth ride was observed (visually). 

--

![alt text][image7]
![alt text][image8]
--


## To run the code

Clone this repository and enter following commands in a terminal

`mkdir build && cd build`

`cmake .. && make`

`./pid`

After execution of `./pid`, simulator should be opened and it should be started by selecting the appropriate project for this implementation (namely, `PID-Control`)# PID-Controller-Project
