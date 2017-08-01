# Project 9: PID Control
[//]: # (Image References)

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


Overview
---
Implement a PID controller in C++ to maneuver the vehicle around a simulated track

#### The goals / steps of this project are the following:
* Build a PID controller 
* Tune the PID hyperparameters by testing the model on the track
* Make sure no tire leaves the drivable portion of the track

Project Deliverables
---
* `main.cpp` for calling the model and communicating with the simulator
* `PID.cpp` defines the PID class with code for initialization, updating error, and returning the total error

Results
---
**The components of PID**

* "P" for proportional makes the vehicle steer in proportion to the cross-track error(CTE). Basically, this makes the vehicle steer in the correct direction. However, a P value that is too high will result in an unstable oscillating driving behavior as the vehicle will constantly try to correct its trajectory and overshoot. The vehicle with just the "P" value alone will never converge and keep oscillating.
* "D" for derivative helps compensate for the overshooting behavior caused by the "P" component by making the controller output proportional to the rate of change of CTE. This counters the oscillations, leading to a more stable driving behavior. However, too high of a "D" value will cause constant change of steering angle.
* "I" for integral takes the integral of the CTE overtime to compensate for systematic bias. This allows the vehicle to head back towards the middle.

**How I tuned the parameters**

I started out by finding a reliable "P" value while keeping the other parameters to 0. Once I found a value that kept the vehicle on track for the most part(although very wobbly), I started tuning the "D" value to reduce the oscillating behavior. Interestingly, the "I" value had little effect in the simulation. This may be because the simulator does not account for systematic bias.

In the end, P = 0.3, I = 0.002, D = 3.0 were chosen as the final parameters.

---

## Dependencies

* Udacity Simulator [download](https://github.com/udacity/self-driving-car-sim/releases)
* uWebSocket [download](https://github.com/uWebSockets/uWebSockets)
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
