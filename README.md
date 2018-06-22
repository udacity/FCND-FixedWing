# FCND - Fixed Wing Project #

This is the readme for the Udacity Flying Car Nanodegree Fixed Wing Project

For easy navigation throughout this document, here is an outline:

 - [Development environment setup](#development-environment-setup)
 - [Simulator walkthrough](#simulator-walkthrough)
 - [Control the Simulator with Python](#pyton-control)
 - [The Scenarios](#the-scenarios)
 - [Evaluation](#evaluation)


## Development Environment Setup ##

Regardless of your development platform, the first step is to download or clone this repository.

### Udacidrone ###

Next you'll need to get the latest version of [Udacidrone](https://udacity.github.io/udacidrone/docs/getting-started.html).
If you've previously installed Udacidrone, ensure that you are working with version 0.3.2 or later.

### Unity Simulation ###

Finally, download the version of the simulator that's appropriate for your operating system [from this repository](https://github.com/udacity/FCND-FixedWing/releases). (v0.1.2 or higher for longitudinal scenarios)

## Simulator Walkthrough ##

Now that you have all the code on your computer and the simulator running, let's walk through some of the elements of the code and the simulator itself.

### Fixed Wing UI ###

The simulator interface should look fairly familiar to the Unity quadcopter simulation with a few additions:

- Airspeed is displayed along with the GPS information
- Scenario menu (explained more below), the simulation will start in the Sandbox mode
- Throttle setting (displays the throttle setting between 0 (no throttle) and 1 (full throttle)

### Sandbox Mode ###

Try flying around in sandbox mode. There are several different flight modes available, you'll start in manual.

#### Manual Mode ####

You have direct control over the throttle, ailerons, elevators, and rudder. Use the following keyboard commands to control the aircraft:

- C/Space: throttle control (incremental)
- Up/Down Arrow: elevator control
- Left/Right Arrow: aileron control
- Q/E: rudder control
- W/S: elevator trim (incremental)

#### Other Flight Modes ####

TBD

### Scenario Selection ###

Click the scenario selection menu to see a drop-down list of possible scenarios. When you select a scenario, the aircraft will be reset to a specified starting location and you will be shown a start-up screen. The start-up screen will describe the scenario task and the evaluation criteria. In the start-up screen, you'll have two options to choose from:

- Tune parameters: select control parameters for the Unity simulation will be displayed for tuning prior to starting the scenario. The scenario will be run with the unity simulation
- Run Python code: a screen will appear waiting for Python controller to connect/arm. The screen will disappear and the scenario will start once the vehicle is armed from Python

Most of the scenarios have a time limit associated with them. Upon completion (pass or fail) there will be a window showing your result. From there, you can select another scenario or continue retry the current scenario.

### Tuning Unity Parameters

The gains of the control system used internally on the Unity simulation can be tuned. The longitudinal and lateral control gains are tuned independently but the control gains within each type build upon each other (i.e. you cannot tune the other loop without first tuning the inner loop gains). To tune the gains:

1. Default gains are loaded from the text file, gains.txt. The file will be located in the same folder as the simulation executable. Set the values prior to running th program.
2. Choose the appropriate scenario from the scenario selection menu
3. Click the "Tune Parameter" button

![tune_param](Diagrams/tuning1.png)

4. A list of parameters will appear. Adjust the sliders or the set the fields to the values you'd like for the parameters.

![params](Diagrams/tuning2.png)

5. Run the scenario!
6. If you want to save the parameters to a text file, click the save button on the top right corner. The parameters are saved to a file named gains_new.txt. To use these default values next time the simulation is executed, rename the file to gains.txt

![save](Diagrams/tuning3.png)


## Python Control ##

The simulation can also be controlled using a Python script and the Udacidrone API. There are three relevant python files found in the FixedWing project repository:

- plane_control.py: this is where you will fill in the control code
- plane_drone.py: contains PlaneDrone, a sub-class of the Udacidrone drone class with additional commands specific to the fixed wing project
- fixed_wing_project.py: contains a subclass of PlaneDrone specifically set-up to run the scenarios

### Running a scenario ###
To run a scenario from Python:
1. Select the scenario within the Unity simulation
2. Select the "Run From Python" button, you should see a "Waiting for Python" message
3. Change fixed_wing_project.py to execute the same scenario (see below)
3. Execute fixed_wing_project.py and the scenario should start autonomatically:

~~~
python fixed_wing_project.py
~~~

If fixed_wing_project.py is run before being prompted onscreen in the Unity simulation, the python code will be unable to connect.

TODO: This needs to be easier to execute for the students possible with a command line argument
To select a different scenario, you need to change the last line of fixed_wing_project.py prior to executing it. For example the following will run the Trim scenario

~~~
drone.run_scenario(Scenario.TRIM)
~~~

The following are valid scenario names:
* Scenario.SANDBOX
* Scenario.TRIM
* Scenario.AIRSPEED
* Scenario.ALTITUDE
* Scenario.CLIMB
* Scenario.LONGITUDINAL
* Scenario.ROLL
* Scenario.TURN
* Scenario.YAW
* Scenario.LINE
* Scenario.ORBIT
* Scenario.LATERAL 


## The Scenarios ##

You'll be implementing several Python controllers in the plane_control.py in order to complete all the scenarios. The scenarios are divided into 2 categories:

- Longitudinal Scenarios
- Lateral/Directional Scenarios

The scenarios within a category will build on one another, so you will need to implement and tune them in order. Each category will end in a challenge which will use the control loops you set up in the preceding scenarios.

Prior to completing a scenario, it's suggested that you first use the Unity based controller to tune the gains. If implemented correctly on Python, the Unity controller gains should get close to meeting the objectives of the scenario, although minor tuning may be required.


### Longitudinal Scenarios ###

The longitudinal scenarios are designed to incrementally implement control loops to command the aircrafts airspeed, pitch, and altitude using the elevator and the throttle. When running these scenarios from Python, a Unity based lateral controller will maintain a near-zero bank and sideslip.

#### Scenario #1: Trim (Unity Only) ####

The objective of this scenario is to find a fixed throttle trim for level flight with no elevator input.

To achieve success in this scenario two objectives must be met for at least 5 seconds:

- The vertical speed must be less than 0.5 m/s
- The airspeed rate must be less than 0.1 m^2/s

This scenario will run indefinitely.

Completing this scenario will help find a route estimate for your feed-forward throttle setting.

Tips:

- Try small increments in your throttle
- If both the airspeed AND altitude are increasing, the throttle is probably too high.
- If both the airspeed AND altitude are decreasing, the throttle is probably too low.
- If the airspeed and altitude are opposite from one another, you'll have to wait for the the phugoid mode to damp out.
- The phugoid oscillations can be very lightly damped, the damping can be assisted by using the elevator controls.



#### Scenario #2: Altitude Hold ####

![altitude](Diagrams/altitude_hold.png)

The objective of this scenario is to tune/implement a controller to maintain a constant altitude using the elevator. The throttle will be set to a fixed value. The altitude hold should be implemented using successive loop closure as shown above. The inner loop will be a PD controller on the aircraft pitch. The outer loop will be a PI controller on the aircraft altitude. Ensure to implement anti-windup for the integrator.


To complete this scenario:

- The altitude must be within +/-3 meters of the target altitude (450 meters) within 10s
- The altitude must maintain within those bounds for 5 seconds

This controller should be implemented in plane_control.py, by filling in the following functions:
~~~py

"""Used to calculate the elevator command required to acheive the target
    pitch

	Args:
		pitch: in radians
		pitch_rate: in radians/sec
		pitch_cmd: in radians

	Returns:
		elevator_cmd: in percentage elevator [-1,1]
"""
def pitch_loop(self, pitch, pitch_rate, pitch_cmd):
	elevator_cmd = 0.0
	# STUDENT CODE HERE
	return elevator_cmd

"""Used to calculate the pitch command required to maintain the commanded
    altitude
    
    Args:
    altitude: in meters (positive up)
    altitude_cmd: in meters (positive up)
    dt: timestep in seconds

	Returns:
		pitch_cmd: in radians
"""
def altitude_loop(self, altitude, altitude_cmd, dt):
	pitch_cmd = 0.0
	# STUDENT CODE HERE
	return pitch_cmd
~~~


Tips:

- Implement and tune the inner most loop first and work outward.
- Increase the proportional pitch gain until the aircraft is nearly unstable OR set a maximum pitch angle and set the gain to be full elevator at the maximum pitch angle
- Increase the derivative pitch gain to achieve a nice dynamic response (fast but not too many oscillations)
- Next, increase the proportional altitude gain until your achieve a nice step response
- Finally, increase the integral altitude gain to meet the scenario objective threshold.
 

#### Scenario #3: Airspeed Hold ####

![airspeed_hold](Diagrams/airspeed_hold.png)

The objective of this scenario is to tune/implement a controller to maintain a constant airspeed. The altitude will be maintained using the altitude controller from the previous scenario. The controller should be implemented as a PI controller using the throttle setting. Ensure to implement anti-windup for the integrator.

To complete this scenario:

- The airspeed must be within +/- 0.5 meters/sec of the target airspeed (41 meters/sec) within 10s
- The airspeed must maintain the airspeed within those bounds for 5 seconds

This controller should be implemented in plane_control.py, by filling in the following function:
~~~py

"""Used to calculate the throttle command required command the target 
    airspeed

	Args:
		airspeed: in meters/sec
		airspeed_cmd: in meters/sec
        dt: timestep in seconds

	Returns:
		throttle_command: in percent throttle [0,1]
"""
def airspeed_loop(self, airspeed, airspeed_cmd, dt):
	throttle_cmd = 0.0
	# STUDENT CODE HERE
	return throttle_cmd  
~~~



Tips:

- First, set your throttle feed-forward value determined from the trim analysis
- Next, increase the proportional gain until you get an acceptable step response (fast but not too many oscillations)
- Increase the integral gain to increase the rate at which the steady state error is removed
- If you notice a large contribution from the integral portion of your controller at steady state, adjust your feed-forward throttle setting accordingly. This should allow you to decrease the value of you integral gain (and provide a better dynamic response).


#### Scenario #4: Steady Climb ####

![climb](Diagrams/airspeed_pitch_hold.png)

The objective of this scenario is to tune/design a controller to maintain a constant airspeed using the elevator with full throttle and a PI controller as shown above. This will put the aircraft in a steady climb. In the previous scenario, the pitch angle was used to control altitude. In this scenario, the pitch angle will be used to control the airspeed. Ensure to implement anti-windup for the integrator.

To complete this scenario:

- The airspeed must be within +/-1 meters/sec of the target airspeed (41 meters/sec) within 10s
- The airspeed must maintain within those bounds for 5 seconds

This controller should be implemented in plane_control.py, by filling in the following functions:
~~~py

"""Used to calculate the pitch command required to maintain the commanded
    airspeed

	Args:
		airspeed: in meters/sec
		airspeed_cmd: in meters/sec
        dt: timestep in seconds

	Returns:
		pitch_cmd: in radians
"""

def airspeed_pitch_loop(self, airspeed, airspeed_cmd, dt):
	pitch_cmd = 0.0
	# STUDENT CODE HERE
	return pitch_cmd
~~~

Tips:

- Use the same inner loop pitch controller from the previous scenario. If you change the inner loop gains, return to the previous scenario to ensure they meet the altitude hold objectives.
- If you saturated the commanded pitch inside the inner loop, you may have to increase the limits. A steady climb at full throttle will need to command higher pitch angles than a constant altitude controller.
- Start with increasing the proportional airspeed gain to achieve a nice dynamic response (fast but not too many oscillations)
- Finally, increase the integral airspeed gain to meet the scenario objective threshold.


### Scenario #5: Longitudinal Challenge ###

The objective of this challenge is to successfully fly through a series of virtual gates in the sky. 
To do this, tune/implement a longitudinal state machine: 
 
 - If the vehicle is within a specified threshold of the target altitude, use the maintain altitude controller.
 - If the vehicle is below or above the target altitude by the specified threshold, use the steady climb/descend controller with full or min throttle (respectively)

To complete the challenge, your altitude must be within +/-3 meters when arriving at the gate.
The gate locations (x = horizontal distance from the start location):

 - Gate #1: x = 200m, altitude=200m
 - Gate #2: x = 1100m, altitude = 300m
 - Gate #3: x = 1400m, altitude = 280m
 - Gate #4: x = 2200m, altitude = 200m

This controller should be implemented in plane_control.py, by filling in the following functions:

~~~py

"""Used to calculate the pitch command and throttle command based on the
    aicraft altitude error

	Args:
		airspeed: in meter/sec
		altitude: in meters (positive up)
		airspeed_cmd: in meters/sec
		altitude_cmd: in meters/sec (positive up)
        dt: timestep in seconds

	Returns:
		pitch_cmd: in radians
		throttle_cmd: in in percent throttle [0,1]
"""
def longitudinal_loop(self, airspeed, altitude, airspeed_cmd, altitude_cmd, dt):
	pitch_cmd = 0.0
	throttle_cmd = 0.0
	# STUDENT CODE HERE
	return[pitch_cmd, throttle_cmd]
~~~

Tips:

- The only parameter to tune in the challenge should be the threshold at which to switch between the different modes of control. The altitude hold scenario starts the aircraft 20m below the target altitude, which would be a good starting point for this threshold.
- Well tuned parameters from the Longitudinal scenarios will help make this challenge less challenging. If you change your control gains for this challenge, return to the scenarios to ensure the new gains meet the objectives of all previous scenarios.

### Lateral/Directional Scenarios ###

The lateral/directional scenarios are designed to incrementally implement control loops to command the aircrafts airspeed, pitch, and altitude using the elevator and the throttle. When running these scenarios from Python, a Unity based longitudinal controller will maintain altitude and airspeed.

#### Scenario #6: Stabilized Roll Angle ####

TBD

#### Scenario #7: Coordinated Turn ####

TBD

#### Scenario #8: Constant Course ####

TBD

#### Scenario #9: Straight Line Following ####

TBD

#### Scenario #10: Orbit Following ####

TBD

### Scenario #11: Lateral/Directional Challenge ###

TBD

### Scenario #12: Full 3D Challenge ###

TBD

## Evaluation ##

The longitudinal, lateral/directional, and full 3D challenges will be evaluated for successful completion of the objectives. Success completion of the controller

### Inconsistent Results ###

The scenarios/challenges are evaluated within Unity using the true aircraft position. Due to asynchronous communication between the Python controller and Unity, successive runs of the same scenario with the same controller may yield different results depending on the speed of your machine and/or other processes running on it.

If this is the case here are some tips to help get a consistent result:

- Don't stress! Implementing controllers for aircraft in the real world yield different results every time they are tested! If you can get a successful result more than 50% of the time, save the log file from one of the successful runs. Most likely the implementation will be a success when evaluated because it will be run on sufficiently fast machines.
- Tune the gains to not only meet the objectives of the scenarios but also exceed them.
- Try being less aggressive on your inner loops (smaller gains)
- Use the simple graphics setting available in the simulation.