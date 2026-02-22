# Quadcopter Project: 6DOF MATLAB Simulation
**Part 1 of building a quadcopter** <br>
<a href="https://github.com/ex33/drone_sim" class="tag tag-big">GITHUB</a>
---
<figure style="text-align: center;">
  <img
    src="simulation_videos/progression/trajectory_video_full_sim.gif"
    alt="order-0-video2"
    style="width: 100%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.8em">
    Full Closed Loop Simulation w/ Filter & LQR 
  </figcaption>
</figure>




## Summary
This is <strong>Part 1</strong> of my quadcopter project. This part will go over my 6DOF simulation in MATLAB completed with: <br>
<p>• Sensor modeling w/ noise </p>
<p>• Motor modeling w/ time delay</p>
<p>• LQR Controller for setpoint regulation (waypoint)</p>
<p>• 18-State Error-State Kalman Filter w/ Bias States </p>
<p>• Closed-loop propagation through non-linear dynamics for end-to-end GNC validation</p>

For more background material, see [Part 0](quadcopterPart0.html). This section contains little to no context on some of the things that goes on, so it somewhat requires taking a quick look at the previous section. I will try to come back here and make this more stand-alone at some point.


This was the status of my Simulation as of [1/15/2026]

## Table of Content
<span style="font-size:1.3em;">• <a href="#introduction">Introduction</a></span><br>

<span style="font-size:1.3em;">• <a href="#overview">Overview</a></span><br>

<span style="font-size:1.3em;">• <a href="#simulation-modules">Simulation Modules</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#sensors">Sensors</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#error-state-kalman-filter">Error State Kalman Filter</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#guidance">Guidance</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#control">Control</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#motors">Motors</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#Dynamics">Dynamics</a></span><br>


<span style="font-size:1.3em;">• <a href="#simlation-progression">Simulation Progression</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#order-0-simulation">Order 0 Simulation</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#order-1-simulation">Order 1 Simulation</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#order-2-simulation">Order 2 Simulation</a></span><br>

<span style="font-size:1.3em;">• <a href="#utilizing-the-simulation">Utilizing the Simulation</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#hardware-selection">Hardware Selection</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#airframe">Airframe</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#battery">Battery</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#motors">Motors</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#propellers">Propellers</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#microcontroller">Microcontroller</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#sensors">Sensors</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#electronic-speed-controller">Electronic Speed Controller</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#flight-computer-board">Flight Computer Board</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#system-parameters">System Parameters</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#sensor-noise-parameters">Sensor Noise Parameters</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#vehicle-parameters">Vehicle Parameters</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#tuning-guidelines">Tuning Guidelines</a></span><br>


<span style="font-size:1.3em;">• <a href="#whats-next">Whats Next?</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#integral-control-to-lqr-lqi">Integral Control to LQR (LQI)</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#optimal-trajectories">Optimal Trajectories</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#other-uav-platforms">Other UAV Platforms</a></span><br>






<h2 id="Introduction">Introduction</h2>
Since this is the first part, the introduction to this project will be placed here. 

From my first full time role, I was able to gain lots of experience in simulation and modeling of satellites, having the opportunities to determine necessary answers to mature a design from conception, and executing the accompanying analysis needed for design choices. I was able to deepen my understanding of many GNC concepts from my coursework through real world applications, particularly in state estimation, guidance algorithms (orbital maneuvers/trajectories and attitude pointing), and system level integration of hardware. I also became more confident in my software skills, having to learn RPO and C++ at the same time, and spending hundreds (if not thousands) of hours contributing to the many simulation tools and reviewing/approving pull requests.  

However, at the start of this, my main knowledge gap still lies within the transition from simulation onto real hardware and all the associated challeges that come with that, which is what I'd hoped to bridge with this project. While I would have loved to build a satellite at home, I do not have the funding to launch it, leading me to select a quadcopter as it requires the same algorithms/skills to design from scratch and is controllable in all 6 degrees of freedom. 

Since drones and satellites have nothing else in common (aside from the control and estimation theories), I should be safe from unknowingly using any sensitive knowledge. 

<strong>Disclaimer: </strong> <br>
This project is mostly for me to learn what it takes to get a customized GNC system flying on a quadcopter. Given that I have a full time job, I wasn't planning to build the best quadcopter or do anything novel yet. The focus was for me to learn and try to see how well I can make the final product myself (so no references to open source software like Betaflight or Ardupilot). 

Additionally, I chose to focus on the simulation and translation of the algorithms onto hardware. This meant using breakout boards with compatible libraires to avoid developing the low level interfaces, and skipping the PCB design of the flight computer (I use a Teesny4.1 and soldered everything to a prototyping board), but more on the hardware choices later. While I do hope to make these custom at some point, it would've added severe delay to a first prototype, which would just be validating the GNC algorithms. 

---

<h2 id="Overview">Overview</h2>
My simulation was written in MATLAB due to the ability to quickly diagnose bugs with its robust ability to put breakpoint and step through files line by line, extracting out each variable value. 

While theoretically this could've all been done in a faster language, I didn't plan on extended studies requiring a large batches. This was meant to give me a sandbox to quicky iterate through software designs, tune my filters / controllers, and observe behaviors given uncertainties in the system parameters. 

Things like observing if sequential updates had a difference from the standard block updates, and if that further differs from scalar sequential updates (which is implemented in the flight software). From literature, they should all be the same given the assumption that the measurements are uncorrelated, but being able to see it for myself is more convincing than reading about it. Or given my measured thrust constant, is it worse to under or over estimate it? The answer would logically be underestimating is worse, given that you would be requesting higher spin rates given a controller output, which could be quickly verified.

Admittedly, I do have goals of eventually using this platform to implement a onboard optimizer for the guidance module, which would require all the different level of fidelity to validate. At that point, I may re-use the code for the FSW to transfer onto a C++ implementation to be able to more formally analyze the system through Monte-carlo and make claims on stability and robustness.



<h2 id="simulation-modules">Simulation Modules</h2>
Below is a high level flow diagram of my simulation:
<figure style="text-align: center;">
  <img
    src="images/SimulationOverview.png"
    alt="Simulation Overview"
    style="width: 60%; max-width: 500px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 1: Simulation Overview
  </figcaption>
</figure>

This layout should be pretty standard, with the exception that if it were more complex, there may be an extra environments block that adds in extra aerodynamic disturbances. I also at this time did not implement a Finite-State Machine into the simulation, as I wasn't sure how this would've looked on board just yet, nor what the modes themselves would look like. 

The simulation starts with calling run_sim('input.jsonx') in the MATLAB command window, where input.jsonx contains all the simulation parameters. This includes things like simulation time, vehicle parameters, and sensors / guidance / control / navigation / motor parameters.

I won't go through all the algorithms here as those were compiled into a different page ([Quadcopter Part 0](quadcopterPart0.html)), but just quickly go through the different modules mentioned above. 

<h3 id="sensors">Sensors</h3>
There are 4 sensor classes: IMU, Magnetometer, Altimeter, and GPS. 

Within the main loop, there is a "getSensors" function that given the simulation time, current state and control, and all the sensor instances, collects all avaliable sensor measurements for the given timestep, and compile them into a vector.

All of these sensors return the RAW measurements, such that I am book-keeping all the different conversions I need to have in my Sensors Module on the FSW. This includes converting LLA to NED positions for the GPS, pressure to height for the altimeter, and general rotations to convert the sensor frame components into the body frame. 

<div class="latex-algorithm">
  <div class="alg-title">getSensors.m</div>

  <div class="alg-body">
    <div class="alg-line">
      <span class="kw">Input:</span> Simulation Time ($\mathbf{t}$), Truth State ($\mathbf{x_k}$), True Force / Torques ($\mathbf{u_k}$), Sensor Classes ($\mathbf{sensors}$)
    </div>

    <div class="alg-line indent">
      1) Form true acceleration ($\mathbf{a}$) and extract the true states.
    </div>

    <div class="alg-line indent">
      2) Obtain the raw measurements ($\mathbf{sensors}$.measurement($\mathbf{t}$, $\mathbf{x_k}$, $\mathbf{a}$))
    </div>

    <div class="alg-line indent">
      3) Processing of any raw measurements as necessary (rotations, conversions, etc)
    </div>

    <div class="alg-line indent">
      4) Compile results 
    </div>


    <div class="alg-line">
      <span class="kw">return</span> Measurement Vector $\mathbf{z}$
    </div>
  </div>
</div>



These steps are not needed if I was trying to do this as quickly as possible, but I wanted the simulation to be a reference and act as a checklist as I started to translate things. 

<h3 id="error-state-kalman-filter">Error State Kalman Filter</h3>
The measurement vector then is passed into "run_navigation", which also takes in the simulation time and the instance of the filter class, and runs one loop of it and returns the estimated system state. 
<div class="latex-algorithm">
  <div class="alg-title">run_navigation.m</div>

  <div class="alg-body">
    <div class="alg-line">
      <span class="kw">Input:</span> Simulation Time ($\mathbf{t}$), Measurement Vector ($\mathbf{z}$), Filter Class ($\mathbf{eskf}$)
    </div>

    <div class="alg-line indent">
      1) Step forward filter with $\mathbf{eskf}$.step($\mathbf{t}$,$\mathbf{z}$)
    </div>

    <div class="alg-line indent">
      2) Get current state estimate from $\mathbf{eskf}$.xOut
    </div>

    <div class="alg-line">
      <span class="kw">return</span> Estimated State $\mathbf{x_{k1}}$
    </div>
  </div>
</div>


There isn't too much to add here as the function call is just two lines (one that calls on eskf.step, and the other forms the output). All the calculations happens internally within the filter class, which follows the standard predict update loop. The one thing that could be mentioned here is that I ended up adding both a version of the filter with and without bias state estimations, and also matrix vs scalar sequential updates. 

The two versions of the filters came from more of a testing perspective. It was considerably easier to tune the no bias state filter, giving me confidence that if there was a bug or implementation error that I could not figure out, I still have something to proceed with and not become a blocker. Additionally, at the time it wasn't clear if I wanted to even implement the bias states within the filter for the FSW, as my flights would not nearly last long enough for the bias of the IMU to grow to a point that would impact the state estimation. They perhaps would be useful for taking care of unaccounted biases and allow the filter to account for them, but for something like the magnetometer bias, this behavior lead to some undesireable results (discussed more in Part 2).

As for the update step, I started off with a matrix update. This just means if the GPS measurement comes in for example, its processed with a matrix inversion. However, to avoid needing to do that for my FSW, I also added in the option to process all measurements as a scalar by breaking up the components, treating them as uncorrelated, and processing them individually. There may be a loss of accuracy doing this, which was why I wanted to verify it was fine in simulation first. 

<h3 id="guidance">Guidance</h3>
The true / estimated states (selected from input file) along with user inputted guidance parameters are passed into "run_guidance" to then calculate the LQR system matrices linearized about the desired reference points and nominal control. 
<div class="latex-algorithm">
  <div class="alg-title">run_guidance.m</div>

  <div class="alg-body">
    <div class="alg-line">
      <span class="kw">Input:</span> Simulation Time ($\mathbf{t}$), State Vector ($\mathbf{x}$), Guidance Struct ($\mathbf{guid}$), Vehicle Struct ($\mathbf{vehicle}$)
    </div>

    <div class="alg-line indent">
      1) Extract all necessary parameters from the Guidance ($\mathbf{guid}$) and Vehicle Parameters ($\mathbf{vehicle}$)
    </div>

    <div class="alg-line indent">
      2) Based on the inputs, select the reference state ($\mathbf{x_{ref}}$) and calculate the nominal control ($\mathbf{u_{ref}}$) 
    </div>

    <div class="alg-line indent">
      3) Calculate system matrices ($\mathbf{A}$ & $\mathbf{B}$)
    </div>

    <div class="alg-line">
      <span class="kw">return</span> Reference State $\mathbf{x_{ref}}$, Nominal Control ($\mathbf{u_{ref}}$), System Matrices ($\mathbf{A}$ & $\mathbf{B}$)
    </div>
  </div>
</div>

While re-calculating the system matrices are unnecessary since they are dependent upon the reference quaternion and body rates (which are mostly constant), this feature was placed here for more complex guidance algorithms that may want to change those. However, even in that case, it is unclear whether its sufficent to just change the attitude references without changing the system matrices.

Currently, the only options for reference types are step vs ramp. As the names suggests, step just uses a constant value as the reference, while ramp allows for the reference to slowly ramp up to the final value based on a given slope. 

The nominal control is basically always just the amount of thrust needed to counteract gravity. I haven't come across any situation in which this would change, except maybe in the cases of optimally generated trajectories based on different constraints requiring different nominal thrusts to achieve a specified acceleration. 

<h3 id="control">Control</h3>
The true / estimated states, reference states, nominal control, and system matrices along with the user inputted gain matrices are passed into "run_controller" to then calculate the desired control effort.
<div class="latex-algorithm">
  <div class="alg-title">run_controller.m</div>

  <div class="alg-body">
    <div class="alg-line">
      <span class="kw">Input:</span> State Vector ($\mathbf{x}$), Reference State $\mathbf{x_{ref}}$, Nominal Control ($\mathbf{u_{ref}}$), System Matrices ($\mathbf{A}$ & $\mathbf{B}$), Gains ($\mathbf{Q}$ & $\mathbf{R}$)
    </div>

    <div class="alg-line indent">
      1) Calculate the state error 
    </div>

    <div class="alg-line indent">
      2) Find the optimal gain matrix ($\mathbf{K}$) using MATLAB's dlqr.m function
    </div>

    <div class="alg-line indent">
      3) Calculate the total control ($\mathbf{u_des}$) according to the control law
    </div>

    <div class="alg-line">
      <span class="kw">return</span> Total Control $\mathbf{u_des}$
    </div>
  </div>
</div>

This part is very straight forward so nothing extra to add. 

<h3 id="motors">Motors</h3>
The calculated control then is passed into "run_motors", which also takes in the simulation time and the instance of the motor class, and calculates the equivalent commands, returning the produced thrust and torques after a slight time delay.
<div class="latex-algorithm">
  <div class="alg-title">run_motors.m</div>

  <div class="alg-body">
    <div class="alg-line">
      <span class="kw">Input:</span> Simulation Time ($\mathbf{t}$), Total Control ($\mathbf{u_des}$), Motors Class ($\mathbf{motors}$)
    </div>

    <div class="alg-line indent">
      1) Calculate motor commands ($\mathbf{PWM}$) via the inverse of the allocation matrix ($\mathbf{M^{-1}}$) to convert $\mathbf{u_des}$
    </div>

    <div class="alg-line indent">
      2) Apply a small time delay via a difference equation thats parameterized by its time constant $\tau$
    </div>

    <div class="alg-line indent">
      3) Calculates the resulting thrust and torques using the allocation matrix ($\mathbf{M}$) 
    </div>

    <div class="alg-line">
      <span class="kw">return</span> Outputted Thrust and Torques $\mathbf{u_k}$
    </div>
  </div>
</div>

So far, nothing is really done with the motor commands inside the simulation aside for plotting, and for reference for unit tests for the FSW. 

In reality, the commanded PWM should be combined with some battery model to determine the actual out putted spin rates of each motor, but that is a lot higher fidelity than needed. 

The time delay here is also modeled as a first order transfer function ($\frac{1}{\tau s + 1}$). While this does slightly change the gain of the system, its impacts are only on the high frequency spectrum and is negliable as the pole is far to the left hand side of the s-plane (so on a bode plot, the point in which the gain changes is probably not within the region that matters). The main desired effect is the time delay it introduces to the system, where the settling time of the response is ~$4\tau$. 

<h3 id="Dynamics">Dynamics</h3>
There isn't any algorithm here to mention, as this is just a call ode45 to propagate true states forward with the thrust and torques. 

<h2 id="simulation-progression">Simulation Progression</h2>
As with all simulations, this didn't come about overnight. As I have learned from building up simulations at work, its always tempting to just jump into simulating the highest fidelity with all the bits and bobs, especially when it is all built out and avaliable to you as an analyst. I've quickly got into the habit of thinking about how to get the same result for the least amount of effort, and slowly adding fidelity only when I am confident in the results, which I applied religiously to this project. I probably have the experience to build this entire 6DOF simulation at once, but it would be riddled with bugs and weird interactions between classes and functions that I could not have foreseen. I do not have a team of people ready to review my PRs, so I had to be slower in the way this was built up. 

While I don't have notes on every single progression, there were three phases I had planned to break this simulation up into from the start:

Order 0: LQR + Dynamics

Order 1: Navigation + Sensors (Still giving truth states to LQR)

Order 2: Closed loop validation (estimated states to LQR) + Motor Delay

<h3 id="order-0-simulation">Order 0 Simulation </h3>
At the minimum, the simulation needs to be able to simulate the LQR controller and dynamical model. This would validate that the chosen controller will work under perfect state knowledge and the current best model of the system. 
While that isn't particularly hard, here is a video from one of my first commits to my repo: (I don't think github supports videos so they are all converted into gifs)

<figure style="text-align: center;">
  <img
    src="simulation_videos/progression/trajectory_video_8_3_25.gif"
    alt="order-0-video2"
    style="width: 100%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 2: Order-0 Simulation Progress 1
  </figcaption>
</figure>

At this point, I was commanding just a reference height, and every other states was 0 (quaternion was identity). So it looks like the controller is mostly working, until something happens at the very top and the quadcopter started to flip over. 

This made me think it could have been a numerical precision issue, where the gains are tuned poorly (identity matrices were used for Q and R), so any attitude error causes the system to become unstable. Following this theory, I added a deadband on the state error, such that if they fell below 1e-6, it would just ignore it. 

While this did resolve this specific case, the error came back worse when I tried commanding a position in any North / East direction so that didn't seem to be the root cause:

<figure style="text-align: center;">
  <img
    src="simulation_videos/progression/trajectory_video_8_5_25.gif"
    alt="order-0-video2"
    style="width: 100%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 3: Over-0 Simulation Progress 2
  </figcaption>
</figure>

This confirmed to me there was a bug in the simulation. Luckily, only two components of this simulation existed so it didn't take long to find the issues (something that would have exponeitally increased the debugging time if I tried to build everything at once). 

Given that I had a script to symbolically generated the LQR matrices, I was able to compare the code index by index to ensure there wasn't a typo. I also verified whether the gain matrices I was producing made sense (i.e. error in this component of the attitude should produce a torque about this axis), along with the A and B matrices. 

From there, I was fairly confident that the error was within the dynamics, where I ended up finding an extra negative sign on the thrust (due to the way my inertial frame is defined, I was double counting the negative sign from another calculation), causing it to be applied in the opposite direction that the quadcopter was tilting in. 


<figure style="text-align: center;">
  <img
    src="simulation_videos/progression/trajectory_video_8_10_25.gif"
    alt="order-0-video3"
    style="width: 100%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 4: Over-0 Simulation Progress 3
  </figcaption>
</figure>

With this part of the simulation working, I felt confident adding on some noise and testing it under uncertainties in the state information. 

<h3 id="order-1-simulation">Order 1 Simulation </h3>
This part of the implementation didn't include videos and were just a series of plots trying to debug my filter so not a lot of visuals were recorded. I decided to add in the sensor models first, as the outputs can be simulated without the filter to verify if they are tracking the truth, before adding in the filter.

From my notes, it seem like adding in the sensor models was straight foward since I was able to simulate my scenario and generate plots comparing the measurements with the truth to confirm.

The main issues came when I coded up the Error State Kalman Filter, which saw the next couple days being a iterative loop of tuning the filter, seeing an undesired response, checking my code, checking my sources, bug fixes, and repeat.

Some issues included a typo within my skew symmetric matrix function, typo when forming the measurement noise matrix for one of the sensors, a slight misunderstanding in the way the magnetometer is modeled (I was trying to account for Magnetic North having a declination angle from the True North, which I was applying at the wrong place in the simulation), and probably much more. 

But at the end of the day, I was able to systematically track down each one and resolve them.

<h3 id="order-2-simulation">Order 2 Simulation </h3>
At this point, I am ready to combine everything from the simulation and pass in the estimated state to the controller. If I had done everything right, any issues should be related to poor tuning of either the controller or filter, which luckily, this was the case. After verification, I also added a slight delay to the motor outputs so that the force / torques aren't instantly aquired, opting for this to be modeled with a first order transfer function that I implemented as a difference equation. [See Thumbnail GIF] 

I also used a reference that ramps up to the final target to avoid a large initial control, which can be seen that this works well. After reaching the final reference, the quadcopter is able to maintain the final position without signs of instability (some very slight wobble coming from the noise). This does not include any cutoff radius around the waypoints, which can also be adding extra jitter. This is usually done due to the noise on GPS measurements causing the position estimate to not always settle out, so achieving a position that is close enough often causes the position controller to turn off until the next waypoint.

<h2 id="utilizing-the-simulation">Utilizing the Simulation</h2>
So far, this simlation was used to validate the algorithms before quickly moving onto the FSW. This includes using the best estimate of the system parameters and tuning the filters accordingly such that closed loop performance is achieved as desired. This section will briefly go over how I did that.

<h3 id="hardware-selection">Hardware Selection</h3>
Taking a step away from the simulation, at a minimum, all the components needed to be selected in order to know what needs to be simulated and how. This included things like the airframe, motors and propellers, ESCs, battery, and sensors / microcontroller. 

At the size and hobby level, most of these items will perform similarly given the price points I considered.

Below are just a couple items I thought were worth mentioning:

<h4 id="airframe">Airframe</h4>
The airframe selected is a 5-inch X shaped frame, which is a moderate size for this project as it isn't large enough that I'd be concern it would do much damage indoors, but also not small enough that it becomes overly reactive. The X shaped frame also means its fully symmetric in terms of moment arms to the pitch and roll axes, and the 5-inch represents diameter of the recommended propellers.

<h4 id="battery">Battery</h4>
The battery selected was a 6S Li-Po composition, the 6S representing 6 4.2V cell connected in series to provide a max NOMINAL voltage of 22.2V (so typically batteries shows the 22.2V rather than the actual 25.2V). The cost of higher number of cells is mostly mass (due to battery itself weighing more and requiring higher rated components), but you end up getting longer flight times and generally less resistive losses (since $P_{loss} = I^2R$, and the current decreases for higher voltage). I ended up opting for a 6S to maximize flight times. 

<h4 id="motors">Motors</h4>
Brushless motors are typically parameterized by the rated KV, which determines how much RPM per volt the motor can output. I ended up with a 1800KV motor given that I selected a higher cell number, I didn't need a higher KV motor. Selecting a KV that's too high for the battery while can allow for higher spin rates, but then it'll request more currents to meet the demanded torque output, causing potential issues for the electronics and also loss of any benefits from the previous point. 

<h4 id="propellers">Propellers</h4>
The propellers were selected to be 5x4x3, so they were 5 inches in diameter, 3 bladed, with a 4 inch pitch. The 5 inches were determined by the airframe selected. The pitch represents how angled the propellers are, which a higher pitch representing a more aggressive drone (at the cost of more load), while a smaller value has better efficency and is smoother. The number of blades has about the same relationship, where more blades represents more thrust while less blades results in less thrust. Again, there isn't too much options here, and a pretty standard configuration is a 5x4x3, so that was selected. 

<h4 id="microcontroller">Microcontroller</h4>
The microcontroller was selected to be the Teensy 4.1. due to its extensive number of I/O pins, high processing speeds and memory, and its processor (ARM Cortex-M) is used in other commerical Flight controllers (Betaflight / PX4). I had considered using the ESP32 due to its Wifi modules for downlinking real time telementry, but I figured I should use a dedicated ratio module in the future instead (the Teensy beats out the ESP32 on the other fronts mentioned).

<h4 id="sensors">Sensors</h4>
All the sensors were breakout boards from Adafruit. The tradeoff here was the extra unused pins and connectors, resulting slightly more mass and more importantly, volume. However, these boards came with libraries that significantly reduced the development and debugging time. I was confident that the sensors will return the expected measurements, and all I needed was the proper interfacing to extract out the measurements, and provide it to my filter. This in itself was extremely valuable as I could proceed with the rest of my FSW development in a modular fashion, and simply replace the components of the sensors with customized software once I get to the point of developing my own PCB. 
Had this been a system with stringent performance requirements, I'd put a lot more thought into the hardware selections. 

<h4 id="battery">Electronic Speed Controller</h4>
The ESCs were from hobby wing, where there was only really one option that would be rated up to the 6S battery. Admittedly, the choice here was mostly due to familiarity with this component in the past, and knowing that it was able to handle much higher loads than this quadcopter gave me peace of mind. It also comes with easy calibration instructions for compatability to the PWM signal the Teesny 4.1 will be sending. 

<h4 id="power-distribution-board">Power Distribution Board</h4>
Lastly, I had to select a power distribution board (PDB) to connect the batteries to the rest of my electronics. Surprisingly, there weren't a lot of options for a cheap and readily avaliable 6S compatable power distribution board online for quadcopters. Many of them were just shy of the rating at 4S. I did end up finding one from Matek systems, which is seemingly discontinued. So if I burn through all my spares, this may eventually force me to go down to a 4S battery. 

<h4 id="flight-computer-board">Flight Computer Board</h4>
Given all the different board selection, after testing on a breadboard, I had to move them all onto a prototyping board that is more compact to fit onto the drone. 

Below is a overview of the layout:

<figure style="text-align: center;">
  <img
    src="images/FlightComputerLayout.png"
    alt="order-0-video3"
    style="width: 100%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 5: Flight Computer Board layout
  </figcaption>
</figure>

I soldered female header pins onto the boards such that only the pins I needed from each board would make contact. This also allowed me to swap out the components in case something went wrong rather than needing to un-solder the entire thing and provided more space route each wire. 

I did not pay any particular focus on keeping the wire lengths about the same as one would do for PCB designs. My main goal was to just get everything fitting onto this board as compactly as possible, which meant there are probably countless best practices from an eletronics standpoint that I ended up ignoring. I hope to research and remedy the majority of them once the PCB is customized, but for now this was good enough to run and test my FSW.


<h3 id="system-parameters">System Parameters</h3>
Going back to the simulation, with the components selected and system built, each of the parameters into the simulation now needs to be defined such that it accurately reflects the expected performances. 

<h4 id="sensor-noise-parameters">Sensor Noise Parameters</h4>
The majority of the sensors are just parameterized by a normally distributed white noise, which can be found on the datasheets. This included the GPS, Altimeter, Magnetometer:

• $\mathbf{\sigma_{GPS} = 3 m}$  <br>
• $\mathbf{\sigma_{Mag} = 0.3 \mu T}$  <br>
• $\mathbf{\sigma_{Alt} = 2 Pa}$  <br>

The white noise for the IMU is also provided, but in terms of a noise spectral density. The $\sigma$ values are dependent upon the sampling frequency, which at 100Hz, comes out to:

• $\mathbf{\sigma_{Acc} = 0.028 \frac{m}{s^2}}$  <br>
• $\mathbf{\sigma_{Gyro} = 0.0031 \frac{rad}{s}}$  <br>

The Rate Random Walk parameters were not provided, but an Allan Variance Curve can be generated from a long period of data collection to obtain this. 

<figure style="text-align: center;">
  <img
    src="images/AllanVariance.png"
    alt="order-0-video3"
    style="width: 80%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 6: Allan Variance Plot
  </figcaption>
</figure>

Above is an example of a generated Allan Variance Plot from logging the stationary IMU for 10 hours. This specific plot is of the X component of the accelerometer. 

As a refresher, this plot has the 3 regions of interest already plotted on here. The Red dotted line represents a slope of -0.5, and is the white noise region. In this region, the line evaluated at $\tau = 1$ is what the white noise parameter would be.

On the opposite end, the Yellow dotted line represents a slope of +0.5, and is the rate-random walk region. Here, the line evaluated at $\tau = 3$ is what the random walk parameter would be. 

This plot also includes an additional bias instability region, which is where the slope is 0. This effectively just means the lowest point of the curve would be taken to be the bias instability parameter (but this isn't modeled).

So for this specific example, the white noise would have a $\sigma_{WN} = 5.5e-3$, while the RRW would have a $\sigma_{RW} = 1.6e-4$

Strangely, the measured white noise is smaller than the spec sheet predicts. While its unclear of the exact environment tested in the spec sheet, there could be a number of factors causing the difference. So to be conservative, the value from the data sheet will be assumed to be what is expected. Additionally, from this, it can also be seen that the RRW value is about 1 order of magnitude smaller than the white noise, which is verified by reviewing other IMU data sheets that does provide this value. Thus, once again, to err on the side of conservatism, it will be assumed the RRW parameter is 1 order of magnitude smaller.



<h4 id="vehicle-parameters">Vehicle Parameters</h4>
While in-accuracies in the sensor noise parameters are limited to the filter (and thus can be counter-acted by on-board tuning), the vehicle parameters themselves are slighly more important in getting a representative simulation and testing out the stability of the controller. 

These included the following:

• Moment of Inertia Matrix ($\mathbf{J}$)<br>
• Moment Arm ($\mathbf{L}$)<br>
• Mass ($\mathbf{m}$)<br>
• Thrust and Torque Constants ($\mathbf{k_T}$ & $\mathbf{k_M}$)<br>


The MOI ($\mathbf{J}$) and moment arm ($\mathbf{L}$) were obtained from the CAD model, while the mass ($\mathbf{m}$) was obtained by simply weighing the system. 

The Thrust Constant ($\mathbf{k_T}$) was obtained by putting on the propellers upside down, and using a hand-held tachometer, measuring the rotation of the motor for a given PWM command, and recording the resulting weight change on a scale. This was then plotted and a line of best fit was obtained, where the linear slope describing the square of the spin-rate versus thrust is the Thrust Constant:

<figure style="text-align: center;">
  <img
    src="images/ThrustConstantPlot.png"
    alt="thrust-constant-plot"
    style="width: 75%; max-width: 750;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 6: Thrust Constant Plot
  </figcaption>
</figure>

The Torque Constant wasn't easily obtainable given the current resources I had, so I opted to obtain this value online. This should be okay as this value only is for the yaw control, which doesn't really matter as much for stability as the thrust constant.

UIUC keeps a data base of the performances of propellers, which I used to find about 3 propellers that were approximately 5 x 4 x 3 (some may have slightly longer pitch), but unfortunately they were all of different airfoil shapes. The database provides the thrust and torque coefficents under static conditions (no free stream velocity), which when combined with the propeller specifications, returned the Thrust / Torque Constants. 

While I just needed the torque constant, I also found that the measured Thrust constant was within the expected range of values, which reassured me that this would give me an appropriate guess at the Torque constant and somewhat validated the value I obtained experimentally.  

The final values are as follows:

• $\mathbf{J} = \begin{bmatrix}0.003 & 2.19e-6 & 0 \\ 2.19e-6 & 0.004 & 2.993e-6 \\ 0 & 2.993e-6 & 0.005  \end{bmatrix} \frac{kg}{m^2}$<br>
• $\mathbf{L} = 0.08 m$<br>
• $\mathbf{m} = 0.83 kg$<br>
• $\mathbf{k_T} = 6.583e-5 \frac{Ns^2}{rev^2}$ <br>
• $\mathbf{k_M} = 7.51e-7 \frac{Nm*s^2}{rev^2}$ <br>


<h2 id="tuning-guidelines">Tuning Guidelines</h2>
With the simulation parameters defined and somewhat within the realms of reality to the actual system, the Controller and Filter can be tuned.

The controller was first tuned according to Bryson's Rule, which is helpful when the states or control vector have various magnitudes and units. It requires identifiying the max allowable state and control error from the references, then using that to scale the diagonals of the Q and R matrices such that all the error coming in are weighted to about the same magnitude:
$$
Q_{ii} = \frac{1}{\Delta x_{i,max}^2}
$$
$$
R_{jj} = \frac{1}{\Delta u_{jj,max}^2}
$$
where the cost function is: 
$$J = \int{(\Delta x^TQ\Delta x + \Delta u^TR\Delta u)}dt$$

While this is good at a starting point and often gives decent performance, the controller was then further tuned to achieve the following requirements for a STEP input in the position: 

• Stable Response in States <br>
• No overshoot in Position <br>
• Settling Time ~10s <br>
• Motor speed under limits (assumed ~70% of max spin rates) <br>

Below is the results with the controller ingesting truth states (no filtering / sensors):
<figure style="text-align: center;">
  <img
    src="images/StepInput.png"
    alt="step-input-plot"
    style="width: 75%; max-width: 750;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 7: Step Response
  </figcaption>
</figure>
While the plot only shows the position states, it can be visually seen that these are stable. By the way the system is set up, this can only be achieved if all the other states are stable in their responses in maintaining 0 or the identity quaternion (so requirement 1 is satisfied). 

Visually, there is no overshoot in the position here, and the system achieves the reference position in about 10 seconds, so requirement 2 and 3 are satisfied.

Lastly, while there is a sharp initial jump in the requested spin rates of the motors, they are still below the set motor limits. 

But despite this, that initial jump isn't desirable. If the commanded reference is big enough, the initial response will surpass the motor limits. This is why the references are often ramped up, such that at a given point in the trajectory, the error to the reference state should be tiny as to not cause this.

Below demonstrates this effect with a simple linear ramp:
<figure style="text-align: center;">
  <img
    src="images/RampInput.png"
    alt="step-input-plot"
    style="width: 75%; max-width: 750;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 8: Ramp Response
  </figcaption>
</figure>

This was able to remedy the initial spike at the cost of no longer meeting the settling time requirement. While this could be tuned such that the controller is slightly more aggressive, the requirements for the step inputs still holds, so no further tuning is required. 

After this, comes the tuning of the filter. There is less of a technique here aside from starting off with a reasonable guess at the initial covariance, and using the datasheet specifications for the process and measurement noises. Note for this filter, the process noise are largely due to the IMU noise since those measurements are used to propagate the state forward, while the biases are dependent upon the RRW values. 

From there, the initial covariance and process noise can be tuned slightly. While you can get some desired improvements from tuning the measurement noise, often times there are more robust ways to do so without deviating from the datasheet values. 

The tuning was also done by running the controller with truth data, while checking the associated filter plots to ensure that the covariance matrix properly bounds the state errors, and that the NIS and NEES values are as expected (should average out to the respective DOF for the measurements / state vector). If this was to be proven more rigiously, I would do a Monte-Carlo, but since there will be on-board tuning anyways, I decided to not spend too much time on that.

After the filter was seemingly performing well enough, I then the controller ingesting the estimated state, fully closing the loop:

<figure style="text-align: center;">
  <img
    src="images/FilterRampInput.png"
    alt="step-input-plot"
    style="width: 75%; max-width: 750;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 9: Ramp Response with Estimated States
  </figcaption>
</figure>

From this, it can be seen that the states are pretty much still stable. 

There is now some error between the reference and true North and East positions caused by estimation error, but that is to be expected. The only way to update those two states comes from the GPS, which has a 1-$\sigma$ of about 3 meters. And normally, once the drone is within some distance of the waypoint, it would no longer have the position error contribute to the control and can just maintain a stable attitude, which should be known to a much more precise degree. 

With this, the simulation component is largely completed as a minimum viable product, and the FSW can now be written and compared.

<h2 id="whats-next">Whats Next?</h2>

There are a couple of things that I would like to implement in my simulation to test out before planning on adding them to the FSW. These includes in no particular order (but are not limited to):

<h3 id="integral-control-to-lqr-lqi">Integral Control to LQR (LQI)</h3>
Depending on the battery voltage, for a given PWM, the overall average voltage would vary and can cause lower thrust than expected. The thrust constant was measured by recording the thrust outputs when the battery was close to full charge. For longer duration flights, eventually the battery will drain and the drone will not be able to maintain hover, as it will think the same PWM it provided before will be enough to provide the same thrust. LQR / PD control can mitigate this to some degree, but will result in some steady state error. 

The other obvious solution for this would be to ensure the battery is fully charge before a flight, and with close monitoring of the battery health, factor in some dependencies into the controller (either sliding mode with battery voltage, or even simply changing the control allocation as a function of total battery voltage). 

This simulation would also provide a way to test out different wind-up logic, and protections needed to ensure the integral component will not grow out of control. 

<h3 id="optimal-trajectories">Optimal Trajectories</h3>
As previously mentioned, I would like to use this platform to learn more about optimal control, and simulate a couple that I will eventually try to implement onboard. 

This may be challenging given the limitations on the accuracy of the sensors, and flying outside (for GPS measurements) will result in extra disturbances. Many research demonstrations accomplishes this indoors, and often take advantage of optical sensors to provide more stable and accurate navigation. If this becomes the limiting factor, I would have liked to at least been able to implement it in simulation before moving on to the next thing.


<h3 id="other-uav-platforms">Other UAV Platforms</h3>
While my main goal for this project was to learn, I already came into this project already with knowledge of how quadcopter dynamics and controls works. Eventually, I would like to work on a fixed-wing aircraft project that ultlizes control surfaces to do attitude commands, which I have no experience with. 

The reason why I didn't start with this was mostly due to not being able to precisely measure any aerodynamic coefficents. I was concerned that while I could build to simulation to test out algorithms, I had no confidence the system performance will be remotely close. And since I didn't have any experience with this platform, I didn't want to take on too much and not finish in a timely manner (would need to look into airfoil designs for example). 

Luckily, a good chunk of this simulation would be transferrable, with the exception of needing a more robust guidance algorithm (typically something like L1 guidance is popular for UAVs). And if any part isn't, the simulation is modular enough that I could just swap it out. 