# Quadcopter from Scratch: 6DOF MATLAB Simulation

**Part 1 of building a quadcopter from "scratch"**

---

## Summary

This is <strong>Part 1</strong> of my quadcopter project. This part will go over all the theory and derivation of the Control, Estimation, and Dynamics of the system. <br>
This will culminate into a full 6DOF simulation in MATLAB completed with: <br>
<p>• Sensor modeling w/ noise </p>
<p>• Motor modeling w/ time delay</p>
<p>• LQR Controller for setpoint regulation (waypoint)</p>
<p>• 18-State Error-State Kalman Filter w/ Bias States </p>
<p>• Closed-loop propagation through non-linear dynamics for end-to-end GNC validation</p>

<strong>Introduction: </strong> <br>
Since this is the first part of hopefully many, the introduction to this project will be place here. I started this project to learn more about flight software and embedded hardware (specifcally converting simulation to real-time systems), as well as applying what I have learned from my first full-time job as a way to gauge my knowledge. Luckily, this project formed a symbiotic relationship with my current role working closely with one of the Aerocubes program, where I got the opportunities to deepen my understanding of the GNC concepts from my coursework through real world applications, which have now been reinforced and used here. <br>

Seeing as drones and satellites have nothing in common aside from the control and estimation theories, I should be safe from unknowingly using any sensitive knowledge. 

<strong>Disclaimer: </strong> <br>
The L1 goal for this project is to have a fully customized GNC system flying on a quadcopter. This means that I am not trying to build the best system, nor do anything particularly novel (yet). The focus for me is to learn more about GNC through a platform that is controllable and can be done at home after work (so no model rocket). I also make choices that may not be optimal to save on time, as even a couple hours of delay can result in several buisness days until I have the time to work on this again. <br>

I also put the from "scratch" in quotation marks since having a full time job means it isn't feasible to learn every single discipline needed to fully do this all from zero. This means I use breakout boards for sensors with readily implemented libraries and prototyping boards to contain all my electronics (rather than commerically avaliable flight computer boards) to delay having to learn low level programming and PCB design (both of which I hope to do in the future). <br>

I also avoided looking at any open sourced software like Betaflight and Ardupilot as to not be influenced by choices they make and wasting time learning their codebases.

---

## Table of Content
<span style="font-size:1.2em;">• <a href="#dynamic-model">Dynamic Model</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#inertial-body-frame">Inertial & Body Frames</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#force-torque-generation">Force and Torque Generation</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#aerodynamic-relationships">Aerodynamic Relationships</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#motor-conventions">Motor Conventions</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#control-allocation">Control Allocation</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#newton-quaterion-euler">Newton's Second Law, Quaternion Kinematics, and Euler's Rotation Equations</a></span><br>

<span style="font-size:1.2em;">• <a href="#lqr-controller">LQR Controller</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#regulation-with-quaternions">Regulation with Quaternions</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#attitude-error-jacobian">Attitude Error / Jacobian</a></span><br>

<span style="font-size:1.2em;">• <a href="#sensors">Sensors</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#imu-model">IMU Model</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#mag-model">Magnetometer Model</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#gps-model">GPS Model</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#lla-conversions">LLA Conversion</a></span><br>
    <span style="font-size:0.9em;">&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#ned-frame-conversions">NED Frame Conversions</a></span><br>
  <span style="font-size:1em;">&nbsp;&nbsp;&nbsp;&nbsp;↳ <a href="#alt-model">Altimeter Model</a></span><br>

<span style="font-size:1.2em;">• <a href="#error-state-kalman-filter">Error State Kalman Filter</a></span><br>

<span style="font-size:1.2em;">• <a href="#simulation-results">Simulation Results</a></span><br>

<span style="font-size:1.2em;">• <a href="#conclusion">Conclusion</a></span><br>



<h2 id="dynamic-model">Dynamic Model</h2>

As with any control and estimation problem, the starting point must be a model describing how the states of the system will evolve over time due to any imparted forces / torques: <br>
$$
\dot{x} = f(x,u)
$$
Where $x$ represents the state vector, and $u$ represents the control. <br>
The state will be the translational position ($p$) and velocity ($v$), along with the quaternion ($q$) and body rates ($\omega$). <br>
$$ x = \begin{bmatrix} p\\\ v \\\ q \\\ \omega\end{bmatrix} $$
The control will the magnitude of the total thrust ($F_T$), along with the torques along the 3 body axis ($^B \tau_x, ^B\tau_y, ^B\tau_z$). More on this choice of control later. <br>
$$ u = \begin{bmatrix} F_T\\\ ^B\tau_x \\\ ^B\tau_y \\\ ^B\tau_z\end{bmatrix} $$


<h3 id="inertial-body-frame">Inertial & Body Frames</h3>
Before getting any further, the inertial frame must be defined. For this project, it will follow the North East Down [$\hat{N}, \hat{E}, \hat{D}$] convention for its orthogonal axis. This means the identity quaternion represents the attitude when $\hat{B}_x$ aligns with $\hat{N}$, $\hat{B}_y$ aligns with $\hat{E}$,  and $\hat{B}_z$ aligns with $\hat{D}$.
<figure style="text-align: center;">
  <img
    src="images/DroneInertialFrame.png"
    alt="Drone inertial frame"
    style="width: 60%; max-width: 500px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 1: Drone inertial reference frame
  </figcaption>
</figure>

This results in some conventions like gravity being positive (along the +Z / Down axis), and needing to commanding a negative height in order to go "Up". While this doesn't impact the complexity of the math, it needs to be noted. If there seems like theres an extra negative sign or rotation seems off, its likely due to this convention. 

<h3 id="force-torque-generation">Force and Torque Generation</h3>

<h4 id="aerodynamic-relationships">Aerodynamic Relationships</h4>
Generally, thrust generated by each motor / propeller pair is described by the following relationship <sup id="cite-propdb-ref"><a href="https://m-selig.ae.illinois.edu/props/propDB.html" target="_blank">[1]</a></sup>:
$$ F_{T,i} = C_T \rho D^4 n_i^2 $$
where $C_T$ is the thrust coefficient, $\rho$ is the air density, $D$ is the diameter of the propeller, and $n_i$ is the spin rate in revolutions per second.

Additionally, each time the propeller accelerates, a torque must be applied to increase its angular momentum. By the conservation of angular momentum, the total angular momentum of the system must remain constant if there are no external torques acting on the system. This means that if the motor spins the propeller in one direction, the aircraft body will experience an equal and opposite torque to counteract it. This counteracting torque can be expressed as:
$$ \tau_i = C_\tau \rho D^5 n_i^2 $$
where $C_\tau$ is the torque coefficient. <br>

Higher fidelity simulations may model $C_T$ and $C_\tau$ as a function of the advance ratio and reynolds number, while $\rho$ as a function of the pressure ($P$) and temperature ($T$). However, without access to a wind tunnel, treating these values as constants works pretty well. This allows for the simplified relationships:
$$ F_{T,i} = k_T n_i^2 $$
$$ \tau_i = k_M n_i^2 $$
where $k_T$ is the thrust constant, and $k_M$ is the torque / moment constant.

<h4 id="motor-conventions">Motor Conventions</h4>

With these relationships defined, it is clear why quadcopters spin their motors in very specific pairings which will be explained below. Firstly, for this project, the motor conventions are as follows:
<figure style="text-align: center;">
  <img
    src="images/MotorsSpinDirection.png"
    alt="Motor Spin Directions"
    style="width: 60%; max-width: 500px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 2: Motors Spin Directions (CW / CCW w.r.t. $\hat{B}_z$)
  </figcaption>
</figure>

<strong> Thrust / Yaw Torque</strong> <br>
If we want to increase or decrease the total vertical thrust, the four motors must spin together to sum up to the desired thrust ($F_T$). 

However, this doesn't necesarily constrain the spin direction of each motor nor the spin rates, which only comes about by considering torques. Starting with the "yaw", the axis of rotation for each motor is aligned along the $\hat{B}_z$ direction, meaning each revolution of the motors will sum up to a torque about that direction ($^B\tau_z$). 

From observation, in order to have independent control over the $F_T$, you must spin all four motors at the same rate and have equal numbers of CW and CCW motors such that  $^B\tau_z = 0$. This allows for control over $^B\tau_z$ by adjusting the speeds of the CW and CCW motor pairs, while maintaining the desired $F_T$.

<strong> Roll / Pitch Torque </strong> <br>

Now that we know how many motors must spin in each direction, the next step is figuring out the orientation of them with respect to each other. The answer here is that adjacent motors spin in opposite directions, while diagonal motors spins in the same direction as shown by Figure 2. 

In this configuration, if you wanted to roll about the $\hat{B}_x$ axis, you can increase the spin rates of the motor pair M1 and M4, while decreasing the spin rates of the motor pair M2 and M3 to maintain the desired thrust level.

The counteracting torques between M1/M4, aand M2/M3 will cancel each other out if they spin at the same rates, resulting in $^B\tau_z = 0$. And due to these pairs having the same moment arm from the $\hat{B}_y$ axis, the torques induced by the moment arms will also cancel out such that $^B\tau_y = 0$, resulting in independent control of $^B\tau_x$

The same logic applies for pitching about the $\hat{B}_y$ axis, except the motor pairs are now M1/M2, and M3/M4. 

<h4 id="control-allocation">Control Allocation</h4>

This results in the following mapping between our chosen control vector $u$ and the spin rates of the motors:

$$
\underbrace{
\begin{bmatrix}
F_T\\
^B\tau_x \\
^B\tau_y \\
^B\tau_z
\end{bmatrix}
}_{u}
=
\underbrace{
\begin{bmatrix}
k_T & k_T & k_T & k_T \\
Lk_T & -Lk_T & -Lk_T & Lk_T \\
Lk_T & Lk_T & -Lk_T & -Lk_T \\
-k_M & k_M & -k_M & k_M
\end{bmatrix}
}_{M}
\underbrace{
\begin{bmatrix}
n_1^2\\
n_2^2 \\
n_3^2 \\
n_4^2
\end{bmatrix}
}_{n^2} 
$$

where $M$ is the control allocation matrix. 

From here, the choice of control is hopefully clear, as $M$ allows for the mapping of the requested control to the desired spin rate of all four motors via a matrix inversion. 

<h3 id="newton-quaterion-euler">Newton's Second Law, Quaternion Kinematics, and Euler's Rotation Equations</h3>

Applying the three equations gives us the full dynamic model that will be used. 

$$
\dot{x} = f(x,u) = 
\begin{bmatrix}
v \\
R(q)^T\frac{^B\vec{F_T}}{m} u+ \vec{g}\\
\frac{1}{2} \begin{bmatrix} -\omega\cdot\vec{q} \\ q_w\omega - \omega \times\vec{q} \end{bmatrix} \\
J^{-1}*(-\omega \times J\omega + ^B\vec{\tau})
\end{bmatrix}
$$

where $^B\vec{F_T}$ represents the total thrust in the body frame ($^B[0; 0; F_T]$), R(q) is the rotation from the inertial frame to the body frame, $\vec{g}$ represents the gravity vector in the inertial frame ($^I[0; 0; g]$), $J$ is the moment of inertia matrix, and $^B\vec{\tau}$ represents the total torques around the body axis ($^B[\tau_x; \tau_y; \tau_z]$). 

The only force modeled will be from gravity and the thrust of the motors (ignoring drag). However, because the thrust is constrained along the $\hat{B}_z$ axis, it must be rotated into the inertial frame in order to properly apply Newton's Second Law. 

<h2 id="lqr-controller">LQR Controller</h2>

With the non-linear model, normally you can just taylor series expand about a equilibrium point to find your system matrices ($A$ & $B$). A typical point to linearize quadcopters around is the "hover" condition ($\bar{x}$ & $\bar{u}$), which means the states should be closed to the identity quaternion with zero body rates (position and velocity doesn't factor into the linearization), along with enough thrust to counteract gravity:
$$
A = I + \frac{\partial f(x,u)}{\partial x}\Big|_{\bar{x}, \bar{u}} \Delta t
$$
$$
B =\frac{\partial f(x,u)}{\partial u}\Big|_{\bar{x}, \bar{u}} \Delta t
$$
$$
\bar{x} = \begin{bmatrix}0; & 0; & 0; & 0; & 0; & 0; & 1; & 0; & 0; & 0; & 0; & 0; & 0 \end{bmatrix}
$$
$$
\bar{u} = \begin{bmatrix}mg; & 0; & 0; & 0\end{bmatrix}
$$

<h3 id="regulation-with-quaternions">Regulation with Quaternions</h3>

While this may seem straight forward, doing the steps above with the quaternion as part of the state vector will result in the system not having full controllability. This can be observed by forming the controllability matrix $C$: 
$$
C = \begin{bmatrix}B & AB & ... A^{n_x-1}B \end{bmatrix}
$$
where $n_x$ is the length of the state vector (13). 

The system is controllable if $C$ is full rank, which means $rank(C) = n_x = 13$, which is not the case. Instead, the rank of $C$ is 12 when linearized about a set point, meaning there is only 12 DOF for this system. The missing DOF comes from the unit-norm constraint on the quaternion. If this were a true regulation problem, you would want to drive all state to 0, which obviously cannot happen with the quaternion as it must maintain a magnitude of 1. 

But if we just operate about the setpoint where the quaternion towards the identity quaternion, the scalar component would effectively be 1 and we can just regulate the vector part. Doing so would effectively shrink the DOF if this system by 1 and give us control over the remaining states, the only thing left is to mathemtically formalize this. 

The steps for doing will be shown here at a high level, but is explained in detail by this lecture from Prof. Manchester at CMU <sup id="cite-quatOpt-ref"><a href="https://www.youtube.com/watch?v=gSeRXxaC5CM" target="_blank">[2]</a></sup>:

<h3 id="attitude-error-jacobian">Attitude Error / Jacobian</h3>

Instead of using the full quaternion, the controller will be regulating a small attitude error $\alpha$. There are multiple representations that can be used, but the one chosen will be the vector component of the quaternion error ($\delta q$)
which is defined as follows:
$$
$q_{ref} = \delta q \otimes q \rightarrow \delta q = q_{ref} \otimes q$
$$
$$
\alpha = \delta q_{xyz}
$$

Where $\delta q$ represents a tiny error in rotation between our current attitude and the desired one, and $alpha$ is simply the vector component of this. The operator $\otimes$ is the quaternion multiplication (Kronecker Product).

Using this definition, an attitude jacobian ($G(q)$) can be aquired that maps a small change in $\alpha$ to $\delta q$,linearized about the current attitude $q$. 
$$
G(q) = \begin{bmatrix} -q_x & -q_y & -q_z \\ q_w & q_z & -q_y \\ -q_z & q_w & q_z \\ q_y & -q_x & q_w \end{bmatrix}
$$  

With this defined, the system matrices can be shrunk to achieve the desired impacts mentioned above:
$$
E(q) = 
\begin{bmatrix} 
I_{3x3} & 0_{3x3}   & 0_{3x3}   & 0_{3x3}  \\
0_{3x3} & I_{3x3}   & 0_{3x3}   & 0_{3x3}  \\
0_{4x3} & 0_{4x3}   & G(q)      & 0_{4x3}  \\
0_{3x3} & 0_{3x3}   & 0_{3x3}   & I_{3x3}  \\
\end{bmatrix}
\rightarrow
\begin{align}
\tilde{A} &= E^T A E\\
\tilde{B} &= E^T B
\end{align}
$$
After linearizing the new system matrices ($\tilde{A}$ & $\tilde{B}$) about hover conditions, the rest of this controller follows the normal LQR steps (select gains $Q$, $R$, solve the Algebraic Riccati equation, obtain an optimal gain $K$). 

As a further note, the controllability test can be performed once again to verify that:
$$
\tilde{C} = \begin{bmatrix}\tilde{B} & \tilde{A}\tilde{B} & ... \tilde{A}^{\tilde{n}_x-1}\tilde{B} \end{bmatrix}
$$
Results in a system that is fully controllable, that is, $rank(\tilde{C}) = \tilde{n}_x = 12$.


<h2 id="sensors">Sensors</h2>
The sensor suite will consist of an IMU for high frequency measurements of the acceleration and body rates for propagation, fused with GPS, Altimeter, and Magnetometer data to provide updates for the position, velocity, and quaternion. 

<h3 id="imu-model">IMU Model</h3>

While there are many sources for the IMU, the main ones will be white noise and a random walk term to simulate bias growth (so ignoring any scale factor, non-orthogonality, $g / g^2$ sensitivity etc), resulting in the following models for the Accelerometer:
$$
^A\tilde{a}(t) = ^Aa_{A/I}(t) + ^Ab_a(t) - R_{I\rightarrow A}(t) ^I g + ^A\eta_a
$$
where $^A\tilde{a}(t)$ is the measured acceleration of the IMU, $^Aa_{A/I}(t)$ is the true net acceleration of the IMU, $^Ab_a(t)$ is the time varying bias of the accelerometer, $R_{I\rightarrow A}(t) ^I g$ is the rotated inertial gravity, and $^A\eta_a$ is the white noise, all expressed in the accelerometer frame.

And Gyro:
$$
^G\tilde{\omega}(t) = ^G\omega_{G/I}(t) + ^Gb_g(t) + ^G\eta_g 
$$
where $^G\tilde{\omega}(t)$ is the measured rotational rate of the IMU,$^G\omega_{G/I}(t)$ is the true rotational rate of the IMU with respect to the inertial frame, $^Gb_g(t)$ is the time varying bias of the gyro (accelerometer measures with respect to free fall), and $^G\eta_g$ is the white noise, all expressed in the gyro frame.

For all intents and purposes, the Accelerometer and Gyro share the same frame, which is the IMU frame. 
The random walk term will be grouped into the bias model, rather than treated as being grouped with the noise.
While the white noise parameter can readily be found on the datasheets, the RRW parameter may not always show up on low cost hobbiest sensors. In this case, a RRW parameter was taken from a similar IMU with some added conservatism and verified with the sensor itself via an Allan Variance curve.  

<h3 id="mag-model">Magnetometer Model</h3>

A magnetometer measures the local magnetic field vector. If you point any of the axis directly along the magnetic field vector, only that axis would return a value, while the other two should be close to zero (with some noise). This results in the following model:
$$
^M\tilde{m} = R_{M\rightarrow I}(t) ^Im_{ref} + ^M\eta_m 
$$
where $^M\tilde{m}$ is the measured magnetic field vector, $R_{M\rightarrow I}(t) ^Im_{ref}$ is the rotated true magnetic field vector, and $^M\eta_m$ is some white noise term, all expressed in the Magnetometer frame.
The white noise term can be found on the magnetometer data sheet.

<h3 id="gps-model">GPS Model</h3>

While a typical GPS model for simulation could be the inertial position and velocity with some added noise, the actual GPS returns a "Sentence", which just is a string of characters containing the information. This typically comes in the form of Latitude, Longitude, Altitude, Speed over Ground (SOG), and Course over Ground (COG), which then must be converted into the associated states. Since this conversion is non-linear, the noise added at the state level vs the measurement level results in different effects. Since the noise terms are typically given on the state level (position / velocity $1-\sigma$), the steps for converting from the 5 measured quantities to the inertial position / velocities will be provided here. The sensor model would then be adding noise to the true states, and then inversing the steps to generate measurements (for the simulation, this is trivial since we could've just taken the noisy state, but the conversion was added to ensure it will be correct for FSW).

<h4 id="lla-conversions">LLA Conversion</h4>
Using the WSG84 Ellipsoid Model, which is what the Latitude, Longitude, and Altitude are in reference to, we can use the following conversion to obtain the conversion from LLA directly into a position vector in the Earth-Centered Earth-Fixed (ECEF) frame <sup id="cite-savransky-ref"><a href="https://sioslab.com/wp-content/uploads/2021/09/MAE4060_Handouts.pdf#page=19" target="_blank">[3]</a></sup>:
$$
\begin{bmatrix}x \\ y\\ z \end{bmatrix}_{ECEF}
=
\begin{bmatrix}
(a/(\sqrt{1-e^2sin^2(\phi)}) + h)cos(\phi)cos(\theta) \\
(a/(\sqrt{1-e^2sin^2(\phi)}) + h)cos(\phi)sin(\theta) \\
(a(1-e^2)/(\sqrt{1-e^2sin^2(\phi)}) + h)sin(\phi)
\end{bmatrix}
$$

where $a$ and $e$ are the semi-major axis and eccentricity of the earth under the WSG84 model, and $\phi$, $\theta$, $h$ are the Latitude, Longitude, and altitude respectively.  

<h4 id="ned-frame-conversions">NED Frame Conversions</h4>
Note that before this can be converted into the inertial frame of choice, there must be a reference vector that initializes the origin of the inertial frame on the ECEF frame, such that the deltas between the measured ECEF, and the reference ECEF vector is the position and velocity relative to the origin of the NED frame. 

$$
^{ECEF} r_{NED} = ^{ECEF} r_{ref} - ^{ECEF} r_{meas}
$$

From there, the rotation that describes the orientation of the ECEF frame to the local NED frame is required, which is dependent upon the reference LLA. This results in:
$$
^{NED} r_{NED}= R_{ECEF\rightarrow NED}^{ECEF} r_{NED}
$$
where $R_{ECEF\rightarrow NED}$ describes this rotation and is:
$$
R_{ECEF\rightarrow NED} = R_2(-\phi_{ref} - \frac{\pi}{2}) R_3(\theta_{ref}) 
$$
This reads as starting with the ECEF, rotate the coordinate around the Z axis by the reference longitude. This aligns the Y axis with the East Direction. Another rotation about the X axis by the quantity of the reference latitude minus 90 degrees will align the Z with the down axis, and X with the North. 

These conversions can be summarized here <sup id="cite-wiki-tangentplanes-ref"><a href="https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates" target="_blank">[4]</a></sup>. 

The datasheet typically gives the noise of the position in terms of a 50% CEP, which can be roughly translated into a $1-\sigma$ value via $\sigma = CEP_{50}/1.18$. The velocity 1-sigma is guess-stimated to be ~1 order of magnitude smaller since its not always provided. 

<h3 id="alt-model">Altimeter Model</h3>
The altimeter works by measuring pressure, and comparing it to the local pressure in order to a height. Since there are no plans to fly this to extreme altitudes, the following exponential model is sufficent:

$$
P = P_0 exp(\frac{-gMh}{RT_0}) + \eta_p
$$
where $P_0, T_0$ are the reference pressure and temperature, while $g, M, R$ are constants and are the gravity, mean molecular weight of air, and the ideal gas constant. $\eta_p$ represents the white noise added onto the pressure model.

The height can computed from a pressure reading, which will be used to compute the "Down" component of the inertial position:
$$
^{NED} z = h_0 - h
$$

where $^{NED} z$ is the z component of the position in the NED frame, and $h_0$ is the reference height at the start. 
Note that the sign here is reversed. This is because as the height coming out of the altimeter increases, $^{NED} z$ becomes more negative since it represents the DOWN direction. 

The white noise 1-sigma is readily avliable in data sheets.


<h2 id="error-state-kalman-filter">Error State Kalman Filter</h2>
<h2 id="simulation-results">Simulation Results</h2>
<h2 id="conclusion">Conclusion</h2>
<hr>

<section id="references" class="references">
  <h2>References</h2>
  <ol>

<li id="cite-propdb">
  Brandt, J. B., Deters, R. W., Ananda, G. K., Dantsker, O. D., & Selig, M. S., 
  <i>UIUC Propeller Database</i>, UIUC Department of Aerospace Engineering
  <span style="margin-left:0.5em; font-size:0.8em;">
    <a href="#cite-propdb-ref" title="Back to main text">↑</a>
  </span><br>
  <a href="https://m-selig.ae.illinois.edu/props/propDB.html" target="_blank">
    https://m-selig.ae.illinois.edu/props/propDB.html
  </a>
</li>

<li id="cite-quatOpt">
  Z. Manchester, 
  <i>“Optimal Control and Reinforcement Learning Lecture 14”</i>, CMU Robotic Exploration Lab
  <span style="margin-left:0.5em; font-size:0.8em;">
    <a href="#cite-quatOpt-ref" title="Back to main text">↑</a>
  </span><br>
  <a href="https://www.youtube.com/watch?v=gSeRXxaC5CM" target="_blank">
    https://www.youtube.com/watch?v=gSeRXxaC5CM
  </a>
</li>

<li id="cite-savransky">
  D. Savransky, 
  <i>“Spaceflight Mechanics Lecture Notes”</i>, Cornell University Space Imaging and Optical Systems Lab
  <span style="margin-left:0.5em; font-size:0.8em;">
    <a href="#cite-savransky-ref" title="Back to main text">↑</a>
  </span><br>
  <a href="https://sioslab.com/wp-content/uploads/2021/09/MAE4060_Handouts.pdf#page=19" target="_blank">
    https://sioslab.com/wp-content/uploads/2021/09/MAE4060_Handouts.pdf#page=19
  </a>
</li>

<li id="cite-wiki-tangentplanes">
  Wikipedia, <i>“Local tangent plane coordinates”</i>.
  <span style="margin-left:0.5em; font-size:0.8em;">
    <a href="#cite-wiki-tangentplanes-ref" title="Back to main text">↑</a>
  </span><br>
  <a href="https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates#Local_north,_east,_down_(NED)_coordinates" target="_blank">
    https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates
  </a>
</li>

  </ol>
</section>

