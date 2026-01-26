# Quadcopter from Scratch: 6DOF MATLAB Simulation

**Part 1 of building a quadcopter from "scratch"**

---

## Summary

This is <strong>Part 1</strong> of my quadcopter project. This part will go over all the theory and derivation of the Control, Estimation, and Dynamics of the system. <br>
This will culminate into a full 6DOF simulation in MATLAB completed with: <br>
<p>• Sensor modeling w/ noise </p>
<p>• Motor modeling w/ time delay</p>
<p>• LQR Controller for setpoint regulation (waypoint)</p>
<p>• 19-State Error-State Kalman Filter</p>
<p class="sub-bullet">– Position (3)</p>
<p class="sub-bullet">– Velocity (3)</p>
<p class="sub-bullet">– Attitude (Quaternion, 4)</p>
<p class="sub-bullet">– Body Rates (3)</p>
<p class="sub-bullet">– Accelerometer Bias (3)</p>
<p class="sub-bullet">– Gyroscope Bias (3)</p>
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




## Dynamic Model

As with any control and estimation problem, the starting point must be a model describing how the states of the system will evolve over time due to any imparted forces / torques: <br>
$$
\dot{x} = f(x,u)
$$
Where $x$ represents the state vector, and $u$ represents the control. <br>
The state will be the translational position ($p$) and velocity ($v$), along with the quaternion ($q$) and body rates ($\omega$). <br>
$$ x = \begin{bmatrix} p\\\ v \\\ q \\\ \omega\end{bmatrix} $$
The control will the magnitude of the total thrust ($F_T$), along with the torques along the 3 body axis ($^B \tau_x, ^B\tau_y, ^B\tau_z$). More on this choice of control later. <br>
$$ u = \begin{bmatrix} F_T\\\ ^B\tau_x \\\ ^B\tau_y \\\ ^B\tau_z\end{bmatrix} $$
Before getting any further, the inertial frame must be defined. For this project, it will follow the North East Down (NED) convention for its orthogonal axis. This results in some conventions like gravity being positive (along the +Z / Down axis), and needing to commanding a negative height in order to go "Up". While this doesn't make any of the math more complicated, it needs to be noted in the case a extra negative sign or rotation seems off, its due to this convention. Naturally, this means the identity quaternion that describes the body axis being aligned with the inertial frame will have $\hat{b}$

$$ x = \begin{bmatrix} x \\\ x \end{bmatrix} $$
[INSERT PICTURE HERE]


## Operational Modes

AgentBreeder operates in three distinct modes, each serving different research and deployment needs:

### 🔵 BlueAgentBreeder (Defense)
- **Objective**: Maximize both safety and capability
- **Use case**: Develop robust, safe multi-agent systems
- **Result**: 79.4% average safety improvement while maintaining capability

BlueAgentBreeder focuses on creating scaffolds that enhance both the safety and performance of multi-agent systems. This mode is ideal for production deployments where safety is paramount.

### 🔴 RedAgentBreeder (Attack)
- **Objective**: Maximize capability while minimizing safety
- **Use case**: Red-team testing and vulnerability discovery
- **Result**: Reveals how scaffolding can inadvertently expose safety weaknesses

RedAgentBreeder serves as an adversarial testing tool, helping researchers understand potential vulnerabilities in multi-agent scaffolds. This mode is crucial for identifying safety risks before deployment.

### 🎯 CapableAgentBreeder (Capability)
- **Objective**: Maximize capability only
- **Use case**: Baseline comparison and pure performance optimization
- **Result**: Competitive performance with existing approaches

CapableAgentBreeder provides a baseline for comparison by focusing solely on performance optimization without safety constraints.

---

## Citation

