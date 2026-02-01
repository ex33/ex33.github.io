# Quadcopter from Scratch: Flight Software
**Part 2 of building a quadcopter from "scratch"** <br>
<a href="https://github.com/ex33/drone_sim" class="tag tag-big">GITHUB</a>
---
<div style="display: flex; justify-content: center; margin: 1.5rem 0;">
  <img
    src="simulation_videos/progression/trajectory_video_full_sim.gif"
    alt="order-0-video3"
    style="max-width: 100%; height: auto;"
  />
</div>


## Summary
This is <strong>Part 1</strong> of my quadcopter project. This part will go over my 6DOF simulation in MATLAB completed with: <br>
<p>• Sensor modeling w/ noise </p>
<p>• Motor modeling w/ time delay</p>
<p>• LQR Controller for setpoint regulation (waypoint)</p>
<p>• 18-State Error-State Kalman Filter w/ Bias States </p>
<p>• Closed-loop propagation through non-linear dynamics for end-to-end GNC validation</p>

For more background material, see [Part 0](quadcopterPart0.html). This section contains little to no context on some of the things that goes on, so it somewhat requires taking a quick look at the previous section. I will try to come back here and make this more stand-alone at some point.

