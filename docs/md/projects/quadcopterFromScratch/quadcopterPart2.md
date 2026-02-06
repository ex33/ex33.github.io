# Quadcopter from Scratch: Flight Software
**Part 2 of building a quadcopter from "scratch"** <br>
<a href="https://github.com/ex33/drone_sim" class="tag tag-big">GITHUB</a>
---

<figure style="text-align: center;">
  <img
    src="images/FlightComputer.png"
    alt="order-0-video2"
    style="width: 50%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.8em">
    Flight Computer Board
  </figcaption>
</figure>


## Summary
<div class="latex-algorithm">
  <div class="alg-title">Flight Software Pseudo Code</div>

  <div class="alg-body">
    <div class="alg-line">
      Initialize all instances of classes, and global variables.
    </div>

    <div class="alg-line">
      <span class="kw">void setup()</span>
    </div>

    <div class="alg-line indent">1) Check for SD card</div>
    <div class="alg-line indent">2) Initialize I2C bus for sensors</div>
    <div class="alg-line indent">3) Check and start up all sensors</div>
    <div class="alg-line indent">4) Run any static calibrations for sensors (start-up biases and references)</div>
    <div class="alg-line indent">5) Begin Logger class</div>
    <div class="alg-line indent">6) Transition state to Idle</div>

    <div class="alg-line"><span class="kw">void loop()</span></div>

    <div class="alg-line indent">7) Get current Loop start time</div>
    <div class="alg-line indent">8) Check Sensors for measurements within respective loops</div>
    <div class="alg-line indent">&nbsp;&nbsp;- If IMU measurement, predict filter state and covariance to current time</div>
    <div class="alg-line indent">&nbsp;&nbsp;- If all other measurement, update filter error state</div>
    <div class="alg-line indent">&nbsp;&nbsp;- Log measurements to logger</div>
    <div class="alg-line indent">9) If any measurements processed for update, inject filter error state and reset</div>
    <div class="alg-line indent">10) Run the FSM and update state if necessary</div>
    <div class="alg-line indent">11) Run the controller and calculate the control effort</div>
    <div class="alg-line indent">12) Run the motor and command the corresponding PWM</div>
    <div class="alg-line indent">13) Log GNC to logger</div>
    <div class="alg-line indent">14) Check Logger for frequency to flush files</div>
  </div>
</div>
