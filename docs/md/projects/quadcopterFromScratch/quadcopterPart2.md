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
This is <strong>Part 2</strong> of my quadcopter project. This part will go over the transition from my 6DOF MATLAB simulation in the previous part onto hardware. 

I will go over the sensor boards selected, major modules on the FSW, software system architecture decisions, some physical integration discussions, testing and verification, and any other challenges that arised.




<h2 id="Overview">Overview</h2>
My FSW is written in C++ using the Platform.io extension on VSCode. 

Platform.io is a development ecosystem for embedded systems, which handles the compiler, dependencies, library managers, and provides a testing / debugging feature. Since my board was a Teensy 4.1., either the Arduino IDE or Platform.io is supported, but I was trying to ensure I had access to as many tools as I could to make this transition seamless. Platform.io also promotes a more standard codebase structure, with your includes, source files, libraries, testing, etc, while Ardiuno typically is for monolithic code that doesn't scale super well.

However, I am still using the Ardiuno framework inside of Platform.io. The majority of my modules do not rely on this, as its mostly for interfacing with the pre-existing libraries for my sensors. In the future if I choose to switch, I should be able to transition with minimal changes to the majority of my code.

While there will be a section talking about testing and verification, I ended up testing at each and every single phase of this. Typically, this included short tests like spinning up a motor, or rotating my sensor and observing the outputs. 

<h2 id="sensor-breakout-boards">Sensor Breakout Boards</h2>
All my sensors are from Adafruit, which provided the sensors that I needed in convenient breakout boards for me to plug straight into my breadboard for testing. It also was extremely attractive that Adafruit has accompanying libraries for their sensors that are easily implementable to get outputs as soon as the board is plugged in. 

My sensor suite includes the following:

• IMU: ICM20649 <br>
• Magnetometer: LIS2MDL <br>
• Barometric Altimeter: BMP390 <br>
• GPS: MTK3333 <br>

<figure style="text-align: center;">
  <img
    src="images/SensorBreakoutBoards.png"
    alt="order-0-video3"
    style="width: 100%; max-width: 1000px;"
  >
  <figcaption style="font-size: 0.9em">
    Figure 1: Sensor Breakout Boards Side-by-side
  </figcaption>
</figure>


There isn't much to talk about on the selection, as the GPS, Barometric Altimeter, and Magnetometer were whats avaliable at the time on their website. Some of the key features are as follows:

• The GPS board is able to run up to 10Hz and 66 channels. <br>
• The Altimeter has a relative accuracy of +/- 0.25m of accuracy with settings in the datasheet for Quadcopter applications (internal filter setting and output rates)<br>
• The Magnetometer is pretty standard, being able to run at a fequency much higher than needed (up to 3.4 MHz) while having relatively low current draw (4mA).<br>

I did end up selecting this specific IMU due to the larger range of acceleration and rotational rates it could record, with a max of +/- 30g for acceleration, and up to 4000dps for rates. Typically, IMUs at this price range would only go up to half of these ranges. This is definitely overkill for my quadcopter, but I wanted the flexibility to explore more aggressive applications in the future (and have a "Do-It-All" chip for transfering to other projects).

Additionally, there was the option to select a 9DOF IMU + Magnetometer board, which I ended up not choosing as it included some firmware that is able to accomplish attitude estimation for you. I had no real interest in this feature so it would've just been taking up extra processing in the background. The two sensors (IMU & Magnetometer) also had different use cases within my Error-State Kalman Filter (ESKF), so I liked the ability of being able to mix and match between the two and keeping them seperated. In the future, I would absolutely consider a chip that has the two packaged into one once I get more involved with board design. 


<h2 id="flight-software">Flight Software</h2>
This section will be all about the code. I will try to give an overview of each of the modules and their purposes, followed by some system level architecture decisions. 

<h3 id="flight-software">Modules</h3>
Below is an overview of all my modules, which are under drone_flight_software/lib.

There are 7 main modules:

• <a href="#math-module">Math</a>  <br>
• <a href="#sensor-module">Sensors</a> <br>
• <a href="#navigation-module">Navigation </a> <br>
• <a href="#control-module">Control </a> <br>
• <a href="#motors-module">Motors </a> <br>
• <a href="#Logger-module">Logger </a> <br>
• <a href="#finite-state-machine-module">FiniteStateMachine </a> <br>


<h4 id="math-module">Math Module</h4>
This contains header only files for my math related containers. 

Within here, I have classes for: 

• Matrix18f [18x18] <br>
• Matrix9f [9x9] <br>
• Vector3f [3x1] <br>
• Quaternion <br>
• Rotation <br>

As the names implies, these are my implementations for common functions I need for each of the containers. For example, my matrix classes must be able to be zero-ed out or set to identity, handle standard math operations (add,subtract,multiply), along with all the standard operations (copy assignment, indexing elements, etc).

My vector class is only really there to go along with my Rotation classes, which is really a special implementation of a 3x3 matrix. And of course, I have a quaternion class for convenience as that is my attitude representation. 

Within this module, I also have a "Mathpk.h" file, where I import all the math container header files. This files exists for two main reasons. The first being an easy way to import all the math containers without having to do individual includes. Second, I depend any interactions between my classes here. For vectors, this includes dot and cross products. For quaternions, this includes some helper functions to do quaternion multiplication and propagation. I could've placed each of these within the classes themselves, but at the time, I thought this would be more clear and allows for each header to not have to import the other ones (although admittedly, this was added later so some of my classes may not strictly follow this, particularly Quaternion and Rotation).

<h4 id="sensor-module">Sensor Module</h4>
This is one of the bigger modules I have, and was actually the first part of the software I wrote. This takes care of the interfaces between my hand-written libraries, and the ones provided by Adafruit. 

At the very top, I import all of my Adafruit header files, which gives me access to create instances of their classes that contains each sensors:

<div class="code-snippet">
<span class="preprocessor">#include</span> <span class="header">&lt;Adafruit_Sensor.h&gt;</span>
<span class="comment">// IMU</span>
<span class="preprocessor">#include</span> <span class="header">&lt;Adafruit_ICM20X.h&gt;</span>
<span class="preprocessor">#include</span> <span class="header">&lt;Adafruit_ICM20649.h&gt;</span>
<span class="comment">// Magnetometer</span>
<span class="preprocessor">#include</span> <span class="header">&lt;Adafruit_LIS2MDL.h&gt;</span>
<span class="comment">// Altimeter</span>
<span class="preprocessor">#include</span> <span class="header">&lt;Adafruit_BMP3XX.h&gt;</span>
<span class="comment">// GPS</span>
<span class="preprocessor">#include</span> <span class="header">&lt;Adafruit_GPS.h&gt;</span>
</div>

There are lot of functions here, but at the very top will be my initializer along with my general functions for this class (checkout, calibrate, setup). This is then followed by specific functions for my IMU, Magnetometer, Altimeter, and GPS in that order. 

For each sensor type, there will be 1-3 functions that sets up the measurement ranges, update rates, internal bit settings, and more. These can easily be identified by looking at the inputs and seeing some non-standard parameter that typically comes from one of the header files above. 

For each sensor, there will be the following 3 functions:
<div class="code-snippet">
<span class="preprocessor">bool</span> sensorUpdate(<span class="header">uint32_t</span> now);
<span class="preprocessor">float</span> getSensorMeas();
<span class="preprocessor">float</span> processSensorMeas(<span class="header">float</span> sensorMeas);
</div>

Each sensor keeps track of when it was last updated in order to ensure that a stale measurement isn't being extracted and treated as new. The "sensorUpdate" will take in the current time, and if the difference between that and the last time the sensor was updated is greater than the user inputted frequency (should be the same one the ODR is set to), then this goes will update the current measurement by calling "getSensorMeas", and return TRUE to signal a measurement is ready. 

Since this is just getting the RAW measurement straight from the register (plus whatever processing Adafruit does), there needs to be some additional operations to ensure the measurement will be handed to the filter as expected. This includes stuff like frame conversions from the sensor frame to the body frame, subtracting off any start-up biases, applying Soft/Hard iron calibration for the magnetometer, conversion from pressure to height for the altimeter, and the associated GPS outputs into the corresponding position and velocity in our inertial frame. 

The remaining functions are helpers that are used for calculations associated with post processing (which are translated from my simulation). Within the sensor stuff, the only parts that actually relies on Adafruit libraries are all the functions that sets the setting for the sensors and getSensorMeas, the rest are functions I wrote for interfacing, making this module still very portable if I ever wanted to move away from these libraries.

<h4 id="navigation-module">Navigation Module</h4>
After the sensor module, the next one I wrote was naturally my Navigation / State Estimation module, which contains my ESKF. 

Within the prediction step, there are the following relevant functions:
<div class="code-snippet">
<span class="preprocessor">void</span> <span class="preprocessor">predict</span>(const <span class="header">std::array&lt;float,6&gt;</span> imuMeas, <span class="header">uint32_t</span> now);
<span class="preprocessor">void</span> <span class="preprocessor">propagateCovariance</span>(const <span class="header">Quaternion</span>& q, const <span class="header">Vector3f</span>& accelBias, const <span class="header">Vector3f</span>& gyroBias, const <span class="header">Vector3f</span>& accelMeas, const <span class="header">Vector3f</span>& gyroMeas, const <span class="header">float</span> dt);
<span class="header">Matrix18f</span> <span class="preprocessor">getQd</span>(<span class="header">float</span> dt);
<span class="header">Matrix18f</span> <span class="preprocessor">getSTM</span>(const <span class="header">Quaternion</span>& q, const <span class="header">Vector3f</span>& accelBias, const <span class="header">Vector3f</span>& gyroBias, const <span class="header">Vector3f</span>& accelMeas, const <span class="header">Vector3f</span>& gyroMeas, const <span class="header">float</span> dt) <span class="preprocessor">const</span>;
</div>

The "predict" function propagates the state and covariance, and is the only one accessed within the main loop, while the others are functions that are internally called. 

The filter does Dynamic Model Replacement / Dead-Reckoning, which just means it uses the IMU measurements as part of the dynamic model. When called, the current IMU measurement and current time must be provided. The filter will use the current time and its internal record of the last time its at in order to determine the timestep to propagate things by. 

After propagating the state, the function calls on "propagateCovaraince" to (as the name suggests) propagate the covariance. This depends on the discrete time process noise matrix, which "getQd" calculates, along with the state transition matrix, which "getSTM" calculates. 

For the update step, below are the following relevant functions:
<div class="code-snippet">
<span class="header">tiltData</span> <span class="preprocessor">updateTiltMeas</span>(const <span class="header">std::array&lt;float,3&gt;</span> accelMeas);
<span class="header">magData</span> <span class="preprocessor">updateMagMeas</span>(const <span class="header">std::array&lt;float,3&gt;</span> magMeas);
<span class="header">altData</span> <span class="preprocessor">updateAltMeas</span>(const <span class="header">float</span> altMeas);
<span class="header">gpsData</span> <span class="preprocessor">updateGPSMeas</span>(const <span class="header">std::array&lt;float,4&gt;</span> gpsMeas);
<span class="preprocessor">inline void</span> <span class="preprocessor">scalarMagTiltUpdate</span>(const <span class="header">float</span>& c1, const <span class="header">float</span>& c2, const <span class="header">float</span>& c3, const <span class="header">float</span>& z, const <span class="header">float</span>& hx, const <span class="header">float</span>& R2, <span class="header">float</span>& NIS, <span class="header">Matrix9f</span>& P, <span class="header">std::array&lt;float,9&gt;</span>& del_xk, <span class="header">std::array&lt;float,9&gt;</span>& K, <span class="header">std::array&lt;float,9&gt;</span>& P_row) {};
<span class="preprocessor">inline void</span> <span class="preprocessor">scalarGPSAltUpdate</span>(const <span class="header">int</span>& idx, const <span class="header">float</span>& z, const <span class="header">float</span>& hx, const <span class="header">float</span>& R2, <span class="header">float</span>& NIS, <span class="header">Matrix9f</span>& P, <span class="header">std::array&lt;float,9&gt;</span>& del_xk, <span class="header">std::array&lt;float,9&gt;</span>& K, <span class="header">std::array&lt;float,9&gt;</span>& P_row) {};
</div>

Firstly, I am doing sequential updates of my measurements, hence the seemingly redundant "updateMeas" calls I have. This is to ensure that measurements are able to be processed asynchrounously. Furthermore, within each measurement, I am doing scalar updates, which just means that for the multi-dimensional measurements, I am processing each component seperately. 

As mentioned in my other pages, this should be statistically equivalent to doing batch updates (as one might in simulations) given that the measurements are uncorrelated with each other, and the components of the measurements are uncorrelated with the others. In practice, this isn't completely true and so there is a loss of accuracy (due to not including as much of the off-diagonal information of the covariance into the updates). But this avoids large matrix inversions, which is more than worth the trade-off for a small system like this. 

Additionally, for this scalar sequential update, given the nature of the measurement types (translation vs attitude), there only really needs to be two distinct function calls to process the four measurements. This is what my "scalarMagTiltUpdate" and "scalarGPSAltUpdate" are. The inputs are a little convoluted without my comments in the code, but essentially, given the sparsity of the observation matrices for each of these, you can generalize the functions enough such that you only need these two. But this is what goes through the update steps of the filter, calculating the innovation covariance, NIS statistic, kalman gain, and updates the error state vector and covariance.

I also do some gating here using the NIS and 99% confidence interval for a 1DOF chi-square distribution. So if any of the components of any measurements returns a NIS greater than 6.635, it won't go through with updating the error state or covariance.

Lastly, after the updates, there is one function that injects the calculated error and moves the state estimate from being a-priori to a-posteriori:
<div class="code-snippet">
<span class="preprocessor">inline void</span> <span class="preprocessor">injectError</span>() {};
</div>

Within the ESKF class, there is a flag that gets flipped whenever ANY updates has been done. Calling on "injectError" will only do something if this flag is high, in which it signals that there is some error state that information can be extracted from. Once the step is complete, it will flip the flag back to low such that the next time this function is called, it won't do anything until the error state has been udpated with a measurement. This also resets the error states back to 0 to signal that all the information has been injected into the state estimate. 

<h4 id="control-module">Control Module</h4>
After my state estimation is done, I need to be able to pass it into my Control module that does the calculations for the requested control effort. The main functions are below:
<div class="code-snippet">
<span class="preprocessor">bool</span> <span class="preprocessor">updateControl</span>(<span class="header">uint32_t</span> now, const <span class="header">Vector3f</span>&amp; p_hat, const <span class="header">Vector3f</span>&amp; v_hat, const <span class="header">Quaternion</span>&amp; q_hat, const <span class="header">Vector3f</span>&amp; w_hat);
<span class="preprocessor">void</span> <span class="preprocessor">updateError</span>(const <span class="header">Vector3f</span>&amp; p_hat, const <span class="header">Vector3f</span>&amp; v_hat, const <span class="header">Quaternion</span>&amp; q_hat, const <span class="header">Vector3f</span>&amp; w_hat);
</div>

Not shown here is the initializer, which takes in the reference the reference states, nominal control, and the gain matrix. These are all saved as memebers of this class. I did include some additional functions that allows for all of these inputs to be modified afterwards, but this isn't used as a Guidance mode hasn't been implemented yet.

The function accessed by the main loop is "updateControl", which returns a boolean that signifies whether this is a control will be calculated at the current time its called. Like my other modules, the control instance keeps track of when the last calculation occured such that it only calculates a new one close to the set frequency. When there is a new control to be calculated, the function internally calls on "updateError" to calculate the various state error based on the incoming state estimate, then using the gain, calcuates the control effort required. 

Using the simulation, the gain can be precalculated then uploaded to the flight software. In my sim, I calculate the system matrices and the associated gain every timestep mostly to account for any potential changes in the reference attitude and body rates. However, this is way overkill for this application as the A and B matrices are almost always constant. Even if I were to implement a guidance module here, at most it would just be changing the reference states to feed into my controller. I assume some more advance guidance schemes may want to change the reference quaternion and body rates, but I'm not convinced that having to recalculate the gains will do much here. 

<h4 id="motors-module">Motors Module</h4>
After calculating the control effort, the information needs to be converted into the correct PWM signals for the motors. My Motors module is really just a class that contains 4 different "Servo" instances (an Ardiuno class), which controls the PWM signal from the relevant pins. The class has other functions I used for testing (mostly to command one, or all motors given a command), and functions to quickly arm and disarm the motors. But the main ones that ingests the control effort are as follows:

<div class="code-snippet">
<span class="preprocessor">void</span> <span class="preprocessor">commandControl</span>(const <span class="header">std::array&lt;float,4&gt;</span>&amp; u);
<span class="header">std::array&lt;float,4&gt;</span> <span class="preprocessor">allocateControl</span>(const <span class="header">std::array&lt;float,4&gt;</span>&amp; u);
</div>

The function that takes in the output from my controller is "commandControl". Since this function is only really called following a successful control calcluation, there doesn't need to be extra guards or checking the time. When this function is called, it will pass the control vector into "allocateControl", which using the psuedo-inverse of the allocation matrix (which I implemented symbolically to be formed based on the inputs), will calculate the spin rate square of each motors. From there, this is mapped onto a PWM command and any cautionary saturation is applied (to ensure it doesn't command the motor to use the max spin rate).

<h4 id="logger-module">Logger Module</h4>
Once I implemented the bulk of my actual closed loop control loop, I needed a robust way to log all the data of interest from my sensors / filter / controller / motors onto my SD card for post flight reviews, which is what my Logger module is. 

Within the folder, there are three header files:

• DataTypes.h <br>
• RingBuffer.h <br>
• Logger.h <br>

DataTypes.h contains all the definitions to all the structs I will be using that represents one singular piece of data. Going through this, I have definitions for all my classes, with each struct containing a memeber for the time, followed by any information I want to be able to access later. For the sensors, this is the measurements and NIS, while for something like the ESKF, its the estimated state and just the diagonal of the covariance (to check growth). So if you scroll back up, it can be seen that my Navigation functions will return these datatypes.

Here is an example of my imuData:

<div class="code-snippet">
<span class="preprocessor">struct</span> <span class="header">imuData</span> {
  <span class="header">float</span> time;
  <span class="header">float</span> ax, ay, az;
  <span class="header">float</span> gx, gy, gz;

  <span class="header">imuData</span>() : time(0), ax(0), ay(0), az(0), gx(0), gy(0), gz(0) { }

  <span class="header">imuData</span>(const <span class="header">float</span> flightTime, const <span class="header">std::array&lt;float,6&gt;</span>&amp; imuMeas) {
    <span class="preprocessor">this</span>-&gt;time = flightTime;
    <span class="preprocessor">this</span>-&gt;ax = imuMeas[0];
    <span class="preprocessor">this</span>-&gt;ay = imuMeas[1];
    <span class="preprocessor">this</span>-&gt;az = imuMeas[2];
    <span class="preprocessor">this</span>-&gt;gx = imuMeas[3];
    <span class="preprocessor">this</span>-&gt;gy = imuMeas[4];
    <span class="preprocessor">this</span>-&gt;gz = imuMeas[5];
  }
};
</div>

Following this, I have my RingBuffer.h, which is a FIXED Size data structure that allows me to save my data for each component to log to the SD card at a later time. The main feature of this is that it keeps track of the head (where I am inserting new data into the buffer), and the tail (when did I last log data up to into my SD card). The when the head reaches the end of the buffer, it will loop back around to the first index and begin overwritting data (hence the "Ring" part). As I log the data from my ring buffer, it will extract out all the information between the tail and head, incrementing the tail until it reaches where the head is. 

There are two main functions here:

<div class="code-snippet">
<span class="preprocessor">inline bool</span> <span class="preprocessor">push</span>(const <span class="header">T</span>&amp; item) {}
<span class="preprocessor">inline bool</span> <span class="preprocessor">pop</span>(<span class="header">T</span>&amp; item) {}
</div>

Given a data type "T", "push" will insert the item into where the header currently is and increment the header. Meanwhile, "pop" will check to ensure that the tail is not at the head (meaning we have un-extracted data), and return the item at tail by updating the item directly that has been passed in by reference. 

Lastly, the Logger.h is where this all comes together, and really just holds onto all my ring buffers, and controls when to write to the SD card. This also is where all my files are being initialized, written to, and flushed at regular intervals. I made this class so I am not writing to my files every loop, and can do so at a much slower frequency than any of my other modules. Additionally, my sensors and filters are producing way more information than I need at an extremely fast rate. This class also allows me to log the data types at at a slower rate (i.e., IMU runs at 100Hz, but I only take in every 5th measurement at 20Hz). 

This is also a templated class based on the size of the buffers (recall the FIXED size), so there needs to be some care into what an approriate size would be. I have two frequencies here, the first is the one at which I log data types into their respective buffers, and a second one that determines the frequency at which I run through the buffer, extract out the data from tail to the head, then flush the logs (i.e. save my data in case of failure). This means all my ring buffer needs to be large enough such that its able to contain the number of data types that will be inserted in between flushes. 

For example, my IMU data is being logged at 25Hz, meaning every 4th IMU sample is being recorded. My logger will flush the data every 2Hz. This means, every 2Hz, I will have about 12.5 IMU samples, so my buffer will need to be at least bigger than this for the IMU. Since I haven't found that I am RAM limited, all my ring buffers are about 1 magnitude larger than needed, and aims to hold ~5-10 seconds worth of each data type.

The other major component of this, aside from logging to the buffers, is the flushing feature. As previously mentioned, if I were to flush often, that would introduce delay and slow down my loops. But if I don't automatically flush, I risk losing the entire's flight log when something gets disconnected. This allows me to save to my files at a regular cadance. 

The main function is as follows:

<div class="code-snippet">
<span class="preprocessor">void</span> <span class="preprocessor">flush</span>(<span class="header">uint32_t</span> now) {
    <span class="preprocessor">if</span> (<span class="preprocessor">this</span>-&gt;lastLog_ == <span class="header">UINT32_MAX</span>) {
        <span class="preprocessor">this</span>-&gt;lastLog_ = now;
    } <span class="preprocessor">else</span> {
        <span class="preprocessor">if</span> (<span class="preprocessor">static_cast</span>&lt;<span class="header">float</span>&gt;(now - <span class="preprocessor">this</span>-&gt;lastLog_) &gt;= <span class="preprocessor">this</span>-&gt;freqFlush_) {
            flushIMU();
            flushTilt();
            flushMag();
            flushAlt();
            flushESKF();
        }
    }
};
</div>


This checks to see if it enough time has passed since the last flush, before calling on all the helper functions. Each one of these functions does the same thing. They will go through their respective buffers, extract out the data between the tail and head, form a large "char" that formats the information from the data type into a nice row, before writing it to the respective file and flushing it. 

Once again, there needs to be some consideration of the size of the "char" that contains all this data that is being written to the file. The way I try to estimate this is as follows: 

$\#_{floats} \times 11 \times \frac{f_{data type log}} { f_{logger flush}}$ 

where $\#_{floats}$ is the number of floats within the data type, 11 comes from a conservative estimate of how much a float takes up, $f_{data type log}$ is the rate the data type is logged, and $f_{logger flush}$ is the rate the logger will flush its files.

So for the IMU, which contains 7 floats per data type, and logs data at a frequency of 25 Hz, while the logger flush datas at a frequency of 2Hz, this comes out to ~962.5 characters. I did this rough estimate with each one, and add a generous amount of margin to account for any other unaccounted for characters when logging.

Unfortunately, this was hardcoded so if the set logging frequency of anything ever changes, I would need to keep in mind that the sizes here should be adjusted accordingly. I could maybe further template the Logger class on this size as well, but I recall some issues trying to initialize the filter without having this statically defined. 


<h4 id="finite-state-machine-module">FiniteStateMachine Module</h4>
So technically, this module is not fully fleshed out. I currently have this implemented as a header only, and in reality, its meant to only take care of transitioning the current state of the FSW. However, since I didn't have a Guidance model, I also use it to directly modify the references of the controller module for some pre-planned testing (which isn't the safest behavior). 

Currently, this consists of hard-coded state transitions based on time to execute some pre-determined behavior. I do have a safe state "BOOT", that I use to transition into once the test is concluded. 

The other main responsibility of the Finite State Machine (FSM) is to handle flight safety. Currently, the FSM will output a flag that must be true in order for the controller to calculate any controls, and for the motor to recieve a PWM command. Going into the safe mode will set both of these to be false for quick disabling of the quadcopter if any off-nominal scenario happens. 

As of now, the only safety related logic I have added is to ensure that the tilt angle at ANY given time does not surpass 90 degrees (in which it most likely means the quadcopter has lost control).

<h2 id="flight-software">System Level Archetecture</h2>
- How i decided to use Bias vs No Bias
- How I am handling sensor frequency, and navigation frequency, etc.
Everyone keeping track internally of their time. Should really do something like with interrupts



<h2 id="flight-software">Testing</h2>

This was the first part of the code that I wrote. 

Talk about how I used a breadboard at the time to connect and can test the stuff.'

all the issues with eskf copying over that unit testing helped


This was tested by implementing it within my simulation. I actually wrote the scalar sequential updates in my FSW first, before writing it into my sim for testing.

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
