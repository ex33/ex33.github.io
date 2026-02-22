# All Projects

Here is a comprehensive list of my current and completed projects. 

<div class="projects-list">

  <p><strong>Current Projects</strong></p>

  <p>• <a href="#quadcopter-sim">Quadcopter Project: 6DOF MATLAB Simulation</a></p>
  <p>• <a href="#quadcopter-fsw">Quadcopter Project: Flight Software</a></p>

  <p><strong>Completed Personal Projects</strong></p>

  <p>• <a href="#sims-flanagan">Sims-Flanagan Method Implementation</a></p>
  <p>• <a href="#quadcopter-theory">Quadcopter Project: Theory and Background</a></p>

  <p><strong>Past University Projects</strong></p>

  <p>• <a href="#alfano-collision">Collision Detection between Objects in Orbit via Alfano's Method</a></p>
  <p>• <a href="#sac-rpo">Recreation of a Simple Adaptive Controller (SAC)</a></p>
  <p>• <a href="#comp-eng_physics">Computational Engineering Physics Python Library</a></p>
  <p>• <a href="#formation_reconfig_repo">Formation Flying Python Library</a></p>
  <p>• <a href="#magnus-evtol">Exploration of Initial Control System Architecture for an E-VTOL utilizing Magnus Effect</a></p>
  <p>• <a href="#cubesat-structure">Structural Design and Analysis of 12U Cubesat Bus</a></p>

</div>



## Current Personal Projects
<div class="project-card" id="quadcopter-sim">
    <div class="project-image">
        <img src="/md/projects/quadcopterFromScratch/simulation_videos/progression/trajectory_video_full_sim.gif" alt="Quadcopter Part1">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/quadcopterFromScratch/quadcopterPart1.html" class="project-link" target="_self">
                Quadcopter Project: 6DOF MATLAB Simulation
            </a>
        </h3>
        <div class="project-venue">Personal Project</div>
        <div class="project-description"> <strong>Part 1 of building a quadcopter. </strong> <br> 
        As with all GNC systems, adequate modeling and simulation of its component is necessary for tuning its algorithm, hardware selection, and a first pass at validation through analyses such as Monte-Carlo. This project contains details on a simulation built in MATLAB featuring sensor / motor models, state estimation, and closed loop control of the position, velocity, attitude (quaternion), and body rates.</div>
        <div class="project-tags">
            <span class="tag tag-language">MATLAB</span>
            <span class="tag tag-control">LQR Control</span>
            <span class="tag tag-estimation">Error State Kalman Filter</span>
            <span class="tag tag-misc">Stochastic Sensor Modeling</span>
            <a href="https://github.com/ex33/drone_sim" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>

<div class="project-card" id="quadcopter-fsw">
    <div class="project-image">
        <img src="/md/projects/quadcopterFromScratch/images/FlightComputer.png" alt="Quadcopter Part2">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/quadcopterFromScratch/quadcopterPart2.html" class="project-link" target="_self">
                Quadcopter Project: Flight Software
            </a>
        </h3>
        <div class="project-venue">Personal Project</div>
        <div class="project-description"> <strong>Part 2 of building a quadcopter. </strong> <br> 
        With a simulation built, the next major step in this project is translating its components onto hardware. This part mostly goes over writing the flight software, briefly touching integration, and any other lessons learned from translating the algorithms from the MATLAB simulation onto the physical system. Towards the end, there are multiple videos containing closed-loop testing, verification, and validation of the attitude controller. </div>
        <div class="project-tags">
            <span class="tag tag-language">C++</span>
            <span class="tag tag-software">Flight Software (FSW)</span>
            <a href="https://github.com/ex33/drone_flight_software" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>



## Completed Personal Projects
<div class="project-card" id="sims-flanagan">
    <div class="project-image">
        <img src="/md/projects/simsFlanagan/sims_flanagan_thumbnail.png" alt="Sims Flanagan">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/simsFlanagan/simsFlanagan.html" class="project-link" target="_self">
                Sims-Flanagan Method Implementation
            </a>
        </h3>
        <div class="project-venue">Personal Project</div>
        <div class="project-description"> 
        Sims-Flanagan is a trajectory optimization method that is used for preliminary design of interplanetary low-thrust missions, developed by Jon Sims and Steve Flanagan at NASA JPL. <br>
        This is my implementation of it using MATLAB's fmincon and was an opportunity for me to learn some of the practical tuning considerations involved with trajectory optimization problems (variable scaling, step sizes, smoothness of objective/constraint functions, etc).
        </div>
        <div class="project-tags">
            <span class="tag tag-control">MATLAB</span>
            <span class="tag tag-estimation">Trajectory Optimization</span>
            <a href="https://github.com/ex33/sims_flanagan" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>

<div class="project-card" id="quadcopter-theory">
    <div class="project-image">
        <img src="/md/projects/quadcopterFromScratch/images/DroneInertialFrame.png" alt="Quadcopter Part0">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/quadcopterFromScratch/quadcopterPart0.html" class="project-link" target="_self">
                Quadcopter Project: Theory and Background
            </a>
        </h3>
        <div class="project-venue">Personal Project</div>
        <div class="project-description"> <strong>Part 0 of building a quadcopter. </strong> <br> 
        This isn't really a project, but attempts to contain a comprehensive background of all the theories and algorithms that is within the simulation and eventually the flight software. This is here in case
        something seems weird within the code or something needs more explanation. Its also a resource for me to quickly refer back to when developing features, so warning if you're sensitive to grammatic errors.</div>
        <div class="project-tags">
            <span class="tag tag-control">LQR Control</span>
            <span class="tag tag-estimation">Error State Kalman Filter</span>
            <span class="tag tag-misc">Sensor Modeling</span>
        </div>
    </div>
</div>


## Past University Projects
<div class="project-card" id="alfano-collision">
    <div class="project-image">
        <img src="/md/projects/cornell/mae6760/mae6760_thumbnail.png" alt="mae6760">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/cornell/mae6760/mae6760.html" class="project-link">
                Collision Detection between Objects in Orbit via Alfano's Method
            </a>
        </h3>
        <div class="project-venue">Class Project [MAE 6760]</div>
        <div class="project-description">
        This was a project I did for my class on Model Based Estimation. I wanted to explorer other applications of filtering aside from typical state estimation, landing on Collision Detection. This uses the EKF linearized model in order to propagate the covariance / uncertainty forward in time, calculate a conjunction, and sequentially update the propabability as time goes and GPS measurement shrinks the covariance. This did not model any collision avoidance maneuvers.</div>
        <div class="project-tags">
            <span class="tag tag-language">Extended Kalman Filter [EKF]</span>
            <span class="tag tag-control">Collision Avoidance</span> 
        </div>
    </div>
</div>


<div class="project-card" id="sac-rpo">
    <div class="project-image">
        <img src="/md/projects/cornell/mae6780/mae6780_thumbnail.png" alt="mae6780">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/cornell/mae6780/mae6780.html" class="project-link">
                Recreation of a Simple Adaptive Controller (SAC) for Spacecraft Proximity Operations
            </a>
        </h3>
        <div class="project-venue">Class Project [MAE 6780]</div>
        <div class="project-description">
        This was a project I did for my class on Multi-variable Controls. I wanted to explore a controller technique I haven't been exposed to before and get some RPO experience before my first job.</div>
        <div class="project-tags">
            <span class="tag tag-language">Simulink</span>
            <span class="tag tag-control">Adaptive Control</span>
            <a href="https://github.com/ex33/mae_6780_final_project" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>

<div class="project-card" id="comp-eng_physics">
    <div class="project-image">
        <!-- <img src="/md/projects/cornell/aep5830/aep5830_thumbnail.png" alt="aep5830"> -->
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="https://github.com/ex33/Computational-Engineering-Physics" class="project-link">
                Computational Engineering Physics Python Library
            </a>
        </h3>
        <div class="project-venue">Class Project [AEP 5830]</div>
        <div class="project-description">
        This is a repo of all the relevant python scripts written for the various concepts taught in the course, ranging from simple root finding, to integrators, to finite difference methods. 
        [Page under construction...]</div>
        <div class="project-tags">
            <span class="tag tag-language">Python</span>
            <span class="tag tag-control">Numerical Methods</span>
            <a href="https://github.com/ex33/Computational-Engineering-Physics" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>

<div class="project-card" id="formation_reconfig_repo">
    <div class="project-image">
        <!-- <img src="/md/projects/cornell/aep5830/aep5830_thumbnail.png" alt="aep5830"> -->
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="https://github.com/ex33/Formation_Reconfig" class="project-link">
                Formation Flying Python Library
            </a>
        </h3>
        <div class="project-venue">Space Imaging and Optical Systems [SIOS]</div>
        <div class="project-description">
        This is a repo of all the relevant python scripts written for the projects I have worked on under the SIOS lab in relations to formation flying / RPO of spacecrafts. 
        [Page under construction...]</div>
        <div class="project-tags">
            <span class="tag tag-language">Python</span>
            <span class="tag tag-control">Astrodynamics</span>
            <a href="https://github.com/ex33/Formation_Reconfig" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>


<div class="project-card" id="magnus-evtol">
    <div class="project-image">
        <img src="/md/projects/cornell/magnus_ultralight/magnus_ultralight_thumbnail.png" alt="mae6780">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/cornell/magnus_ultralight/magnus_ultralight.html" class="project-link">
                Exploration of Initial Control System Architecture for an E-VTOL utilizing Magnus Effect
            </a>
        </h3>
        <div class="project-venue">Space System Design Studio [SSDS]</div>
        <div class="project-description">
        Magnus Ultralight was a E-VTOL that featured a spinning cylinder in the middle that would generate extra lift from the incoming air flow due to the Magnus effect. <br>
        This was my first time working on a control system, and I have definitely learned a lot since then. This was during a time where I was concurrently taking a lot of the courses I needed for a project like this, leading to many decisions that I would've made differently in hindsight (see my Quadcopter project for how I would've approached this now). Nevertheless, this helped build a lot of fundamentals and it was a ton of fun to take the project out to a field and attempting to fly it.  
        </div>
        <div class="project-tags">
            <span class="tag tag-language">Simulink</span>
            <span class="tag tag-control">PID</span>
        </div>
    </div>
</div>


<div class="project-card" id="cubesat-structure">
    <div class="project-image">
        <img src="/md/projects/cornell/cislunar_explorers/cislunar_explorers_thumbnail.png" alt="mae6780">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/md/projects/cornell/cislunar_explorers/cislunar_explorers.html" class="project-link">
                Structural Design and Analysis of 12U Cubesat Bus
            </a>
        </h3>
        <div class="project-venue">Space System Design Studio [SSDS]</div>
        <div class="project-description">
        The Cislunar Explorers were a pair of 3U cubesats that featured water electrolysis as propulsion, along with a novel optical navigation system. This project came from one of NASA's Cubesat challenges,but due to delays, the cubesat was not able to launch with Artemis. This allowed for an opportunity to overhaul the design that was passed through multiple teams and become extremely overconstrained. <br>
        This was my undergraduate senior design project, where my cubesat team was undergoing a design change from a 6U to a 12U Cubesat. Admittedly, this was my first time doing structural analysis and analyzing frequency modes, but I learned a ton from this. I was honest with a lot of my self-"discoveries" in this report, learning about meshing, representative mass models and boundary conditions, and more.  
        </div>
        <div class="project-tags">
            <span class="tag tag-language">ANSYS</span>
            <span class="tag tag-control">Frequency Analysis</span>
        </div>
    </div>
</div>
