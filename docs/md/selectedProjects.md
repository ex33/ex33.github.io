# Selected Projects

More under "Projects" tab.


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


