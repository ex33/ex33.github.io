# All Projects

Here is a comprehensive list of my current and completed projects. 

<div class="projects-list">

  <p><strong>Current Projects</strong></p>

  <p>• [Project 1] Short description or focus</p>
  <p>• [Project 2] Short description or focus</p>
  <p>• [Project 3] Short description or focus</p>

  <p><strong>Completed Projects</strong></p>

  <p>• [Project A] Short description or focus</p>
  <p>• [Project B] Short description or focus</p>
  <p>• [Project C] Short description or focus</p>

</div>


## Current Projects
<div class="project-card">
    <div class="project-image">
        <img src="assets/AgentBreederDiagramJPG.jpg" alt="AgentBreeder project">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="md/projects/quadcopterFromScratch/quadcopterPart1.html" class="project-link">
                Quadcopter from Scratch: 6DOF MATLAB Simulation
            </a>
        </h3>
        <div class="project-venue">Personal Project</div>
        <div class="project-description"> <strong>Part 1 of building a quadcopter from "scratch". </strong> <br> 
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

<div class="project-card">
    <div class="project-image">
        <img src="assets/AgentBreederDiagramJPG.jpg" alt="AgentBreeder project">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/AgentBreeder" class="project-link">
                Quadcopter from Scratch: Flight Software
            </a>
        </h3>
        <div class="project-venue">Personal Project</div>
        <div class="project-description"> <strong>Part 2 of building a quadcopter from "scratch". </strong> <br> 
        With a simulation built, the next major step in this project is translating its components onto hardware / flight software. This part goes over the hardware selection, writing the flight software, and any other lessons learned from translating the algorithms from the MATLAB simulation onto the physical system. </div>
<!--         MOVET HIS PART INTO THE ACTUAL PAGE
        The flight computer chosen is the Teensy 4.1 due to its high performance MCU, large memory, abundant I/O, and interfaces with the ardunio Platform, all of which allowed for ease of mind for a first prototype. The sensors were selected from Adafruit, which featured breakout boards with comptatible libraries readily avaliable to handle the low level programming. Naturally, Platform.io was selected as the development environment due to having more features than the Ardiuno IDE to handle scaling up the software. </div> -->
        <div class="project-tags">
            <span class="tag tag-language">C++</span>
            <span class="tag tag-software">Data Buffers</span>
            <span class="tag tag-software">Flight Software (FSW)</span>
            <a href="https://github.com/ex33/drone_flight_software" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>



## Completed Projects

<div class="project-card">
    <div class="project-image">
        <img src="assets/AgentBreederDiagramJPG.jpg" alt="AgentBreeder project">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="/AgentBreeder" class="project-link">
                AgentBreeder: Mitigating the AI Safety Impact of Multi-Agent Scaffolds via Self-Improvement
            </a>
        </h3>
        <div class="project-venue">NeurIPS 2025 spotlight</div>
        <div class="project-authors">J Rosser, Jakob Foerster</div>
        <div class="project-year">2025</div>
        <div class="project-tags">
            <span class="tag tag-safety">Multi-Agent Safety</span>
            <a href="https://arxiv.org/abs/2502.00757" class="tag tag-arxiv">ARXIV</a>
            <a href="https://github.com/J-Rosser-UK/AgentBreeder" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>

<div class="project-card">
    <div class="project-image">
        <img src="assets/stream_jpg.jpg" alt="Stream project">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="https://openreview.net/pdf?id=5HGu0ZqBl9" class="project-link">
                Stream: Scaling Mechanistic Interpretability to Long Context in LLMs via Sparse Attention
            </a>
        </h3>
        <div class="project-venue">NeurIPS 2025 Mech Interp Workshop</div>
        <div class="project-authors">J Rosser, José Luis Redondo García, Gustavo Penha, Konstantina Palla, Hugues Bouchard</div>
        <div class="project-year">2025</div>
        <div class="project-tags">
            <span class="tag tag-safety">Mechanistic Interpretability</span>
            <a href="https://openreview.net/pdf?id=5HGu0ZqBl9" class="tag tag-arxiv">ARXIV</a>
        </div>
    </div>
</div>

<div class="project-card">
    <div class="project-image">
        <img src="assets/mapping_faithful.jpg" alt="Mapping Faithful Reasoning project">
    </div>
    <div class="project-content">
        <h3 class="project-title">
            <a href="https://openreview.net/pdf?id=NJNr5KbW3m" class="project-link">
                Mapping Faithful Reasoning in Language Models
            </a>
        </h3>
        <div class="project-venue">NeurIPS 2025 Mech Interp Workshop</div>
        <div class="project-authors">Jiazheng Li, Andreas Damianou, J Rosser, José Luis Redondo García, Konstantina Palla</div>
        <div class="project-year">2025</div>
        <div class="project-tags">
            <span class="tag tag-safety">Mechanistic Interpretability</span>
            <a href="https://openreview.net/pdf?id=NJNr5KbW3m" class="tag tag-arxiv">ARXIV</a>
        </div>
    </div>
</div>

<div class="project-card">
    <div class="project-content">
        <h3 class="project-title">
            <a href="https://github.com/jrosseruk/CyberAgentBreeder" class="project-link">
                CyberAgentBreeder: An Evolutionary Framework for Breeding LLM Cybersecurity Agents
            </a>
        </h3>
        <div class="project-venue">UK AISI Bounty Programme • Project</div>
        <div class="project-authors">J Rosser, Joe Skinner</div>
        <div class="project-year">2025</div>
        <div class="project-tags">
            <span class="tag tag-safety">Agentic Scaffolds</span>
            <a href="https://github.com/jrosseruk/CyberAgentBreeder" class="tag tag-github">GITHUB</a>
        </div>
    </div>
</div>

<div class="project-card">
    <div class="project-content">
        <h3 class="project-title">
            <a href="https://www.lesswrong.com/posts/qwAiKvomuAm5ekC4D/subliminal-learning-transmitting-misalignment-via" class="project-link">
                Transmitting Misalignment with Subliminal Learning via Paraphrasing
            </a>
        </h3>
        <div class="project-venue">LessWrong</div>
        <div class="project-authors">Matthew Bozoukov, Taywon Min, J Rosser, Callum McDougall</div>
        <div class="project-year">2025</div>
        <div class="project-tags">
            <span class="tag tag-safety">AI Safety</span>
            <a href="https://www.lesswrong.com/posts/qwAiKvomuAm5ekC4D/subliminal-learning-transmitting-misalignment-via" class="tag tag-arxiv">LESSWRONG</a>
        </div>
    </div>
</div>
