# Writeup (Due 10-13)

In your ROS package create a README.md file to hold your project writeup. Your writeup should touch on the following topics. We expect this writeup to be done in such a way that you are proud to include it as part of your professional portfolio. As such, please make sure to write the report so that it is understandable to an external audience. Make sure to add pictures to your report, links to Youtube videos, embedded animated Gifs (these can be recorded with the tool peek).

### What was the goal of your project? (Rajiv)

### How did you solve the problem? (Rajiv)

(Note: this doesn’t have to be super-detailed, you should try to explain what you did at a high-level so that others in the class could reasonably understand what you did).

[Rajiv Make State Diagram]

- moving particles with odom (including high-level overview of math) [Rajiv Draw]
- updating weights with laser (original vs. optimization)
- normalizing particle weights (and where)
- resampling particles (new particle blind to parent vs. aware of parent weight) (Alex add/review)
- locate robot (mean of particles - effective due to convergence of particles) (Alex)
  - tried clustering - too slow

### Describe a design decision you had to make when working on your project and what you ultimately did (and why)?

These design decisions could be particular choices for how you implemented some part of an algorithm or perhaps a decision regarding which of two external packages to use in your project.

- optimizations (efficiency and effectiveness) (Alex)
  - weights
    - transition to matrix for efficiency
    - using step function for weight instead of gaussian
  - resampling (efficiency and effectiveness)
- teaming (Rajiv)
  - code structure
  - peer reviews

### What if any challenges did you face along the way?

- each add a few if needed
- structure of helper functions and starter code (Alex)
  - upderstanding flow of code
  - conflict b/w what what would most effective vs. what is computationally good and what is necessary
    - clustering algorithm to locate robot (Alex)
- not being able to use linux (Rajiv)
  - benefits of teaming design

### What would you do to improve your project if you had more time? (Alex)

- each add more if needed
- robot kidnapping (things that would need to change in this approach, ex: odom to update particles)
- experiment with other weighting algorithms with direct comparisons (which converges first on the same bag file)

### Did you learn any interesting lessons for future robotic programming projects? (Alex)

These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

- github issues - very helpful
  - branches and PRs to catch partner up on code
- good scoping on the project - steady workload across the three weeks
- using the starter code still allowed us to investigate areas that we were excited about

### TODO for Writeup and Presentation

- collect bag files and document in readme (Alex)
- collect gif of algorithm running (Alex)
  - also old version of resampling
- Diagrams/visuals (Rajiv)
