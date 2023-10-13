# Robot Localization Project

Course: A Computational Introduction to Robotics, Fall 2023

Professor: Paul Ruvolo

### What was the goal of your project?

Robot localization is the field of robotics that uses sensor measurements and environment information to place a robot within a known or unknown field with a reasonable degree of certainty. The goal of this project was to implement the particle-filter approach to robot localization on the Neato robotic vacuum platform bots in ROS by using lidar-scanning data in combination with a known occupancy-field-based map to locate the robot in space.

A particle filter is an algorithm in robotics which typically initializes particles representing a simulated robot within a known map. It then compares their simulated sensor values to the robot’s actual sensor values to remove poorly matching points and place new points on top of well matching points, while moving each point in accordance with its pose and the robot movement data. Through this continual moving of points and resampling, the poorly fitting particles are removed and through repopulation of well fitting particles the highest weighted particles ideally approach the true position of the robot.

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

Because we decided to build off of the starter code, and the overall structure of the algorithm was generally fixed, many of our design decisions came down to optimizations of our algorithm’s efficiency and effectiveness.

A big part of where the particle filter slows down is in calculating weights for each particle after every resample. This is because, at a high level, each of the 360 lidar data points need to be evaluated for each particle. For our first pass through this algorithm this was implemented as a nested for loop that iterated through each particle and evaluated each data point one at a time, keeping track of a cumulative total; included in this was a transformation from polar to cartesian for each individual data point. This slowed our algorithm down so much that we needed to reduce the number of particles to less than 50 in order for it to run without significant lag. The outer for loop can not be avoided, as each particle has a unique reference frame; however, the evaluation of the lidar data was improved by transforming and evaluating the points all at once using matrix operations. Furthermore, we found that the simple approach of counting the number of lidar points whose distance from the map was below some acceptable threshold value was a sufficient and low-cost way to assign an overall weight to the particle, which removed the need to set up a gaussian function and apply it to all the data points.

A place where we had the opportunity to pursue modifications to the algorithm to increase its effectiveness was during the resampling step. Specifically, we wanted to have the resample radius for each of the good seed particles to be variable, dependent on its weight, or confidence. Our final design decision was to decrease the standard deviation of resampled particles around each seed particle exponentially with the weight of the seed particle. This has the effect of gradually converging the particles around one final point; see the comparison below for the difference between when resample radius is constant (left) and when the resampled particles “know” the confidence of their seed particle (right).

<figure
  style=
    "display: flex;
    width:50%;"
>
  <div
    style=
      "flex: 50%;
      padding: 5px;"
  >
    <img 
      src="./gifs/original_particles.gif"
      alt="Resampling with constant radius"
    >
  </div>
  <div
    style=
      "flex: 50%;
      padding: 5px;"
  >
    <img 
      src="./gifs/converged_particles.gif"
      alt="Resampling with adaptive radius"
    >
  </div>
</figure>

During the initial scoping of this project, we decided that we wanted to engage in this code work in a more rigidly managed structure than is typically required in a two person project. We used github’s native issue and branch management tools to submit pull requests for each new code expansion, conducted code reviews before merging any code to the main branch, and used issues to control the scope and direction of our development. Although this structure seems cumbersome, it actually massively sped up development time because it provided a scaffolded, non-personal, and automatic structure to justify the many design decisions that we were faced with within the codebase itself. This also allowed us to easily see where we were going at any point in the development cycle, and track ourselves in relation to our overall goals to reduce or increase our scope when necessary. The most unexpected benefit of this design decision was that it was massively beneficial when we were reduced to one working linux install between the two of us.

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

- Diagrams/visuals (Rajiv)
