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
    "display: flex;"
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

Early on, the largest challenge we faced was getting integrated into the starter code. We made sure that we spent some time to fully understand the flow of code in the particle filter node, as there were many helper functions that complicated the control structure. We found that the best was to understand how the code works was to implement a very quick pass through each of the missing components in the starter repository, and go back and refactor, optimize, and scale up once the behavior of the particles was as expected.

As we got deeper into the implementation, another challenge was balancing what would be most effective and what is necessary and computationally efficient. A great example of this is when we tried to implement the method that picks the robot’s location, given the placement and weights of the particles. We thought it would be interesting to implement a clustering algorithm that groups the particles and returns the sizes and centroids of the clusters; this could be used if the particle filter ends up splitting into clusters instead of converging. It turns out that this was extremely computationally expensive, and we noticed that the particles were actually converging decently well, even right at the beginning; therefore, we decided a simpler and equally effective approach would be to use the mean particle pose as the robot’s pose.

As mentioned previously, one of our linux installs was lost two days into the project. Luckily, the structure which we had chosen early on into the project lended itself to discretizing development chunks that could be reviewed before implementation. As a result, even though there was no way for Rajiv to test his code, he was still able to participate by building functionality in branches that would go to Alex for review before their merge. Though github and exception handling in tandem with thorough f-string print statement, Rajiv was still able to participate actively in development. Lastly, because this structure lended itself to frequent recalibrating discussions to create new issues and blaze the path forward in development, that soft-skills and management portion of the project was unaffected by access to linux and was of large emphasis and utility to the project flow.

### What would you do to improve your project if you had more time?

We have a couple of areas of interest that we’d like to pursue if we had more time to improve the project. The first one would be to take on a more challenging variation of this problem, such as the robot kidnapping problem, in which the robot can be picked up and relocated at any time. Some of our assumptions and algorithmic approaches that worked for our project would not work for the robot kidnapping problem, which would require us to make the particle filter more robust. For example, while we can still update particles with the robot’s odometry, it wouldn’t make much sense if all of the particles are on the opposite side of the map from the robot after it has been kidnapped, and resampling around those particles would continue to yield bad results. Therefore, we would need to figure out a way to reseed particles across the entire map if we sense that the robot has been kidnapped.

Additionally, we would like to look into and experiment with different weighting algorithms and make direct comparisons between them. By running different algorithms on the same bag file as a baseline, we can compare how different weighting algorithms affect how quickly the particles located the robot. Since we used a very simple weighting algorithm, described above, it would be interesting to see if something more complicated, such as assigning weights based on a gaussian curve, actually results in better performance.

### Did you learn any interesting lessons for future robotic programming projects? (Alex)

These could relate to working on robotics projects in teams, working on more open-ended (and longer term) problems, or any other relevant topic.

- github issues - very helpful
  - branches and PRs to catch partner up on code
- good scoping on the project - steady workload across the three weeks
- using the starter code still allowed us to investigate areas that we were excited about

### TODO for Writeup and Presentation

- Diagrams/visuals (Rajiv)
