# RRT-KUKA-Simulation
This project is a simulation of RRT path-planning with the KUKA robot arm, developed as a research project at the Laboratory of Intelligent Decisions and Autonomous Robots at Georgia Tech. The end-effector reaches a desired point in Euclidean space by using RRT to calculate a vector of joint angles. The simulation was created using [Drake](https://drake.mit.edu/) and [OMPL](https://ompl.kavrakilab.org/).

![](https://github.com/jerredchen/RRT-KUKA-Simulation/blob/main/rrt1.gif)

This is an example of a simulation of the KUKA arm's end effector reaching a desired point in space using RRT.

![](https://github.com/jerredchen/RRT-KUKA-Simulation/blob/main/rrt2.gif)

This demonstrates the KUKA arm's end effector able to reach another desired point in space. Notice that the RRT algorithm is *not* optimal.
