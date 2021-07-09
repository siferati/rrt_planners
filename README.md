# rrt_planners

A collection of global planners based on Rapidly-exploring Random Trees for move_base.

*Developed and tested only for ROS Melodic Morenia.*

### Usage

Set the move_base paramenter `base_global_planner` to one of the following:

* **rrt_planners/RRTPlanner** - LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning.

* **rrt_planners/RRTStarPlanner** - Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. The international journal of robotics research, 30(7), 846-894.

* **rrt_planners/RRTXPlanner** - Otte, M., & Frazzoli, E. (2015). RRTX: Real-Time Motion Planning/Replanning for Environments with Unpredictable Obstacles. In Algorithmic Foundations of Robotics XI (pp. 461-478). Springer, Cham.

### Published Topics

* **plan** - The computed global plan.

* **rapidly_exploring_random_tree** - The computed rapidly-exploring random tree.
