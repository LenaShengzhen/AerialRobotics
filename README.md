# AerialRobotics
Simulate the path planning and trajectory planning of quadrotors/UAVs.



## Motion Planning project
#### The main purpose:
- A new method to improve path planning

- Compare JPS and new method

#### Functions already implemented:

- Path planning algorithms (Jump point search,  JPS-PF )

- Generating Convex Polytopes(Safe Flight Corridors)

- Time Allocation(Average time allocation, Trapezoidal time allocation)

- Trajectory planning(QP)

- Trajectory following control(PD controller)

The more detailed data is in [Motion_Planning.md](Motion_Planning.md)

#### Demo animation: 

<img src="gifs/pro2/head.gif" alt="polyhedron-1" width="700">

<img src="gifs/pro2/3dCorner-new-ys.gif" alt="3dCorner-new-ys" width="550">

## Trajectory Planning project
#### The main purpose:
- Compare different pathfinding algorithms(Dijkstra,  Astar,  Jump point search)
- Compare different trajectory planning methods(Ax = b, QP)

#### Functions already implemented:
- Path planning algorithms (Dijkstra,  Astar,  Jump point search)
- Generating Convex Polytopes(Safe Flight Corridors)
- Trajectory planning(Ax = b, QP)
- Trajectory following control(PD controller)

The more detailed data is in [Trajectory_Planning.md](Trajectory_Planning.md)

#### Demo animation: 

<img src="gifs/pro1/head.gif" alt="polyhedron-1" width="400">







**More related knowledge introduction on  [Wiki](https://github.com/LenaShengzhen/AerialRobotics/wiki)**

**New feature schedule on [projects](https://github.com/LenaShengzhen/AerialRobotics/projects/2)**

**Bug and issue tracking on [projects](https://github.com/LenaShengzhen/AerialRobotics/projects/1)**





## Reference

[1] D. Harabor and A. Grastien. 2011. "**Online Graph Pruning for Pathfinding on Grid Maps**". In Proceedings of the 25th National Conference on Artificial Intelligence (AAAI), San Francisco, USA.

[2] R. Deits and R. Tedrake, "**Computing large convex regions of obstacle-free space through semidefinite programming**",in Algorithmic Foundations of Robotics XI. Berlin, Germany: Springer, 2015, pp. 109–124.

[3] S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor, et al., "**Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-d complex environments**", IEEE Robotics and Automation Let- ters, vol. 2, no. 3, pp. 1688-1695, July 2017.

[4] S. Savin, "**An algorithm for generating convex obstacle-free regions based on stereographic projection**,” in International Siberian Confer- ence on Control and Communications, 2017.

[5] Xingguang Zhong, Yuwei Wu, Dong Wang, Qianhao Wang, Chao Xu, and Fei Gao, "**Generating Large Convex Polytopes Directly on Point Clouds**",2020

[6] D.W.Mellinger,"**Trajectory generation and control for quadrotors**"Ph.D. dissertation, Univ. Pennsylvania, Philadelphia, PA, 2012.

[7] D. Mellinger and V. Kumar, "**Minimum snap trajectory generation and control for quadrotors**", inProc. 2011 IEEE Int. Conf. Robot.Autom.,2011

[8] C. Richter, A. Bry, and N. Roy, “**Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments**,” in *Proc. Int. Symp. Robot. Res.*, 2016.

[9] T. Lee, M. Leoky, and N. H. McClamroch, "**Geometric tracking control of a quadrotor UAV on SE (3)**", in *Proc. 49th IEEE Conf. Decis. Control*. IEEE, 2010, pp. 5420–5425.

[10] Daniel Mellinger, Alex Kushleyev and Vijay Kumar,"**mixed-integer quadratic program trajectory generation for heterogeneous quadrotor teams**", IEEE, 2012