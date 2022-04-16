# Trajectory Planning



<img src="gifs/pro1/head.gif" alt="polyhedron-1" width="400">



- Path planning algorithms (Dijkstra,  Astar,  Jump point search)
- Generating Convex Polytopes(Safe Flight Corridors)
- Trajectory planning(Ax = b, QP)
- Trajectory following control(PD controller)



## Required

MATLAB(R2019b is tested)



## Pic



### 1. Path Planning

| dijkstra                                                 | Astar                                              | Jump Point Search                              |
| -------------------------------------------------------- | -------------------------------------------------- | ---------------------------------------------- |
| <img src="gifs/pro1/dijkstra.gif" alt="dijkstra" width="270"> | <img src="gifs/pro1/Astar.gif" alt="Astar" width="270"> | <img src="gifs/pro1/JPS.gif" alt="JPS" width="270"> |
| 1.642271 seconds                                         | 0.755504 seconds                                   | 0.451664 seconds                               |

 

### 2. Generating Convex Polytopes

### 	Safe Flight Corridors

- ####  find_ellipsoid

  <img src="imgs/pro1/ellipsoid-1.png" alt="polyhedron-1" width="400"><img src="imgs/pro1/ellipsoid-2.png" alt="polyhedron-2" width="400">

- #### find_polyhedron

<img src="gifs/pro1/polyhedron-1.gif" alt="polyhedron-1" width="400"><img src="gifs/pro1/polyhedron-2.gif" alt="polyhedron-2" width="400">



### 3. Trajectory planning

Compare different Trajectory planning：

- #### Trajectory 1: use Ax = b get Trajectory.

<img src="gifs/pro1/closeForm.gif" alt="closeForm" width="270"><img src="imgs/pro1/close_postion.jpg" alt="closeForm" width="270"><img src="imgs/pro1/close_velocity.jpg" alt="closeForm" width="270"> 

The Trajectory pass every Path point.

- #### Trajectory 2: use 'Quadratic Programming' get Trajectory. use corridor constraints make  Ax< b. x y z separately find minimum-snap.

<img src="gifs/pro1/corridorConstraints.gif" alt="corridorConstraints" width="270"><img src="imgs/pro1/corridorConstraints_postion.jpg" alt="corridorConstraints" width="270"><img src="imgs/pro1/corridorConstraints_velocity.jpg" alt="corridorConstraints" width="270"> 

The Trajectory don't need to pass every Path point.

minSnapValue(X + Y + Z) is : 9376.0901



- #### Trajectory 3: use 'Quadratic Programming' get Trajectory. use SFC make Ax < b.

<img src="gifs/pro1/SFC.gif" alt="SFC" width="270"><img src="imgs/pro1/SFC_postion.jpg" alt="SFC" width="270"><img src="imgs/pro1/SFC_velocity.jpg" alt="SFC" width="270"> 

The Trajectory don't need to pass every Path point.

minSnapValue(X + Y + Z) is : 2958.5877



### 4. Trajectory following control

Trajectory following by use PD Controller, you can learn by [coursera](https://www.coursera.org/learn/robotics-flight/home/welcome).



## Reference

##### Paper:

[1] D. Harabor and A. Grastien. 2011. "**Online Graph Pruning for Pathfinding on Grid Maps**". In Proceedings of the 25th National Conference on Artificial Intelligence (AAAI), San Francisco, USA.

[2] S. Liu, M. Watterson, K. Mohta, K. Sun, S. Bhattacharya, C.J. Taylor, et al., "**Planning dynamically feasible trajectories for quadrotors using safe flight corridors in 3-d complex environments**", IEEE Robotics and Automation Let- ters, vol. 2, no. 3, pp. 1688-1695, July 2017.

[3] D.W.Mellinger,"**Trajectory generation and control for quadrotors**"Ph.D. dissertation, Univ. Pennsylvania, Philadelphia, PA, 2012.

[4] D. Mellinger and V. Kumar, "**Minimum snap trajectory generation and control for quadrotors**", inProc. 2011 IEEE Int. Conf. Robot.Autom.,2011

[5] T. Lee, M. Leoky, and N. H. McClamroch, "**Geometric tracking control of a quadrotor UAV on SE (3)**" in *Proc. 49th IEEE Conf. Decis. Control*. IEEE, 2010, pp. 5420–5425.

##### Code:

safe flight corridors: [C++ version](https://github.com/sikang/DecompUtil)

