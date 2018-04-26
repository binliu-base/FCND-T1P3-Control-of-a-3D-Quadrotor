# FCND-Term1-P3-3D-Quadrotor-Controller
Udacity Flying Car Nanodegree - Term 1 - Project 3 - 3D Quadrotor Controller

### 1. Project Overview
Goal of this project is to design and implement a trajectory following controller, which control the quadrotors  
to find the sequance of propeller turn rates so that the drone to fly the desired 3D trajectory.

The project is in two parts. First, We using the Python/Unity simulator environment for rapid prototyping of the desired controller architecture. 
Once it meet the requirements for a working controller, then we migrate that control architecture to a C++ project containing real vehicle code.
Finally, We test the C++ code controller with another simulator that udacity provided, provides a higher fidelity model of the real vehicle this code would be running on.

#### 1.1 3D Control Architecture

![ Cascade Control Architecture](./images/3d-control-arch.png)


The 3D Control Architecture Diagram shows a cascade PID control system with 5 controllers, 4 actuator (u1 to u4) acting on two loop processes in series.

#### Trajectory Overview
On the left of the diagram, The trajectory gets split into three path, Path 1: Altitude or z part, Path 2: Lateral Position or x and y part,  Path 3: Yaw. The z trajectory is handled by the altitude controller, which generate the collective thrust (u1). The x and y trajectory is first handled by the lateral position controller, which outputs acceleration targets of x and y direction. 
Finally, the yaw trajectory is handled by the yaw controller.

#### Detils of each path
Path 1: Altitude 
The altitude controller is responsible for ensuring the vehicle stays close to target position and velocity by computing a target thrust value (u1).
The inputs to the altitude controller including everything related to the target and actual altitude, plus the current estimated attitude. Output of the altitude controller gets send to the roll-pitch controller, because the current commanded thrust is going to be shared in the x,y and z directions, and the portion that points in the x and y will 
determine acceleration in those directions. The other input of the roll-pitch controller comes from the lateral position controller.

Path 2: Lateral Position
The lateral position controller is just a PD controller in the 2D, It generate an acceleration command in the x and y directions, which is send to the roll-pitch controller. The complicated angular control logic in the roll-pitch controller. This roll-pitch controller is the most interesting of all of them. it's job is to take a thrust command as well as the desired x and y accelerations and attitude pitch, roll, yaw and p, q, r.
and output a target roll and pitch rate. These commanded  p and q values are send to the body rate controller. The body rate controller is just the p controller that convert p, q and r command into three rotational moment commands u2, u3 and u4. The r commands come from the yaw controller.

Path 3: Yaw
The yaw controller is controlled through the reactive moment command and that command only affects yaw.


### 2. Project Rubric 

#### 2.1 Implemented Controller 

##### 2.1.1 Implemented Controller In Python

##### 2.1.1 Implemented Controller In C++

#### 2.2 Flight Evaluation

##### 2.2.1 Flight In Python   

##### 2.2.2 Flight In C++

##### Scenario 1
##### Scenario 2
##### Scenario 3
##### Scenario 4
##### Scenario 5





