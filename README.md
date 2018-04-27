# FCND-Term1-P3-3D-Quadrotor-Controller
Udacity Flying Car Nanodegree - Term 1 - Project 3 - 3D Quadrotor Controller

### 1. Project Overview
The goal of this project is to design and build a cascade PID controller, which will control the quadrotors to fly the desired trajectory in the 3D environment. 

The project is in two parts. First, We using the Python/Unity simulator environment for rapid prototyping of the desired controller architecture. 
Once it meets the requirements for a working controller, then we migrate that control architecture to a C++ project containing real vehicle code.
Finally, We test the C++ code controller with another simulator that udacity provided, provides a higher fidelity model of the real vehicle this code would be running on.

#### 1.1 3D Control Architecture

![ Cascade Control Architecture](./images/3d-control-arch.png)

The 3D Control Architecture Diagram shows a cascade PID control system with 5 controllers, 4 actuators (u1 to u4) acting on cascade control loops. A feed-forward approach was used in the system, which adapts the motion parameters sent to the vehicle controller. For the theory behind the controller design, Please read sections 3 and 4 of this paper [Feed-Forward Parameter Identification for Precise Periodic Quadrocopter Motions](http://www.dynsyslab.org/wp-content/papercite-data/pdf/schoellig-acc12.pdf).The author of this paper is our instructor, [Angela P. Schoellig](http://www.dynsyslab.org/prof-angela-schoellig/)


#### Trajectory Breakdown
On the left of the diagram, The trajectory gets split into three paths, Path 1: Altitude or z part, Path 2: Lateral Position or x and y part,  Path 3: Yaw. The z trajectory is handled by the altitude controller, which generate the collective thrust (u1). The x and y trajectory is first handled by the lateral position controller, which outputs acceleration targets of x and y directions. Finally, the yaw trajectory is handled by the yaw controller.

#### Detils of Each path
Path 1: Altitude 
The altitude controller is responsible for ensuring the vehicle stays close to target position and velocity by computing a target thrust value (u1).
The inputs to the altitude controller including everything related to the target and actual altitude, plus the currently estimated attitude. The output of the altitude controller gets send to the roll-pitch controller, because the current commanded thrust is going to be shared in the x,y and z directions, and the portion that points in the x and y will determine acceleration in those directions. The other input of the roll-pitch controller comes from the lateral position controller.

Path 2: Lateral Position
The lateral position controller is just a PD controller in the 2D, It generates an acceleration command in the x and y directions, which is sent to the roll-pitch controller. The complicated angular control logic in the roll-pitch controller. This roll-pitch controller is the most interesting of all of them. its job is to take a thrust command as well as the desired x and y accelerations and attitude pitch, roll, yaw and p, q, r.
and output a target roll and pitch rate. These commanded p and q values are sent to the body rate controller. The body rate controller is just the P controller that convert p, q and r command into three rotational moment commands u2, u3, and u4. The r commands come from the yaw controller.

Path 3: Yaw
The yaw controller is controlled by the reactive moment command and that command only affects yaw.


### 2. Project Rubric 

#### 2.1 Implemented Controller 

##### 2.1.1 Implement body rate control in python and C++

The body rate control is a P controller on body rates to commanded moments. Steps of body rate control as follows,
```python
        1. compute current body rate error in all direction (r, p, and q)
        2. compute angular acceleration 
        3. generate the rotational moment
        4. constrain the desired moment within a set of bounds (-MAX_TORQUE, MAX_TORQUE)
```

- python: lines 165 to 178 in controller.py
- C++: lines 97 to 117 in QuadControl.cpp
  
##### 2.1.2  Implement roll pitch control 
The roll-pitch control is also a P controller in the body frame, Which is to take a thrust command as well as the desired x and y accelerations and attitude pitch, roll, yaw and p, q, r. and output a target roll and pitch rate.
Steps of roll-pitch control as follows,

```python
        1. get collective acceleration: c = -thrust_cmd / DRONE_MASS
        2. actual portion of acceleration on x and y direction from rotation matrix
        3. target portion of acceleration on x, y and z direction: b_c = acceleration_cmd / c
        4. compute current position error of b term:  b_err = b_c - b
        5. compute target change rate of b term: b_dot_c = self.k_p_euler_angles[:2][::-1] * b_err
        6. r = np.array([[R[1, 0], -R[0, 0]],
                        [R[1, 1], -R[0, 1]]],
                        dtype=np.float)
        7. generate the target roll and pitch rate with matrix multiplication: pq_c = np.dot(r, b_dot_c) / R[2, 2]

```
- python: lines 138 to 164 in controller.py
- C++: lines 125 to 168 in QuadControl.cpp

##### 2.1.3 Implement altitude control 
Steps of altitude control as follows,
```python
        1. get actual b term on z-direction from a rotation matrix
        2. compute current position error in z-direction
        3. compute current vertical error in z-direction
        4. compute target acceleration in z-direction
        5. generate the thrust command 
        6. constrain the thrust command within a set of bounds (0.1, MAX_THRUST)
```
- python: lines 113 to 136 in controller.py
- C++: lines 170 to 207 in QuadControl.cpp

##### 2.1.4 Implement lateral position control
Steps of lateral position control as follows,
```python
        1. compute current position error in all direction in world frame
        2. compute current velocity error in all direction in world frame	
        3. generate the target acceleration command 
        4. constrain the target acceleration within a set of bounds (-maxAccelXY, maxAccelXY)
```
- python: lines 94 to 110 in controller.py
- C++: lines 210 to 251 in QuadControl.cpp

##### 2.1.5 Implement yaw control 
Steps of yaw control as follows,
```python
        1. compute current yaw error
        2. generate the target yaw rate command
```
- python: lines 181 to 198 in controller.py
- C++: lines 255 to 274 in QuadControl.cpp


#### 2.2 Flight Evaluation
The quadrotors controlled by the trajectory following controller successfully fly around the desired 3D trajectory within the desired limits in both python and C++ simulator environment.

##### 2.2.1 python result
![python result](./images/python_result.png) 

![horizontal_err](./images/horizontal_err.png)  ![vertical_err](./images/vertical_err.png)

##### 2.2.2 Flight In C++

##### Scenario 2
![scenario2](./images/scenario2.png)  
##### Scenario 3
![scenario3](./images/scenario3.png)  
##### Scenario 4    
![scenario4](./images/scenario4.png)  
##### Scenario 5
![scenario5](./images/scenario5.png)  
