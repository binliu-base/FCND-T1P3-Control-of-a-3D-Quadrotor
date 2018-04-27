# FCND-Term1-P3-3D-Quadrotor-Controller
Udacity Flying Car Nanodegree - Term 1 - Project 3 - 3D Quadrotor Controller

### 1. Project Overview
Goal of this project is to design and build a 3D controller, which will control the quadrotors to fly the desired 3D trajectory in the simulator. 

The project is in two parts. First, We using the Python/Unity simulator environment for rapid prototyping of the desired controller architecture. 
Once it meet the requirements for a working controller, then we migrate that control architecture to a C++ project containing real vehicle code.
Finally, We test the C++ code controller with another simulator that udacity provided, provides a higher fidelity model of the real vehicle this code would be running on.

#### 1.1 3D Control Architecture

![ Cascade Control Architecture](./images/3d-control-arch.png)


The 3D Control Architecture Diagram shows a cascade PID control system with 5 controllers, 4 actuator (u1 to u4) acting on two loop processes in series.

#### Trajectory Breakdown
On the left of the diagram, The trajectory gets split into three path, Path 1: Altitude or z part, Path 2: Lateral Position or x and y part,  Path 3: Yaw. The z trajectory is handled by the altitude controller, which generate the collective thrust (u1). The x and y trajectory is first handled by the lateral position controller, which outputs acceleration targets of x and y direction. 
Finally, the yaw trajectory is handled by the yaw controller.

#### Detils of each path
Path 1: Altitude 
The altitude controller is responsible for ensuring the vehicle stays close to target position and velocity by computing a target thrust value (u1).
The inputs to the altitude controller including everything related to the target and actual altitude, plus the current estimated attitude. Output of the altitude controller gets send to the roll-pitch controller, because the current commanded thrust is going to be shared in the x,y and z directions, and the portion that points in the x and y will 
determine acceleration in those directions. The other input of the roll-pitch controller comes from the lateral position controller.

Path 2: Lateral Position
The lateral position controller is just a PD controller in the 2D, It generate an acceleration command in the x and y directions, which is send to the roll-pitch controller. The complicated angular control logic in the roll-pitch controller. This roll-pitch controller is the most interesting of all of them. it's job is to take a thrust command as well as the desired x and y accelerations and attitude pitch, roll, yaw and p, q, r.
and output a target roll and pitch rate. These commanded  p and q values are send to the body rate controller. The body rate controller is just the P controller that convert p, q and r command into three rotational moment commands u2, u3 and u4. The r commands come from the yaw controller.

Path 3: Yaw
The yaw controller is controlled through the reactive moment command and that command only affects yaw.


### 2. Project Rubric 

#### 2.1 Implemented Controller 

##### 2.1.1 Implement body rate control in python and C++

The body rate control is a P controller on body rates to commanded moments. Steps of body rate control as follows,
```python
        body_rate_error = body_rate_cmd - body_rate
        angular acceleration = Angle rate gains * body_rate_error
        rotational moment = I * angular acceleration
        rotational moment = np.clip(tau, -MAX_TORQUE, MAX_TORQUE) 
```

- python: lines 165 to 178 in controller.py
- C++: lines 97 to 117 in QuadControl.cpp

##### 2.1.2  Implement roll pitch control 
The roll-pitch control is also a P controller in the body frame, Which is to take a thrust command as well as the desired x and y accelerations and attitude pitch, roll, yaw and p, q, r. and output a target roll and pitch rate.
Steps of roll-pitch control as follows,

```python

        get collective acceleration: c = -thrust_cmd / DRONE_MASS
        get actual portion of acceleration on x and y direction from rotation matrix: b = R[0:2, 2] 
        compute target portion of acceleration on x, y and z direction: b_c = acceleration_cmd / c
        b_err = b_c - b
        b_dot_c = self.k_p_euler_angles[:2][::-1] * b_err
        r = np.array([[R[1, 0], -R[0, 0]],
                      [R[1, 1], -R[0, 1]]],
                     dtype=np.float)
        generate the target roll and pitch rate with matrix multiplication: pq_c = np.dot(r, b_dot_c) / R[2, 2]

```
- python: lines 138 to 164 in controller.py
- C++: lines 125 to 168 in QuadControl.cpp

###### Implement altitude control 
The snipped below (line 112 in controller.py) are the implemented altitude control.
```python
    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude,
                         acceleration_ff=0.0):

        b_z = euler2RM(*attitude)[2,2]
        z_err = altitude_cmd - altitude
        z_dot_err = vertical_velocity_cmd - vertical_velocity

        z_dot_dot_c = self.k_p_z * z_err + self.k_d_z * z_dot_err + acceleration_ff
        
        c = (z_dot_dot_c - GRAVITY)/b_z
        thrust = c * DRONE_MASS_KG
        thrust = np.clip(thrust, 0.1, MAX_THRUST)

        return thrust
```

###### Implement lateral position control
The snipped below (line 93 in controller.py) are the implemented lateral position control.
```python
    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                                 acceleration_ff=np.array([0.0, 0.0])):

        pos_err = local_position_cmd - local_position
        vel_err = local_velocity_cmd - local_velocity
        acc_cmd = self.k_p_posXY * pos_err + self.k_d_posXY * vel_err + acceleration_ff
        return acc_cmd
```

###### Implement yaw control 
The snipped below (line 180 in controller.py) are the implemented yaw control.
```python
    def yaw_control(self, yaw_cmd, yaw):
        # yaw must be within [-pi, pi)
        yaw_cmd = np.fmod(yaw_cmd + np.pi, 2 * np.pi) - np.pi
        yaw_err = yaw_cmd - yaw
        if np.abs(yaw_err) > np.pi:
            direction = -1 if yaw_err > 0 else 1
            yaw_err = yaw_err + direction * 2 * np.pi
        yaw_c = self.k_p_yaw * yaw_err

        return self.k_p_yaw * yaw_err
```

##### 2.1.1 Implemented Controller In C++
###### Implemented body rate control

###### Implement roll pitch control

###### Implement altitude control
###### Implement lateral position 
###### Implement yaw control
###### Implement calculating the motor commands given commanded thrust and moments in C++

#### 2.2 Flight Evaluation

##### 2.2.1 Flight In Python   

##### 2.2.2 Flight In C++

##### Scenario 1
##### Scenario 2
##### Scenario 3
##### Scenario 4
##### Scenario 5





