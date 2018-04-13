# FCND - 3D Motion Planning

### 1. Project Overview
Goal of this project is to integrate the techniques that we have learned to create a planning solution, Which plan and generate safe and smooth trajectories for a simulated drone through an urban environment in a simulator. 

#### 1.1 The 2.5D map of the urban environment is in colliders.csv
The colliders.csv file that we used is a 2.5D grid representation showing downtown San Francisco at roughly one meter resolution. We read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position()).  Starting from the third row is the obstacles data in the map, Position of each obstacle is represented by discrete X,Y,Z coordinates, Size of each obstacle is represented by halfSizeX,halfSizeY,halfSizeZ.

[2.5D map](https://... FIXME)

<!-- # Final Result
Video (https://youtu.be/xWD0j_8Z6gg  FIXME) With this path planner, our drone successfully plan a path through an urban environment and fly around the 2D flight path.

![pathplanner5](https://user-images.githubusercontent.com/24623272/29002135-5933af78-7ace-11e7-8e9a-8fee53692b5f.png FIXME)  -->


### 2. Project Rubric

#### 2.1. Explain the Starter Code
motion_planning.py is a evolutionary version of backyard_flyer_solution.py for simple path planning. A new feature in motion_planning.py is automated path planning with a path planner, While in  backyard_flyer_solution.py, we create a simple square shaped flight path manually.  

The following are the modifications between motion_planning.py and backyard_flyer_solution.py. 

- Add a new state (PLANNING) in the finite-state machine, between ARMING and TAKEOFF. 
- Add a new method plan_path. When the drone is at the state ARMING and it is actually armed (line 66), the transition to the PLANNING state is executed on the method plan_path.
- The method plan_path define a motion planning pipeline. first, read in the hardcoded global home ([-122.397438, 37.7924766, 0.]) and obstacle map from colliders.csv file (line 130 to 133), second, create a grid representation for the drone's environment with the method create_grid (line 136), which defined  in the planning_utils.py file. third, define start (this is just grid center) and goal point in on the grid (line 139 to 143). fourth, Run the path planner (A* Algorithms) to find a path from start to goal
(line 150). Simply convert path to waypoints. finally, send the waypoints to simulator.


#### 2.2 Implementing of the Path Planning Algorithm 

##### 2.2.1 Read and set the global home location

The snipped below (line FIXME in motion_planning.py) to do read in and set the global home location from the first line of the colliders.csv.

```python
filename = "colliders.csv"
lon0, lat0 = read_global_home(filename)
self.set_home_position(lon0, lat0, 0)        
```        

##### 2.2.2 From global position to local position 

The snipped below (line FIXME in motion_planning.py), first, retrieve current global position.
then, convert from global position to local position.

```python
current_global_position = [self._longitude, self._latitude, self._altitude]
self._north, self._east, self._down =  global_to_local(current_global_position, self.global_home)
``` 

##### 2.2.3 Set start point to the current local position

```python
# Read in obstacle map
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)        

# Define a grid for a particular altitude and safety margin around obstacles
grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)        
print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

# convert start position to current position rather than map center
# Set goal as some arbitrary position on the grid
# Define starting point on the grid
grid_start = (int(np.ceil(-north_offset+self.local_position[0])), int(np.ceil(-east_offset+self.local_position[1])))        

``` 

##### 2.2.4 Set goal point to arbitrary location on the grid
```python
grid_goal = (-north_offset + 400, -east_offset + 400)
```

##### 2.2.5 Find a path with A* algorithm

- Add some new action in class Action in planning_utils.py(line 63 to 66)

```python
SOUTH_WEST =(-1,-1, np.sqrt(2))
NORTH_WEST =(1,-1, np.sqrt(2))
SOUTH_EAST =(-1, 1, np.sqrt(2))
NORTH_EAST =(1, 1, np.sqrt(2)) 
```
- Add some check in the valid_actions methon in planning_utils.py (line 97 to 104)

```python
    if y - 1 < 0 or x - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if y - 1 < 0 or x + 1 > n or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if y + 1 > m or x + 1 > n or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if y + 1 > m or x - 1 < 0 or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
```

- Run the A* Algorithms to find a path from start to goal (line 150 in motion_planning.py)

```python
#Compute the lowest cost path with a_star
path, _ = a_star(grid, heuristic, grid_start, grid_goal)
```

##### 2.2.6 Prune path with collinearity

- Using collinearity here to prune path (line FIXME in planning_utils.py)
```python
def prune_path(path):
    pruned_path = [p for p in path]
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i+1])
        p3 = point(pruned_path[i+2])

        if collinearity_check(p1, p2, p3):
            pruned_path.remove(pruned_path[i+1])
        else:
            i += 1
    return pruned_path    
```

### 3. Executing the flight

#### 3.1 Test1: Try a different goal location

#### 3.2 Test2： Try a different start point


