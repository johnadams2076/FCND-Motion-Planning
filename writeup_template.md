## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...
Different states of the drone are defined. State transitions are coded.
Path planning skeleton code has create grid method that takes in data from csv file and creates a grid with obstacles.
Grid start and Grid Goal is hardcoded. A* search algorithm is called to find a path in the grid from start point to the goal point.
Waypoints are created using the path discovered by A* algorithm and sent to the drone via Mavlink protocol.

#And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
#![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.
planning_utils.py has a method get_lat_lon(filename). This method reads the first line of the file and splits the string accordingly to read float values of latitute and longitude.
In motion_planning.py, method plan_path() was modified by adding the line self.set_home_position(lon0, lat0, 0) 


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.
In planning_utils.py, method global_to_local(global_position, global_home) was added to convert global position to local. The conversion is achieved by using utm package's 
from_latlon() method. The method takes in latitude and longitude and returns in NED format.
The co-ordinates for global home are subtracted from the current global position to get local position. 


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!
Currently, the grid start position is hardcoded to ensure A* find a path. Removing the lines
self._latitude = 37.7849088
        self._longitude = -122.4005945
        self._altitude = 0
from the constructor will facilitate picking the current location as start location. 
#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
This feat was achieved by passing global Lon , lat and alt of the goal, as command line arguments.
Global Goal co-ordinates are converted to local goal points by passing it through the global_to_local() method.
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.
In planning_utils.py, just for reference, Class ActionPlus with Enums and method valid_actions_plus are added to support diagonal motions.
However, A* implementation is replaced to accommodate graphs. Method create_grid_and_edges(data, drone_altitude, safety_distance) was added to generate collision-free Vornoi based edges.
A Networkx Graph was created based on generated edges and euclidean distance between them as associated weight.
Method a_star_graph(graph, h, start, goal) in planning_utils.py has core of the search algorithm.
Key concept here is Priority Queue. It returns item with the lowest cost and algorithm works on it.
#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.
Method prune_path(path) in planning_utils.py implements the culling of waypoints.
Method takes 3 points from path and determines its area by using np.linalg.det(m).
If area is less than a pre-determined negligible value such as epsilon=1e-6, then the middle node is removed from path.


### Execute the flight
#### 1. Does it work?
It works!
Please check. Simulator time out.
### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
Coming soon to a simulator near you!

