import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, get_lat_lon, prune_path, global_to_local, a_star_graph, \
    create_grid_and_edges, closest_point, plot_grid
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
import numpy.linalg as LA
import networkx as nx



class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, global_goal_position=np.array([40.0, 40.0, 3.0])):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        self._latitude = 37.7849088
        self._longitude = -122.4005945
        self._altitude = 0

        # initial state
        self.flight_state = States.MANUAL
        self.global_goal_position = global_goal_position

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        colliders_file = 'colliders.csv'
        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = get_lat_lon(colliders_file)
        print(f'Home lat : {lat0}, lon : {lon0}')
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)


        # TODO: retrieve current global position
        print('Current global position', self.global_position)
        # TODO: convert to current local position using global_to_local()
        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)
        print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        #grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print('Pre-create Grid and Edges')
        # This is now the routine using Voronoi
        grid, edges, north_offset, east_offset = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print('Post-create Grid and Edges')
        # TODO: create the graph with the weight of the edges
        # set to the Euclidean distance between the points
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = LA.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)
        print('Post-create Graph', G.number_of_nodes(), G.number_of_edges(), G.size())
        #start_ne = (25, 100)
        #goal_ne = (750., 300.)

        #start_ne_g = closest_point(G, start_ne)
        #goal_ne_g = closest_point(G, goal_ne)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start_north = int(np.ceil(local_north - north_offset))
        grid_start_east = int(np.ceil(local_east - east_offset))
        grid_start = (grid_start_north, grid_start_east)
        graph_start = closest_point(G, grid_start)

        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert
        local_goal_north, local_goal_east, local_goal_down = global_to_local(self.global_goal_position,
                                                                             self.global_home)
        grid_goal_north = int(np.ceil(local_goal_north - north_offset))
        grid_goal_east = int(np.ceil(local_goal_east - east_offset))
        grid_goal = (grid_goal_north, grid_goal_east)
        graph_goal = closest_point(G, grid_goal)

        plot_grid(edges, grid, grid_start, grid_goal)


        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Local Start and Goal: ', graph_start, graph_goal)
        path, path_cost = a_star_graph(G, heuristic, graph_start, graph_goal)
        print('Post a-Star search')
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        pruned_path = prune_path(path)
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    #parser.add_argument('--global_goal_lon', type=str, default='-122.4165994', help="Goal position longitude")
    #parser.add_argument('--global_goal_lat', type=str, default='37.8091669', help="Goal position latitude")
    parser.add_argument('--global_goal_lon', type=str, default='-122.4005945', help="Goal position longitude")
    parser.add_argument('--global_goal_lat', type=str, default='37.7850088', help="Goal position latitude")

    parser.add_argument('--global_goal_alt', type=str, default='0', help="Goal position altitude")
    args = parser.parse_args()
    print(args)
    global_goal_position = np.fromstring(f'{args.global_goal_lon},{args.global_goal_lat},{args.global_goal_alt}',
                                         dtype='Float64', sep=',')

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60000)
    drone = MotionPlanning(conn, global_goal_position=global_goal_position)
    time.sleep(1)
    drone.start()


if __name__ == "__main__":
    main()
