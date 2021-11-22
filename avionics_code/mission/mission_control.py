from avionics_code.path import path_functions as p_f, path_objects as p_o
from avionics_code.flight import flight_profile as f_p
from avionics_code.helpers import global_variables as g_v, geometrical_functions as g_f
from avionics_code.helpers import parameters as para

WAYPOINT_ACCEPTANCE_DISTANCE = para.WAYPOINT_ACCEPTANCE_DISTANCE
OBSTACLE_DISTANCE_FOR_VALID_PASS = para.OBSTACLE_DISTANCE_FOR_VALID_PASS
MISSION_TIME_LENGTH = para.MISSION_TIME_LENGTH

class MissionControl:
    """stores computed path and manipulates it"""

    def __init__(self):

        # current path to be sent/that was sent to the plane
        self.straight_2d_paths_list = None
        self.curved_2d_paths_list = None

        self.chosen_path = None

    def refresh_mission_state(self):
        """Refreshes mission state by checking if a new waypoint was reached,
        then checks if the plane path needs to be recomputed or if time ran out"""

        # update the waypoint status list and make appropriate actions
        self.update_waypoint_status_list()

        # land if time ran out
        if self.check_timeout():
            print("\nTime ran out, starting landing...")
            g_v.ms.land()
            self.compute_path()
            g_v.rf.export_path(self.chosen_path)

        # update path if there is no path or plane deviated from original path
        if self.chosen_path is None or self.check_path_deviation():
            self.compute_path()
            g_v.rf.export_path(self.chosen_path)

    def check_path_deviation(self):
        """Check if path deviated from original computed path"""

        return False

    @staticmethod
    def check_timeout():
        """checks if mission time is over"""

        return g_v.th.last_flight_profile().time_int - g_v.init_time > MISSION_TIME_LENGTH

    def update_waypoint_status_list(self):
        """checks if a waypoint from the mission state was reached"""

        # get info for path computation
        plane_obj = g_v.th.last_flight_profile().plane_obj
        plane_3d_pos = (plane_obj.pos[0], plane_obj.pos[1], plane_obj.z)
        waypoint_list = g_v.ms.waypoint_list
        next_waypoint = waypoint_list[0]
        next_waypoint_3d_pos = (next_waypoint.pos[0], next_waypoint.pos[1], next_waypoint.z)

        # Check if there are no points to fly to
        if len(waypoint_list) == 0:
            print("\nError, no waypoints to fly to")
            print("Called ending mission")
            g_v.rf.end_mission()
            return

        # check if the next waypoint was reached
        if g_f.distance_3d(plane_3d_pos, next_waypoint_3d_pos) < WAYPOINT_ACCEPTANCE_DISTANCE:
            print("\nWaypoint reached!")
            # do what needs to be done at the mission waypoint (like taking a picture)
            self.mission_action(next_waypoint.mission_index)
            del g_v.ms.waypoint_list[0]

    @staticmethod
    def mission_action(mission_index):
        """do an action when the plane reaches a new mission waypoint"""

        if mission_index == 1:
            g_v.rf.drop_ugv()
        elif mission_index == 2:
            g_v.rf.take_picture()
        elif mission_index == 3:
            g_v.rf.take_off_axis_picture()

    def compute_path(self):
        """Computes path to be sent to the plane
        based on the mission state"""

        print("\nComputing path...")

        # get info for path computation
        plane_obj = g_v.th.last_flight_profile().plane_obj
        plane_pos = plane_obj.pos
        plane_z = plane_obj.z
        waypoint_list = g_v.ms.waypoint_list
        profile = g_v.mp

        # check if the plane is inside an obstacle or too close to one
        obstacles = profile.obstacles
        O_D_V = OBSTACLE_DISTANCE_FOR_VALID_PASS
        for obstacle in obstacles:
            # delete obstacle if it is already inside
            if g_f.distance_2d(obstacle.pos, plane_pos) <= obstacle.r:
                profile.delete_obstacle(obstacles.index(obstacle))
                # represent plane as being further away if it is close to it
            elif g_f.distance_2d(obstacle.pos, plane_pos) <= obstacle.r + O_D_V:

                # get vector from obstacle center to current position and scale
                # it till it is at obstacle pass distance
                vec = g_f.sub_vectors(obstacle.pos, plane_pos)
                vec_norm = g_f.norm(vec)
                distance_from_obstacle = obstacle.r + O_D_V
                vec_scaled = g_f.scale_vector(vec, 1 - distance_from_obstacle / vec_norm)

                # get new plane position
                plane_pos = g_f.add_vectors(vec_scaled, plane_pos)

        # Check if there are no points to fly to
        if len(waypoint_list) == 0:
            print("Error, no waypoints to fly to")
            print("Called ending mission")
            g_v.rf.end_mission()
            return

        # compute 2d path list
        plane_obj = f_p.FlightObject(plane_pos, plane_z)
        self.straight_2d_paths_list = p_f.straight_2d_path_finder(plane_obj, waypoint_list, profile)

        # check if any two waypoints could not be connected
        for index, path_group in enumerate(self.straight_2d_paths_list):
            if path_group is None:
                print(f'Error: full mission waypoints {index} and {index+1} '
                      f'could not be linked with a straight line path')
                print(f'Deleting mission waypoint {index+1} from mission state and retrying')
                del g_v.ms.waypoint_list[index]
                print(len(g_v.ms.waypoint_list))
                self.compute_path()
                return

        self.curved_2d_paths_list = p_f.curving_2d(self.straight_2d_paths_list, profile)

        curved_path = self.curved_2d_paths_list[0]
        waypoint_list_new = list()
        for way in curved_path.waypoint_list:

            pre_turn_waypoint = way.pre_turn_waypoint
            if pre_turn_waypoint is not None:
                waypoint_list_new.append(pre_turn_waypoint)

            waypoint_list_new.append(way)

            post_turn_waypoint = way.post_turn_waypoint
            if post_turn_waypoint is not None:
                waypoint_list_new.append(post_turn_waypoint)

        self.chosen_path = p_o.Path(waypoint_list_new)

        print("Path computed.")
