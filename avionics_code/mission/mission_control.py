from avionics_code.path import path_functions as p_f
from avionics_code.helpers import global_variables as g_v

class MissionControl:
    """stores computed path"""

    def __init__(self):

        # current path to be sent/that was sent to the plane
        self.current_path = None
        self.straight_2d_paths_list = None
        self.curved_2d_paths_list = None

    def compute_path(self):
        """Computes path to be sent to the plane
        based on the mission state"""

        print("\nComputing path...")

        # get info for path computation
        plane_pos = g_v.th.last_flight_profile().plane_obj.pos
        waypoint_list = g_v.ms.waypoint_list
        profile = g_v.mp

        # Check if there are no points to fly to
        if len(waypoint_list) == 0:
            print("Error, no waypoints to fly to")
            print("Called ending mission")
            g_v.rf.end_mission()
            return

        # compute 2d path list
        self.straight_2d_paths_list = p_f.straight_2d_path_finder(plane_pos, waypoint_list, profile)

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

        print("Path computed.")


