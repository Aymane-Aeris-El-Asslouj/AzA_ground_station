from avionics_code.path import path_functions as p_f
from avionics_code.helpers import global_variables as g_v

class MissionControl:
    """stores computed path"""

    def __init__(self):

        # current path to be sent/that was sent to the plane
        self.current_path = None

    def compute_path(self):
        """Computes path to be sent to the plane
        based on the mission state"""

        print("\nComputing path...")

        plane_pos = g_v.th.last_flight_profile.plane_obj
        waypoint_list = g_v.ms.waypoint_list
        profile = g_v.mp
        self.current_path = p_f.straight_2d_path_finder(plane_pos, waypoint_list, profile)


