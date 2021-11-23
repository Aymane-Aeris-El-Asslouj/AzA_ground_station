from avionics_code.helpers import global_variables as g_v, geography_functions as gg_f
from avionics_code.helpers import parameters as para
from avionics_code.communications import generate_plan as g_p, export_plan as e_p

import json
import time

import asyncio

MAP_REF = para.MAP_REF

class RFComs:

    def __init__(self):
        pass

    def connect(self):
        """Connects to plane"""

        print("\nfake connection to plane")

    def fetch_plane_status(self):
        """Gets position and velocity/attitude of plane"""

        print("\nfake get plane status (loads local plane.json)")

        with open("extra files/plane.json") as f:
            data = json.load(f)

        plane_pos = gg_f.geographic_to_cartesian_center(data["plane GPS"])
        plane_z = data["plane GPS"]["Altitude"]
        ugv_pos = gg_f.geographic_to_cartesian_center(data["ugv GPS"])
        ugv_z = data["ugv GPS"]["Altitude"]
        time_int = time.time()

        flight_profile = g_v.th.add_flight_profile(plane_pos, plane_z, ugv_pos, ugv_z, time_int)
        g_v.sc.upload_telemetry(flight_profile)

    def end_mission(self):
        """sends plane in emergency landing mode"""

        print("\nfake ending mission: closing system or emergency landing")

    def authorize_airdrop(self):
        """authorizes the airdrop"""

        print("\nfake airdrop authorization")

    def drop_ugv(self):
        """drops ugv"""

        print("\nfake ugv drop")

    def take_picture(self):
        """takes picture"""

        print("\nfake picture")

    def take_off_axis_picture(self):
        """takes off axis picture"""

        print("\nfake off axis picture")

    def export_path(self, path):
        """makes path into plan and exports it to the pixhawk"""

        print("\nExporting path")
        path = path
        border = g_v.mp.border
        center = MAP_REF

        boundary = [vertex.pos for vertex in border.vertices]
        waypoints = [(way.pos, way.z) for way in path.waypoint_list]

        g_p.generate_plan(boundary, waypoints, center)

        print("Path exported.")
