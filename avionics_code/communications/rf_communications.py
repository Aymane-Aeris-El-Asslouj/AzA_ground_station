from avionics_code.helpers import geography_functions as gg_f, global_variables as g_v

import json
import time

class RFComs:

    def __init__(self):
        pass

    def connect(self):
        """Connects to plane"""

        print("fake connection to plane")

    def fetch_plane_status(self):
        """Gets position and velocity/attitude of plane"""

        print("fake get plane status (loads local plane.json)")

        with open("extra files/plane.json") as f:
            data = json.load(f)

        plane_pos = gg_f.geographic_to_cartesian_center(data["plane GPS"])
        ugv_pos = gg_f.geographic_to_cartesian_center(data["ugv GPS"])
        time_int = time.time()

        flight_profile = g_v.th.add_flight_profile(plane_pos, ugv_pos, time_int)
        g_v.sc.upload_telemetry(flight_profile)
