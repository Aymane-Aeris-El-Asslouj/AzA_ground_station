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

