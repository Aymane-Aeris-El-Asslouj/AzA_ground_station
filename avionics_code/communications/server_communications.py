from avionics_code.references import global_variables as g_v

import json


class ServerComs:

    def __init__(self):
        self.connection_status = g_v.StandardStatus.NONE
        self.retrieval_status = g_v.StandardStatus.NONE

    def connect(self):
        """Connects to competition server"""

        self.connection_status = g_v.StandardStatus.STARTED

        self.connection_status = g_v.StandardStatus.SUCCESS

    def get_mission(self):
        """Gets mission info as a json file from the server
        and turns it into a mission profile object"""

        self.retrieval_status = g_v.StandardStatus.STARTED

        with open("extra files/data.json") as f:
            data = json.load(f)
            g_v.mp.from_json(data)
            g_v.gui.to_draw("mission profile")

        self.retrieval_status = g_v.StandardStatus.SUCCESS

    def upload_telemetry(self, flight_profile):
        #print("\nfake telemetry upload")
        pass
