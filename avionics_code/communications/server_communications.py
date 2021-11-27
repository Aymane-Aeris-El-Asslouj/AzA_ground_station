from avionics_code.mission import mission_profile_construction as m_p_c
from avionics_code.helpers import global_variables as g_v


class ServerComs:

    def __init__(self):
        self.connection_status = 0

    def connect(self):
        """Connects to competition server"""

        self.connection_status = 1
        g_v.gui.to_draw["system status"] = True

        self.connection_status = 2
        g_v.gui.to_draw["system status"] = True

    def get_mission(self):
        """Gets mission info as a json file from the server
        and turns it into a mission profile object"""

        g_v.mp.retrieval_status = 1
        g_v.gui.to_draw["system status"] = True

        g_v.mp = m_p_c.json_to_mission_profile("extra files/data.json")

        g_v.mp.retrieval_status = 2
        g_v.gui.to_draw["system status"] = True

    def upload_telemetry(self, flight_profile):
        #print("\nfake telemetry upload")
        pass
