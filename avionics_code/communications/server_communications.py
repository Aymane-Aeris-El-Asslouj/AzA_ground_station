from avionics_code.mission import mission_profile_construction as m_p_c

class ServerComs:
    def __init__(self):
        pass

    def connect(self):
        """Connects to competition server"""

        print("fake connection to server")

    def get_mission(self):
        """Gets mission info as a json file from the server
        and turns it into a mission profile object"""
        
        print("fake get mission (loads local data.json)")
        return m_p_c.json_to_mission_profile("extra files/data.json")

    def upload_telemetry(self, flight_profile):
        print("fake telemetry upload")
        pass
