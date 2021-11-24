from avionics_code.helpers import global_variables as g_v, geography_functions as gg_f
from avionics_code.helpers import parameters as para
from avionics_code.communications import generate_plan as g_p

import aiostream
import time
import asyncio
import mavsdk

FEET_PER_METER = para.FEET_PER_METER
MAP_REF = para.MAP_REF


class RFComs:

    def __init__(self):
        self.pixhawk = None
        self.status = 0

    async def connect(self):
        """Connects to plane"""

        self.status = 1
        g_v.gui.update_system_status()
        self.pixhawk = mavsdk.System()
        await self.pixhawk.connect(system_address="udp://:14540")
        self.status = 2
        g_v.gui.update_system_status()

    async def fetch_plane_status_loop(self):
        """Gets position and velocity/attitude of plane"""

        # Start the tasks
        asyncio.ensure_future(self.get_telemetry_loop(self.pixhawk))

    @staticmethod
    async def get_telemetry_loop(pixhawk):
        i = 0
        a_mix = aiostream.stream.combine.merge(pixhawk.telemetry.position(), pixhawk.telemetry.heading())
        async with a_mix.stream() as streamer:
            mix_iter = streamer.__aiter__()
            while True:
                data_1 = await mix_iter.__anext__()
                data_2 = await mix_iter.__anext__()

                check_1 = isinstance(data_1, mavsdk.telemetry.Position)
                check_2 = isinstance(data_2, mavsdk.telemetry.Heading)
                if check_1 and check_2:
                    position = data_1
                    heading = data_2
                    i += 1
                    if i % 50 == 0:
                        gps = {'latitude': position.latitude_deg, 'longitude': position.longitude_deg}
                        plane_pos = gg_f.geographic_to_cartesian_center(gps)
                        plane_z = position.absolute_altitude_m * FEET_PER_METER
                        time_int = time.time()
                        plane_orientation = heading.heading_deg
                        p_o = plane_orientation
                        print(p_o)
                        g_v.th.add_flight_profile(plane_pos, plane_z, p_o, (0, 0), 0, 0, time_int)

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
