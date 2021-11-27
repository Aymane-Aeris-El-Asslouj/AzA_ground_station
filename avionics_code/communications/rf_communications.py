from avionics_code.helpers import global_variables as g_v, geography_functions as gg_f
from avionics_code.helpers import parameters as para
from avionics_code.communications import generate_plan as g_p

import math
import time
import asyncio
import mavsdk
import threading
from mavsdk import geofence, mission_raw

FEET_PER_METER = para.FEET_PER_METER
MAP_REF = para.MAP_REF
RF_TELEMETRY_RATE = para.RF_TELEMETRY_RATE
RF_TELEMETRY_RATE_SAMPLE = para.RF_TELEMETRY_RATE_SAMPLE
TAKEOFF_COORDINATES = para.TAKEOFF_COORDINATES
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES
LANDING_COORDINATES = para.LANDING_COORDINATES


class RFComs:

    def __init__(self, event_loop):
        self.pixhawk = None
        self.connection_status = 0
        self.parameters_status = 0
        self.geofence_status = 0
        self.download_status = 0
        self.mission_upload_status = 0
        self.mission_status = 0
        self.close_request = False
        self.event_loop = event_loop

    async def connect(self):
        """Connects to plane"""

        self.connection_status = 1
        g_v.gui.to_draw["system status"] = True

        self.pixhawk = mavsdk.System()
        await self.pixhawk.connect(system_address="udp://:14540")

        async for state in self.pixhawk.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered!")
                break

        self.connection_status = 2
        g_v.gui.to_draw["system status"] = True

    async def set_vehicle_parameters(self):
        """sets all vehicle parameters after connecting/reconnecting"""

        self.parameters_status = 1
        g_v.gui.to_draw["system status"] = True

        # set telemetry rate
        await self.pixhawk.telemetry.set_rate_position(RF_TELEMETRY_RATE)
        await self.pixhawk.telemetry.set_rate_velocity_ned(RF_TELEMETRY_RATE)
        await self.pixhawk.telemetry.set_rate_attitude(RF_TELEMETRY_RATE)
        await self.pixhawk.telemetry.set_rate_in_air(RF_TELEMETRY_RATE)
        await self.pixhawk.telemetry.set_rate_landed_state(RF_TELEMETRY_RATE)

        self.parameters_status = 2
        g_v.gui.to_draw["system status"] = True

    async def upload_geofence(self):
        """Upload geofence to plane"""

        self.geofence_status = 1
        g_v.gui.to_draw["system status"] = True

        # delete any previous geofence
        await self.pixhawk.geofence.clear_geofence()

        # Define geofence boundary
        vertices_pos = [gg_f.cartesian_to_geo(vertex.pos) for vertex in g_v.mp.border.vertices]
        vertices_pos = [geofence.Point(point[0], point[1]) for point in vertices_pos]

        # Create a polygon object using vertex points
        polygon = geofence.Polygon(vertices_pos, geofence.Polygon.FenceType.INCLUSION)

        # Uploading the geofence
        await self.pixhawk.geofence.upload_geofence([polygon])

        self.geofence_status = 2
        g_v.gui.to_draw["system status"] = True

    async def subscribe_to_telemetry(self):
        """Gets position and velocity/attitude of plane"""

        tele = self.pixhawk.telemetry
        miss = self.pixhawk.mission_raw

        # shortcut for calling subscribe to data
        def sub(a, b, c):
            asyncio.ensure_future(self.subscribe_to_data(a, b, c))

        # subscribe to plane position
        sub(self.pixhawk.core.connection_state, g_v.th.connection, self.connection_converter)

        # subscribe to plane position
        sub(tele.position, g_v.th.position, self.position_converter)

        # subscribe to plane velocity
        sub(tele.velocity_ned, g_v.th.velocity, self.velocity_converter)

        # subscribe to plane heading
        sub(tele.heading, g_v.th.heading, self.heading_converter)

        # subscribe to armed state
        sub(tele.armed, g_v.th.armed, self.armed_converter)

        # subscribe to in air state
        sub(tele.in_air, g_v.th.in_air, self.in_air_converter)

        # subscribe to landed state
        sub(tele.landed_state, g_v.th.landed, self.landed_converter)

        # subscribe to flight_mode state
        sub(tele.flight_mode, g_v.th.flight_mode, self.flight_mode_converter)

        # subscribe to status text
        sub(tele.status_text, g_v.th.status_text, self.status_text_converter)

        # subscribe to mission progress
        sub(tele.status_text, g_v.th.status_text, self.status_text_converter)

        # subscribe to mission progress
        sub(miss.mission_progress, g_v.th.mission_progress, self.progress_converter)

        asyncio.ensure_future(self.test_data(self.pixhawk.mission.mission_progress))

    @staticmethod
    async def test_data(tele_in):
        """for testing telemetry output"""

        i = 0
        t = time.time()
        frequencies = []
        # subscribe to telemetry data input
        async for data in tele_in():
            i += 1
            delta = time.time() - t
            frequencies.append(1 / delta)
            avg = sum(frequencies) / len(frequencies)
            print(f'{tele_in.__name__} pack {i}, frequency: {1 / delta}, avg: {avg}')
            t = time.time()

            print(data)

    async def subscribe_to_data(self, tele_in, tele_out, tele_converter):
        """subscribes to a telemetry data inputs and sends
        the received data through a converter before going
        to the telemetry history"""

        t = time.time()
        telemtry_rates = []

        # subscribe to telemetry data input
        try:
            async for data in tele_in():

                if self.close_request:
                    break
                new_telemetry_rate = 1/(time.time()-t)
                telemtry_rates.append(new_telemetry_rate)
                if len(telemtry_rates) > RF_TELEMETRY_RATE_SAMPLE:
                    del telemtry_rates[0]
                telemetry_rate = sum(telemtry_rates)/len(telemtry_rates)
                t = time.time()

                # convert the telemetry data based on the converter
                data = tele_converter(data)

                # get telemetry time
                telemetry_time = time.time()

                # send data to telemetry history
                tele_out.set_data(telemetry_time, telemetry_rate, data)
        except Exception as err:
            print(type(err))

    @staticmethod
    def connection_converter(data):
        """converts connection telemetry data to the desired format"""

        return {"connection state": data.is_connected}

    @staticmethod
    def position_converter(data):
        """converts position telemetry data to the desired format"""

        # make geographic conversions for data
        gps = {'latitude': data.latitude_deg, 'longitude': data.longitude_deg}
        plane_pos = gg_f.geographic_to_cartesian_center(gps)
        plane_z = data.absolute_altitude_m * FEET_PER_METER

        # create dummy plane object
        class FlightObj:
            pass
        plane_obj = FlightObj()
        plane_obj.pos = plane_pos
        plane_obj.z = plane_z

        return {"flight object": plane_obj}

    @staticmethod
    def velocity_converter(data):
        """converts velocity telemetry data to the desired format"""

        # create dummy plane object
        class FlightObj:
            pass

        v_x = data.east_m_s * FEET_PER_METER
        v_y = data.north_m_s * FEET_PER_METER
        v_z = data.down_m_s * FEET_PER_METER

        plane_obj = FlightObj()
        plane_obj.vel = (v_x, v_y)
        plane_obj.v_z = v_z
        plane_obj.v = math.hypot(v_x, v_y)
        if round(plane_obj.v) == 0:
            plane_obj.v_ang = None
        else:
            plane_obj.v_ang = math.atan2(v_y, v_x)

        return {"flight object": plane_obj}

    @staticmethod
    def heading_converter(data):
        """converts attitude telemetry data to the desired format"""

        return {"heading": data.heading_deg}

    @staticmethod
    def armed_converter(data):
        """converts armed telemetry data to the desired format"""

        return {"armed": data}

    @staticmethod
    def in_air_converter(data):
        """converts in air telemetry data to the desired format"""

        return {"in air": data}

    @staticmethod
    def landed_converter(data):
        """converts landed state telemetry data to the desired format"""

        return {"landed": data}

    @staticmethod
    def flight_mode_converter(data):
        """converts flight mode telemetry data to the desired format"""

        return {"flight mode": data}

    @staticmethod
    def status_text_converter(data):
        """converts status text telemetry data to the desired format"""

        return {"type": data.type, "text": data.text}

    @staticmethod
    def progress_converter(data):
        """converts mission progress telemetry data to the desired format"""

        return {"current": data.current, "total": data.total}






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

    def launch_download_mission(self):
        """start download mission for updating the mission state"""

        # check that an exportable path has been found and can be uploaded
        if self.download_status == 1:
            g_v.gui.display_message("Can't download mission,", "mission is being downloaded", 0)
        else:
            self.event_loop.create_task(self.download_mission())

    async def download_mission(self):
        """download mission from pixhawk to update the mission state"""

        self.download_status = 1
        g_v.gui.to_draw["system status"] = True

        # check if connection data is received
        if not g_v.th.connection.data_received():
            self.download_status = 0
            g_v.gui.display_message("can't download mission,", "connection is not established", 0)

        # check if connection is active
        elif not g_v.th.connection.data["connection state"]:
            self.download_status = 0
            g_v.gui.display_message("can't download mission,", "connection was lost", 0)

        else:
            # download mission
            try:
                previous_mission = await self.pixhawk.mission_raw.download_mission()
                for mission_item in previous_mission:
                    print(mission_item)
            except mavsdk.system.mission_raw.MissionRawError as err:
                error = str(err._result.result)
                self.download_status = 0
                g_v.gui.display_message("can't download mission,", error, 0)
            else:
                self.download_status = 2
                g_v.gui.to_draw["system status"] = True

    def launch_upload_mission(self):
        """start async mission uploading"""

        # check that an exportable path has been found and can be uploaded
        if g_v.mc.path_computation_status == 0:
            g_v.gui.display_message("Can't upload mission,", "path computation not started", 0)
        elif g_v.mc.path_computation_status == 1:
            g_v.gui.display_message("Can't upload mission,", "path is being computed", 0)
        elif self.mission_upload_status == 1:
            g_v.gui.display_message("Can't upload mission,", "mission is being uploaded", 0)
        else:
            self.event_loop.create_task(self.upload_mission())

    async def upload_mission(self):
        """upload mission to pixhawk"""

        self.mission_upload_status = 1
        g_v.gui.to_draw["system status"] = True

        # check if connection data is received
        if not g_v.th.connection.data_received():
            self.mission_upload_status = 0
            g_v.gui.display_message("can't upload mission,", "connection is not established", 0)

        # check if connection is active
        elif not g_v.th.connection.data["connection state"]:
            self.mission_upload_status = 0
            g_v.gui.display_message("can't upload mission,", "connection was lost", 0)

        else:
            # clear previous mission
            try:
                await self.pixhawk.mission_raw.clear_mission()
            except mavsdk.system.mission_raw.MissionRawError as err:
                error = str(err._result.result)
                self.mission_status = 0
                g_v.gui.display_message("can't clear mission,", error, 0)
            else:
                # create new mission
                mission_items = list()
                MissionItem = mission_raw.MissionItem
                index = 0

                # add takeoff item
                lat, lon = TAKEOFF_COORDINATES["latitude"], TAKEOFF_COORDINATES["longitude"]
                alt = TAKEOFF_COORDINATES["altitude"]/FEET_PER_METER
                mission_items.append(MissionItem(index, 5, 22, 1, 1, 0.0, 0.0, 0.0, 0, lat, lon, alt, 0))
                index += 1

                # add mission waypoints
                for waypoint in g_v.mc.exportable_chosen_path.waypoint_list:
                    lat, lon = gg_f.cartesian_to_geo(waypoint.pos)
                    # scale lat lon for the used global frame 5
                    lat, lon = int(lat*(10**7)), int(lon*(10**7))
                    alt = waypoint.z / FEET_PER_METER
                    mission_items.append(MissionItem(index, 5, 16, 0, 1, 1.0, 2, 0, 0, lat, lon, alt, 0))
                    index += 1

                """add landing pattern"""
                # landing start flair
                alt = LANDING_LOITER_COORDINATES["altitude"] / FEET_PER_METER
                mission_items.append(MissionItem(index, 2, 189,  0, 1, 0, 0, 0, 0, 0, 0, alt, 0))
                index += 1
                # landing loiter
                lat, lon = LANDING_LOITER_COORDINATES["latitude"], LANDING_LOITER_COORDINATES["longitude"]
                alt = LANDING_LOITER_COORDINATES["altitude"]/FEET_PER_METER
                mission_items.append(MissionItem(index, 5, 31, 0, 1, 1, -75.0, 0, 1, lat, lon, alt, 0))
                index += 1
                # landing position
                lat, lon = LANDING_COORDINATES["latitude"], LANDING_COORDINATES["longitude"]
                alt = LANDING_COORDINATES["altitude"]/FEET_PER_METER
                mission_items.append(MissionItem(index, 5, 21, 0,  1, 0, 0, 0, 0, lat, lon, alt, 0))
                index += 1

                # upload mission
                try:
                    await self.pixhawk.mission_raw.upload_mission(mission_items)
                except mavsdk.system.mission_raw.MissionRawError as err:
                    error = str(err._result.result)
                    self.mission_status = 0
                    g_v.gui.display_message("can't upload mission,", error, 0)

                self.mission_upload_status = 2
                g_v.gui.to_draw["system status"] = True

    def launch_start_mission(self):
        """start async mission starting"""

        # check that a mission has been uploaded and is not being started
        m_u_s = self.mission_upload_status
        m_s = self.mission_status
        if m_u_s != 2:
            g_v.gui.display_message("Can't start mission,", "mission not uploaded yet", 0)
        elif m_s == 1:
            g_v.gui.display_message("Can't start mission,", "vehicle is being armed", 0)
        elif m_s == 2:
            g_v.gui.display_message("Can't start mission,", "mission is being started", 0)
        elif m_s == 4:
            g_v.gui.display_message("Can't start mission,", "mission is paused", 0)
        else:
            self.event_loop.create_task(self.start_mission())

    async def start_mission(self):
        """start flight mission"""

        self.mission_status = 1
        g_v.gui.to_draw["system status"] = True

        # check if arming data is received
        if not g_v.th.armed.data_received():
            self.mission_status = 0
            g_v.gui.display_message("can't start mission,", "telemetry is not received", 0)

        # check if arming data is up to date
        elif not g_v.th.armed.data_is_up_to_date():
            self.mission_status = 0
            g_v.gui.display_message("can't start mission,", "telemetry is not up to date", 0)

        else:
            # arm vehicle if not armed already
            try:
                if not g_v.th.armed.data["armed"]:
                    await self.pixhawk.action.arm()
            except mavsdk.system.action.ActionError as err:
                error = str(err._result.result)
                self.mission_status = 0
                g_v.gui.display_message("can't arm vehicle,", error, 0)
            else:
                self.mission_status = 2
                g_v.gui.to_draw["system status"] = True

                # start mission
                try:
                    await self.pixhawk.mission_raw.start_mission()
                except mavsdk.system.mission_raw.MissionRawError as err:
                    error = str(err._result.result)
                    self.mission_status = 0
                    g_v.gui.display_message("can't start mission,", error, 0)
                else:
                    self.mission_status = 3
                    g_v.gui.to_draw["system status"] = True

    def launch_pause_mission(self):
        """start async mission stopping"""

        m_s = self.mission_status
        if m_s == 1:
            g_v.gui.display_message("Can't pause mission,", "vehicle is being armed", 0)
        elif m_s == 2:
            g_v.gui.display_message("Can't pause mission,", "mission is being started", 0)
        elif m_s == 4:
            g_v.gui.display_message("Can't pause mission,", "mission is paused", 0)
        else:
            self.event_loop.create_task(self.pause_mission())
        
    async def pause_mission(self):
        """pause flight mission"""

        self.mission_status = 4
        g_v.gui.to_draw["system status"] = True

        # check if flight_mode data is received
        if not g_v.th.flight_mode.data_received():
            self.mission_status = 0
            g_v.gui.display_message("can't pause mission,", "telemetry is not received", 0)

        # check if flight_mode data is up to date
        elif not g_v.th.flight_mode.data_is_up_to_date():
            self.mission_status = 0
            g_v.gui.display_message("can't pause mission,", "telemetry is not up to date", 0)

        else:
            # pause mission
            try:
                if g_v.th.flight_mode.data["flight mode"].value == 4:
                    await self.pixhawk.mission_raw.pause_mission()
                else:
                    self.mission_status = 0
                    g_v.gui.display_message("can't pause mission,", "vehicle is not in mission mode", 0)
            except mavsdk.system.mission_raw.MissionRawError as err:
                error = str(err._result.result)
                self.mission_status = 0
                g_v.gui.display_message("can't pause mission,", error, 0)
            else:
                self.mission_status = 5
                g_v.gui.to_draw["system status"] = True

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
