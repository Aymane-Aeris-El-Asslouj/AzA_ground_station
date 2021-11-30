from avionics_code.helpers import global_variables as g_v, geography_functions as gg_f
from avionics_code.helpers import parameters as para, geometrical_functions as g_f
from avionics_code.helpers.global_variables import StandardStatus as SS

import math
import time
import asyncio
import mavsdk
import grpc
from mavsdk import geofence, mission_raw

FEET_PER_METER = para.FEET_PER_METER
MAP_REF = para.MAP_REF
RF_TELEMETRY_RATE = para.RF_TELEMETRY_RATE
RF_TELEMETRY_RATE_SAMPLE = para.RF_TELEMETRY_RATE_SAMPLE
TAKEOFF_COORDINATES = para.TAKEOFF_COORDINATES
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES
LANDING_COORDINATES = para.LANDING_COORDINATES
MAX_SPEED = para.MAX_SPEED
CRUISE_SPEED = para.CRUISE_SPEED
ORBIT_VELOCITY = para.ORBIT_VELOCITY
ORBIT_RADIUS = para.ORBIT_RADIUS


class RFComs:
    
    def __init__(self, event_loop):

        # connection status
        self.lost_comms = False
        self.connection_index = 0
        self.pixhawk = None
        self.connection_status = 0
        
        # initial upload status
        self.parameters_status = SS.NONE
        self.geofence_status = SS.NONE
        
        # mission status
        self.download_status = SS.NONE
        self.upload_status = SS.NONE
        self.mission_start = SS.NONE
        self.mission_pause = SS.NONE
        self.go_to_status = SS.NONE
        
        self.close_request = False
        self.event_loop = event_loop
        self.current_plan = None
        self.current_mission_item = None

    def launch_connect(self):
        """schedules connection to be started"""

        self.event_loop.create_task(self.connect())

    async def connect(self):
        """Connects to plane"""

        # setup new connection
        self.connection_index += 1
        self.pixhawk = mavsdk.System()

        # try to connect to plane
        try:

            # get base connection
            await self.pixhawk.connect(system_address="udp://:14540")
            async for state in self.pixhawk.core.connection_state():
                if state.is_connected:
                    break

            # upload parameters
            await self.set_vehicle_parameters()

            # upload geofence
            await self.upload_geofence()

            # subscribe to telemetry
            await self.subscribe_to_telemetry()

        # if connection fails, reconnect
        except grpc._channel._MultiThreadedRendezvous:
            g_v.gui.display_message("connection attempt failed", "reconnection attempt...", 0)
            if not self.lost_comms:
                self.lost_comms = True
                await self.connect()

        # create connection subscription
        asyncio.ensure_future(self.subscribe_to_connection(self.connection_index))
        self.lost_comms = False

        g_v.gui.to_draw["system status"] = True

    async def set_vehicle_parameters(self):
        """sets all vehicle parameters after connecting/reconnecting"""

        self.parameters_status = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # set parameters
        MAX_SPEED_M = MAX_SPEED/FEET_PER_METER
        try:
            await self.pixhawk.action.set_maximum_speed(MAX_SPEED_M)
            await self.pixhawk.telemetry.set_rate_position(RF_TELEMETRY_RATE)
            await self.pixhawk.telemetry.set_rate_velocity_ned(RF_TELEMETRY_RATE)
            await self.pixhawk.telemetry.set_rate_attitude(RF_TELEMETRY_RATE)
            await self.pixhawk.telemetry.set_rate_in_air(RF_TELEMETRY_RATE)
            await self.pixhawk.telemetry.set_rate_landed_state(RF_TELEMETRY_RATE)
        except mavsdk.system.telemetry.TelemetryError:
            self.parameters_status = SS.FAILED
            g_v.gui.display_message("parameters upload failed", "retrying...", 0)
            await self.set_vehicle_parameters()

        self.parameters_status = SS.SUCCESS
        g_v.gui.to_draw["system status"] = True

    async def upload_geofence(self):
        """Upload geofence to plane"""

        self.geofence_status = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # delete any previous geofence
        try:
            await self.pixhawk.geofence.clear_geofence()
        except mavsdk.system.geofence.GeofenceError:
            self.geofence_status = SS.FAILED
            g_v.gui.display_message("geofence upload failed", "retrying...", 0)
            await self.upload_geofence()

        # Define geofence boundary
        vertices_pos = [gg_f.cartesian_to_geo(vertex.pos) for vertex in g_v.mp.border.vertices]
        vertices_pos = [geofence.Point(point[0], point[1]) for point in vertices_pos]

        # Create a polygon object using vertex points
        polygon = geofence.Polygon(vertices_pos, geofence.Polygon.FenceType.INCLUSION)

        # Uploading the geofence
        try:
            await self.pixhawk.geofence.upload_geofence([polygon])
        except mavsdk.system.geofence.GeofenceError:
            self.geofence_status = SS.FAILED
            g_v.gui.display_message("geofence upload failed", "retrying...", 0)
            await self.upload_geofence()

        self.geofence_status = SS.SUCCESS
        g_v.gui.to_draw["system status"] = True

    async def subscribe_to_telemetry(self):
        """Gets position and velocity/attitude of plane"""

        tele = self.pixhawk.telemetry
        miss = self.pixhawk.mission_raw

        # shortcut for calling subscribe to data
        def sub(a, b, c):
            asyncio.ensure_future(self.subscribe_to_data(a, b, c, self.connection_index))

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

    async def subscribe_to_connection(self, connection_index):
        """subscribes to a telemetry data inputs and sends
        the received data through a converter before going
        to the telemetry history"""

        # subscribe to connection status
        try:
            async for data in self.pixhawk.core.connection_state():

                if self.close_request or self.lost_comms or self.connection_index != connection_index:
                    break

                # change connection status if needed
                if data.is_connected and self.connection_status == 0:
                    self.connection_status = 1
                    g_v.gui.to_draw["system status"] = True
                if not data.is_connected and self.connection_status == 1:
                    self.connection_status = 0
                    g_v.gui.to_draw["system status"] = True

        # reconnect if connection lost
        except grpc._channel._MultiThreadedRendezvous:
            g_v.gui.display_message("lost connection error", "reconnection attempt...", 0)
            if (not self.lost_comms) and self.connection_index == connection_index:
                self.lost_comms = True
                await self.connect()
                await self.subscribe_to_telemetry()

    async def subscribe_to_data(self, tele_in, tele_out, tele_converter, connection_index):
        """subscribes to a telemetry data inputs and sends
        the received data through a converter before going
        to the telemetry history"""

        t = time.time()
        telemetry_rates = []
        try:
            # subscribe to telemetry data input
            async for data in tele_in():

                if self.close_request or self.lost_comms or self.connection_index != connection_index:
                    break
                new_telemetry_rate = 1/(time.time()-t)
                telemetry_rates.append(new_telemetry_rate)
                if len(telemetry_rates) > RF_TELEMETRY_RATE_SAMPLE:
                    del telemetry_rates[0]
                telemetry_rate = sum(telemetry_rates)/len(telemetry_rates)
                t = time.time()

                # convert the telemetry data based on the converter
                data = tele_converter(data)

                # get telemetry time
                telemetry_time = time.time()

                # send data to telemetry history
                tele_out.set_data(telemetry_time, telemetry_rate, data)

        # if connection lost reconnect
        except grpc._channel._MultiThreadedRendezvous:
            g_v.gui.display_message("lost connection error", "reconnection attempt...", 0)
            if (not self.lost_comms) and self.connection_index == connection_index:
                self.lost_comms = True
                await self.connect()
                await self.subscribe_to_telemetry()

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

        return {"heading": g_f.clamp_angle(-math.radians(data.heading_deg) + math.pi/2)}

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

    def launch_download_mission(self):
        """start download mission for updating the mission state"""

        # check that an exportable path has been found and can be uploaded
        if self.download_status == SS.STARTED:
            g_v.gui.display_message("Can't download mission,", "mission is being downloaded", 0)
        else:
            self.event_loop.create_task(self.download_mission())

    async def download_mission(self):
        """download mission from pixhawk to update the mission state"""

        self.download_status = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # check if connection data is received
        if self.connection_status == 0:
            self.download_status = SS.FAILED
            g_v.gui.display_message("can't download mission,", "no connection", 0)
        else:
            # download mission
            try:
                previous_mission = await self.pixhawk.mission_raw.download_mission()
                self.current_plan = previous_mission

                # get list of plane waypoint positions
                pos_list = list()

                for index, mission_item in enumerate(previous_mission):
                    if mission_item.current == 1:
                        self.current_mission_item = index
                        break
                    if mission_item.command == 16 or mission_item.command == 31:
                        lat = mission_item.x/(10**7)
                        lon = mission_item.y/(10**7)
                        alt = mission_item.z * FEET_PER_METER
                        point = {"latitude": lat, "longitude": lon}
                        x, y = gg_f.geographic_to_cartesian_center(point)
                        pos_list.append((x, y, alt))

                # update mission state list
                mission_state_list = g_v.ms.waypoint_list

                # deactivate takeoff
                mission_state_list[0].is_mission = 0

                # find first active waypoint
                active_index = g_v.ms.active_index()

                # if all waypoints are inactive, nothing to update
                if active_index is None:
                    return

                # update mission waypoint activity
                for pos_i in pos_list:
                    active_way = mission_state_list[active_index]
                    active_3_d = (active_way.pos[0], active_way.pos[1], active_way.z)
                    if g_f.distance_3d(pos_i, active_3_d) < 10:
                        active_way.is_mission = 0
                        active_index += 1

            except mavsdk.system.mission_raw.MissionRawError as err:
                error = str(err._result.result)
                self.download_status = SS.FAILED
                g_v.gui.display_message("can't download mission,", error, 0)
            except grpc._channel._MultiThreadedRendezvous:
                self.download_status = SS.FAILED
                g_v.gui.display_message("can't download mission,", "connection lost, reconnecting...", 0)
                if not self.lost_comms:
                    self.lost_comms = True
                    await self.connect()
            else:
                self.download_status = SS.SUCCESS
                g_v.gui.to_draw["system status"] = True

    def launch_upload_mission(self):
        """start async mission uploading"""

        # check that an exportable path has been found and can be uploaded
        if g_v.mc.path_computation_status == 0:
            g_v.gui.display_message("Can't upload mission,", "path computation not started", 0)
        elif g_v.mc.path_computation_status == 1:
            g_v.gui.display_message("Can't upload mission,", "path is being computed", 0)
        elif self.upload_status == SS.STARTED:
            g_v.gui.display_message("Can't upload mission,", "mission is being uploaded", 0)
        elif self.lost_comms:
            g_v.gui.display_message("Can't upload mission,", "connection was lost", 0)
        else:
            self.event_loop.create_task(self.upload_mission())

    async def upload_mission(self):
        """upload mission to pixhawk"""

        self.upload_status = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # check if connection data is received
        if self.connection_status == 0:
            self.upload_status = SS.FAILED
            g_v.gui.display_message("can't upload mission,", "no connection", 0)
        else:
            # clear previous mission
            try:
                await self.pixhawk.mission_raw.clear_mission()
            except mavsdk.system.mission_raw.MissionRawError as err:
                error = str(err._result.result)
                self.upload_status = SS.FAILED
                g_v.gui.display_message("can't clear mission,", error, 0)
            except grpc._channel._MultiThreadedRendezvous:
                self.upload_status = SS.FAILED
                g_v.gui.display_message("can't clear mission,", "connection lost, reconnecting...", 0)
                if not self.lost_comms:
                    self.lost_comms = True
                    await self.connect()
            else:
                # create new mission
                mission_items = list()
                MissionItem = mission_raw.MissionItem
                index = 0

                current = int(not g_v.th.in_air.data["in air"])
                # add speed change
                CRUISE_SPEED_M = CRUISE_SPEED / FEET_PER_METER
                item = MissionItem(index, 2, 178, current, 1, 0.0, CRUISE_SPEED_M, -1, 0, 0, 0, 0, 0)
                mission_items.append(item)
                index += 1

                # add takeoff item
                lat, lon = TAKEOFF_COORDINATES["latitude"], TAKEOFF_COORDINATES["longitude"]
                alt = TAKEOFF_COORDINATES["altitude"]/FEET_PER_METER
                item = MissionItem(index, 5, 22, 0, 1, 0.0, 0.0, 0.0, 0, lat, lon, alt, 0)
                mission_items.append(item)
                index += 1
                current = int(g_v.th.in_air.data["in air"])
                # add mission waypoints
                imaging_added = False
                for waypoint in g_v.mc.exportable_chosen_path.waypoint_list:

                    lat, lon = gg_f.cartesian_to_geo(waypoint.pos)
                    # scale lat lon for the used global frame 5
                    lat, lon = int(lat*(10**7)), int(lon*(10**7))
                    alt = waypoint.z / FEET_PER_METER

                    # add imaging start
                    if waypoint.mission_index == 2 and (not imaging_added):
                        imaging_added = True
                        item = MissionItem(index, 2, 2500, current, 1, 0, 0, 0, 0, 0, 0, 0, 0)
                        mission_items.append(item)
                        index += 1
                        current = 0

                    # add mission waypoint
                    item = MissionItem(index, 5, 16, current, 1, 1.0, 0, 0, 0, lat, lon, alt, 0)
                    mission_items.append(item)
                    index += 1
                    current = 0

                # add imaging end
                item = MissionItem(index, 2, 2501, current, 1, 0, 0, 0, 0, 0, 0, 0, 0)
                mission_items.append(item)
                index += 1
                current = 0

                """add landing pattern"""
                # landing start flair
                L_L_C = LANDING_LOITER_COORDINATES
                alt = L_L_C["altitude"] / FEET_PER_METER
                item = MissionItem(index, 2, 189,  current, 1, 0, 0, 0, 0, 0, 0, alt, 0)
                mission_items.append(item)
                index += 1
                # landing loiter
                lat, lon = L_L_C["latitude"], L_L_C["longitude"]
                alt = L_L_C["altitude"]/FEET_PER_METER
                rad = L_L_C["radius"]/FEET_PER_METER
                item = MissionItem(index, 5, 31, 0, 1, 1, rad, 0, 1, lat, lon, alt, 0)
                mission_items.append(item)
                index += 1
                # landing position
                L_C = LANDING_COORDINATES
                lat, lon = L_C["latitude"], L_C["longitude"]
                alt = L_C["altitude"]/FEET_PER_METER
                item = MissionItem(index, 5, 21, 0,  1, 0, 0, 0, 0, lat, lon, alt, 0)
                mission_items.append(item)
                index += 1

                # upload mission
                try:
                    await self.pixhawk.mission_raw.upload_mission(mission_items)
                    self.current_plan = mission_items
                    self.current_mission_item = 0
                except mavsdk.system.mission_raw.MissionRawError as err:
                    error = str(err._result.result)
                    self.upload_status = SS.FAILED
                    g_v.gui.display_message("can't upload mission,", error, 0)
                except grpc._channel._MultiThreadedRendezvous:
                    self.upload_status = SS.FAILED
                    g_v.gui.display_message("can't upload mission,", "connection lost, reconnecting...", 0)
                    if not self.lost_comms:
                        self.lost_comms = True
                        await self.connect()
                else:
                    self.upload_status = SS.SUCCESS
                    g_v.gui.to_draw["system status"] = True

    def launch_start_mission(self):
        """start async mission starting"""

        # check that a mission has been uploaded and is not being started
        if self.mission_start == SS.STARTED:
            g_v.gui.display_message("Can't start mission,", "mission is being started", 0)
        elif self.mission_pause == SS.STARTED:
            g_v.gui.display_message("Can't start mission,", "mission is being paused", 0)
        elif self.go_to_status == SS.STARTED:
            g_v.gui.display_message("Can't start mission,", "sending orbit request", 0)
        elif self.lost_comms:
            g_v.gui.display_message("Can't start mission,", "connection was lost", 0)
        else:
            if g_v.mc.pre_existing_flight:
                self.event_loop.create_task(self.start_mission())
            else:
                if self.upload_status != SS.SUCCESS:
                    g_v.gui.display_message("Can't start mission,", "mission not uploaded yet", 0)
                else:
                    self.event_loop.create_task(self.start_mission())

    async def start_mission(self):
        """start flight mission"""

        self.mission_start = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # check if arming data is received
        if not g_v.th.armed.data_received():
            self.mission_start = SS.FAILED
            g_v.gui.display_message("can't start mission,", "telemetry is not received", 0)
        else:
            # arm vehicle if not armed already
            try:
                if not g_v.th.armed.data["armed"]:
                    await self.pixhawk.action.arm()
            except mavsdk.system.action.ActionError as err:
                error = str(err._result.result)
                self.mission_start = SS.FAILED
                g_v.gui.display_message("can't arm vehicle,", error, 0)
            except grpc._channel._MultiThreadedRendezvous:
                self.mission_start = SS.FAILED
                g_v.gui.display_message("can't arm vehicle,", "connection lost, reconnecting...", 0)
                if not self.lost_comms:
                    self.lost_comms = True
                    await self.connect()
            else:
                # start mission
                try:
                    await self.pixhawk.mission_raw.start_mission()
                except mavsdk.system.mission_raw.MissionRawError as err:
                    error = str(err._result.result)
                    self.mission_start = SS.FAILED
                    g_v.gui.display_message("can't start mission,", error, 0)
                except grpc._channel._MultiThreadedRendezvous:
                    self.mission_start = SS.FAILED
                    g_v.gui.display_message("can't start mission,", "connection lost, reconnecting...", 0)
                    if not self.lost_comms:
                        self.lost_comms = True
                        await self.connect()
                else:
                    self.mission_start = SS.SUCCESS
                    g_v.gui.to_draw["system status"] = True

    def launch_pause_mission(self):
        """start async mission stopping"""

        if self.mission_start == SS.STARTED:
            g_v.gui.display_message("Can't pause mission,", "mission is being started", 0)
        elif self.mission_pause == SS.STARTED:
            g_v.gui.display_message("Can't pause mission,", "mission is being paused", 0)
        elif self.go_to_status == SS.STARTED:
            g_v.gui.display_message("Can't pause mission,", "sending orbit request", 0)
        elif self.lost_comms:
            g_v.gui.display_message("Can't pause mission,", "connection was lost", 0)
        else:
            self.event_loop.create_task(self.pause_mission())
        
    async def pause_mission(self):
        """pause flight mission"""

        self.mission_pause = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # check if flight_mode data is received
        if not g_v.th.flight_mode.data_received():
            self.mission_pause = SS.FAILED
            g_v.gui.display_message("can't pause mission,", "telemetry is not received", 0)
        else:
            # pause mission
            if g_v.th.flight_mode.data["flight mode"].value == 4:
                try:
                    await self.pixhawk.mission_raw.pause_mission()
                except mavsdk.system.mission_raw.MissionRawError as err:
                    error = str(err._result.result)
                    self.mission_pause = SS.FAILED
                    g_v.gui.display_message("can't pause mission,", error, 0)
                except grpc._channel._MultiThreadedRendezvous:
                    self.mission_pause = SS.FAILED
                    g_v.gui.display_message("can't pause mission,", "connection lost, reconnecting...", 0)
                    if not self.lost_comms:
                        self.lost_comms = True
                        await self.connect()
                else:
                    self.mission_pause = SS.SUCCESS
                    g_v.gui.to_draw["system status"] = True
            else:
                self.mission_pause = SS.FAILED
                g_v.gui.display_message("can't pause mission,", "vehicle is not in mission mode", 0)

    def launch_go_to(self, position, alt):
        """start async go to starting"""

        # check that a mission has been uploaded and is not being started
        if self.mission_start == SS.STARTED:
            g_v.gui.display_message("Can't go to position,", "mission is being started", 0)
        elif self.mission_pause == SS.STARTED:
            g_v.gui.display_message("Can't go to position,", "mission is being paused", 0)
        elif self.go_to_status == SS.STARTED:
            g_v.gui.display_message("Can't go to position,", "already sending orbit request", 0)
        elif self.lost_comms:
            g_v.gui.display_message("Can't go to position,", "connection was lost", 0)
        else:
            self.event_loop.create_task(self.start_go_to(position, alt))

    async def start_go_to(self, position, alt):
        """start go to position"""

        self.go_to_status = SS.STARTED
        g_v.gui.to_draw["system status"] = True

        # check if arming data is received
        if not g_v.th.in_air.data_received():
            self.go_to_status = SS.FAILED
            g_v.gui.display_message("can't go to position,", "telemetry is not received", 0)
        else:
            if g_v.th.in_air.data["in air"]:
                lat, lon = gg_f.cartesian_to_geo(position)
                alt_m = alt/FEET_PER_METER
                no_control = mavsdk.action.OrbitYawBehavior(2)
                O_R = ORBIT_RADIUS/FEET_PER_METER
                O_V = ORBIT_VELOCITY/FEET_PER_METER

                try:
                    await self.pixhawk.action.do_orbit(O_R, O_V, no_control, lat, lon, alt_m)
                except mavsdk.system.action.ActionError as err:
                    error = str(err._result.result)
                    self.go_to_status = SS.FAILED
                    g_v.gui.display_message("can't go to position,", error, 0)
                except grpc._channel._MultiThreadedRendezvous:
                    self.go_to_status = SS.FAILED
                    g_v.gui.display_message("can't go to position,", "connection lost, reconnecting...", 0)
                    if not self.lost_comms:
                        self.lost_comms = True
                        await self.connect()
                else:
                    self.go_to_status = SS.SUCCESS
                    g_v.gui.to_draw["system status"] = True

            else:
                self.go_to_status = SS.FAILED
                g_v.gui.display_message("can't go to position,", "vehicle is not in air", 0)
