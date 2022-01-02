from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.communications import telemetry as tel
from avionics_code.communications import pixhawk_commands as p_c
from avionics_code.communications import pixhawk_utility as p_u

import mavsdk
import asyncio


FEET_PER_METER = para.FEET_PER_METER
RF_TELEMETRY_RATE = para.RF_TELEMETRY_RATE
MAX_SPEED = para.MAX_SPEED

SS = g_v.StandardStatus
MT = g_v.MessageType


class RFComs:
    
    def __init__(self, event_loop):

        # connection status
        self.connection_index = 0
        self.pixhawk = mavsdk.System()
        self.connection_status = SS.NONE
        self.running_tasks = []
        
        # mission status
        self.download_status = SS.NONE
        self.upload_status = SS.NONE
        self.mission_start = SS.NONE
        self.mission_pause = SS.NONE
        self.go_to_status = SS.NONE
        self.camera_gimbal = SS.NONE
        
        self.event_loop = event_loop
        self.current_plan = None
        self.current_mission_item = None

    def launch_connect(self):
        """schedules connection to be started"""

        if self.connection_status != SS.STARTED:
            self.event_loop.create_task(self.connect())

    @p_u.reconnect_deco("connection attempt failed",
                        "connection_status")
    async def connect(self):
        """Connects to plane"""

        # setup new connection
        self.connection_index += 1

        # cancel all previous generators
        for task in self.running_tasks:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await asyncio.get_event_loop().shutdown_asyncgens()

        # get base connection
        await self.pixhawk.connect(system_address="udp://:14540")
        async for state in self.pixhawk.core.connection_state():
            if state.is_connected:
                break

        # upload parameters
        await self.set_parameters()
        # subscribe to telemetry
        await tel.subscribe_to_telemetry(self)

        # create connection subscription
        sub_co = tel.subscribe_to_connection
        task = asyncio.ensure_future(sub_co(self, self.connection_index))
        self.running_tasks.append(task)

        self.connection_status = SS.SUCCESS

    async def set_parameters(self):
        """sets all vehicle parameters after
        connecting/reconnecting"""

        tele = self.pixhawk.telemetry

        # set parameters
        MAX_SPEED_M = MAX_SPEED/FEET_PER_METER
        try:
            await self.pixhawk.action.set_maximum_speed(MAX_SPEED_M)
            await self.pixhawk.param.set_param_int("GF_ACTION", 1)
            await tele.set_rate_position(RF_TELEMETRY_RATE)
            await tele.set_rate_velocity_ned(RF_TELEMETRY_RATE)
            await tele.set_rate_attitude(RF_TELEMETRY_RATE)
            await tele.set_rate_in_air(RF_TELEMETRY_RATE)
            await tele.set_rate_landed_state(RF_TELEMETRY_RATE)
        except (mavsdk.system.telemetry.TelemetryError,
                mavsdk.system.action.ActionError):
            self.connection_status = SS.FAILED
            g_v.gui.display_message("parameters upload failed",
                                    "retrying...", MT.COMMS)
            await self.set_parameters()

    """shortcuts"""

    def launch_download_mission(self):
        p_c.launch_download_mission(self)

    def launch_upload_mission(self):
        p_c.launch_upload_mission(self)

    def launch_start_mission(self):
        p_c.launch_start_mission(self)

    def launch_pause_mission(self):
        p_c.launch_pause_mission(self)

    def launch_go_to(self, position, alt):
        p_c.launch_go_to(self, position, alt)

    def launch_camera_gimbal(self):
        p_c.launch_camera_gimbal(self)

    def free_rf(self):
        return p_c.free_rf(self)
