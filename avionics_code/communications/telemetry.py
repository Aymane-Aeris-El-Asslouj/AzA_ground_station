from avionics_code.utility_functions import geometrical_functions as g_f
from avionics_code.utility_functions import geography_functions as gg_f
from avionics_code.references import parameters as para
from avionics_code.references import global_variables as g_v
from avionics_code.communications import pixhawk_utility as p_u

import math
import time
import asyncio
import threading

FEET_PER_METER = para.FEET_PER_METER
RF_TELEMETRY_RATE_SAMPLE = para.RF_TELEMETRY_RATE_SAMPLE

SS = g_v.StandardStatus


@p_u.reconnect_deco("lost connection error",
                    "connection_status", no_start=True)
async def subscribe_to_connection(self, connection_index):
    """subscribes to a telemetry data inputs and sends
    the received data through a converter before going
    to the telemetry history"""

    async for data in self.pixhawk.core.connection_state():

        # check if connection was changed
        if self.connection_index != connection_index:
            break

        # change connection status if needed
        if not data.is_connected:
            await p_u.reconnect(self, "connection lost")


async def subscribe_to_telemetry(self):
    """Gets position and velocity/attitude of plane"""

    tele = self.pixhawk.telemetry
    miss = self.pixhawk.mission_raw

    # shortcut for calling subscribe to data
    def sub(a, b, c):
        sub_data = subscribe_to_data
        co_index = self.connection_index
        return asyncio.ensure_future(sub_data(self, a, b, c, co_index))

    tasks = self.running_tasks

    # subscribe to plane position
    tasks.append(sub(tele.position, g_v.th.position,
                 position_converter))

    # subscribe to plane velocity
    tasks.append(sub(tele.velocity_ned, g_v.th.velocity,
                 velocity_converter))

    # subscribe to plane heading
    tasks.append(sub(tele.heading, g_v.th.heading,
                 heading_converter))

    # subscribe to armed state
    tasks.append(sub(tele.armed, g_v.th.armed,
                 lambda data: {"armed": data}))

    # subscribe to in air state
    tasks.append(sub(tele.in_air, g_v.th.in_air,
                 lambda data: {"in air": data}))

    # subscribe to landed state
    tasks.append(sub(tele.landed_state, g_v.th.landed,
                 lambda data: {"landed": data}))

    # subscribe to flight_mode state
    tasks.append(sub(tele.flight_mode, g_v.th.flight_mode,
                 lambda data: {"flight mode": data}))

    # subscribe to status text
    tasks.append(sub(tele.status_text, g_v.th.status_text,
                 lambda data: {"type": data.type, "text": data.text}))

    # subscribe to mission progress
    tasks.append(sub(tele.status_text, g_v.th.status_text,
                 lambda data: {"type": data.type, "text": data.text}))

    # subscribe to mission progress
    tasks.append(sub(miss.mission_progress, g_v.th.mission_progress,
                 lambda data: {"current": data.current,
                               "total": data.total}))


@p_u.reconnect_deco("lost connection error",
                    "connection_status", no_start=True)
async def subscribe_to_data(self, tele_in, tele_out,
                            tele_converter, connection_index):
    """subscribes to a telemetry data inputs and sends
    the received data through a converter before going
    to the telemetry history"""

    t = time.time()
    telemetry_rates = []

    # subscribe to telemetry data input
    async for data in tele_in():

        with threading.Lock():

            # check if connection was lost
            if self.connection_status != SS.SUCCESS:
                break

            # check if connection was changed
            if self.connection_index != connection_index:
                break

            # add new telemetry rate to history
            new_telemetry_rate = 1/(time.time()-t)
            telemetry_rates.append(new_telemetry_rate)

            # delete last telemetry rates from history
            if len(telemetry_rates) > RF_TELEMETRY_RATE_SAMPLE:
                del telemetry_rates[0]

            # compute average telemetry rate
            telemetry_rate = sum(telemetry_rates)/len(telemetry_rates)
            t = time.time()

            # convert the telemetry data based on the converter
            data = tele_converter(data)

            # get telemetry time
            telemetry_time = time.time()

            # send data to telemetry history
            tele_out.set_data(telemetry_time, telemetry_rate, data)


def position_converter(data):
    """converts position telemetry data to the desired format"""

    # make geographic conversions for data
    gps = {'latitude': data.latitude_deg,
           'longitude': data.longitude_deg}
    plane_pos = gg_f.geographic_to_cartesian_center(gps)
    plane_z = data.absolute_altitude_m * FEET_PER_METER

    # create dummy plane object
    class FlightObj:
        pass
    plane_obj = FlightObj()
    plane_obj.pos = plane_pos
    plane_obj.z = plane_z

    return {"flight object": plane_obj}


def velocity_converter(data):
    """converts velocity telemetry data to the desired format"""

    # create dummy plane object
    class FlightObj:
        pass

    # convert velocities
    v_x = data.east_m_s * FEET_PER_METER
    v_y = data.north_m_s * FEET_PER_METER
    v_z = data.down_m_s * FEET_PER_METER

    # create object holding velocities
    plane_obj = FlightObj()
    plane_obj.vel = (v_x, v_y)
    plane_obj.v_z = v_z
    plane_obj.v = math.hypot(v_x, v_y)
    if round(plane_obj.v) == 0:
        plane_obj.v_ang = None
    else:
        plane_obj.v_ang = math.atan2(v_y, v_x)

    return {"flight object": plane_obj}


def heading_converter(data):
    """converts attitude telemetry data to the desired format"""

    angle = math.radians(data.heading_deg)

    return {"heading": g_f.clamp_angle(-angle + math.pi / 2)}
