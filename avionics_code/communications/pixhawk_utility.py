from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.utility_functions import geography_functions as gg_f
from avionics_code.utility_functions import geometrical_functions as g_f

from mavsdk import mission_raw
import grpc

CRUISE_SPEED = para.CRUISE_SPEED
FEET_PER_METER = para.FEET_PER_METER
TAKEOFF_COORDINATES = para.TAKEOFF_COORDINATES
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES
LANDING_COORDINATES = para.LANDING_COORDINATES

SS = g_v.StandardStatus
MT = g_v.MessageType


def reconnect_deco(text, status, no_start=False):
    """decorator factory to handle reconnections"""

    def deco(func):

        async def return_func(*args):
            self = args[0]
            try:
                if not no_start:
                    setattr(self, status, SS.STARTED)
                await func(*args)
            except grpc._channel._MultiThreadedRendezvous:
                setattr(self, status, SS.FAILED)
                await reconnect(self, text)
        return return_func

    return deco


def reco_except(text, status, given_error):

    def deco(func):

        async def return_func(*args):
            self = args[0]
            try:
                setattr(self, status, SS.STARTED)
                await func(*args)
            except given_error as err:
                in_error = str(err._result.result)
                setattr(self, status, SS.FAILED)
                g_v.gui.display_message(text, in_error, MT.COMMS)
            except grpc._channel._MultiThreadedRendezvous:
                setattr(self, status, SS.FAILED)
                await reconnect(self, text)
            else:
                setattr(self, status, SS.SUCCESS)

        return return_func

    return deco


async def reconnect(self, text):
    """reconnects when connection is lost"""

    msg = g_v.gui.display_message
    msg(text, "connection lost, reconnecting...", MT.COMMS)
    if self.connection_status == SS.SUCCESS:
        self.connection_status = SS.FAILED
        await self.connect()


def create_mission():
    """create pixhawk mission"""

    # create new mission
    mission_items = list()
    MissionItem = mission_raw.MissionItem
    index = 0

    # add speed change
    current = int(not g_v.th.in_air.data["in air"])
    CRUISE_SPEED_M = CRUISE_SPEED / FEET_PER_METER
    item = MissionItem(index, 2, 178, current, 1, 0.0,
                       CRUISE_SPEED_M, -1, 0, 0, 0, 0, 0)
    mission_items.append(item)
    index += 1

    # add takeoff item
    T_C = TAKEOFF_COORDINATES
    lat, lon = T_C["latitude"], T_C["longitude"]
    alt = T_C["altitude"] / FEET_PER_METER
    item = MissionItem(index, 5, 22, 0, 1, 0.0, 0.0,
                       0.0, 0, lat, lon, alt, 0)
    mission_items.append(item)
    index += 1

    # add mission waypoints
    current = int(g_v.th.in_air.data["in air"])
    imaging_added = False

    way_list = g_v.mc.exportable_chosen_path.waypoint_list
    for waypoint in way_list:

        lat, lon = gg_f.cartesian_to_geo(waypoint.pos)
        # scale lat lon for the used global frame 5
        lat, lon = int(lat * (10 ** 7)), int(lon * (10 ** 7))
        alt = waypoint.z / FEET_PER_METER

        # add imaging start
        IMG = g_v.MissionType.IMAGING
        if waypoint.mission_type == IMG and (not imaging_added):
            imaging_added = True
            item = MissionItem(index, 2, 2500, current,
                               1, 0, 0, 0, 0, 0, 0, 0, 0)
            mission_items.append(item)
            index += 1
            current = 0

        # add mission waypoint
        item = MissionItem(index, 5, 16, current, 1, 1.0,
                           0, 0, 0, lat, lon, alt, 0)
        mission_items.append(item)
        index += 1
        current = 0

    # add imaging end
    item = MissionItem(index, 2, 2501, current, 1, 0,
                       0, 0, 0, 0, 0, 0, 0)
    mission_items.append(item)
    index += 1
    current = 0

    """add landing pattern"""
    # landing start flair
    L_L_C = LANDING_LOITER_COORDINATES
    alt = L_L_C["altitude"] / FEET_PER_METER
    item = MissionItem(index, 2, 189, current, 1, 0,
                       0, 0, 0, 0, 0, alt, 0)
    mission_items.append(item)
    index += 1
    # landing loiter
    lat, lon = L_L_C["latitude"], L_L_C["longitude"]
    alt = L_L_C["altitude"] / FEET_PER_METER
    rad = L_L_C["radius"] / FEET_PER_METER
    item = MissionItem(index, 5, 31, 0, 1, 1, rad,
                       0, 1, lat, lon, alt, 0)
    mission_items.append(item)
    index += 1
    # landing position
    L_C = LANDING_COORDINATES
    lat, lon = L_C["latitude"], L_C["longitude"]
    alt = L_C["altitude"] / FEET_PER_METER
    item = MissionItem(index, 5, 21, 0, 1, 0,
                       0, 0, 0, lat, lon, alt, 0)
    mission_items.append(item)
    index += 1

    return mission_items


def load_mission_state(self, previous_mission):
    """uses pixhawk mission to update mission state"""

    # get list of plane waypoint positions
    pos_list = list()

    for index, mission_item in enumerate(previous_mission):
        if mission_item.current == 1:
            self.current_mission_item = index
            break
        if mission_item.command == 16 or mission_item.command == 31:
            lat = mission_item.x / (10 ** 7)
            lon = mission_item.y / (10 ** 7)
            alt = mission_item.z * FEET_PER_METER
            point = {"latitude": lat, "longitude": lon}
            x, y = gg_f.geographic_to_cartesian_center(point)
            pos_list.append((x, y, alt))

    # update mission state list
    mission_state_list = g_v.ms.waypoint_list

    # deactivate takeoff
    mission_state_list[0].is_mission = g_v.Activity.INACTIVE

    # find first active waypoint
    active_index = g_v.ms.active_index()

    # if all waypoints are inactive, nothing to update
    if active_index is None:
        return

    # update mission waypoint activity
    for pos_i in pos_list:
        active_way = mission_state_list[active_index]
        pos = active_way.pos
        active_3_d = (pos[0], pos[1], active_way.z)
        if g_f.distance_3d(pos_i, active_3_d) < 10:
            active_way.is_mission = g_v.Activity.INACTIVE
            active_index += 1
