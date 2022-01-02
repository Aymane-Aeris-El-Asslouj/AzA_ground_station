from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.utility_functions import geography_functions as gg_f
from avionics_code.communications import pixhawk_utility as p_u

import mavsdk
from mavsdk import geofence
import asyncio

CRUISE_SPEED = para.CRUISE_SPEED
ORBIT_VELOCITY = para.ORBIT_VELOCITY
ORBIT_RADIUS = para.ORBIT_RADIUS
FEET_PER_METER = para.FEET_PER_METER

SS = g_v.StandardStatus


def launch_download_mission(self):
    """start download mission for updating the mission state"""

    self.event_loop.create_task(download_mission(self))


@p_u.reco_except("can't download mission,", "download_status",
                 mavsdk.system.mission_raw.MissionRawError)
async def download_mission(self):
    """download mission from pixhawk to update
    the mission state"""

    # download mission
    download = self.pixhawk.mission_raw.download_mission
    self.current_plan = await download()

    # load it
    p_u.load_mission_state(self, self.current_plan)


def launch_upload_mission(self):
    """start async mission uploading"""

    self.event_loop.create_task(upload_mission(self))


@p_u.reco_except("can't upload mission,", "upload_status",
                 (mavsdk.system.geofence.GeofenceError,
                  mavsdk.system.mission_raw.MissionRawError))
async def upload_mission(self):
    """upload mission to pixhawk"""

    # Define geofence boundary
    vertices_pos = [gg_f.cartesian_to_geo(vertex.pos)
                    for vertex in g_v.mp.border.vertices]
    new_vertices_pos = [geofence.Point(point[0], point[1])
                        for point in vertices_pos]

    # Create a polygon object using vertex points
    fence_type = geofence.Polygon.FenceType.INCLUSION
    polygon = geofence.Polygon(new_vertices_pos, fence_type)

    # clear previous geofence
    await self.pixhawk.geofence.clear_geofence()

    # upload current geofence
    await self.pixhawk.geofence.upload_geofence([polygon])

    # clear previous mission
    await self.pixhawk.mission_raw.clear_mission()

    # create and upload mission
    mission_items = p_u.create_mission()
    upload = self.pixhawk.mission_raw.upload_mission
    await upload(mission_items)
    self.current_plan = mission_items
    self.current_mission_item = 0


def launch_start_mission(self):
    """start async mission starting"""

    self.event_loop.create_task(start_mission(self))


@p_u.reco_except("can't start mission,", "mission_start",
                 (mavsdk.system.action.ActionError,
                  mavsdk.system.mission_raw.MissionRawError))
async def start_mission(self):
    """start flight mission"""

    # arm vehicle if not armed already
    if not g_v.th.armed.data["armed"]:
        await self.pixhawk.action.arm()
    # start mission
    await self.pixhawk.mission_raw.start_mission()


def launch_pause_mission(self):
    """start async mission stopping"""

    self.event_loop.create_task(pause_mission(self))


@p_u.reco_except("can't pause mission,", "mission_pause",
                 mavsdk.system.mission_raw.MissionRawError)
async def pause_mission(self):
    """pause flight mission"""

    await self.pixhawk.mission_raw.pause_mission()


def launch_go_to(self, position, alt):
    """start async go to starting"""

    self.event_loop.create_task(start_go_to(self, position, alt))


@p_u.reco_except("can't go to,", "go_to_status",
                 mavsdk.system.action.ActionError)
async def start_go_to(self, position, alt):
    """start go to position"""

    # check if arming data is received
    lat, lon = gg_f.cartesian_to_geo(position)
    alt_m = alt / FEET_PER_METER
    no_control = mavsdk.action.OrbitYawBehavior(2)
    O_R = ORBIT_RADIUS / FEET_PER_METER
    O_V = ORBIT_VELOCITY / FEET_PER_METER

    do_orbit = self.pixhawk.action.do_orbit
    await do_orbit(O_R, O_V, no_control, lat, lon, alt_m)


def launch_camera_gimbal(self):
    """start async camera gimbal"""

    self.event_loop.create_task(camera_gimbal(self))


@p_u.reco_except("can't camera gimbal,", "camera_gimbal",
                 mavsdk.system.camera.CameraError)
async def camera_gimbal(self):
    """camera gimbal command"""

    await run(self)


def free_rf(self):
    """checks if no comm is being performed,
    and connection is present"""

    # check that an exportable path has
    # been found and can be uploaded
    if self.download_status == SS.STARTED:
        return False
    elif self.upload_status == SS.STARTED:
        return False
    elif self.mission_start == SS.STARTED:
        return False
    elif self.mission_pause == SS.STARTED:
        return False
    elif self.go_to_status == SS.STARTED:
        return False
    elif self.connection_status != SS.SUCCESS:
        return False
    else:
        return True


async def run(self):

    asyncio.ensure_future(print_mode(self.pixhawk))
    asyncio.ensure_future(print_status(self.pixhawk))
    asyncio.ensure_future(ba(self.pixhawk))
    asyncio.ensure_future(bc(self.pixhawk))

    print("Setting mode to 'PHOTO'")
    try:
        await self.pixhawk.camera.set_mode(mavsdk.system.camera.Mode.PHOTO)
    except mavsdk.system.camera.CameraError as error:
        print(f"Setting mode failed with error code: {error._result.result}")

    await asyncio.sleep(2)

    print("Taking a photo")
    try:
        await self.pixhawk.camera.take_photo()
    except mavsdk.system.camera.CameraError as error:
        print(f"Couldn't take photo: {error._result.result}")


async def print_mode(drone):
    async for mode in drone.camera.mode():
        print(f"Camera mode: {mode}")


async def print_status(drone):
    async for status in drone.camera.status():
        print(status)


async def bc(drone):
    async for status in drone.camera.information():
        print(status)


async def ba(drone):
    async for status in drone.camera.capture_info():
        print(status)
