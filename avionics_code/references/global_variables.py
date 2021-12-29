"""Global variables for the access of the whole code"""
from enum import Enum, unique, auto

"""Server comms object"""
sc = None

"""Mission profile object"""
mp = None

"""RF comms object"""
rf = None

"""Telemetry history object"""
th = None

"""Mission state object"""
ms = None

"""Mission control object"""
mc = None

"""Graphical interface"""
gui = None


@unique
class StandardStatus(Enum):
    NONE = auto()
    STARTED = auto()
    SUCCESS = auto()
    FAILED = auto()
    EXPIRED = auto()


@unique
class ControllerStatus(Enum):
    WAIT_CONNECTION = auto()
    CHECK_AIR = auto()
    DOWNLOAD_MISSION = auto()
    WAIT_DOWNLOAD_MISSION = auto()
    WAIT_POSITION_TELEMETRY = auto()
    WAIT_COMPUTE_PATH = auto()
    WAIT_REQUESTS = auto()
    WAIT_GENERATION_FOR_DOWNLOAD = auto()
    WAIT_GENERATION_FOR_COMPUTE = auto()


@unique
class MissionType(Enum):
    PLANE = auto()
    TAKEOFF = auto()
    WAYPOINTS = auto()
    IMAGING = auto()
    OFF_AXIS = auto()
    LANDING = auto()


"""close request for the program"""
close_request = False


def get_status(obj, status_name):
    """for getting status of some object"""

    if obj is None:
        return StandardStatus.NONE
    else:
        return getattr(obj, status_name)


def get_success(obj, status_name):
    """check if a status is successful"""

    status = get_status(obj, status_name)
    return status == StandardStatus.SUCCESS
