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

    AWAITING_GENERATION = auto()

    AWAITING_CONNECTION = auto()
    CHECKING_AIR = auto()

    DOWNLOAD_MISSION = auto()
    DOWNLOADING_MISSION = auto()

    GENERATE_MISSION = auto()
    GENERATING_MISSION = auto()

    COMPUTE_PATH = auto()
    COMPUTING_PATH = auto()

    UPLOAD_MISSION = auto()
    UPLOADING_MISSION = auto()

    START_MISSION = auto()
    STARTING_MISSION = auto()

    PAUSE_MISSION = auto()
    PAUSING_MISSION = auto()

    GO_TO = auto()
    GO_TOING = auto()

    LAND = auto()

    CAMERA_GIMBAL_COMMAND = auto()
    CAMERA_GIMBAL_COMMANDING = auto()


@unique
class MissionType(Enum):
    PLANE = auto()
    TAKEOFF = auto()
    WAYPOINTS = auto()
    IMAGING = auto()
    OFF_AXIS = auto()
    LANDING = auto()


@unique
class MessageType(Enum):
    DEFAULT = auto()
    COMMS = auto()
    ERROR = auto()
    CONTROLLER = auto()
    MISSION_STATE = auto()


@unique
class Activity(Enum):
    NA = auto()
    ACTIVE = auto()
    INACTIVE = auto()


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
