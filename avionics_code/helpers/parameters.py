"""Parameters for whole code to access,
All distances are in feet, otherwise sizes are in pixel"""

# Constants
FEET_PER_METER = 3.280839895
KNOTS_PER_FT_PER_S = 0.592484
FLOAT_DIFFERENCE_FOR_EQUALITY = 0.00001

# Map parameters
MAP_REF = {
    "latitude": 38.14468,
    "longitude": -76.428022
    }

# Dashboard parameters
DASHBOARD_SIZE = 650  # default 650 (in pixels) size of dashboard
FRAMES_PER_SECOND = 30  # (default 30) frames per second for the dashboard
REGULAR_UPDATE_PER_SECOND = 1  # (default 1) full display updates per second
WAYPOINT_SIZE = 4  # (in pixels) size of waypoints on dashboard
DEFAULT_MAP_SIZE = 6000  # (ft) map size * don't change it cause the background doesn't scale with it*
ARROW_HEAD_SIZE = 30  # (ft) size of arrow heads
PATH_COLORING_CYCLE = 40  # length of coloring cycle in waypoints for shown path
MESSAGE_DISPLAY_PERIOD = 3  # (s) period for which a message is displayed as new on screen

# RF comms parameters
RF_TELEMETRY_RATE = 1  # (Hz) telemetry request rate for pixhawk, actual rate may vary
RF_TELEMETRY_RATE_SAMPLE = 20  # sample size for determining current telemetry rate
RF_TELEMETRY_ALLOWED_DELAY = 1  # (s) delay for telemetry to no longer be valid

# Mission control parameters
MISSION_STATE_REFRESH_RATE = 15  # (Hz) rate at which the mission state is refreshed
WAYPOINT_ACCEPTANCE_DISTANCE_1 = 40  # (ft) distance to consider a mission waypoint 0/1 crossed
WAYPOINT_ACCEPTANCE_DISTANCE_2 = 130  # (ft) distance to consider a mission waypoint 2/3/4/5 crossed
MISSION_TIME_LENGTH = 1200  # (seconds) time for mission after which plane lands automatically
TAKEOFF_COORDINATES = {
      "latitude": 381429471,
      "longitude": -764262467,
      "altitude": 200
    }
LANDING_LOITER_COORDINATES = {
      "latitude": 381454976,
      "longitude": -764236249,
      "altitude": 200
    }
LANDING_COORDINATES = {
      "latitude": 381449043,
      "longitude": -764274419,
      "altitude": 100
    }
LANDING_LOITER_COORDINATES_FRAME_0 = {
      "latitude": 38.1449043,
      "longitude": -76.4274419,
      "altitude": 200
    }

# CV/Hardware parameters
CV_IMAGE_GROUND_SIZE = 150  # (ft) side of the ground square area covered by CV's camera
OFF_AXIS_IMAGING_DISTANCE = 150  # (ft) distance from border or obstacle for off-axis imaging
OFF_AXIS_IMAGING_RANGE = 350  # (ft) ground range of off-axis imaging
AIR_DROP_ALTITUDE = 200.0  # (ft) altitude for airdrop
IMAGING_ALTITUDE = 200.0  # (ft) altitude for imaging
OFF_AXIS_IMAGING_ALTITUDE = 300.0  # (ft) altitude for off-axis imaging

# Mission generation parameters
WEIGHT_OF_ANGLE = 500000  # weight of angle for computing path cost when covering search area
BIAS_OF_ANGLE = 0.5  # bias of angle for computing path cost  when covering search area
WEIGHT_OF_DISTANCE = 1  # weight of distance for computing path cost  when covering search area
BIAS_OF_DISTANCE = 1  # bias of distance for computing path cost  when covering search area
OFF_AXIS_WEIGHT_OF_ANGLE = 1  # weight of angle for computing path cost when covering off axis searching

# Path finding parameters
PATHS_PER_WAYPOINTS = 6  # max number of paths to find between each two waypoints
MAX_ATTEMPTS_PER_WAYPOINTS = 100  # max number of rounds of exploration to find paths between two waypoints
BACK_TRACKING_DEPTH = 2  # max number of backtracking to be done behind a waypoint before it is marked as unreachable

# Path logic parameters
OBSTACLE_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from obstacles for path to be valid
OBSTACLE_DISTANCE_FOR_VALID_NODE = 80  # (ft) Distance from obstacles at which nodes are created
BORDER_DISTANCE_FOR_VALID_PASS = 80  # (ft) Distance off from border for path to be valid
BORDER_DISTANCE_FOR_VALID_NODE = 100  # (ft) Distance from border vertices at which nodes are created
NODE_MIN_DISTANCE = 10  # (ft) Distance between two nodes such that they are considered the same
OBSTACLE_ORBIT_RATIO = 1.2  # factor that obstacle distance for pass is scaled by for minimum orbiting angle

# Plane parameters
PREFERRED_TURN_RADIUS = 100  # (ft) preferred turning radius for plane
MAX_CLIMBING_RATIO = 0.3  # maximum altitude change per distance crossed
