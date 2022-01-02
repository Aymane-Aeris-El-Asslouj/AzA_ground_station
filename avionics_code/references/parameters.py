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
REGULAR_UPDATES_PER_SECOND = 1  # (default 1) full display updates per second
WAYPOINT_SIZE = 4  # (in pixels) size of waypoints on dashboard
DEFAULT_MAP_SIZE = 6000  # (ft) map size * don't change it cause the background doesn't scale with it*
ARROW_HEAD_SIZE = 10  # (ft) size of arrow heads
PATH_COLORING_CYCLE = 40  # length of coloring cycle in waypoints for shown path
MESSAGE_DISPLAY_PERIOD = 3  # (s) period for which a message is displayed as new on screen
FONT_SIZE = 14
FONT_TYPE = "Arial"
ZOOM_RATIO = 4/3
ZOOM_MAX = 5
PLANE_SIZE = 100

# RF comms parameters
RF_TELEMETRY_RATE = 15  # (Hz) telemetry request rate for pixhawk, actual rate may vary
RF_TELEMETRY_RATE_SAMPLE = 20  # sample size for determining current telemetry rate
RF_TELEMETRY_ALLOWED_DELAY = 1  # (s) delay for telemetry to no longer be valid

# Mission control parameters
MISSION_STATE_REFRESH_RATE = 15  # (Hz) rate at which the mission state is refreshed
WAYPOINT_ACCEPTANCE_DISTANCE_1 = 40  # (ft) distance to consider a mission waypoint 0/1 crossed
WAYPOINT_ACCEPTANCE_DISTANCE_2 = 130  # (ft) distance to consider a mission waypoint 2/3/4/5 crossed
TAKEOFF_COORDINATES = {
      "latitude": 381429471,
      "longitude": -764262467,
      "altitude": 150  # ft
    }
LANDING_LOITER_COORDINATES = {
      "latitude": 381454976,
      "longitude": -764236249,
      "altitude": 200,
      "radius": -246
    }
LANDING_COORDINATES = {
      "latitude": 381449043,
      "longitude": -764274419,
      "altitude": 100
}

# CV/Hardware parameters
CV_IMAGE_GROUND_SIZE = 50  # (ft) side of the ground square area covered by CV's camera
OFF_AXIS_IMAGING_RANGE = 350  # (ft) ground range of off-axis imaging
AIR_DROP_ALTITUDE = 200.0  # (ft) altitude for airdrop
IMAGING_ALTITUDE = 200.0  # (ft) altitude for imaging
OFF_AXIS_IMAGING_ALTITUDE = 300.0  # (ft) altitude for off-axis imaging
OFF_AXIS_IMAGING_DISTANCE = 80  # (ft) distance from border/obstacle for off-axis imaging

# Mission generation parameters
WEIGHT_OF_ANGLE_SCOUT = 500000  # weight of angle for computing path cost when covering search area
BIAS_OF_ANGLE_SCOUT = 0.0001  # bias of angle for computing path cost  when covering search area
WEIGHT_OF_DISTANCE_SCOUT = 1  # weight of distance for computing path cost  when covering search area
BIAS_OF_DISTANCE_SCOUT = 100000  # bias of distance for computing path cost  when covering search area
WEIGHT_OF_ANGLE_OFF = 1  # weight of angle for computing path cost when covering search area
BIAS_OF_ANGLE_OFF = 1  # bias of angle for computing path cost  when covering search area
WEIGHT_OF_DISTANCE_OFF = 1  # weight of distance for computing path cost  when covering search area
BIAS_OF_DISTANCE_OFF = 1  # bias of distance for computing path cost  when covering search area

# Path finding parameters
PATHS_PER_WAYPOINTS = 6  # max number of paths to find between each two waypoints
MAX_ATTEMPTS_PER_WAYPOINTS = 100  # max number of rounds of exploration to find paths between two waypoints
BACK_TRACKING_DEPTH = 2  # max number of backtracking for a waypoint to be marked as unreachable
TURN_SEARCH_ITERATIONS = 50  # steps to check for different possible turn angles at waypoints

# Path logic parameters
OBSTACLE_DISTANCE_FOR_VALID_PASS = 20  # (ft) Distance off from obstacles for path to be valid
OBSTACLE_DISTANCE_FOR_VALID_NODE = 40  # (ft) Min distance from obstacles for node to be valid
OBSTACLE_DISTANCE_FOR_ORBIT = 80  # (ft) Distance from obstacles where orbit nodes are created
OBSTACLE_DISTANCE_FOR_ORBIT_START = 120  # (ft) Distance from obstacles where you enter its orbit zone
BORDER_DISTANCE_FOR_VALID_PASS = 20  # (ft) Distance off from border for path to be valid
BORDER_DISTANCE_FOR_VALID_NODE = 40  # (ft) Distance off from border for node to be valid
BORDER_DISTANCE_FOR_BORDER_NODE = 80  # (ft) Distance from off from border at which nodes are created
NODE_MIN_DISTANCE = 10  # (ft) Distance between two nodes such that they are considered the same
MIN_DEVIATION_FOR_ALTERNATIVE_PATH = 5  # (deg) minimum deviation for a path to not be simplified

# Plane parameters
CONFIRMATION_DISTANCE = 100  # (ft) distance of post-turn waypoints
PREFERRED_TURN_RADIUS = 100  # (ft) turning radius for tight turns
MAX_CLIMBING_RATIO = 0.3  # maximum altitude change per distance crossed
MAX_SPEED = 230  # (ft/s) max airspeed
CRUISE_SPEED = 130  # (ft/s) cruise airspeed
ORBIT_VELOCITY = 60  # (ft/s) orbit velocity for requested orbit
ORBIT_RADIUS = 200  # (ft) orbit radius for requested orbit
ORBIT_RADIUS_VALIDATION = 450

# more parameters
SLEEP_THREADS = False
