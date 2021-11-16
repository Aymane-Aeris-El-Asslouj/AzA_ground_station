"""Parameters for whole code to access,
All distances are in feet, otherwise sizes are in pixel"""

# Constants
FEET_PER_METER = 3.280839895
FLOAT_DIFFERENCE_FOR_EQUALITY = 0.00001

# Map parameters
MAP_REF = {
    "latitude": 38.14468,
    "longitude": -76.428022
    }
LANDING_LOITER_CENTER = {
      "latitude": 38.1461944444444,
      "longitude": -76.4237138888889,
      "altitude": 100.0
    }

# Dashboard parameters
DASHBOARD_SIZE = 650  # default 650 (in pixels) size of dashboard
WAYPOINT_SIZE = 4  # (in pixels) size of waypoints on dashboard
DEFAULT_MAP_SIZE = 6000  # (ft) map size * don't change it cause the background doesn't scale with it*
ARROW_HEAD_SIZE = 30  # (ft) size of arrow heads
PATH_COLORING_CYCLE = 40  # length of coloring cycle in waypoints for shown path

# Communication
TIME_CUT_OFF_FOR_FLIGHT_STATUS = 10  # (seconds) time after which a flight profile is no longer displayed

# CV/Hardware parameters
CV_IMAGE_GROUND_SIZE = 200  # (ft) side of the ground square area covered by CV's camera
OFF_AXIS_IMAGING_DISTANCE = 150  # (ft) distance from border or obstacle for off-axis imaging
OFF_AXIS_IMAGING_RANGE = 350  # (ft) ground range of off-axis imaging
AIR_DROP_ALTITUDE = 200.0  # (ft) altitude for airdrop
IMAGING_ALTITUDE = 200.0  # (ft) altitude for imaging
OFF_AXIS_IMAGING_ALTITUDE = 300.0  # (ft) altitude for off-axis imaging

# Mission generation parameters
WEIGHT_OF_ANGLE = 5000  # weight of angle for computing path cost when covering search area
BIAS_OF_ANGLE = 0.1  # bias of angle for computing path cost  when covering search area
WEIGHT_OF_DISTANCE = 1  # weight of distance for computing path cost  when covering search area
BIAS_OF_DISTANCE = 1  # bias of distance for computing path cost  when covering search area
OFF_AXIS_WEIGHT_OF_ANGLE = 1  # weight of angle for computing path cost when covering off axis searching


# Path finding parameters
PATHS_PER_WAYPOINTS = 1  # number of paths to find between each two waypoints
MAX_ATTEMPTS_PER_WAYPOINTS = 100  # max number of rounds of exploration to find paths between two waypoints
VARIATIONS_FOR_PATH_BUILDING = 0  # max number of curved paths to get stitched during curving of straight paths

# Path logic parameters
OBSTACLE_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from obstacles for path to be valid
OBSTACLE_DISTANCE_FOR_VALID_NODE = 80  # (ft) Distance from obstacles at which nodes are created
BORDER_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from border for path to be valid
BORDER_DISTANCE_FOR_VALID_NODE = 100  # (ft) Distance from border vertices at which nodes are created
NODE_MIN_DISTANCE = 10  # (ft) Distance between two nodes such that they are considered the same
OBSTACLE_ORBIT_RATIO = 1.2  # factor that obstacle distance for pass is scaled by for minimum orbiting angle
MIN_DEVIATION_FOR_ALTERNATIVE_PATH = 10  # (degrees) Angle between two straight path lines so they represent a new path


# Plane parameters
TURN_RADIUS = 50  # (ft) Minimum turning radius of plane
PREFERRED_TURN_RADIUS = 100  # (ft) preferred turning radius for plane
MAX_CLIMBING_RATIO = 0.3  # maximum altitude change per distance crossed
