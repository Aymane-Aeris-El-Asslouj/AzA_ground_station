"""Parameters for whole code to access,
All distances are in feet, otherwise sizes are in pixel"""

# Constants
FEET_PER_METER = 3.280839895
FLOAT_DIFFERENCE_FOR_EQUALITY = 0.00001
MAP_REF = {"latitude": 38.14468, "longitude": -76.428022}

# Dashboard parameters
DASHBOARD_SIZE = 650  # default 650 (in pixels) size of dashboard
WAYPOINT_SIZE = 4  # (in pixels) size of waypoints on dashboard
COLOR_OFFSET_FOR_PATH = 0.5  # (0 to 1 range) Color offset between each two parts of a path for distinguishing them
DEFAULT_MAP_SIZE = 6000  # (ft) map size * don't change it cause the background doesn't scale with it*

# Path finding parameters
PATHS_PER_WAYPOINTS = 2  # number of paths to find between each two waypoints
MAX_ATTEMPTS_PER_WAYPOINTS = 100  # max number of rounds of exploration to find paths between two waypoints
VARIATIONS_FOR_PATH_BUILDING = 2  # max number of curved paths to get stitched during curving of straight paths

# Path logic parameters
OBSTACLE_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from obstacles for path to be valid
OBSTACLE_DISTANCE_FOR_VALID_NODE = 80  # (ft) Distance from obstacles at which nodes are created
BORDER_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from border for path to be valid
BORDER_DISTANCE_FOR_VALID_NODE = 100  # (ft) Distance from border vertices at which nodes are created
NODE_MIN_DISTANCE = 10  # (ft) Distance between two nodes such that they are considered the same
OBSTACLE_ORBIT_RATIO = 1.2  # factor that obstacle distance for pass is scaled by for minimum orbiting angle
MIN_DEVIATION_FOR_ALTERNATIVE_PATH = 10  # (degrees) Angle between two straight path lines so they represent a new path
MAX_CLIMBING_RATIO = 0.3  # maximum altitude change per distance crossed

TURN_RADIUS = 50  # (ft) Minimum turning radius of plane
PREFERRED_TURN_RADIUS = 200  # (ft) preferred turning radius for plane
