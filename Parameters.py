# All distances are in feet, otherwise sizes are in pixel

# Constants
FEET_PER_METER = 3.280839895
FLOAT_DIFFERENCE_FOR_EQUALITY = 0.00001

# Dashboard parameters
DASHBOARD_SIZE = 650  # (in pixels) size of dashboard
WAYPOINT_SIZE = 3.5  # (in pixels) size of waypoints on dashboard
SELECTION_MENU_POSITION = 40  # (in pixels) position of selection menu buttons on dashboard
NUMBER_OF_PATH_TO_SHOW = 2  # maximum number of paths shown on the dashboard
SHIFT_FOR_PATHS_TO_SHOW = 2  # (in pixels) Shift between shown paths to make distinguishing them easier
COLOR_OFFSET_FOR_PATH = 0.3  # (0 to 1 range) Color offset between each two parts of a path for distinguishing them
DEFAULT_MAP_SIZE = 6000  # (ft) map size for empty map (a little bit higher than default map)

# Path finding parameters
PATHS_PER_WAYPOINTS = 2  # number of paths to find between each two waypoints
MAX_ATTEMPTS_PER_WAYPOINTS = 100  # max number of rounds of exploration to find paths between two waypoints

# Path logic parameters
OBSTACLE_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from obstacles for path to be valid
OBSTACLE_DISTANCE_FOR_VALID_NODE = 60  # (ft) Distance from obstacles at which nodes are created
BORDER_DISTANCE_FOR_VALID_PASS = 30  # (ft) Distance off from border for path to be valid
BORDER_DISTANCE_FOR_VALID_NODE = 60  # (ft) Distance from border vertices at which nodes are created
NODE_MIN_DISTANCE = 10  # (ft) Distance between two nodes such that they are considered the same
OBSTACLE_ORBIT_RATIO = 1.1  # factor that obstacle distance for pass is scaled by for minimum orbiting angle
