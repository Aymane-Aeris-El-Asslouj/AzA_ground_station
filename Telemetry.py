import utm
import json
import Geometrical_functions
import Path_objects
import Parameters

FEET_PER_METER = Parameters.FEET_PER_METER

# Perform geographic to cartesian transformation on a list of points and give them in reference to the center of map
def Geographic_to_cartesian_list(Points, Center):
    New_points = []
    for Point in Points:
        New_point = Geographic_to_cartesian(Point)
        New_points.append(Geometrical_functions.Sub_vectors(New_point, Center))
    return New_points


# Use dictionary with longitude and latitude to get cartesian position in feet
def Geographic_to_cartesian(Point):
    (x, y, p, q) = utm.from_latlon(Point['latitude'], Point['longitude'])
    return x * FEET_PER_METER, y * FEET_PER_METER


# Extract list of values from dictionary through key
def Dictionary_extraction(pts, key):
    values = []
    for pt in pts:
        values.append(pt[key])
    return values


# load Telemetry from json file
class Map_data_telemetry:
    def __init__(self, file='data.json'):
        with open(file) as f:
            data = json.load(f)
        # get map data and transform it in cartesian coordinates
        self.Center = Geographic_to_cartesian(data['mapCenterPos'])
        self.Border_points = Geographic_to_cartesian_list(data['flyZones'][0]['boundaryPoints'], self.Center)
        self.Waypoint_points = Geographic_to_cartesian_list(data['waypoints'], self.Center)
        self.Obstacle_points = Geographic_to_cartesian_list(data['stationaryObstacles'], self.Center)
        self.Obstacle_radii = Dictionary_extraction(data['stationaryObstacles'], 'radius')

        # transform map data into path objects
        self.Map_center = Path_objects.Waypoint(self.Center)
        self.Border = Path_objects.Border(
            list(Path_objects.Border_vertex(Border_point_i) for Border_point_i in self.Border_points))
        self.Obstacles = list(
            Path_objects.Obstacle(self.Obstacle_points[index], self.Obstacle_radii[index]) for index in
            range(len(self.Obstacle_points)))
        self.Mission_waypoint_list = list(
            Path_objects.Waypoint(Waypoint_point_i) for Waypoint_point_i in self.Waypoint_points)
