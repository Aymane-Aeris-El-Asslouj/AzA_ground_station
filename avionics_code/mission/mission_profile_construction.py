import utm
import json
import Geometrical_functions as G_f
import Path_objects as P_o
import Parameters as Para

FEET_PER_METER = Para.FEET_PER_METER


"""This function takes the information from a Json file provided by the server
and uses it to construct a mission profile"""
def Json_to_mission_profile(file):
    with open(file) as f:
        data = json.load(f)
    # unpack Json data and transform it in cartesian coordinates
    Center_tuple = Geographic_to_cartesian(data['mapCenterPos'])
    Border_tuples = Geographic_to_cartesian_list(data['flyZones'][0]['boundaryPoints'], Center_tuple)
    XY_tuples = Geographic_to_cartesian_list(data['waypoints'], Center_tuple)
    Altitude_tuples = list(W_i["altitude"] for W_i in data['waypoints'])
    Mission_tuples = list(zip(XY_tuples, Altitude_tuples))
    Obstacle_points = Geographic_to_cartesian_list(data['stationaryObstacles'], Center_tuple)
    Obstacle_radii = Dictionary_extraction(data['stationaryObstacles'], 'radius')

    # no current position is available now in this code, so assuming current position is average of Mission_waypoints
    Current_x = sum(M_i[0][0]for M_i in Mission_tuples)/len(Mission_tuples)
    Current_y = sum(M_i[0][1]for M_i in Mission_tuples)/len(Mission_tuples)
    Current_tuple = ((Current_x, Current_y), 300)
    Obstacle_tuples = list(zip(Obstacle_points, Obstacle_radii))

    return P_o.Mission_profile(Center_tuple, Current_tuple, Mission_tuples, Obstacle_tuples, Border_tuples)


"""Perform geographic to cartesian transformation on a list of points
and give them in the frame of reference of the center of map"""
def Geographic_to_cartesian_list(Points, Center):
    New_points = []
    for Point in Points:
        New_point = Geographic_to_cartesian(Point)
        New_points.append(G_f.Sub_vectors(New_point, Center))
    return New_points


"""Use dictionary with longitude and latitude to get cartesian position in feet"""
def Geographic_to_cartesian(Point):
    (x, y, p, q) = utm.from_latlon(Point['latitude'], Point['longitude'])
    return x * FEET_PER_METER, y * FEET_PER_METER


"""Extract list of values from dictionary through key"""
def Dictionary_extraction(pts, key):
    values = []
    for pt in pts:
        values.append(pt[key])
    return values
