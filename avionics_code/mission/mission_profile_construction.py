import json
from avionics_code.helpers import geometrical_functions as g_f, parameters as para, geography_functions as gg_f
from avionics_code.path import path_objects as p_o
from avionics_code.mission import mission_profile as m_p


def json_to_mission_profile(file):
    """This function takes the information from a Json file
    provided by the server and uses it to construct a mission profile"""

    m_p_var = m_p.MissionProfile()

    with open(file) as f:
        data = json.load(f)

    g_t_c_c = gg_f.geographic_to_cartesian_center
    g_t_c_l = gg_f.geographic_to_cartesian_list

    # lost coms
    m_p_var.lost_comms_object = p_o.MapObject(g_t_c_c(data['lostCommsPos']))

    # flight area
    m_p_var.border_altitude_min = data['flyZones'][0]['altitudeMin']
    m_p_var.border_altitude_max = data['flyZones'][0]['altitudeMax']
    pos_iterator = g_t_c_l(data['flyZones'][0]['boundaryPoints'])
    m_p_var.border = p_o.Border(list(p_o.Vertex(pos) for pos in pos_iterator))

    # Mission waypoints
    mission_xy_tuples = g_t_c_l(data['waypoints'])
    altitude_tuples = list(w_i["altitude"] for w_i in data['waypoints'])
    waypoint_tuples = list(zip(mission_xy_tuples, altitude_tuples))
    m_p_var.mission_waypoints = list(p_o.Waypoint(0, t[0], t[1], is_mission=True) for t in waypoint_tuples)

    # Search grid
    pos_iterator = g_t_c_l(data['searchGridPoints'])
    m_p_var.search_area = p_o.MapArea(list(p_o.Vertex(pos) for pos in pos_iterator))

    # off axis object
    m_p_var.off_axis_object = p_o.MapObject(g_t_c_c(data['offAxisOdlcPos']))

    # Emergent object last known position
    m_p_var.emergent_object = p_o.MapObject(g_t_c_c(data['emergentLastKnownPos']))

    # air drop boundary
    pos_iterator = g_t_c_l(data['airDropBoundaryPoints'])
    m_p_var.ugv_area = p_o.MapArea(list(p_o.Vertex(pos) for pos in pos_iterator))

    # air drop position
    m_p_var.airdrop_object = p_o.MapObject(g_t_c_c(data['airDropPos']))

    # ugv driving position
    m_p_var.ugv_goal_object = p_o.MapObject(g_t_c_c(data['ugvDrivePos']))

    # obstacles
    obstacle_points = g_t_c_l(data['stationaryObstacles'])
    obstacle_radii = dictionary_extraction(data['stationaryObstacles'], 'radius')
    zipped = zip(obstacle_points, obstacle_radii)
    m_p_var.obstacles = list(p_o.Obstacle(o[0], o[1]) for o in zipped)

    # map height
    height = data["mapHeight"]
    c_x, c_y = g_t_c_c(data['mapCenterPos'])
    pairs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    map_list = [p_o.Vertex((c_x+pair[0]*height*(8/9), c_y+pair[1]*height/2)) for pair in pairs]
    m_p_var.mapping_area = p_o.MapArea(map_list)

    return m_p_var


def dictionary_extraction(pts, key):
    """Extract list of values from dictionary through key"""

    values = []
    for pt in pts:
        values.append(pt[key])
    return values
