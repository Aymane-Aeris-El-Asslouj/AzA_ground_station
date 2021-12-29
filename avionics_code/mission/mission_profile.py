from avionics_code.path import path_objects as p_o
from avionics_code.references import global_variables as g_v
from avionics_code.utility_functions import geography_functions as gg_f
from avionics_code.utility_functions import geometrical_functions as g_f


def mp_update_decorator(func):
    """makes all updates set the mission state
    and path to expired and draws mission profile"""

    def new_func(*args):
        func(*args)
        g_v.ms.generation_status = g_v.StandardStatus.EXPIRED
        g_v.mc.path_computation_status = g_v.StandardStatus.EXPIRED
        g_v.rf.upload_status = g_v.StandardStatus.EXPIRED
        g_v.gui.to_draw("mission profile")
        g_v.gui.to_draw("system status")

    return new_func


class MissionProfile:
    """stores all mission info received from the server"""

    def __init__(self):

        # lost coms (MapObject)
        self.lost_comms_object = None

        # flight area (int + int + border)
        self.border_altitude_min = None
        self.border_altitude_max = None
        self.border = None

        # Mission waypoints (Waypoint list)
        self.mission_waypoints = None

        # Search grid (MapArea)
        self.search_area = None

        # off axis object (MapObject)
        self.off_axis_object = None

        # Emergent object last known position (MapObject)
        self.emergent_object = None

        # air drop boundary (MapArea)
        self.ugv_area = None

        # air drop position (MapObject)
        self.airdrop_object = None

        # ugv driving position (MapObject)
        self.ugv_goal_object = None

        # obstacles (obstacle list)
        self.obstacles = None

        # map height (MapArea)
        self.mapping_area = None

    @mp_update_decorator
    def add_obstacle(self, obs_pos):
        self.obstacles.append(p_o.Obstacle(obs_pos[0], obs_pos[1]))

    @mp_update_decorator
    def clear_obstacles(self):
        self.obstacles.clear()

    @mp_update_decorator
    def delete_obstacle(self, i):
        del self.obstacles[i]

    @mp_update_decorator
    def add_border_vertex(self, i, vertex_tuple):
        self.border.vertices.insert(i, p_o.Vertex(vertex_tuple))

    @mp_update_decorator
    def clear_border(self):
        self.border.vertices.clear()

    @mp_update_decorator
    def delete_border_vertex(self, i):
        del self.border.vertices[i]

    @mp_update_decorator
    def add_search_vertex(self, i, vertex_tuple):
        self.search_area.vertices.insert(i, p_o.Vertex(vertex_tuple))

    @mp_update_decorator
    def clear_search(self):
        self.search_area.vertices.clear()

    @mp_update_decorator
    def delete_search_vertex(self, i):
        del self.search_area.vertices[i]

    @mp_update_decorator
    def add_waypoint(self, i, waypoint_tuple, altitude):
        way = p_o.Waypoint(0, waypoint_tuple, altitude, is_mission=1)
        self.mission_waypoints.insert(i, way)

    @mp_update_decorator
    def clear_waypoints(self):
        self.mission_waypoints.clear()

    @mp_update_decorator
    def delete_waypoint(self, i):
        del self.mission_waypoints[i]

    @mp_update_decorator
    def set_airdrop(self, pos):
        self.airdrop_object.pos = pos

    @mp_update_decorator
    def set_airdrop_goal(self, pos):
        self.ugv_goal_object.pos = pos

    @mp_update_decorator
    def set_lostcomms(self, pos):
        self.lost_comms_object.pos = pos

    @mp_update_decorator
    def set_offaxis_obj(self, pos):
        self.off_axis_object.pos = pos

    @mp_update_decorator
    def set_emergent_obj(self, pos):
        self.emergent_object.pos = pos

    @mp_update_decorator
    def set_mapping_area(self, pos1, pos2):
        # determine center of two points
        center = g_f.center_2d(pos1, pos2)
        # determine height of mapping area
        height = abs(pos2[1]-pos1[1])
        # compute vertices of the area
        self.mapping_area = self.map_from_cen_height(center, height)

    @mp_update_decorator
    def clear_mapping_area(self):
        self.set_mapping_area((0, 0), (0, 0))

    def from_json(self, data):
        """This function takes the information from
        a Json file provided by the server and uses
        it to construct a mission profile"""
        
        dict_ext = self.dictionary_extraction

        g_t_c_c = gg_f.geographic_to_cartesian_center
        g_t_c_l = gg_f.geographic_to_cartesian_list

        # lost coms
        self.lost_comms_object = p_o.MapObject(g_t_c_c(data['lostCommsPos']))

        # flight area
        self.border_altitude_min = data['flyZones'][0]['altitudeMin']
        self.border_altitude_max = data['flyZones'][0]['altitudeMax']
        pos_iter = g_t_c_l(data['flyZones'][0]['boundaryPoints'])
        self.border = p_o.Border(list(p_o.Vertex(pos) for pos in pos_iter))

        # Mission waypoints
        mission_xy_tuples = g_t_c_l(data['waypoints'])
        altitude_tuples = list(w_i["altitude"] for w_i in data['waypoints'])
        way_tuples = list(zip(mission_xy_tuples, altitude_tuples))
        WAY_TYPE = g_v.MissionType.WAYPOINTS
        self.mission_waypoints = list(
            p_o.Waypoint(WAY_TYPE, t[0], t[1], is_mission=1) for t in way_tuples)

        # Search grid
        pos_iter = g_t_c_l(data['searchGridPoints'])
        obj = p_o.MapArea(list(p_o.Vertex(pos) for pos in pos_iter))
        self.search_area = obj

        # off axis object
        obj = p_o.MapObject(g_t_c_c(data['offAxisOdlcPos']))
        self.off_axis_object = obj

        # Emergent object last known position
        obj = p_o.MapObject(g_t_c_c(data['emergentLastKnownPos']))
        self.emergent_object = obj

        # air drop boundary
        pos_iter = g_t_c_l(data['airDropBoundaryPoints'])
        obj = p_o.MapArea(list(p_o.Vertex(pos) for pos in pos_iter))
        self.ugv_area = obj

        # air drop position
        obj = p_o.MapObject(g_t_c_c(data['airDropPos']))
        self.airdrop_object = obj

        # ugv driving position
        obj = p_o.MapObject(g_t_c_c(data['ugvDrivePos']))
        self.ugv_goal_object = obj

        # obstacles
        obstacle_points = g_t_c_l(data['stationaryObstacles'])
        obstacle_radii = dict_ext(data['stationaryObstacles'], 'radius')
        zipped = zip(obstacle_points, obstacle_radii)
        self.obstacles = list(p_o.Obstacle(o[0], o[1]) for o in zipped)

        # map height
        height = data["mapHeight"]
        center = g_t_c_c(data['mapCenterPos'])
        self.mapping_area = self.map_from_cen_height(center, height)

        # generate border nodes
        self.border.create_vertex_nodes(self)

    @staticmethod
    def dictionary_extraction(pts, key):
        """Extract list of values from dictionary through key"""

        values = []
        for pt in pts:
            values.append(pt[key])
        return values

    @staticmethod
    def map_from_cen_height(center, height):
        """returns map from center and height"""
        corners = g_f.rect_from_cen_size(center, height * (16 / 9), height)
        map_list = [p_o.Vertex(corner) for corner in corners]
        return p_o.MapArea(map_list)
