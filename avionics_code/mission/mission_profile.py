import avionics_code.path.path_objects as p_o

class MissionProfile:
    """stores all mission info received from the server"""

    def __init__(self):
        # lost coms (MapObject)
        self.lost_comms = None

        # flight area (int + int + border)
        self.border_altitude_min = None
        self.border_altitude_max = None
        self.border = None

        # Mission waypoints (Waypoint list)
        self.mission_waypoints = None

        # Search grid (MapArea)
        self.search_boundary = None

        # off axis object (MapObject)
        self.off_axis_object = None

        # Emergent object last known position (MapObject)
        self.emergent_object = None

        # air drop boundary (MapArea)
        self.ugv_boundary = None

        # air drop position (MapObject)
        self.airdrop_obj = None

        # ugv driving position (MapObject)
        self.ugv_goal = None

        # obstacles (obstacle list)
        self.obstacles = None

        # map height (MapArea)
        self.mapping_area = None

    def add_obstacle(self, obstacle_tuple):
        self.obstacles.append(p_o.Obstacle(obstacle_tuple[0], obstacle_tuple[1]))

    def clear_obstacles(self):
        self.obstacles.clear()

    def delete_obstacle(self, i):
        del self.obstacles[i]

    def add_border_vertex(self, i, vertex_tuple):
        self.border.vertices.insert(i, p_o.Vertex(vertex_tuple))

    def clear_border(self):
        self.border.vertices.clear()

    def delete_border_vertex(self, i):
        del self.border.vertices[i]

    def add_search_vertex(self, i, vertex_tuple):
        self.search_boundary.vertices.insert(i, p_o.Vertex(vertex_tuple))

    def clear_search(self):
        self.search_boundary.vertices.clear()

    def delete_search_vertex(self, i):
        del self.search_boundary.vertices[i]

    def add_waypoint(self, i, waypoint_tuple):
        self.mission_waypoints.insert(i, p_o.Waypoint(waypoint_tuple))

    def clear_waypoints(self):
        self.mission_waypoints.clear()

    def delete_waypoint(self, i):
        del self.mission_waypoints[i]

    def set_airdrop(self, pos):
        self.airdrop_obj.pos = pos

    def set_airdrop_goal(self, pos):
        self.ugv_goal.pos = pos

    def set_lostcomms(self, pos):
        self.lost_comms.pos = pos

    def set_offaxis_obj(self, pos):
        self.off_axis_object.pos = pos

    def set_emergent_obj(self, pos):
        self.emergent_object.pos = pos

    def set_mapping_area(self, pos1, pos2):
        # determine center of two points
        c_x, c_y = ((pos1[0]+pos2[0])/2, (pos1[1]+pos2[1])/2)
        # determine height of mapping area
        height = abs(pos2[1]-pos1[1])
        # compute vertices of the area
        pairs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
        map_list = [p_o.Vertex((c_x+pair[0]*height*(8/9), c_y+pair[1]*height/2)) for pair in pairs]
        self.mapping_area.vertices = map_list

    def clear_lostcomms(self):
        self.set_lostcomms((0, 0))

    def clear_emergent_obj(self):
        self.set_emergent_obj((0, 0))

    def clear_offaxis_obj(self):
        self.set_offaxis_obj((0, 0))

    def clear_airdrop(self):
        self.set_airdrop((0, 0))

    def clear_airdrop_goal(self):
        self.set_airdrop_goal((0, 0))

    def clear_mapping_area(self):
        self.set_mapping_area((0, 0), (0, 0))

    def clear_all(self):
        self.clear_search()
        self.clear_border()
        self.clear_waypoints()
        self.clear_obstacles()
        self.clear_lostcomms()
        self.clear_emergent_obj()
        self.clear_offaxis_obj()
        self.clear_airdrop()
        self.clear_airdrop_goal()
        self.clear_mapping_area()
