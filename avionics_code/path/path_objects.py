import math
import avionics_code.helpers.geometrical_functions as g_f
import avionics_code.path.path_functions as p_f
import avionics_code.helpers.parameters as para

OBSTACLE_DISTANCE_FOR_VALID_PASS = para.OBSTACLE_DISTANCE_FOR_VALID_PASS
OBSTACLE_DISTANCE_FOR_VALID_NODE = para.OBSTACLE_DISTANCE_FOR_VALID_NODE
BORDER_DISTANCE_FOR_VALID_PASS = para.BORDER_DISTANCE_FOR_VALID_PASS
BORDER_DISTANCE_FOR_VALID_NODE = para.BORDER_DISTANCE_FOR_VALID_NODE
NODE_MIN_DISTANCE = para.NODE_MIN_DISTANCE
OBSTACLE_ORBIT_RATIO = para.OBSTACLE_ORBIT_RATIO
MIN_DEVIATION_FOR_ALTERNATIVE_PATH = math.radians(para.MIN_DEVIATION_FOR_ALTERNATIVE_PATH)
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS


class MapObject:
    """Parent class for Map objects (waypoints, border vertices, obstacles)"""

    def __init__(self, position_tuple):
        self.pos = position_tuple


class Vertex(MapObject):
    """border vertex object with x,y position"""

    pass


class MapArea:
    """Parent class for Map areas (border, search area, mapping, etc)"""

    def __init__(self, vertices):
        self.vertices = vertices
        
    def compute_simple_edges(self):
        """Returns list of edges of the border made with two vertices"""

        edges_list = list()
        if len(self.vertices) > 1:
            for i in range(len(self.vertices)):
                edges_list.append((self.vertices[i - 1], self.vertices[i]))
        return edges_list

    def polygon(self):
        """Returns a polygon of the positions of the vertices"""

        return [vertex.pos for vertex in self.vertices]

    def waypoint_in_area(self, map_object):
        """checks if MapObject is inside area"""

        if len(self.vertices) > 2:
            return g_f.point_inside_polygon(map_object.pos, self.polygon())
        else:
            return None


class Obstacle(MapObject):
    """obstacle object with x,y position and radius r"""

    def __init__(self, position_tuple, r):
        super().__init__(position_tuple)
        self.r = r

    def create_tangent_nodes(self, waypoint_1):
        """Creates tangent nodes from the given waypoint
        toward the obstacle to allow dodging it"""

        m_i = waypoint_1.mission_index

        distance_to_waypoint = g_f.distance_2d(waypoint_1.pos, self.pos)
        # Check if Waypoint is the same as center of obstacle
        if g_f.float_eq(distance_to_waypoint, 0):
            return None, None
        # Check if Waypoint is far enough for normal node creation
        elif distance_to_waypoint >= self.r + 2 * OBSTACLE_DISTANCE_FOR_VALID_NODE:
            # safe distance radius
            safe_r = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            n_1, n_2 = g_f.tangent_points(waypoint_1.pos, self.pos, safe_r)
            return Waypoint(m_i, n_1, parent_obstacle=self), Waypoint(m_i, n_2, parent_obstacle=self)

        # Check if Waypoint is inside obstacle
        elif distance_to_waypoint <= self.r:
            return None, None
        # Check if Waypoint is in obstacle orbit for dense node creation
        else:
            # find angle of rotation around orbit
            factor_1 = self.r + OBSTACLE_DISTANCE_FOR_VALID_PASS * OBSTACLE_ORBIT_RATIO
            factor_2 = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            orbit_angle = 2 * math.acos(factor_1 / factor_2)

            # find vector from center of obstacle to waypoint
            axis_vector = g_f.sub_vectors(waypoint_1.pos, self.pos)

            # rotate axis vector around orbit_angle
            new_axis_1 = g_f.rotate_vector(axis_vector, orbit_angle)
            new_axis_2 = g_f.rotate_vector(axis_vector, -orbit_angle)

            # add vectors to center of obstacle location type
            node_1 = Waypoint(m_i, g_f.add_vectors(self.pos, new_axis_1), parent_obstacle=self)
            node_2 = Waypoint(m_i, g_f.add_vectors(self.pos, new_axis_2), parent_obstacle=self)

            return node_1, node_2


class Border(MapArea):
    """border object that create waypoints on its concave vertices"""
    
    def __init__(self, vertices):
        super().__init__(vertices)
        # list of waypoints on concave border vertices
        # that allow to dodge the border
        self.border_waypoints = self.create_vertex_nodes()

    def create_vertex_nodes(self):
        """Create vertex nodes on concave border vertices to dodge edges"""

        node_list = list()
        if len(self.vertices) > 2:
            # go through all vertices to see if a node can be created on them
            for vertex_index in range(len(self.vertices)):
                # Get current, previous and next border vertices
                cur_vertex = self.vertices[vertex_index % len(self.vertices)]
                prev_vertex = self.vertices[(vertex_index - 1) % len(self.vertices)]
                next_vertex = self.vertices[(vertex_index + 1) % len(self.vertices)]

                # Get the vectors linking them
                current_to_previous = g_f.sub_vectors(prev_vertex.pos, cur_vertex.pos)
                current_to_next = g_f.sub_vectors(next_vertex.pos, cur_vertex.pos)
                # Check if the vectors are non-nul
                null_1 = g_f.float_eq_2d(current_to_previous, (0, 0))
                null_2 = g_f.float_eq_2d(current_to_next, (0, 0))
                if not (null_1 or null_2):

                    current_to_previous = g_f.unit_vector(current_to_previous)
                    current_to_next = g_f.unit_vector(current_to_next)

                    # find direction away from border vertex and add a node there
                    vector_away = g_f.add_vectors(current_to_previous, current_to_next)

                    # Check if the vectors do not add up to a null vector
                    if not g_f.float_eq_2d(vector_away, (0, 0)):
                        unit_away = g_f.unit_vector(vector_away)
                        offset_away = g_f.scale_vector(unit_away, BORDER_DISTANCE_FOR_VALID_NODE)
                        new_node = g_f.sub_vectors(cur_vertex.pos, offset_away)
                        node_list.append(Waypoint(-1, new_node, parent_vertex=cur_vertex))
        return node_list

    def waypoint_is_too_close(self, way):
        """checks waypoint is too close assuming
        it is inside the border"""

        vertices = self.vertices
        for vertex_index in range(len(vertices)):
            vertex_1 = vertices[vertex_index].pos
            vertex_2 = vertices[(vertex_index + 1) % len(vertices)].pos
            safe_radius = BORDER_DISTANCE_FOR_VALID_NODE
            if g_f.seg_to_disk_intersection(vertex_1, vertex_2, way.pos, safe_radius):
                return True

        return False


class Waypoint(MapObject):
    """z = None for Waypoints created during straight path generation,
    
    Waypoint object with x,y,z position, parent object,
    and turn waypoint
    
    parent obstacle: node created on an obstacle during straight
                path generation
    Parent border vertex:  node created on a border vertex during
                straight path generation
    pre turn waypoint: waypoint to make sure the plane goes through it
    post turn waypoint: waypoint to make sure the plane goes through it
    mission index: index for what part of the mission they are
    is mission: is part of the mission state, so it was not added
                for another purpose
    target: target for action, likely off axis imaging"""
    
    def __init__(self, mission_index, position_tuple, z=None,
                 parent_obstacle=None, parent_vertex=None, is_mission=False, target=None):
        super().__init__(position_tuple)
        self.mission_index = mission_index
        self.z = z
        self.parent_obstacle = parent_obstacle
        self.parent_vertex = parent_vertex
        self.pre_turn_waypoint = None
        self.post_turn_waypoint = None
        self.is_mission = is_mission
        self.target = target
    
    def distance_2d_to(self, other_way):
        """computes 2d distance to other Waypoint"""
        
        return g_f.distance_2d(self.pos, other_way.pos)

    def distance_3d_to(self, other_way):
        """computes 2d distance to other Waypoint"""
    
        return math.hypot(self.pos[0] - other_way.pos[0], self.pos[1] - other_way.pos[1], self.z - other_way.z)

    def is_valid(self, profile_1):
        """Checks if this waypoint is valid in that it is
        not inside an obstacles or outside the border"""

        # check if is is inside some obstacle
        for obstacle_i in profile_1.obstacles:
            # check if the seg connecting the two waypoints intersects with the obstacle
            center = obstacle_i.pos
            safe_radius = obstacle_i.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            if g_f.point_inside_circle(self.pos, center, safe_radius):
                return False

        # check if point is outside the border
        inside_border = profile_1.border.waypoint_in_area(self)
        if not(inside_border or inside_border is None):
            return False

        # check if point is too close to border
        if profile_1.border.waypoint_is_too_close(self):
            return False

        return True

    def is_connectable_to(self, other_way, profile_1):
        """Checks if this waypoint can be connected to another through
        a straight line without hitting an obstacle/border"""

        # Extract map info
        obstacles = profile_1.obstacles
        border_1 = profile_1.border

        # check if there is a collision with some obstacle
        for obstacle_i in obstacles:
            # check if the seg connecting the two waypoints intersects with the obstacle
            center = obstacle_i.pos
            safe_radius = obstacle_i.r + OBSTACLE_DISTANCE_FOR_VALID_PASS
            other_pos = other_way.pos
            if g_f.seg_to_disk_intersection(self.pos, other_pos, center, safe_radius):
                return False

        # check if there is a collision with some border line
        for vertex_index in range(len(border_1.vertices)):
            vertex_1 = border_1.vertices[vertex_index]
            vertex_2 = border_1.vertices[(vertex_index + 1) % len(border_1.vertices)]
            # check if the seg connecting the two waypoints intersects with the border line
            w_node = other_way.pos
            v_node_1 = vertex_1.pos
            v_node_2 = vertex_2.pos
            b_s = BORDER_DISTANCE_FOR_VALID_PASS
            if g_f.seg_to_seg_with_safety(self.pos, w_node, v_node_1, v_node_2, b_s):
                return False

        return True

    def is_going_backwards(self, waypoint_1, waypoint_2):
        """Check if path is going backwards"""
    
        last_to_current = g_f.sub_vectors(self.pos, waypoint_1.pos)
        current_to_next = g_f.sub_vectors(waypoint_2.pos, self.pos)
        if g_f.distinct_points_3(last_to_current, current_to_next, (0, 0)):
            return abs(g_f.find_angle(last_to_current, current_to_next)) > math.pi / 2
        else:
            return False

    def is_bouncing(self, waypoint_1, waypoint_2):
        """Check if path through node is not crossing
        over obstacle or border (bouncing off it instead)"""
    
        # cannot bounce without a node created on a map object
        if self.parent_obstacle is None and self.parent_vertex is None:
            return False
        else:
            if self.parent_obstacle is not None:
                w_node_1 = waypoint_1.pos
                w_node_2 = waypoint_2.pos
                o_node = self.parent_obstacle.pos
                return not g_f.is_crossing_over_edge(w_node_1, w_node_2, self.pos, o_node)
            elif self.parent_vertex is not None:
                w_node_1 = waypoint_1.pos
                w_node_2 = waypoint_2.pos
                v_node = self.parent_vertex.pos
                return not g_f.is_crossing_over_edge(w_node_1, w_node_2, self.pos, v_node)

    def is_across_viable_edge(self, waypoint_1, waypoint_2, profile_1):
        """Check if path is going through a node in a straight line"""

        if waypoint_2.is_connectable_to(waypoint_1, profile_1):
            last_to_current = g_f.sub_vectors(self.pos, waypoint_1.pos)
            current_to_next = g_f.sub_vectors(waypoint_2.pos, self.pos)
            if g_f.distinct_points_3(last_to_current, current_to_next, (0, 0)):
                DIV = MIN_DEVIATION_FOR_ALTERNATIVE_PATH
                return abs(g_f.find_angle(last_to_current, current_to_next)) < DIV
            else:
                return False
        else:
            return False

    def turn_before(self, way_1, way_2, profile):
        """adds an turn waypoint to make the turn easier considering pixhawk 4"""

        P_R = PREFERRED_TURN_RADIUS

        node_1 = way_1.pos
        node_2 = self.pos
        node_3 = way_2.pos
        if not g_f.distinct_points_3(node_1, node_2, node_3):
            return None

        # move node 2 by a turning radius away from node 3
        # to take into account the pixhawk 4 turing when it reaches it
        dis = P_R + g_f.distance_2d(node_2, node_3)
        node_2 = g_f.homothety_unit(node_2, node_3, dis)

        # find two possible turn centers that would allow
        # to turn through node 2 to go into node 3
        n_1, n_2 = g_f.unit_normal_vectors_to_line(node_2, node_3)
        turn_cen_1 = g_f.add_vectors(node_2, g_f.scale_vector(n_1, P_R))
        turn_cen_2 = g_f.add_vectors(node_2, g_f.scale_vector(n_2, P_R))

        # Check which one is on the side of node 1
        if not g_f.seg_to_line_intersection(node_1, turn_cen_1, node_2, node_3):
            turn_cen = turn_cen_1
        else:
            turn_cen = turn_cen_2

        # Make sure node 1 is not inside the turn circle
        if g_f.distance_2d(node_1, turn_cen) > P_R:

            # find tangent points on the turn circle coming node_1
            node_a, node_b = g_f.tangent_points(node_1, turn_cen, P_R)

            # find which tangent point gives the right orientation for turning
            vec_1 = g_f.sub_vectors(node_2, turn_cen)
            vec_2 = g_f.sub_vectors(node_3, node_2)
            vec_3 = g_f.sub_vectors(node_a, node_1)
            vec_4 = g_f.sub_vectors(turn_cen, node_a)
            a_1 = g_f.find_angle(vec_1, vec_2)
            a_2 = g_f.find_angle(vec_3, vec_4)
            if a_1 * a_2 > 0:
                turn_w_pos = node_a
            else:
                turn_w_pos = node_b

            # move the turning waypoint by a turning radius away from node 1
            # to take into account the pixhawk 4 turing when it reaches it
            dis = P_R + g_f.distance_2d(turn_w_pos, node_1)
            new_pos = g_f.homothety_unit(turn_w_pos, node_1, dis)
            turn_w = Waypoint(self.mission_index, new_pos)

            # check that the picked turning waypoint is valid
            if turn_w.is_connectable_to(way_1, profile) and turn_w.is_connectable_to(self, profile):
                self.pre_turn_waypoint = turn_w
                return True
            else:
                return False
        else:
            return False

    def turn_after(self, way_1, way_2, profile):
        """adds a turn waypoint after itself to make sure the plane
        goes through it"""

        P_R = PREFERRED_TURN_RADIUS

        # get all three points involved in the turn
        node_1 = way_1.pos
        node_2 = self.pos
        node_3 = way_2.pos

        if not g_f.distinct_points_3(node_1, node_2, node_3):
            return None

        # find a point after node 2 that is a turn radius away from it
        dis = g_f.distance_2d(node_1, node_2) + P_R
        turn_pos = g_f.homothety_unit(node_2, node_1, dis)

        TURN_NUM = 50
        node_1_to_2 = g_f.sub_vectors(node_2, node_1)
        node_2_to_3 = g_f.sub_vectors(node_3, node_2)
        base_angle = g_f.find_angle(node_1_to_2, node_2_to_3)

        for turn_index in range(TURN_NUM):
            turn_pos = g_f.rotate_vector_with_center(turn_pos, node_2, base_angle/TURN_NUM)

            # check the turn waypoint is valid
            turn_w = Waypoint(self.mission_index, turn_pos)
            if turn_w.is_connectable_to(self, profile) and turn_w.is_connectable_to(way_2, profile):
                if turn_index == 0:
                    self.post_turn_waypoint = turn_w
                    return True
                elif self.turn_before(way_1, turn_w, profile):
                    self.post_turn_waypoint = turn_w
                    return True

        # no turning points could be made
        return False


class Path:
    """path made from multiple waypoints"""

    def __init__(self, waypoint_list):
        self.waypoint_list = waypoint_list
        self.simple_distance_2d = None

    def compute_simple_edges(self):
        """Returns list of edges of the path without turn waypoints"""

        edges_list = list()
        if len(self.waypoint_list) > 1:
            for i in range(len(self.waypoint_list) - 1):
                edges_list.append((self.waypoint_list[i], self.waypoint_list[i + 1]))
        return edges_list

    def compute_simple_distance_2d(self):
        """computes 2D distance of the path without
        considering altitude or turn waypoints"""

        return sum(edge[0].distance_2d_to(edge[1]) for edge in self.compute_simple_edges())

    def compute_true_distance_2d(self):
        """computes 2D distance of the path with turn waypoints"""

        return sum(edge[0].distance_2d_to(edge[1]) for edge in self.compute_simple_edges())

    def distance_2d_update(self):
        """updates 2D distance"""

        self.simple_distance_2d = self.compute_simple_distance_2d()

    def path_distance_to_waypoint(self, next_waypoint):
        """find total distance of path plus an extra waypoint"""

        last_node = self.waypoint_list[- 1]
        self.distance_2d_update()
        return self.simple_distance_2d + g_f.distance_2d(last_node.pos, next_waypoint.pos)

    def curve(self, profile):
        """Curve path by adding turn waypoints"""

        if len(self.waypoint_list) > 2:
            for way_index in range(1, len(self.waypoint_list) - 1):

                prev_way = self.waypoint_list[way_index - 1]
                cur_way = self.waypoint_list[way_index]
                next_way = self.waypoint_list[way_index + 1]

                if prev_way.post_turn_waypoint is not None:
                    prev_way = prev_way.post_turn_waypoint

                # if mission point, make waypoint to go through it
                if cur_way.is_mission:
                    cur_way.turn_after(prev_way, next_way, profile)

    def is_valid(self, profile_1):
        """Check if path is valid including turn waypoints"""

        for w_index, waypoint_i in enumerate(self.waypoint_list):
            if 0 < w_index < len(self.waypoint_list) - 1:
                c_p = self.waypoint_list[w_index-1]
                if c_p.off_waypoint is not None:
                    c_p = c_p.off_waypoint
                c_w = waypoint_i
                if c_w.turn_waypoint is not None:
                    c_p = c_w.turn_waypoint
                next_waypoint = self.waypoint_list[w_index+1]
                c_off_w = c_w.off_waypoint
                c_off_next = next_waypoint.off_waypoint
                al_w = next_waypoint.turn_waypoint
                if c_off_w is not None:
                    turn_cen = c_off_w.turn_waypoint
                    if c_off_w.pos != (None, None):
                        if not c_w.is_arc_connectable_to(c_p, c_off_w, turn_cen, profile_1):
                            return False
                        c_w = c_off_w
                    else:
                        center = g_f.center_2d(c_w.pos, next_waypoint.pos)
                        Danger_area = Danger_zone(center, 2*PREFERRED_TURN_RADIUS)
                        if not Danger_area.is_free(profile_1):
                            return False

                if al_w is None:
                    if not c_w.is_connectable_to(next_waypoint, profile_1):
                        return False
                else:
                    turn_cen_2 = c_off_next.turn_waypoint
                    if not c_w.is_connectable_to(al_w, profile_1):
                        return False
                    if not al_w.is_arc_connectable_to(c_w, next_waypoint, turn_cen_2, profile_1):
                        return False
        return True


class Danger_zone:
    """Area where the flight is hard to predict"""

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def is_free(self, profile_1):
        """Check if the area has no obstacles or borders in it"""

        # Extract map info
        obstacles = profile_1.obstacles
        border_1 = profile_1.border

        # check if there is a collision with some obstacle
        for obstacle_i in obstacles:

            # check if the seg connecting the two waypoints intersects with the obstacle
            center = obstacle_i.pos
            if g_f.circle_to_circle_intersection(self.center, self.radius, center, obstacle_i.r):
                return False

        # check if there is a collision with some border line
        for vertex_index in range(len(border_1.vertices)):
            vertex_1 = border_1.vertices[vertex_index]
            vertex_2 = border_1.vertices[(vertex_index + 1) % len(border_1.vertices)]
            # check if the seg connecting the two waypoints intersects with the border line
            v_node_1 = vertex_1.pos
            v_node_2 = vertex_2.pos
            if g_f.seg_to_disk_intersection(v_node_1, v_node_2, self.center, self.radius):
                return False

        return True
