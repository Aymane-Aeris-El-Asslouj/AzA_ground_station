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
    and alleviation waypoint
    
    parent obstacle: node created on an obstacle during straight
                path generation
    Parent border vertex:  node created on a border vertex during
                straight path generation
    alleviation waypoint: waypoint added during path curving to
                make the turn on the waypoint easier
    off waypoint: The position that the plane will go to
                because of inertia after crossing the waypoint
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
        self.alleviation_waypoint = None
        self.off_waypoint = None
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

    def is_arc_connectable_to(self, base_w, initial_w, turn_cen_w, profile_1):
        """Checks if an arc can be constructed from self
            to initial_w with center turn_cen_w coming from base_w"""
        
        node_1 = base_w.pos
        node_2 = self.pos
        node_3 = initial_w.pos
        turn_cen = turn_cen_w.pos
        if g_f.distinct_points_4(node_1, node_2, node_3, turn_cen):
            middle = g_f.center_2d(node_2, node_3)
            distance_to_center = g_f.distance_2d(self.pos, turn_cen)
            if g_f.distinct_points_2(middle, turn_cen):
                middle_1 = g_f.homothety_unit(middle, turn_cen, distance_to_center)
                middle_2 = g_f.homothety_unit(middle, turn_cen, -distance_to_center)
            else:
                n_1, n_2 = g_f.unit_normal_vectors_to_line(node_2, node_3)
                vec1 = g_f.scale_vector(n_1, distance_to_center)
                vec2 = g_f.scale_vector(n_2, distance_to_center)
                middle_1 = g_f.add_vectors(middle, vec1)
                middle_2 = g_f.add_vectors(middle, vec2)
            
            vec_1 = g_f.sub_vectors(turn_cen, node_2)
            vec_2 = g_f.sub_vectors(turn_cen, middle_1)
            a_1 = g_f.find_angle(vec_1, g_f.sub_vectors(node_1, node_2))
            a_2 = g_f.find_angle(vec_2, g_f.sub_vectors(node_3, middle_1))

            # find which tangent point gives the right orientation for turning
            if a_1 * a_2 < 0:
                arc_middle_w = Waypoint(self.mission_index, middle_1)
            else:
                arc_middle_w = Waypoint(self.mission_index, middle_2)

            c_1 = arc_middle_w.is_connectable_to(self, profile_1)
            c_2 = arc_middle_w.is_connectable_to(initial_w, profile_1)
            return c_1 and c_2
        else:
            return True
        
        return g_f.float_eq_2d(self.pos, waypoint_1.pos)

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

    def alleviate(self, waypoint_1, waypoint_2, profile_1):
        """adds an alleviation waypoint to make the turn easier"""

        p_r = PREFERRED_TURN_RADIUS

        node_1 = waypoint_1.pos
        node_2 = self.pos
        node_3 = waypoint_2.pos
        if not g_f.distinct_points_3(node_1, node_2, node_3):
            return

        # find two possible turn centers that would allow
        # to turn through node 2 to go into node 3
        n_1, n_2 = g_f.unit_normal_vectors_to_line(node_2, node_3)
        turn_cen_1 = g_f.add_vectors(node_2, g_f.scale_vector(n_1, p_r))
        turn_cen_2 = g_f.add_vectors(node_2, g_f.scale_vector(n_2, p_r))

        # Check which one is on the side of node 1
        if not g_f.seg_to_line_intersection(node_1, turn_cen_1, node_2, node_3):
            turn_cen = turn_cen_1
        else:
            turn_cen = turn_cen_2

        alleviation_w = None

        a_1 = g_f.find_angle((0, 1), g_f.sub_vectors(turn_cen, node_2))
        a_2 = g_f.find_angle((0, 1), g_f.sub_vectors(node_3, node_2))
        if -3.14 < a_2 - a_1 < 0 or 3.14 < a_2 - a_1 < 6.30:
            rotation_unit = -0.1
        else:
            rotation_unit = +0.1

        new_turn_cen = turn_cen
        # Angle by which the turn center is shifted when the default turn is not possible
        turn_offset = 0
        
        vec_1 = g_f.sub_vectors(node_1, node_2)
        vec_2 = g_f.sub_vectors(new_turn_cen, node_2)
        a_x = g_f.find_angle(vec_1, vec_2)

        while -math.pi/2 < turn_offset < math.pi/2 or -math.pi/2 < a_x < math.pi/2:

            a_x = g_f.find_angle(vec_1, vec_2)

            # Make sure node 1 is not inside the turn circle
            if g_f.distance_2d(node_1, new_turn_cen) > p_r:

                # find tangent points on the turn circle coming node_1
                node_a, node_b = g_f.tangent_points(node_1, new_turn_cen, p_r)
                vec_1 = g_f.sub_vectors(node_2, new_turn_cen)
                vec_2 = g_f.sub_vectors(node_3, node_2)
                vec_3 = g_f.sub_vectors(node_a, node_1)
                vec_4 = g_f.sub_vectors(new_turn_cen, node_a)
                a_1 = g_f.find_angle(vec_1, vec_2)
                a_2 = g_f.find_angle(vec_3, vec_4)

                # find which tangent point gives the right orientation for turning
                if a_1 * a_2 > 0:
                    picked_w = Waypoint(self.mission_index, node_a)
                else:
                    picked_w = Waypoint(self.mission_index, node_b)

                new_center_w = Waypoint(self.mission_index, new_turn_cen)

                if picked_w.is_connectable_to(waypoint_1, profile_1):
                    if picked_w.is_arc_connectable_to(waypoint_1, self, new_center_w, profile_1):
                        if -math.pi/2 < turn_offset < math.pi/2:
                            alleviation_w = picked_w
                            break

            # Change the turn center
            turn_offset += rotation_unit
            new_turn_cen = g_f.rotate_vector_with_center(turn_cen, node_2, turn_offset)

        if alleviation_w is None:
            self.alleviation_waypoint = alleviation_w
        elif g_f.distance_2d(alleviation_w.pos, self.pos) > 2*NODE_MIN_DISTANCE:
            self.alleviation_waypoint = alleviation_w
        else:
            self.alleviation_waypoint = None

        # store turn center as the alleviation waypoint of off waypoint
        self.off_waypoint = Waypoint(self.mission_index, (0, 0))
        self.off_waypoint.alleviation_waypoint = Waypoint(self.mission_index, new_turn_cen)

    def off_shoot(self, prev_waypoint, next_waypoint):
        """Determines where the off waypoint is"""

        p_r = PREFERRED_TURN_RADIUS
        center = self.off_waypoint.alleviation_waypoint.pos
        node_1 = prev_waypoint.pos
        node_2 = self.pos
        node_3 = next_waypoint.pos

        if g_f.distance_2d(node_3, center) > p_r:
            # find tangent points on the turn circle coming node_1
            node_a, node_b = g_f.tangent_points(node_3, center, p_r)
            a_1 = g_f.find_angle(g_f.sub_vectors(center, node_2), g_f.sub_vectors(node_1, node_2))
            a_2 = g_f.find_angle(g_f.sub_vectors(center, node_a), g_f.sub_vectors(node_3, node_a))

            a_3 = g_f.find_angle(g_f.sub_vectors(node_1, node_2), g_f.sub_vectors(node_3, node_2))

            if abs(a_3) < math.pi / 2 and self.alleviation_waypoint is not None:
                # find which tangent point gives the right orientation for turning
                if g_f.distance_2d(node_a, node_2) < g_f.distance_2d(node_b, node_2):
                    self.off_waypoint.pos = node_a
                else:
                    self.off_waypoint.pos = node_b
            else:
                # find which tangent point gives the right orientation for turning
                if a_1 * a_2 < 0:
                    self.off_waypoint.pos = node_a
                else:
                    self.off_waypoint.pos = node_b
        else:
            self.off_waypoint.pos = None, None


class Path:
    """path made from multiple waypoints"""

    def __init__(self, waypoint_list):
        self.waypoint_list = waypoint_list
        self.simple_distance_2d = None

    def compute_simple_edges(self):
        """Returns list of edges of the path without alleviation waypoints"""

        edges_list = list()
        if len(self.waypoint_list) > 1:
            for i in range(len(self.waypoint_list) - 1):
                edges_list.append((self.waypoint_list[i], self.waypoint_list[i + 1]))
        return edges_list

    def compute_simple_distance_2d(self):
        """computes 2D distance of the path without
        considering altitude or alleviation waypoints"""

        return sum(edge[0].distance_2d_to(edge[1]) for edge in self.compute_simple_edges())

    def compute_true_distance_2d(self):
        """computes 2D distance of the path with alleviation waypoints"""

        return sum(edge[0].distance_2d_to(edge[1]) for edge in self.compute_simple_edges())

    def distance_2d_update(self):
        """updates 2D distance"""

        self.simple_distance_2d = self.compute_simple_distance_2d()

    def path_distance_to_waypoint(self, next_waypoint):
        """find total distance of path plus an extra waypoint"""

        last_node = self.waypoint_list[- 1]
        self.distance_2d_update()
        return self.simple_distance_2d + g_f.distance_2d(last_node.pos, next_waypoint.pos)

    def curve(self, profile_1):
        """Curve path by adding alleviation waypoints"""

        """add internal alleviation waypoints inside each path"""
        if len(self.waypoint_list) > 2:
            for way_index in range(1, len(self.waypoint_list) - 1):
                prev_waypoint = self.waypoint_list[- 1 - way_index - 1]
                cur_waypoint = self.waypoint_list[- 1 - way_index]
                next_waypoint = self.waypoint_list[- 1 - way_index + 1]
                if next_waypoint.alleviation_waypoint is not None:
                    next_waypoint = next_waypoint.alleviation_waypoint
                cur_waypoint.alleviate(prev_waypoint, next_waypoint, profile_1)

    def off_shoot(self):
        """finds off waypoints for path"""

        if len(self.waypoint_list) > 2:
            for way_index in range(1, len(self.waypoint_list) - 1):
                prev_waypoint = self.waypoint_list[way_index - 1]
                cur_waypoint = self.waypoint_list[way_index]
                next_waypoint = self.waypoint_list[way_index + 1]
                if cur_waypoint.alleviation_waypoint is not None:
                    prev_waypoint = cur_waypoint.alleviation_waypoint
                if next_waypoint.alleviation_waypoint is not None:
                    next_waypoint = next_waypoint.alleviation_waypoint
                cur_waypoint.off_shoot(prev_waypoint, next_waypoint)

    def is_valid(self, profile_1):
        """Check if path is valid including alleviation waypoints"""

        for w_index, waypoint_i in enumerate(self.waypoint_list):
            if 0 < w_index < len(self.waypoint_list) - 1:
                c_p = self.waypoint_list[w_index-1]
                if c_p.off_waypoint is not None:
                    c_p = c_p.off_waypoint
                c_w = waypoint_i
                if c_w.alleviation_waypoint is not None:
                    c_p = c_w.alleviation_waypoint
                next_waypoint = self.waypoint_list[w_index+1]
                c_off_w = c_w.off_waypoint
                c_off_next = next_waypoint.off_waypoint
                al_w = next_waypoint.alleviation_waypoint
                if c_off_w is not None:
                    turn_cen = c_off_w.alleviation_waypoint
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
                    turn_cen_2 = c_off_next.alleviation_waypoint
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
