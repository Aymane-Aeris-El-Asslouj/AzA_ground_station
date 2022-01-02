from avionics_code.references import parameters as para
from avionics_code.utility_functions import geometrical_functions as g_f
from avionics_code.references import global_variables as g_v

import math

OBSTACLE_DISTANCE_FOR_VALID_PASS = para.OBSTACLE_DISTANCE_FOR_VALID_PASS
OBSTACLE_DISTANCE_FOR_VALID_NODE = para.OBSTACLE_DISTANCE_FOR_VALID_NODE
OBSTACLE_DISTANCE_FOR_ORBIT = para.OBSTACLE_DISTANCE_FOR_ORBIT
OBSTACLE_DISTANCE_FOR_ORBIT_START = para.OBSTACLE_DISTANCE_FOR_ORBIT_START

BORDER_DISTANCE_FOR_VALID_PASS = para.BORDER_DISTANCE_FOR_VALID_PASS
BORDER_DISTANCE_FOR_VALID_NODE = para.BORDER_DISTANCE_FOR_VALID_NODE
BORDER_DISTANCE_FOR_BORDER_NODE = para.BORDER_DISTANCE_FOR_BORDER_NODE

NODE_MIN_DISTANCE = para.NODE_MIN_DISTANCE
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS
CONFIRMATION_DISTANCE = para.CONFIRMATION_DISTANCE
TURN_SEARCH_ITERATIONS = para.TURN_SEARCH_ITERATIONS
MAX_CLIMBING_RATIO = para.MAX_CLIMBING_RATIO
MIN_DEVIATION_FOR_ALTERNATIVE_PATH = para.MIN_DEVIATION_FOR_ALTERNATIVE_PATH


class MapObject:
    """Parent class for Map objects (waypoints,
    border vertices, obstacles)"""

    def __init__(self, position_tuple):
        self.pos = position_tuple


class Vertex(MapObject):
    """border vertex object with x,y position"""

    pass


class MapArea:
    """Parent class for Map areas
    (border, search area, mapping, etc.)"""

    def __init__(self, vertices):
        self.vertices = vertices

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

    def create_tangent_nodes(self, waypoint_1, profile, mission_type):
        """Creates tangent nodes from the given waypoint
        toward the obstacle to allow dodging it"""

        m_i = mission_type

        distance_to_waypoint = g_f.distance_2d(waypoint_1.pos, self.pos)
        # Check if Waypoint is far enough for normal node creation
        if distance_to_waypoint >= self.r + OBSTACLE_DISTANCE_FOR_ORBIT_START:
            # safe distance radius
            safe_r = self.r + OBSTACLE_DISTANCE_FOR_ORBIT
            n_1, n_2 = g_f.tangent_points(waypoint_1.pos, self.pos, safe_r)

            # add vectors to center of obstacle location type
            node_1 = Waypoint(m_i, n_1, parent_obstacle=self)
            node_2 = Waypoint(m_i, n_2, parent_obstacle=self)

            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if not node_1.is_valid(profile, O_V_N, B_V_N):
                node_1 = None
            if not node_2.is_valid(profile, O_V_N, B_V_N):
                node_2 = None

            return node_1, node_2

        # Check if Waypoint is inside obstacle
        elif distance_to_waypoint <= self.r:
            return None, None
        # Check if Waypoint is in obstacle orbit for dense node creation
        else:
            # find angle of rotation around orbit
            factor_1 = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            factor_2 = self.r + OBSTACLE_DISTANCE_FOR_ORBIT
            orbit_angle = 2 * math.acos(factor_1 / factor_2)

            # find vector from center of obstacle to waypoint with orbit radius
            axis_vector = g_f.sub_vectors(waypoint_1.pos, self.pos)
            axis_vector = g_f.unit_vector(axis_vector)
            axis_vector = g_f.scale_vector(axis_vector, factor_2)

            # rotate axis vector around orbit_angle
            new_axis_1 = g_f.rotate_vector(axis_vector, orbit_angle)
            new_axis_2 = g_f.rotate_vector(axis_vector, -orbit_angle)

            # add vectors to center of obstacle location type
            node_1 = Waypoint(m_i, g_f.add_vectors(self.pos, new_axis_1),
                              parent_obstacle=self)
            node_2 = Waypoint(m_i, g_f.add_vectors(self.pos, new_axis_2),
                              parent_obstacle=self)

            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if not node_1.is_valid(profile, O_V_N, B_V_N):
                node_1 = None
            if not node_2.is_valid(profile, O_V_N, B_V_N):
                node_2 = None

            return node_1, node_2


class Border(MapArea):
    """border object that create waypoints on its concave vertices"""
    
    def __init__(self, vertices):
        super().__init__(vertices)
        # list of waypoints on concave border vertices
        # that allow to dodge the border
        self.border_waypoints = None

    def create_vertex_nodes(self, profile):
        """Create vertex nodes on concave border vertices to dodge edges"""

        node_list = list()
        if len(self.vertices) > 2:
            # go through all vertices to see if a node can be created on them
            for vertex_index in range(len(self.vertices)):
                # Get current, previous and next border vertices
                len_v = len(self.vertices)
                cur_vertex = self.vertices[vertex_index % len_v]
                prev_vertex = self.vertices[(vertex_index - 1) % len_v]
                next_vertex = self.vertices[(vertex_index + 1) % len_v]

                # Get the vectors linking them
                sub = g_f.sub_vectors
                cur_to_previous = sub(prev_vertex.pos, cur_vertex.pos)
                cur_to_next = sub(next_vertex.pos, cur_vertex.pos)
                # Check if the vectors are non-nul
                null_1 = g_f.float_eq_2d(cur_to_previous, (0, 0))
                null_2 = g_f.float_eq_2d(cur_to_next, (0, 0))
                if not (null_1 or null_2):

                    cur_to_previous = g_f.unit_vector(cur_to_previous)
                    cur_to_next = g_f.unit_vector(cur_to_next)

                    # find direction away from border vertex and add a node there
                    vector_away = g_f.add_vectors(cur_to_previous, cur_to_next)

                    # Check if the vectors do not add up to a null vector
                    if not g_f.float_eq_2d(vector_away, (0, 0)):
                        unit_away = g_f.unit_vector(vector_away)
                        B_B_N = BORDER_DISTANCE_FOR_BORDER_NODE
                        offset_away = g_f.scale_vector(unit_away, B_B_N)
                        new_node = g_f.sub_vectors(cur_vertex.pos, offset_away)
                        border_way = Waypoint(-1, new_node,
                                              parent_vertex=cur_vertex)
                        O_V_D = OBSTACLE_DISTANCE_FOR_VALID_NODE
                        B_V_D = BORDER_DISTANCE_FOR_VALID_NODE
                        if border_way.is_valid(profile, O_V_D, B_V_D):
                            node_list.append(border_way)
        self.border_waypoints = node_list

    def waypoint_is_too_close(self, way, distance):
        """checks waypoint is too close assuming
        it is inside the border"""

        vertices = self.vertices
        for vertex_index in range(len(vertices)):
            vertex_1 = vertices[vertex_index].pos
            vertex_2 = vertices[(vertex_index + 1) % len(vertices)].pos
            safe_radius = distance
            if g_f.seg_to_disk_intersection(vertex_1, vertex_2,
                                            way.pos, safe_radius):
                return True

        return False


class Waypoint(MapObject):
    """z = None for Waypoints created during straight path generation,
    
    parent obstacle: node created on an obstacle during straight
                path generation
    Parent border vertex:  node created on a border vertex during
                straight path generation

    pre turn waypoint: waypoint to make sure the plane goes through it
    post turn waypoint: waypoint to make sure the plane goes through it

    mission type: index for what part of the mission they are

    is mission: -1 if not a mission waypoint, 1 if an active mission
                waypoint 0 if an inactive mission waypoint

    target: target for action, likely off axis imaging"""
    
    def __init__(self, mission_type, position_tuple, z=None,
                 parent_obstacle=None, parent_vertex=None,
                 is_mission=g_v.Activity.NA, target=None):
        super().__init__(position_tuple)
        self.mission_type = mission_type
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
        """computes 3d distance to other Waypoint"""

        xyz_1 = self.pos[0], self.pos[1], self.z,
        xyz_2 = other_way.pos[0], other_way.pos[1], other_way.z,
        return g_f.distance_3d(xyz_1, xyz_2)

    def is_valid(self, profile_1, obstacle_distance, border_distance):
        """Checks if this waypoint is valid in that it is
        not inside an obstacles or outside the border"""

        # check if is inside some obstacle
        for obstacle_i in profile_1.obstacles:
            # check if the seg connecting the two waypoints
            # intersects with the obstacle
            center = obstacle_i.pos
            safe_radius = obstacle_i.r + obstacle_distance
            if g_f.point_inside_circle(self.pos, center, safe_radius):
                return False

        # check if point is outside the border
        inside_border = profile_1.border.waypoint_in_area(self)
        if not(inside_border or inside_border is None):
            return False

        # check if point is too close to border
        if profile_1.border.waypoint_is_too_close(self, border_distance):
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
            # check if the seg connecting the two waypoints
            # intersects with the obstacle
            center = obstacle_i.pos
            safe_radius = obstacle_i.r + OBSTACLE_DISTANCE_FOR_VALID_PASS
            other_pos = other_way.pos
            if g_f.seg_to_disk_intersection(self.pos, other_pos,
                                            center, safe_radius):
                return False

        # check if there is a collision with some line of border
        len_v = len(border_1.vertices)
        for vertex_index in range(len_v):
            vertex_1 = border_1.vertices[vertex_index]
            
            vertex_2 = border_1.vertices[(vertex_index + 1) % len_v]
            # check if the seg connecting the two waypoints
            # intersects with the line of border
            w_node = other_way.pos
            v_node_1 = vertex_1.pos
            v_node_2 = vertex_2.pos
            b_s = BORDER_DISTANCE_FOR_VALID_PASS
            if g_f.seg_to_seg_with_safety(self.pos, w_node, v_node_1,
                                          v_node_2, b_s):
                return False

        return True

    def is_going_backwards(self, waypoint_1, waypoint_2):
        """Check if path is going backwards"""
    
        last_to_cur = g_f.sub_vectors(self.pos, waypoint_1.pos)
        cur_to_next = g_f.sub_vectors(waypoint_2.pos, self.pos)
        if g_f.distinct_points_3(last_to_cur, cur_to_next, (0, 0)):
            angle = abs(g_f.find_angle(last_to_cur, cur_to_next))
            return angle > math.pi / 2
        else:
            return False

    def is_bouncing(self, waypoint_1, waypoint_2):
        """Check if path through node is not crossing
        over obstacle or border (bouncing off it instead)"""
    
        # cannot bounce without a node created on a map object
        if self.parent_obstacle is None and self.parent_vertex is None:
            return False
        else:
            cross = g_f.is_crossing_over_edge
            if self.parent_obstacle is not None:
                w_node_1 = waypoint_1.pos
                w_node_2 = waypoint_2.pos
                o_node = self.parent_obstacle.pos
                return not cross(w_node_1, w_node_2, self.pos, o_node)
            elif self.parent_vertex is not None:
                w_node_1 = waypoint_1.pos
                w_node_2 = waypoint_2.pos
                v_node = self.parent_vertex.pos
                return not cross(w_node_1, w_node_2, self.pos, v_node)

    def is_across_viable_edge(self, waypoint_1, waypoint_2, profile_1):
        """Check if path is going through a node in a straight line"""

        if waypoint_2.is_connectable_to(waypoint_1, profile_1):
            last_to_cur = g_f.sub_vectors(self.pos, waypoint_1.pos)
            cur_to_next = g_f.sub_vectors(waypoint_2.pos, self.pos)
            non_zero_1 = g_f.distinct_points_2(last_to_cur, (0, 0))
            non_zero_2 = g_f.distinct_points_2(cur_to_next, (0, 0))
            if non_zero_1 and non_zero_2:
                M_D_A_P = MIN_DEVIATION_FOR_ALTERNATIVE_PATH * (math.pi/180)
                angle = abs(g_f.find_angle(last_to_cur, cur_to_next))
                return angle < M_D_A_P
            else:
                return False
        else:
            return False

    def turn_before(self, way_1, way_2, profile):
        """adds a turn waypoint to make the turn
        easier considering pixhawk 4"""

        C_D = CONFIRMATION_DISTANCE

        node_1 = way_1.pos
        node_2 = self.pos
        node_3 = way_2.pos

        # if the points are not distinct, there is no point in curving
        if not g_f.distinct_points_3(node_1, node_2, node_3):
            return None

        # move node 2 by a turning radius away from node 3
        # to take into account the pixhawk 4 turing when it reaches it
        dis = C_D + g_f.distance_2d(node_2, node_3)
        node_2 = g_f.homothety_unit(node_2, node_3, dis)

        # find two possible turn centers that would allow
        # turning through node 2 to go into node 3
        n_1, n_2 = g_f.unit_normal_vectors_to_line(node_2, node_3)
        turn_cen_1 = g_f.add_vectors(node_2, g_f.scale_vector(n_1, C_D))
        turn_cen_2 = g_f.add_vectors(node_2, g_f.scale_vector(n_2, C_D))

        # Check which one is on the side of node 1
        inter = g_f.seg_to_line_intersection
        if not inter(node_1, turn_cen_1, node_2, node_3):
            turn_cen = turn_cen_1
        else:
            turn_cen = turn_cen_2

        # Make sure node 1 is not inside the turn circle
        if g_f.distance_2d(node_1, turn_cen) > C_D:

            # find tangent points on the turn circle coming node_1
            node_a, node_b = g_f.tangent_points(node_1, turn_cen, C_D)

            # find which tangent point gives
            # the right orientation for turning
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

            if g_f.float_eq_2d(turn_w_pos, node_1):
                return True

            # move the turning waypoint by a turning radius away from node 1
            # to take into account the pixhawk 4 turing when it reaches it
            dis = C_D + g_f.distance_2d(turn_w_pos, node_1)
            new_pos = g_f.homothety_unit(turn_w_pos, node_1, dis)
            turn_w = Waypoint(self.mission_type, new_pos)

            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if turn_w.is_valid(profile, O_V_N, B_V_N):
                con = turn_w.is_connectable_to
                if con(way_1, profile) and con(self, profile):
                    self.pre_turn_waypoint = turn_w
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False

    def turn_after(self, way_1, way_2, profile, no_next_way):
        """adds a turn waypoint after itself to make sure the plane
        goes through it"""

        C_D = CONFIRMATION_DISTANCE

        # get all three points involved in the turn
        node_1 = way_1.pos
        node_2 = self.pos
        node_3 = None
        if not no_next_way:
            node_3 = way_2.pos

        # if the points are not distinct, there is no point in curving
        if not no_next_way:
            if not g_f.distinct_points_3(node_1, node_2, node_3):
                return None
        else:
            if not g_f.distinct_points_2(node_1, node_2):
                return None

        # find a point after node 2 that is a turn radius away from it
        dis = g_f.distance_2d(node_1, node_2) + C_D
        turn_pos = g_f.homothety_unit(node_2, node_1, dis)

        for turn_index in range(TURN_SEARCH_ITERATIONS):
            turn_angle = math.pi/TURN_SEARCH_ITERATIONS
            rot_c = g_f.rotate_vector_with_center
            turn_pos = rot_c(turn_pos, node_2, turn_angle)

            turn_w = Waypoint(self.mission_type, turn_pos)

            if not no_next_way:
                valid = turn_w.is_connectable_to(way_2, profile)
            else:
                valid = True

            O_V_N = OBSTACLE_DISTANCE_FOR_VALID_NODE
            B_V_N = BORDER_DISTANCE_FOR_VALID_NODE
            if turn_w.is_valid(profile, O_V_N, B_V_N):
                if turn_w.is_connectable_to(self, profile) and valid:
                    if turn_index == 0:
                        self.post_turn_waypoint = turn_w
                        return True
                    elif self.turn_before(way_1, turn_w, profile):
                        self.post_turn_waypoint = turn_w
                        return True

        # no turning points could be made
        return False

    def check_valid_turn(self, next_waypoint, profile):
        """checks if the turn made through itself toward the
        next waypoint is valid"""

        node_1 = self.pos
        node_2 = self.post_turn_waypoint.pos
        node_3 = next_waypoint.pos

        # check if the turn is not tight
        vec_2_to_1 = g_f.sub_vectors(node_1, node_2)
        vec_2_to_3 = g_f.sub_vectors(node_3, node_2)
        turn_angle = g_f.find_geometrical_angle(vec_2_to_1, vec_2_to_3)
        if not turn_angle < math.pi/4:
            return True

        # find turning waypoints
        P_R = PREFERRED_TURN_RADIUS
        n_1, n_2 = g_f.unit_normal_vectors_to_line(node_1, node_2)
        scaled_n_1 = g_f.scale_vector(n_1, P_R)
        scaled_n_2 = g_f.scale_vector(n_2, P_R)
        turn_1 = g_f.add_vectors(node_1, scaled_n_1)
        turn_2 = g_f.add_vectors(node_1, scaled_n_2)

        # pick the right turning waypoint
        if g_f.seg_to_line_intersection(node_3, turn_1, node_1, node_2):
            turn = turn_2
        else:
            turn = turn_1

        return area_is_valid(turn, P_R, profile, False)

    def curve(self, prev_way, next_way, profile, no_next_way=False):
        """adds turning waypoints to make the plane pass
        through the waypoint if needed"""

        need_curving = self.mission_type == g_v.MissionType.WAYPOINTS
        # if mission point, make waypoint to go through it
        if self.is_mission != g_v.Activity.NA and need_curving:
            # get rid of any previous turning points
            self.pre_turn_waypoint = None
            self.post_turn_waypoint = None

            need_curving = prev_way.mission_type == g_v.MissionType.WAYPOINTS
            if prev_way.post_turn_waypoint is not None and need_curving:
                prev_way = prev_way.post_turn_waypoint

            curved_worked = self.turn_after(prev_way, next_way,
                                            profile, no_next_way)

            # return if there was a problem with the curving
            if curved_worked is not None:
                if not curved_worked:
                    return False
        return True


class Path:
    """path made from multiple waypoints"""

    def __init__(self, waypoint_list):
        self.waypoint_list = waypoint_list

    def distance_2d(self):
        """computes 2D distance of the path without
        considering turn waypoints"""

        distance = 0
        if len(self.waypoint_list) > 1:
            for i in range(len(self.waypoint_list) - 1):
                way_i = self.waypoint_list[i]
                way_i_1 = self.waypoint_list[i + 1]
                distance += way_i.distance_2d_to(way_i_1)
        return distance

    def distance_2d_with_turn_waypoint(self):
        """computes 2D distance of the path while
        considering turn waypoints"""

        distance = 0
        if len(self.waypoint_list) > 1:
            for i in range(len(self.waypoint_list) - 1):
                way_1 = self.waypoint_list[i]
                if way_1.post_turn_waypoint is not None:
                    distance += way_1.distance_2d_to(way_1.post_turn_waypoint)
                    way_1 = way_1.post_turn_waypoint
                way_2 = self.waypoint_list[i + 1]
                if way_2.pre_turn_waypoint is not None:
                    distance += way_2.distance_2d_to(way_2.pre_turn_waypoint)
                    way_2 = way_2.pre_turn_waypoint
                distance += way_1.distance_2d_to(way_2)
        return distance

    def path_distance_to_waypoint(self, next_waypoint):
        """find total distance of path plus an extra waypoint"""

        last_node = self.waypoint_list[- 1]
        dis = g_f.distance_2d(last_node.pos, next_waypoint.pos)
        return self.distance_2d() + dis

    def curve(self, profile):
        """Curve path by adding turn waypoints"""

        # check if the turn coming from the first waypoint can be made
        way_0 = self.waypoint_list[0]
        way_1 = self.waypoint_list[1]
        if way_0.post_turn_waypoint is not None:
            if not way_0.check_valid_turn(way_1, profile):
                return False

        # go over all points except the last one
        for way_index in range(1, len(self.waypoint_list)):

            # for the first waypoint, the last node given
            # is the previous waypoint
            prev_way = self.waypoint_list[way_index - 1]
            cur_way = self.waypoint_list[way_index]

            if way_index == len(self.waypoint_list) - 1:
                # if there was a problem with curving, return false
                if not cur_way.curve(prev_way, None, profile,
                                     no_next_way=True):
                    return False
            else:
                next_way = self.waypoint_list[way_index + 1]
                # if there was a problem with curving, return false
                if not cur_way.curve(prev_way, next_way, profile):
                    return False

        # return that the curving worked
        return True

    def altitude_set(self):
        """add altitude to waypoints of path"""

        z_0 = self.waypoint_list[0].z
        z_f = self.waypoint_list[-1].z

        delta_z = z_f - z_0
        distance_crossed = self.distance_2d_with_turn_waypoint()

        if g_f.float_eq(distance_crossed, 0):
            if g_f.float_eq(delta_z, 0):
                climb_ratio = 0
            else:
                return False
        else:
            climb_ratio = delta_z/distance_crossed

        if abs(climb_ratio) > MAX_CLIMBING_RATIO:
            return False

        # go over all points except the first and last one
        for way_index in range(1, len(self.waypoint_list)):

            # make the plane climb between the two
            # waypoints at the climb ratio
            prev_way = self.waypoint_list[way_index - 1]
            prev_way_post = prev_way.post_turn_waypoint
            if prev_way_post is not None:
                dis = prev_way.distance_2d_to(prev_way_post)
                prev_way_post.z = prev_way.z + climb_ratio * dis
                prev_way = prev_way_post

            cur_way = self.waypoint_list[way_index]
            cur_way_pre = cur_way.pre_turn_waypoint
            if cur_way_pre is not None:
                dis = prev_way.distance_2d_to(cur_way_pre)
                cur_way_pre.z = prev_way.z + climb_ratio * dis
                dis = cur_way_pre.distance_2d_to(cur_way)
                cur_way.z = cur_way_pre.z + climb_ratio * dis
            else:
                dis = prev_way.distance_2d_to(cur_way)
                cur_way.z = prev_way.z + climb_ratio * dis

        # return that the curving worked
        return True


def area_is_valid(area_center, area_radius, profile,
                  node_safety_level):
    """check that an area has no obstacles
    in it and does not cross the border"""

    # check that area is valid
    for obs in profile.obstacles:
        if node_safety_level:
            safety_radius = obs.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
        else:
            safety_radius = obs.r + OBSTACLE_DISTANCE_FOR_VALID_PASS
        inter = g_f.circle_to_circle_intersection
        if inter(area_center, area_radius, obs.pos, safety_radius):
            return False

    vertices = profile.border.vertices
    for vertex_index in range(len(vertices)):
        # check if the seg connecting the two waypoints
        # intersects with the obstacle

        vertex_1 = vertices[vertex_index].pos
        vertex_2 = vertices[(vertex_index+1) % len(vertices)].pos

        if node_safety_level:
            safety_radius = area_radius + BORDER_DISTANCE_FOR_VALID_NODE
        else:
            safety_radius = area_radius + BORDER_DISTANCE_FOR_VALID_PASS
        inter = g_f.seg_to_disk_intersection
        if inter(vertex_1, vertex_2, area_center, safety_radius):
            return False

    return True
