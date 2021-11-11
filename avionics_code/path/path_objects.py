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
TURN_RADIUS = para.TURN_RADIUS
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS


class MapObject:
    """Parent class for Map objects (waypoints, border vertices, Obstacles)"""

    def __init__(self, position_tuple):
        self.pos = position_tuple


class Vertex(MapObject):
    """Border vertex object with x,y position"""

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


class Obstacle(MapObject):
    """Obstacle object with x,y position and radius r"""

    def __init__(self, position_tuple, r):
        self.r = r
        self.pos = position_tuple

    def create_tangent_nodes(self, Waypoint_1):
        """Creates tangent nodes from the given waypoint toward the Obstacle to allow dodging it"""

        Distance_to_waypoint = g_f.Distance_2d(Waypoint_1.point_2d(), self.point_2d())
        # Check if Waypoint is the same as center of Obstacle
        if g_f.float_eq(Distance_to_waypoint, 0):
            return None, None
        # Check if Waypoint is far enough for normal node creation
        elif Distance_to_waypoint >= self.r + 2 * OBSTACLE_DISTANCE_FOR_VALID_NODE:

            # safe distance radius
            Safe_r = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            N_1, N_2 = g_f.Tangent_points(Waypoint_1.point_2d(), self.point_2d(), Safe_r)
            return Waypoint(N_1, Parent_Obstacle=self), Waypoint(N_2, Parent_Obstacle=self)

        # Check if Waypoint is inside Obstacle
        elif Distance_to_waypoint <= self.r:
            return None, None
        # Check if Waypoint is in Obstacle orbit for dense node creation
        else:
            # Find angle of rotation around orbit
            Factor_1 = self.r + Obstacle_DISTANCE_FOR_VALID_PASS * OBSTACLE_ORBIT_RATIO
            Factor_2 = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            Orbit_angle = 2 * math.acos(Factor_1 / Factor_2)

            # find vector from center of Obstacle to waypoint
            Axis_vector = g_f.sub_vectors(Waypoint_1.point_2d(), self.point_2d())

            # rotate axis vector around Orbit_angle
            new_axis_vector_1 = g_f.Rotate_vector(Axis_vector, Orbit_angle)
            new_axis_vector_2 = g_f.Rotate_vector(Axis_vector, -Orbit_angle)

            # add vectors to center of Obstacle location type
            node_1 = Waypoint(g_f.add_vectors(self.point_2d(), new_axis_vector_1), Parent_Obstacle=self)
            node_2 = Waypoint(g_f.add_vectors(self.point_2d(), new_axis_vector_2), Parent_Obstacle=self)

            return node_1, node_2


class Border(MapArea):
    """Border object that create waypoints on its concave vertices"""

    def create_vertex_nodes(self):
        """Create vertex nodes on concave border vertices to dodge edges"""

        node_list = list()
        if len(self.vertices) > 2:
            # go through all vertices to see if a node can be created on them
            for vertex_index in range(len(self.vertices)):
                # Get current, previous and next Border vertices
                current_vertex = self.vertices[vertex_index % len(self.vertices)]
                previous_vertex = self.vertices[(vertex_index - 1) % len(self.vertices)]
                next_vertex = self.vertices[(vertex_index + 1) % len(self.vertices)]

                # Get the vectors linking them
                current_to_previous_vertex = g_f.sub_vectors(previous_vertex.point_2d(), current_vertex.point_2d())
                current_to_next_vertex = g_f.sub_vectors(next_vertex.point_2d(), current_vertex.point_2d())
                # Check if the vectors are non-nul
                null_1 = g_f.float_eq_2d(current_to_previous_vertex, (0, 0))
                null_2 = g_f.float_eq_2d(current_to_next_vertex, (0, 0))
                if not (null_1 or null_2):

                    current_to_previous_vertex = g_f.unit_vector(current_to_previous_vertex)
                    current_to_next_vertex = g_f.unit_vector(current_to_next_vertex)

                    # Find direction away from border vertex and add a node there
                    vector_away = g_f.add_vectors(current_to_previous_vertex, current_to_next_vertex)

                    # Check if the vectors do not add up to a null vector
                    if not g_f.float_eq_2d(vector_away, (0, 0)):
                        unit_away = g_f.unit_vector(vector_away)
                        offset_away = g_f.scale_vector(unit_away, BORDER_DISTANCE_FOR_VALID_NODE)
                        new_node = g_f.sub_vectors(current_vertex.point_2d(), offset_away)
                        node_list.append(Waypoint(new_node, Parent_Vertex=current_vertex))
        return node_list


"""z = None for Waypoints created during straight path generation
Waypoint object with x,y,z position, parent object, and alleviation waypoint
parent Obstacle: node created on an Obstacle during straight path generation
Parent Border vertex:  node created on a border vertex during straight path generation
Alleviation waypoint: waypoint added during path curving to make the turn on the waypoint easier
Off waypoint: The position that the plane will go to because of inertia after crossing the waypoint"""
class Waypoint(MapObject):
    def __init__(self, position_tuple, z=None, Parent_Obstacle=None, Parent_Vertex=None):
        self.pos = position_tuple
        self.z = z
        self.Parent_Obstacle = Parent_Obstacle
        self.Parent_Vertex = Parent_Vertex
        self.Alleviation_waypoint = None
        self.Off_waypoint = None

    """computes 2d distance to other Waypoint"""
    def Distance_2d_to(self, Other_waypoint):
        return math.hypot(self.x - Other_waypoint.x, self.y - Other_waypoint.y)

    """computes 2d distance to other Waypoint"""
    def Distance_3d_to(self, Other_waypoint):
        return math.hypot(self.x - Other_waypoint.x, self.y - Other_waypoint.y, self.z - Other_waypoint.z)

    """Checks if this waypoint can be connected to another through
    a straight line without hitting an Obstacle/border"""
    def Is_connectable_to(self, Other_waypoint, Mission_profile_1):

        # Extract map info
        Obstacles = Mission_profile_1.Obstacles
        Border_1 = Mission_profile_1.Border

        # check if there is a collision with some Obstacle
        for Obstacle_i in Obstacles:
            # check if the segment connecting the two waypoints intersects with the Obstacle
            Center = Obstacle_i.point_2d()
            Safe_radius = Obstacle_i.r + Obstacle_DISTANCE_FOR_VALID_PASS
            if g_f.Segment_to_disk_intersection(self.point_2d(), Other_waypoint.point_2d(), Center, Safe_radius):
                return False

        # check if there is a collision with some border line
        for vertex_index in range(len(Border_1.vertices)):
            Vertex_1 = Border_1.vertices[vertex_index]
            Vertex_2 = Border_1.vertices[(vertex_index + 1) % len(Border_1.vertices)]
            # check if the segment connecting the two waypoints intersects with the border line
            W_node = Other_waypoint.point_2d()
            V_node_1 = Vertex_1.point_2d()
            V_node_2 = Vertex_2.point_2d()
            B_S = BORDER_DISTANCE_FOR_VALID_PASS
            if g_f.Segment_to_segment_with_safety(self.point_2d(), W_node, V_node_1, V_node_2, B_S):
                return False

        return True

    """Checks if an arc can be constructed from self
    to Initial_w with center Turn_center_w coming from Base_w"""
    def Is_arc_connectable_to(self, Base_w, Initial_w, Turn_center_w, Mission_profile_1):
        node_1 = Base_w.point_2d()
        node_2 = self.point_2d()
        node_3 = Initial_w.point_2d()
        Turn_center = Turn_center_w.point_2d()
        if g_f.Distinct_points_4(node_1, node_2, node_3, Turn_center):
            Middle = g_f.Center_2d(node_2, node_3)
            Distance_to_center = g_f.Distance_2d(self.point_2d(), Turn_center)
            Middle_1 = g_f.Homothety_unit(Middle, Turn_center, Distance_to_center)
            Middle_2 = g_f.Homothety_unit(Middle, Turn_center, -Distance_to_center)

            A_1 = g_f.Find_angle(g_f.sub_vectors(Turn_center, node_2), g_f.sub_vectors(node_1, node_2))
            A_2 = g_f.Find_angle(g_f.sub_vectors(Turn_center, Middle_1), g_f.sub_vectors(node_3, Middle_1))

            # Find which tangent point gives the right orientation for turning
            if A_1 * A_2 < 0:
                Arc_middle_w = Waypoint(Middle_1)
            else:
                Arc_middle_w = Waypoint(Middle_2)

            C_1 = Arc_middle_w.Is_connectable_to(self, Mission_profile_1)
            C_2 = Arc_middle_w.Is_connectable_to(Initial_w, Mission_profile_1)
            return C_1 and C_2
        else:
            return True

    """Check if waypoint coincides with another waypoint"""
    def Coincides_with(self, Waypoint_1):
        return g_f.float_eq_2d(self.point_2d(), Waypoint_1.point_2d())

    """Check if path is going backwards"""
    def Is_going_backwards(self, Waypoint_1, Waypoint_2):
        Last_to_current = g_f.sub_vectors(self.point_2d(), Waypoint_1.point_2d())
        current_to_next = g_f.sub_vectors(Waypoint_2.point_2d(), self.point_2d())
        if g_f.Distinct_points_3(Last_to_current, current_to_next, (0, 0)):
            return abs(g_f.Find_angle(Last_to_current, current_to_next)) > math.pi / 2
        else:
            return False

    """Check if path through node is not crossing over Obstacle or border (bouncing off it instead)"""
    def Is_bouncing(self, Waypoint_1, Waypoint_2):
        # cannot bounce without a node created on a map object
        if self.Parent_Obstacle is None and self.Parent_Vertex is None:
            return False
        else:
            if self.Parent_Obstacle is not None:
                W_node_1 = Waypoint_1.point_2d()
                W_node_2 = Waypoint_2.point_2d()
                O_node = self.Parent_Obstacle.point_2d()
                return not g_f.Is_crossing_over_edge(W_node_1, W_node_2, self.point_2d(), O_node)
            elif self.Parent_Vertex is not None:
                W_node_1 = Waypoint_1.point_2d()
                W_node_2 = Waypoint_2.point_2d()
                V_node = self.Parent_Vertex.point_2d()
                return not g_f.Is_crossing_over_edge(W_node_1, W_node_2, self.point_2d(), V_node)

    """Check if path is going through a node in a straight line"""
    def Is_across_viable_edge(self, Waypoint_1, Waypoint_2, Mission_profile_1):

        # Extract map info
        Obstacle_1 = Mission_profile_1.Obstacles
        Border_1 = Mission_profile_1.Border

        if Waypoint_2.Is_connectable_to(Waypoint_1, Mission_profile_1):
            Last_to_current = g_f.sub_vectors(self.point_2d(), Waypoint_1.point_2d())
            current_to_next = g_f.sub_vectors(Waypoint_2.point_2d(), self.point_2d())
            if g_f.Distinct_points_3(Last_to_current, current_to_next, (0, 0)):
                return abs(g_f.Find_angle(Last_to_current, current_to_next)) < MIN_DEVIATION_FOR_ALTERNATIVE_PATH
            else:
                return False
        else:
            return False

    """adds an alleviation waypoint to make the turn easier"""
    def Alleviate(self, Waypoint_1, Waypoint_2, Mission_profile_1):

        P_R = PREFERRED_TURN_RADIUS

        node_1 = Waypoint_1.point_2d()
        node_2 = self.point_2d()
        node_3 = Waypoint_2.point_2d()
        if not g_f.Distinct_points_3(node_1, node_2, node_3):
            return

        # find two possible turn centers that would allow to turn through node 2 to go into node 3
        N_1, N_2 = g_f.unit_normal_vectors_to_line(node_2, node_3)
        Turn_center_1 = g_f.add_vectors(node_2, g_f.scale_vector(N_1, P_R))
        Turn_center_2 = g_f.add_vectors(node_2, g_f.scale_vector(N_2, P_R))

        # Check which one is on the side of node 1
        if not g_f.Segment_to_line_intersection(node_1, Turn_center_1, node_2, node_3):
            Turn_center = Turn_center_1
        else:
            Turn_center = Turn_center_2

        Alleviation_w = None

        A_1 = g_f.Find_angle((0, 1), g_f.sub_vectors(Turn_center, node_2))
        A_2 = g_f.Find_angle((0, 1), g_f.sub_vectors(node_3, node_2))
        if -3.14 < A_2 - A_1 < 0 or 3.14 < A_2 - A_1 < 6.30:
            Rotation_unit = -0.1
        else:
            Rotation_unit = +0.1

        new_turn_center = Turn_center
        # Angle by which the turn center is shifted when the default turn is not possible
        Turn_offset = 0

        A_x = g_f.Find_angle(g_f.sub_vectors(node_1, node_2), g_f.sub_vectors(new_turn_center, node_2))

        while -math.pi/2 < Turn_offset < math.pi/2 or -math.pi/2 < A_x < math.pi/2:

            A_x = g_f.Find_angle(g_f.sub_vectors(node_1, node_2), g_f.sub_vectors(new_turn_center, node_2))

            # Make sure node 1 is not inside the turn circle
            if g_f.Distance_2d(node_1, new_turn_center) > P_R:

                # Find tangent points on the turn circle coming node_1
                node_a, node_b = g_f.Tangent_points(node_1, new_turn_center, P_R)
                A_1 = g_f.Find_angle(g_f.sub_vectors(node_2, new_turn_center), g_f.sub_vectors(node_3, node_2))
                A_2 = g_f.Find_angle(g_f.sub_vectors(node_a, node_1), g_f.sub_vectors(new_turn_center, node_a))

                # Find which tangent point gives the right orientation for turning
                if A_1 * A_2 > 0:
                    Picked_w = Waypoint(node_a)
                else:
                    Picked_w = Waypoint(node_b)

                new_center_w = Waypoint(new_turn_center)

                if Picked_w.Is_connectable_to(Waypoint_1, Mission_profile_1):
                    if Picked_w.Is_arc_connectable_to(Waypoint_1, self, new_center_w, Mission_profile_1):
                        if -math.pi/2 < Turn_offset < math.pi/2:
                            Alleviation_w = Picked_w
                            break

            # Change the turn center
            Turn_offset += Rotation_unit
            new_turn_center = g_f.Rotate_vector_with_center(Turn_center, node_2, Turn_offset)

        if Alleviation_w is None:
            self.Alleviation_waypoint = Alleviation_w
        elif g_f.Distance_2d(Alleviation_w.point_2d(), self.point_2d()) > 2*NODE_MIN_DISTANCE:
            self.Alleviation_waypoint = Alleviation_w
        else:
            self.Alleviation_waypoint = None

        # store turn center as the alleviation waypoint of off waypoint
        self.Off_waypoint = Waypoint((0, 0))
        self.Off_waypoint.Alleviation_waypoint = Waypoint(new_turn_center)

    """Determines where the off waypoint is"""
    def Off_shoot(self, previous_waypoint, next_waypoint):
        P_R = PREFERRED_TURN_RADIUS
        Center = self.Off_waypoint.Alleviation_waypoint.point_2d()
        node_1 = previous_waypoint.point_2d()
        node_2 = self.point_2d()
        node_3 = next_waypoint.point_2d()

        if g_f.Distance_2d(node_3, Center) > P_R:
            # Find tangent points on the turn circle coming node_1
            node_a, node_b = g_f.Tangent_points(node_3, Center, P_R)
            A_1 = g_f.Find_angle(g_f.sub_vectors(Center, node_2), g_f.sub_vectors(node_1, node_2))
            A_2 = g_f.Find_angle(g_f.sub_vectors(Center, node_a), g_f.sub_vectors(node_3, node_a))

            A_3 = g_f.Find_angle(g_f.sub_vectors(node_1, node_2), g_f.sub_vectors(node_3, node_2))

            if abs(A_3) < math.pi / 2 and self.Alleviation_waypoint is not None:
                # Find which tangent point gives the right orientation for turning
                if g_f.Distance_2d(node_a, node_2) < g_f.Distance_2d(node_b, node_2):
                    self.Off_waypoint.x, self.Off_waypoint.y = node_a
                else:
                    self.Off_waypoint.x, self.Off_waypoint.y = node_b
            else:
                # Find which tangent point gives the right orientation for turning
                if A_1 * A_2 < 0:
                    self.Off_waypoint.x, self.Off_waypoint.y = node_a
                else:
                    self.Off_waypoint.x, self.Off_waypoint.y = node_b
        else:
            self.Off_waypoint.x, self.Off_waypoint.y = None, None


"""path made from multiple waypoints"""
class Path:
    Simple_distance_2d = None

    def __init__(self, Waypoint_list):
        self.Waypoint_list = Waypoint_list

    """Returns list of edges of the path without alleviation waypoints"""
    def Compute_simple_edges(self):
        edges_list = list()
        if len(self.Waypoint_list) > 1:
            for i in range(len(self.Waypoint_list) - 1):
                edges_list.append((self.Waypoint_list[i], self.Waypoint_list[i + 1]))
        return edges_list

    """computes 2D distance of the path without considering altitude or alleviation waypoints"""
    def Compute_simple_distance_2d(self):
        return sum(Edge[0].Distance_2d_to(Edge[1]) for Edge in self.Compute_simple_edges())

    """computes 2D distance of the path with alleviation waypoints"""
    def Compute_true_distance_2d(self):
        return sum(Edge[0].Distance_2d_to(Edge[1]) for Edge in self.Compute_simple_edges())

    """updates 2D distance"""
    def Distance_2d_update(self):
        self.Simple_distance_2d = self.Compute_simple_distance_2d()

    """Check if waypoint is part of path"""
    def Contains_waypoint(self, Waypoint_1):
        Found = False
        for Waypoint_i in self.Waypoint_list:
            if g_f.Distance_2d(Waypoint_i.point_2d(), Waypoint_1.point_2d()) < NODE_MIN_DISTANCE:
                Found = True
        return Found

    """find total distance of path plus an extra waypoint"""
    def Path_distance_to_waypoint(self, next_waypoint):
        Last_node = self.Waypoint_list[len(self.Waypoint_list) - 1]
        self.Distance_2d_update()
        return self.Simple_distance_2d + g_f.Distance_2d(Last_node.point_2d(), next_waypoint.point_2d())

    """Curve path by adding alleviation waypoints"""
    def Curve(self, Mission_profile_1):

        """add internal alleviation waypoints inside each path"""
        if len(self.Waypoint_list) > 2:
            for Waypoint_index in range(1, len(self.Waypoint_list) - 1):
                previous_waypoint = self.Waypoint_list[len(self.Waypoint_list) - 1 - Waypoint_index - 1]
                current_waypoint = self.Waypoint_list[len(self.Waypoint_list) - 1 - Waypoint_index]
                next_waypoint = self.Waypoint_list[len(self.Waypoint_list) - 1 - Waypoint_index + 1]
                if next_waypoint.Alleviation_waypoint is not None:
                    next_waypoint = next_waypoint.Alleviation_waypoint
                current_waypoint.Alleviate(previous_waypoint, next_waypoint, Mission_profile_1)

    """Finds off waypoints for path"""
    def Off_shoot(self):
        if len(self.Waypoint_list) > 2:
            for Waypoint_index in range(1, len(self.Waypoint_list) - 1):
                previous_waypoint = self.Waypoint_list[Waypoint_index - 1]
                current_waypoint = self.Waypoint_list[Waypoint_index]
                next_waypoint = self.Waypoint_list[Waypoint_index + 1]
                if current_waypoint.Alleviation_waypoint is not None:
                    previous_waypoint = current_waypoint.Alleviation_waypoint
                if next_waypoint.Alleviation_waypoint is not None:
                    next_waypoint = next_waypoint.Alleviation_waypoint
                current_waypoint.Off_shoot(previous_waypoint, next_waypoint)

    """Check if path is valid including alleviation waypoints"""
    def Is_valid(self, Mission_profile_1):

        for W_index, Waypoint_i in enumerate(self.Waypoint_list):
            if 0 < W_index < len(self.Waypoint_list) - 1:
                C_p = self.Waypoint_list[W_index-1]
                if C_p.Off_waypoint is not None:
                    C_p = C_p.Off_waypoint
                C_w = Waypoint_i
                if C_w.Alleviation_waypoint is not None:
                    C_p = C_w.Alleviation_waypoint
                next_waypoint = self.Waypoint_list[W_index+1]
                C_off_w = C_w.Off_waypoint
                C_off_next = next_waypoint.Off_waypoint
                Al_w = next_waypoint.Alleviation_waypoint
                if C_off_w is not None:
                    Turn_center = C_off_w.Alleviation_waypoint
                    if C_off_w.x is not None:
                        if not C_w.Is_arc_connectable_to(C_p, C_off_w, Turn_center, Mission_profile_1):
                            return False
                        C_w = C_off_w
                    else:
                        Center = g_f.Center_2d(C_w.point_2d(), next_waypoint.point_2d())
                        Danger_area = Danger_zone(Center, 2*PREFERRED_TURN_RADIUS)
                        if not Danger_area.Is_free(Mission_profile_1):
                            return False

                if Al_w is None:
                    if not C_w.Is_connectable_to(next_waypoint, Mission_profile_1):
                        return False
                else:
                    Turn_center_2 = C_off_next.Alleviation_waypoint
                    if not C_w.Is_connectable_to(Al_w, Mission_profile_1):
                        return False
                    if not Al_w.Is_arc_connectable_to(C_w, next_waypoint, Turn_center_2, Mission_profile_1):
                        return False
        return True


"""Area where the flight is hard to predict"""
class Danger_zone:
    def __init__(self, Center, Radius):
        self.Center = Center
        self.Radius = Radius

    """Check if the area has no Obstacles or borders in it"""
    def Is_free(self, Mission_profile_1):
        # Extract map info
        Obstacles = Mission_profile_1.Obstacles
        Border_1 = Mission_profile_1.Border

        # check if there is a collision with some Obstacle
        for Obstacle_i in Obstacles:

            # check if the segment connecting the two waypoints intersects with the Obstacle
            Center = Obstacle_i.point_2d()
            if g_f.Circle_to_circle_intersection(self.Center, self.Radius, Center, Obstacle_i.r):
                return False

        # check if there is a collision with some border line
        for vertex_index in range(len(Border_1.vertices)):
            Vertex_1 = Border_1.vertices[vertex_index]
            Vertex_2 = Border_1.vertices[(vertex_index + 1) % len(Border_1.vertices)]
            # check if the segment connecting the two waypoints intersects with the border line
            V_node_1 = Vertex_1.point_2d()
            V_node_2 = Vertex_2.point_2d()
            if g_f.Segment_to_disk_intersection(V_node_1, V_node_2, self.Center, self.Radius):
                return False

        return True
