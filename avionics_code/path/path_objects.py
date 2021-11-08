import math
import Geometrical_functions as G_f
import path_functions as P_f
import Parameters as Para

OBSTACLE_DISTANCE_FOR_VALID_PASS = Para.OBSTACLE_DISTANCE_FOR_VALID_PASS
OBSTACLE_DISTANCE_FOR_VALID_NODE = Para.OBSTACLE_DISTANCE_FOR_VALID_NODE
BORDER_DISTANCE_FOR_VALID_PASS = Para.BORDER_DISTANCE_FOR_VALID_PASS
BORDER_DISTANCE_FOR_VALID_NODE = Para.BORDER_DISTANCE_FOR_VALID_NODE
NODE_MIN_DISTANCE = Para.NODE_MIN_DISTANCE
OBSTACLE_ORBIT_RATIO = Para.OBSTACLE_ORBIT_RATIO
MIN_DEVIATION_FOR_ALTERNATIVE_PATH = math.radians(Para.MIN_DEVIATION_FOR_ALTERNATIVE_PATH)
TURN_RADIUS = Para.TURN_RADIUS
PREFERRED_TURN_RADIUS = Para.PREFERRED_TURN_RADIUS

"""Mission object grouping map data and computing mission paths"""


class Mission_profile:

    """Construct profile from map data and initialize different path types"""
    def __init__(self, Center_tuple, Current_tuple, Mission_tuples, Obstacles_tuples, Border_tuples):
        # transform map data into path objects
        self.Map_center = Waypoint(Center_tuple)
        self.Current_position = Waypoint(Current_tuple[0], Current_tuple[1])
        self.Border = Border(list(Vertex(B_i) for B_i in Border_tuples))
        self.Obstacles = list(Obstacle(O_i[0], O_i[1]) for O_i in Obstacles_tuples)
        self.Mission_waypoint_list = list(Waypoint(W_i[0], z=W_i[1]) for W_i in Mission_tuples)

        # different path types that are generated from the map data
        self.Straight_2D_paths_list = None
        self.Curved_2D_paths_list = None
        self.Curved_3D_paths_list = None
        self.Chosen_3D_path = None

    """Goes through all steps of path finding and stores all of the resultant paths"""

    def Compute_path(self):
        if len(self.Mission_waypoint_list) > 0:
            self.Straight_2D_paths_list = P_f.Straight_2D_path_finder(self)
            self.Curved_2D_paths_list = P_f.Curving_2D(self, self.Straight_2D_paths_list)
            self.Curved_3D_paths_list = P_f.Altitude_extension(self.Curved_2D_paths_list)
            self.Chosen_3D_path = P_f.Path_selector(self.Curved_3D_paths_list)
        else:
            self.Straight_2D_paths_list = list()
            self.Curved_2D_paths_list = list()
            self.Curved_3D_paths_list = list()
            self.Chosen_3D_path = list()

    """Change obstacles list"""

    def Add_obstacle(self, Obstacle_tuple):
        self.Obstacles.append(Obstacle(Obstacle_tuple[0], Obstacle_tuple[1]))

    def Clear_obstacles(self):
        self.Obstacles.clear()

    def Delete_obstacle(self, i):
        del self.Obstacles[i]

    """Change Border"""

    def Add_vertex(self, i, Vertex_tuple):
        self.Border.Vertex_list.insert(i, Vertex(Vertex_tuple))

    def Clear_border(self):
        self.Border.Vertex_list.clear()

    def Delete_vertex(self, i):
        del self.Border.Vertex_list[i]

    """Change mission waypoint list"""

    def Add_waypoint(self, i, Waypoint_tuple):
        self.Mission_waypoint_list.insert(i, Waypoint(Waypoint_tuple))

    def Clear_waypoints(self):
        self.Mission_waypoint_list.clear()

    def Delete_waypoint(self, i):
        del self.Mission_waypoint_list[i]


"""Parent class for Map objects (waypoints, border vertices, obstacles)"""
class Map_object:
    # return 2d coordinates
    def Point_2d(self):
        return self.x, self.y


"""Parent class for Map structures (border, path)"""
class Map_structure:
    pass


"""obstacle object with x,y position and radius r"""
class Obstacle(Map_object):
    def __init__(self, Coordinates_2d_tuple, r):
        self.r = r
        self.x = Coordinates_2d_tuple[0]
        self.y = Coordinates_2d_tuple[1]

    """Creates tangent nodes from the given waypoint toward the obstacle to allow dodging it"""
    def Create_tangent_nodes(self, Waypoint_1):
        Distance_to_waypoint = G_f.Distance_2d(Waypoint_1.Point_2d(), self.Point_2d())
        # Check if Waypoint is the same as center of obstacle
        if G_f.Float_eq(Distance_to_waypoint, 0):
            return None, None
        # Check if Waypoint is far enough for normal node creation
        elif Distance_to_waypoint >= self.r + 2 * OBSTACLE_DISTANCE_FOR_VALID_NODE:

            # safe distance radius
            Safe_r = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            N_1, N_2 = G_f.Tangent_points(Waypoint_1.Point_2d(), self.Point_2d(), Safe_r)
            return Waypoint(N_1, Parent_obstacle=self), Waypoint(N_2, Parent_obstacle=self)

        # Check if Waypoint is inside obstacle
        elif Distance_to_waypoint <= self.r:
            return None, None
        # Check if Waypoint is in obstacle orbit for dense node creation
        else:
            # Find angle of rotation around orbit
            Factor_1 = self.r + OBSTACLE_DISTANCE_FOR_VALID_PASS * OBSTACLE_ORBIT_RATIO
            Factor_2 = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE
            Orbit_angle = 2 * math.acos(Factor_1 / Factor_2)

            # find vector from center of obstacle to waypoint
            Axis_vector = G_f.Sub_vectors(Waypoint_1.Point_2d(), self.Point_2d())

            # rotate axis vector around Orbit_angle
            New_axis_vector_1 = G_f.Rotate_vector(Axis_vector, Orbit_angle)
            New_axis_vector_2 = G_f.Rotate_vector(Axis_vector, -Orbit_angle)

            # add vectors to center of obstacle location type
            Node_1 = Waypoint(G_f.Add_vectors(self.Point_2d(), New_axis_vector_1), Parent_obstacle=self)
            Node_2 = Waypoint(G_f.Add_vectors(self.Point_2d(), New_axis_vector_2), Parent_obstacle=self)

            return Node_1, Node_2


"""Border vertex object with x,y position"""
class Vertex(Map_object):
    def __init__(self, Coordinates_2d_tuple):
        self.x = Coordinates_2d_tuple[0]
        self.y = Coordinates_2d_tuple[1]


"""Border object made with multiple vertex objects"""
class Border(Map_structure):
    def __init__(self, Vertex_list):
        self.Vertex_list = Vertex_list

    """Returns list of edges of the border"""
    def Compute_simple_edges(self):
        Edges_list = list()
        if len(self.Vertex_list) > 1:
            for i in range(len(self.Vertex_list)):
                Edges_list.append((self.Vertex_list[i - 1], self.Vertex_list[i]))
        return Edges_list

    """Create vertex nodes on concave border vertices"""
    def Create_vertex_nodes(self):
        Node_list = list()
        if len(self.Vertex_list) > 2:
            # go through all vertices to see if a node can be created on them
            for Vertex_index in range(len(self.Vertex_list)):
                # Get current, previous and next Border vertices
                Current_vertex = self.Vertex_list[Vertex_index % len(self.Vertex_list)]
                Previous_vertex = self.Vertex_list[(Vertex_index - 1) % len(self.Vertex_list)]
                Next_vertex = self.Vertex_list[(Vertex_index + 1) % len(self.Vertex_list)]

                # Get the vectors linking them
                Current_to_previous_vertex = G_f.Sub_vectors(Previous_vertex.Point_2d(), Current_vertex.Point_2d())
                Current_to_next_vertex = G_f.Sub_vectors(Next_vertex.Point_2d(), Current_vertex.Point_2d())
                # Check if the vectors are non-nul
                Null_1 = G_f.Float_eq_2d(Current_to_previous_vertex, (0, 0))
                Null_2 = G_f.Float_eq_2d(Current_to_next_vertex, (0, 0))
                if not (Null_1 or Null_2):

                    Current_to_previous_vertex = G_f.Unit_vector(Current_to_previous_vertex)
                    Current_to_next_vertex = G_f.Unit_vector(Current_to_next_vertex)

                    # Find direction away from border vertex and add a node there
                    Vector_away = G_f.Add_vectors(Current_to_previous_vertex, Current_to_next_vertex)

                    # Check if the vectors do not add up to a null vector
                    if not G_f.Float_eq_2d(Vector_away, (0, 0)):
                        Unit_away = G_f.Unit_vector(Vector_away)
                        Offset_away = G_f.Scale_vector(Unit_away, BORDER_DISTANCE_FOR_VALID_NODE)
                        New_node = G_f.Sub_vectors(Current_vertex.Point_2d(), Offset_away)
                        Node_list.append(Waypoint(New_node, Parent_Vertex=Current_vertex))
        return Node_list


"""z = None for Waypoints created during straight path generation
Waypoint object with x,y,z position, parent object, and alleviation waypoint
parent Obstacle: node created on an obstacle during straight path generation
Parent Border vertex:  node created on a border vertex during straight path generation
Alleviation waypoint: waypoint added during path curving to make the turn on the waypoint easier
Off waypoint: The position that the plane will go to because of inertia after crossing the waypoint"""
class Waypoint(Map_object):
    def __init__(self, Coordinates_2d_tuple, z=None, Parent_obstacle=None, Parent_Vertex=None):
        self.x = Coordinates_2d_tuple[0]
        self.y = Coordinates_2d_tuple[1]
        self.z = z
        self.Parent_obstacle = Parent_obstacle
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
    a straight line without hitting an obstacle/border"""
    def Is_connectable_to(self, Other_waypoint, Mission_profile_1):

        # Extract map info
        Obstacles = Mission_profile_1.Obstacles
        Border_1 = Mission_profile_1.Border

        # check if there is a collision with some obstacle
        for Obstacle_i in Obstacles:
            # check if the segment connecting the two waypoints intersects with the Obstacle
            Center = Obstacle_i.Point_2d()
            Safe_radius = Obstacle_i.r + OBSTACLE_DISTANCE_FOR_VALID_PASS
            if G_f.Segment_to_disk_intersection(self.Point_2d(), Other_waypoint.Point_2d(), Center, Safe_radius):
                return False

        # check if there is a collision with some border line
        for Vertex_index in range(len(Border_1.Vertex_list)):
            Vertex_1 = Border_1.Vertex_list[Vertex_index]
            Vertex_2 = Border_1.Vertex_list[(Vertex_index + 1) % len(Border_1.Vertex_list)]
            # check if the segment connecting the two waypoints intersects with the border line
            W_node = Other_waypoint.Point_2d()
            V_node_1 = Vertex_1.Point_2d()
            V_node_2 = Vertex_2.Point_2d()
            B_S = BORDER_DISTANCE_FOR_VALID_PASS
            if G_f.Segment_to_segment_with_safety(self.Point_2d(), W_node, V_node_1, V_node_2, B_S):
                return False

        return True

    """Checks if an arc can be constructed from self
    to Initial_w with center Turn_center_w coming from Base_w"""
    def Is_arc_connectable_to(self, Base_w, Initial_w, Turn_center_w, Mission_profile_1):
        Node_1 = Base_w.Point_2d()
        Node_2 = self.Point_2d()
        Node_3 = Initial_w.Point_2d()
        Turn_center = Turn_center_w.Point_2d()
        if G_f.Distinct_points_4(Node_1, Node_2, Node_3, Turn_center):
            Middle = G_f.Center_2d(Node_2, Node_3)
            Distance_to_center = G_f.Distance_2d(self.Point_2d(), Turn_center)
            Middle_1 = G_f.Homothety_unit(Middle, Turn_center, Distance_to_center)
            Middle_2 = G_f.Homothety_unit(Middle, Turn_center, -Distance_to_center)

            A_1 = G_f.Find_angle(G_f.Sub_vectors(Turn_center, Node_2), G_f.Sub_vectors(Node_1, Node_2))
            A_2 = G_f.Find_angle(G_f.Sub_vectors(Turn_center, Middle_1), G_f.Sub_vectors(Node_3, Middle_1))

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
        return G_f.Float_eq_2d(self.Point_2d(), Waypoint_1.Point_2d())

    """Check if path is going backwards"""
    def Is_going_backwards(self, Waypoint_1, Waypoint_2):
        Last_to_current = G_f.Sub_vectors(self.Point_2d(), Waypoint_1.Point_2d())
        Current_to_Next = G_f.Sub_vectors(Waypoint_2.Point_2d(), self.Point_2d())
        if G_f.Distinct_points_3(Last_to_current, Current_to_Next, (0, 0)):
            return abs(G_f.Find_angle(Last_to_current, Current_to_Next)) > math.pi / 2
        else:
            return False

    """Check if path through node is not crossing over obstacle or border (bouncing off it instead)"""
    def Is_bouncing(self, Waypoint_1, Waypoint_2):
        # cannot bounce without a node created on a map object
        if self.Parent_obstacle is None and self.Parent_Vertex is None:
            return False
        else:
            if self.Parent_obstacle is not None:
                W_node_1 = Waypoint_1.Point_2d()
                W_node_2 = Waypoint_2.Point_2d()
                O_node = self.Parent_obstacle.Point_2d()
                return not G_f.Is_crossing_over_edge(W_node_1, W_node_2, self.Point_2d(), O_node)
            elif self.Parent_Vertex is not None:
                W_node_1 = Waypoint_1.Point_2d()
                W_node_2 = Waypoint_2.Point_2d()
                V_node = self.Parent_Vertex.Point_2d()
                return not G_f.Is_crossing_over_edge(W_node_1, W_node_2, self.Point_2d(), V_node)

    """Check if path is going through a node in a straight line"""
    def Is_across_viable_edge(self, Waypoint_1, Waypoint_2, Mission_profile_1):

        # Extract map info
        Obstacle_1 = Mission_profile_1.Obstacles
        Border_1 = Mission_profile_1.Border

        if Waypoint_2.Is_connectable_to(Waypoint_1, Mission_profile_1):
            Last_to_current = G_f.Sub_vectors(self.Point_2d(), Waypoint_1.Point_2d())
            Current_to_Next = G_f.Sub_vectors(Waypoint_2.Point_2d(), self.Point_2d())
            if G_f.Distinct_points_3(Last_to_current, Current_to_Next, (0, 0)):
                return abs(G_f.Find_angle(Last_to_current, Current_to_Next)) < MIN_DEVIATION_FOR_ALTERNATIVE_PATH
            else:
                return False
        else:
            return False

    """Adds an alleviation waypoint to make the turn easier"""
    def Alleviate(self, Waypoint_1, Waypoint_2, Mission_profile_1):

        P_R = PREFERRED_TURN_RADIUS

        Node_1 = Waypoint_1.Point_2d()
        Node_2 = self.Point_2d()
        Node_3 = Waypoint_2.Point_2d()
        if not G_f.Distinct_points_3(Node_1, Node_2, Node_3):
            return

        # find two possible turn centers that would allow to turn through Node 2 to go into Node 3
        N_1, N_2 = G_f.Unit_normal_vectors_to_line(Node_2, Node_3)
        Turn_center_1 = G_f.Add_vectors(Node_2, G_f.Scale_vector(N_1, P_R))
        Turn_center_2 = G_f.Add_vectors(Node_2, G_f.Scale_vector(N_2, P_R))

        # Check which one is on the side of Node 1
        if not G_f.Segment_to_line_intersection(Node_1, Turn_center_1, Node_2, Node_3):
            Turn_center = Turn_center_1
        else:
            Turn_center = Turn_center_2

        Alleviation_w = None

        A_1 = G_f.Find_angle((0, 1), G_f.Sub_vectors(Turn_center, Node_2))
        A_2 = G_f.Find_angle((0, 1), G_f.Sub_vectors(Node_3, Node_2))
        if -3.14 < A_2 - A_1 < 0 or 3.14 < A_2 - A_1 < 6.30:
            Rotation_unit = -0.1
        else:
            Rotation_unit = +0.1

        New_turn_center = Turn_center
        # Angle by which the turn center is shifted when the default turn is not possible
        Turn_offset = 0

        A_x = G_f.Find_angle(G_f.Sub_vectors(Node_1, Node_2), G_f.Sub_vectors(New_turn_center, Node_2))

        while -math.pi/2 < Turn_offset < math.pi/2 or -math.pi/2 < A_x < math.pi/2:

            A_x = G_f.Find_angle(G_f.Sub_vectors(Node_1, Node_2), G_f.Sub_vectors(New_turn_center, Node_2))

            # Make sure Node 1 is not inside the turn circle
            if G_f.Distance_2d(Node_1, New_turn_center) > P_R:

                # Find tangent points on the turn circle coming Node_1
                Node_a, Node_b = G_f.Tangent_points(Node_1, New_turn_center, P_R)
                A_1 = G_f.Find_angle(G_f.Sub_vectors(Node_2, New_turn_center), G_f.Sub_vectors(Node_3, Node_2))
                A_2 = G_f.Find_angle(G_f.Sub_vectors(Node_a, Node_1), G_f.Sub_vectors(New_turn_center, Node_a))

                # Find which tangent point gives the right orientation for turning
                if A_1 * A_2 > 0:
                    Picked_w = Waypoint(Node_a)
                else:
                    Picked_w = Waypoint(Node_b)

                New_center_w = Waypoint(New_turn_center)

                if Picked_w.Is_connectable_to(Waypoint_1, Mission_profile_1):
                    if Picked_w.Is_arc_connectable_to(Waypoint_1, self, New_center_w, Mission_profile_1):
                        if -math.pi/2 < Turn_offset < math.pi/2:
                            Alleviation_w = Picked_w
                            break

            # Change the turn center
            Turn_offset += Rotation_unit
            New_turn_center = G_f.Rotate_vector_with_center(Turn_center, Node_2, Turn_offset)

        if Alleviation_w is None:
            self.Alleviation_waypoint = Alleviation_w
        elif G_f.Distance_2d(Alleviation_w.Point_2d(), self.Point_2d()) > 2*NODE_MIN_DISTANCE:
            self.Alleviation_waypoint = Alleviation_w
        else:
            self.Alleviation_waypoint = None

        # store turn center as the alleviation waypoint of off waypoint
        self.Off_waypoint = Waypoint((0, 0))
        self.Off_waypoint.Alleviation_waypoint = Waypoint(New_turn_center)

    """Determines where the off waypoint is"""
    def Off_shoot(self, Previous_waypoint, Next_waypoint):
        P_R = PREFERRED_TURN_RADIUS
        Center = self.Off_waypoint.Alleviation_waypoint.Point_2d()
        Node_1 = Previous_waypoint.Point_2d()
        Node_2 = self.Point_2d()
        Node_3 = Next_waypoint.Point_2d()

        if G_f.Distance_2d(Node_3, Center) > P_R:
            # Find tangent points on the turn circle coming Node_1
            Node_a, Node_b = G_f.Tangent_points(Node_3, Center, P_R)
            A_1 = G_f.Find_angle(G_f.Sub_vectors(Center, Node_2), G_f.Sub_vectors(Node_1, Node_2))
            A_2 = G_f.Find_angle(G_f.Sub_vectors(Center, Node_a), G_f.Sub_vectors(Node_3, Node_a))

            A_3 = G_f.Find_angle(G_f.Sub_vectors(Node_1, Node_2), G_f.Sub_vectors(Node_3, Node_2))

            if abs(A_3) < math.pi / 2 and self.Alleviation_waypoint is not None:
                # Find which tangent point gives the right orientation for turning
                if G_f.Distance_2d(Node_a, Node_2) < G_f.Distance_2d(Node_b, Node_2):
                    self.Off_waypoint.x, self.Off_waypoint.y = Node_a
                else:
                    self.Off_waypoint.x, self.Off_waypoint.y = Node_b
            else:
                # Find which tangent point gives the right orientation for turning
                if A_1 * A_2 < 0:
                    self.Off_waypoint.x, self.Off_waypoint.y = Node_a
                else:
                    self.Off_waypoint.x, self.Off_waypoint.y = Node_b
        else:
            self.Off_waypoint.x, self.Off_waypoint.y = None, None


"""path made from multiple waypoints"""
class Path(Map_structure):
    Simple_distance_2d = None

    def __init__(self, Waypoint_list):
        self.Waypoint_list = Waypoint_list

    """Returns list of edges of the path without alleviation waypoints"""
    def Compute_simple_edges(self):
        Edges_list = list()
        if len(self.Waypoint_list) > 1:
            for i in range(len(self.Waypoint_list) - 1):
                Edges_list.append((self.Waypoint_list[i], self.Waypoint_list[i + 1]))
        return Edges_list

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
            if G_f.Distance_2d(Waypoint_i.Point_2d(), Waypoint_1.Point_2d()) < NODE_MIN_DISTANCE:
                Found = True
        return Found

    """find total distance of path plus an extra waypoint"""
    def Path_distance_to_waypoint(self, Next_waypoint):
        Last_node = self.Waypoint_list[len(self.Waypoint_list) - 1]
        self.Distance_2d_update()
        return self.Simple_distance_2d + G_f.Distance_2d(Last_node.Point_2d(), Next_waypoint.Point_2d())

    """Curve path by adding alleviation waypoints"""
    def Curve(self, Mission_profile_1):

        """Add internal alleviation waypoints inside each path"""
        if len(self.Waypoint_list) > 2:
            for Waypoint_index in range(1, len(self.Waypoint_list) - 1):
                Previous_waypoint = self.Waypoint_list[len(self.Waypoint_list) - 1 - Waypoint_index - 1]
                Current_waypoint = self.Waypoint_list[len(self.Waypoint_list) - 1 - Waypoint_index]
                Next_waypoint = self.Waypoint_list[len(self.Waypoint_list) - 1 - Waypoint_index + 1]
                if Next_waypoint.Alleviation_waypoint is not None:
                    Next_waypoint = Next_waypoint.Alleviation_waypoint
                Current_waypoint.Alleviate(Previous_waypoint, Next_waypoint, Mission_profile_1)

    """Finds off waypoints for path"""
    def Off_shoot(self):
        if len(self.Waypoint_list) > 2:
            for Waypoint_index in range(1, len(self.Waypoint_list) - 1):
                Previous_waypoint = self.Waypoint_list[Waypoint_index - 1]
                Current_waypoint = self.Waypoint_list[Waypoint_index]
                Next_waypoint = self.Waypoint_list[Waypoint_index + 1]
                if Current_waypoint.Alleviation_waypoint is not None:
                    Previous_waypoint = Current_waypoint.Alleviation_waypoint
                if Next_waypoint.Alleviation_waypoint is not None:
                    Next_waypoint = Next_waypoint.Alleviation_waypoint
                Current_waypoint.Off_shoot(Previous_waypoint, Next_waypoint)

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
                Next_waypoint = self.Waypoint_list[W_index+1]
                C_off_w = C_w.Off_waypoint
                C_off_next = Next_waypoint.Off_waypoint
                Al_w = Next_waypoint.Alleviation_waypoint
                if C_off_w is not None:
                    Turn_center = C_off_w.Alleviation_waypoint
                    if C_off_w.x is not None:
                        if not C_w.Is_arc_connectable_to(C_p, C_off_w, Turn_center, Mission_profile_1):
                            return False
                        C_w = C_off_w
                    else:
                        Center = G_f.Center_2d(C_w.Point_2d(), Next_waypoint.Point_2d())
                        Danger_area = Danger_zone(Center, 2*PREFERRED_TURN_RADIUS)
                        if not Danger_area.Is_free(Mission_profile_1):
                            return False

                if Al_w is None:
                    if not C_w.Is_connectable_to(Next_waypoint, Mission_profile_1):
                        return False
                else:
                    Turn_center_2 = C_off_next.Alleviation_waypoint
                    if not C_w.Is_connectable_to(Al_w, Mission_profile_1):
                        return False
                    if not Al_w.Is_arc_connectable_to(C_w, Next_waypoint, Turn_center_2, Mission_profile_1):
                        return False
        return True


"""Area where the flight is hard to predict"""
class Danger_zone:
    def __init__(self, Center, Radius):
        self.Center = Center
        self.Radius = Radius

    """Check if the area has no obstacles or borders in it"""
    def Is_free(self, Mission_profile_1):
        # Extract map info
        Obstacles = Mission_profile_1.Obstacles
        Border_1 = Mission_profile_1.Border

        # check if there is a collision with some obstacle
        for Obstacle_i in Obstacles:

            # check if the segment connecting the two waypoints intersects with the Obstacle
            Center = Obstacle_i.Point_2d()
            if G_f.Circle_to_circle_intersection(self.Center, self.Radius, Center, Obstacle_i.r):
                return False

        # check if there is a collision with some border line
        for Vertex_index in range(len(Border_1.Vertex_list)):
            Vertex_1 = Border_1.Vertex_list[Vertex_index]
            Vertex_2 = Border_1.Vertex_list[(Vertex_index + 1) % len(Border_1.Vertex_list)]
            # check if the segment connecting the two waypoints intersects with the border line
            V_node_1 = Vertex_1.Point_2d()
            V_node_2 = Vertex_2.Point_2d()
            if G_f.Segment_to_disk_intersection(V_node_1, V_node_2, self.Center, self.Radius):
                return False

        return True
