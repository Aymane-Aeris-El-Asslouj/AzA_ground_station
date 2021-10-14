import math
import Geometrical_functions
import Parameters

OBSTACLE_DISTANCE_FOR_VALID_PASS = Parameters.OBSTACLE_DISTANCE_FOR_VALID_PASS
OBSTACLE_DISTANCE_FOR_VALID_NODE = Parameters.OBSTACLE_DISTANCE_FOR_VALID_NODE
BORDER_DISTANCE_FOR_VALID_PASS = Parameters.BORDER_DISTANCE_FOR_VALID_PASS
BORDER_DISTANCE_FOR_VALID_NODE = Parameters.BORDER_DISTANCE_FOR_VALID_NODE
NODE_MIN_DISTANCE = Parameters.NODE_MIN_DISTANCE
OBSTACLE_ORBIT_RATIO = Parameters.OBSTACLE_ORBIT_RATIO


# obstacle object with x,y position and radius r
class Obstacle:
    def __init__(self, Coordinates_2d_tuple, r):
        self.r = r
        self.x = Coordinates_2d_tuple[0]
        self.y = Coordinates_2d_tuple[1]

    # Creates tangent nodes from the given waypoint toward the obstacle to allow dodging it
    def Create_tangent_nodes(self, Waypoint_1):
        Distance_to_waypoint = Geometrical_functions.Distance_2d(Waypoint_1.Point_2d(), self.Point_2d())
        # Check if Waypoint is the same as center of obstacle
        if Geometrical_functions.Float_eq(Distance_to_waypoint, 0):
            return None, None
        # Check if Waypoint is far enough for normal node creation
        elif Distance_to_waypoint >= self.r + 2 * OBSTACLE_DISTANCE_FOR_VALID_NODE:

            # safe distance radius
            Safe_r = self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE

            # Get Axis vector from Waypoint to center of Obstacle
            Axis_vector = Geometrical_functions.Sub_vectors(self.Point_2d(), Waypoint_1.Point_2d())
            Axis_unit = Geometrical_functions.Unit_vector(Axis_vector)
            Axis_distance = Geometrical_functions.Norm(Axis_vector)
            # Get normal vectors to the line connecting the Waypoint and the center of the obstacle
            N_vector_1, N_vector_2 = Geometrical_functions.Unit_normal_vectors_to_line(Waypoint_1.Point_2d(),
                                                                                       self.Point_2d())
            # distance on axis from Waypoint to Obstacle center for the nodes
            Node_Axis_distance = (Axis_distance ** 2 - Safe_r ** 2) / Axis_distance

            # distance on normal axis for offset of the new nodes
            Node_normal_distance = (Safe_r / Axis_distance) * (math.sqrt(Axis_distance ** 2 - Safe_r ** 2))

            # Scale vectors
            Axis_scaled = Geometrical_functions.Scale_vector(Axis_unit, Node_Axis_distance)
            S_vector_1 = Geometrical_functions.Scale_vector(N_vector_1, Node_normal_distance)
            S_vector_2 = Geometrical_functions.Scale_vector(N_vector_2, Node_normal_distance)

            # compute vectors from waypoint to nodes
            Waypoint_to_node_1 = Geometrical_functions.Add_vectors(Axis_scaled, S_vector_1)
            Waypoint_to_node_2 = Geometrical_functions.Add_vectors(Axis_scaled, S_vector_2)
            # add vectors to Waypoint location type
            Node_1 = Waypoint(Geometrical_functions.Add_vectors(Waypoint_1.Point_2d(), Waypoint_to_node_1),
                              Parent_obstacle=self)
            Node_2 = Waypoint(Geometrical_functions.Add_vectors(Waypoint_1.Point_2d(), Waypoint_to_node_2),
                              Parent_obstacle=self)
            return Node_1, Node_2

        # Check if Waypoint is inside obstacle
        elif Distance_to_waypoint <= self.r:
            return None, None
        # Check if Waypoint is in obstacle orbit for dense node creation
        else:
            # Find angle of rotation around orbit
            Orbit_angle = 2 * math.acos(
                (self.r + OBSTACLE_DISTANCE_FOR_VALID_PASS * OBSTACLE_ORBIT_RATIO) / (
                        self.r + OBSTACLE_DISTANCE_FOR_VALID_NODE))

            # find vector from center of obstacle to waypoint
            Axis_vector = Geometrical_functions.Sub_vectors(Waypoint_1.Point_2d(), self.Point_2d())

            # rotate axis vector around Orbit_angle
            New_axis_vector_1 = Geometrical_functions.Rotate_vector(Axis_vector, Orbit_angle)
            New_axis_vector_2 = Geometrical_functions.Rotate_vector(Axis_vector, -Orbit_angle)

            # add vectors to center of obstacle location type
            Node_1 = Waypoint(Geometrical_functions.Add_vectors(self.Point_2d(), New_axis_vector_1),
                              Parent_obstacle=self)
            Node_2 = Waypoint(Geometrical_functions.Add_vectors(self.Point_2d(), New_axis_vector_2),
                              Parent_obstacle=self)

            return Node_1, Node_2

    # return 2d coordinates
    def Point_2d(self):
        return self.x, self.y


# Border vertex object with x,y position
class Border_vertex:
    def __init__(self, Coordinates_2d_tuple):
        self.x = Coordinates_2d_tuple[0]
        self.y = Coordinates_2d_tuple[1]

    # return 2d coordinates
    def Point_2d(self):
        return self.x, self.y


# Border object made with multiple vertex objects
class Border:
    def __init__(self, Border_vertex_list):
        self.Border_vertex_list = Border_vertex_list

    # returns a path made from the Border vertices
    def Get_Path(self):
        return Path(list(Waypoint(Vertex.Point_2d()) for Vertex in self.Border_vertex_list))

    # Returns list of edges of the border
    def Compute_simple_edges(self):
        Edges_list = list()
        if len(self.Border_vertex_list) > 1:
            for i in range(len(self.Border_vertex_list)):
                Edges_list.append((self.Border_vertex_list[i - 1], self.Border_vertex_list[i]))
        return Edges_list

    # Create vertex nodes on concave border vertices
    def Create_vertex_nodes(self):
        Node_list = list()
        if len(self.Border_vertex_list) > 2:
            # go through all vertices to see if a node can be created on them
            for Border_vertex_index in range(len(self.Border_vertex_list)):
                # Get current, previous and next Border vertices
                Current_vertex = self.Border_vertex_list[Border_vertex_index % len(self.Border_vertex_list)]
                Previous_vertex = self.Border_vertex_list[(Border_vertex_index - 1) % len(self.Border_vertex_list)]
                Next_vertex = self.Border_vertex_list[(Border_vertex_index + 1) % len(self.Border_vertex_list)]

                # Get the vectors linking them
                Current_to_previous_vertex = Geometrical_functions.Sub_vectors(Previous_vertex.Point_2d(),
                                                                               Current_vertex.Point_2d())
                Current_to_next_vertex = Geometrical_functions.Sub_vectors(Next_vertex.Point_2d(),
                                                                           Current_vertex.Point_2d())
                # Check if the vectors are non-nul
                if not (Geometrical_functions.Float_eq_2d(Current_to_previous_vertex,
                                                          (0, 0)) or Geometrical_functions.Float_eq_2d(
                        Current_to_next_vertex, (0, 0))):

                    Current_to_previous_vertex = Geometrical_functions.Unit_vector(Current_to_previous_vertex)
                    Current_to_next_vertex = Geometrical_functions.Unit_vector(Current_to_next_vertex)

                    # Find direction away from border vertex and add a node there
                    Vector_away = Geometrical_functions.Add_vectors(Current_to_previous_vertex, Current_to_next_vertex)

                    # Check if the vectors do not add up to a null vector
                    if not Geometrical_functions.Float_eq_2d(Vector_away, (0, 0)):
                        Unit_away = Geometrical_functions.Unit_vector(Vector_away)
                        Offset_away = Geometrical_functions.Scale_vector(Unit_away, BORDER_DISTANCE_FOR_VALID_NODE)
                        Node_list.append(
                            Waypoint(Geometrical_functions.Sub_vectors(Current_vertex.Point_2d(), Offset_away),
                                     Parent_border_vertex=Current_vertex))
        return Node_list


# z = None for Waypoints created during straight path generation
# Waypoint object with x,y,z position, parent object, and alleviation waypoint
# parent Obstacle: node created on an obstacle during straight path generation
# parent Border vertex:  node created on a border vertex during straight path generation
# Alleviation waypoint: waypoint added during path curving to make the turn on the waypoint easier
class Waypoint:
    def __init__(self, Coordinates_2d_tuple, z=None, Parent_obstacle=None, Parent_border_vertex=None):
        self.x = Coordinates_2d_tuple[0]
        self.y = Coordinates_2d_tuple[1]
        self.z = z
        self.Parent_obstacle = Parent_obstacle
        self.Parent_border_vertex = Parent_border_vertex
        self.Alleviation_waypoint = None

    # computes 2d distance to other Waypoint
    def Distance_2d_to(self, Other_waypoint):
        return math.hypot(self.x - Other_waypoint.x, self.y - Other_waypoint.y)

    # computes 2d distance to other Waypoint
    def Distance_3d_to(self, Other_waypoint):
        return math.hypot(self.x - Other_waypoint.x, self.y - Other_waypoint.y, self.z - Other_waypoint.z)

    # Checks if this waypoint can be connected to another through a straight line without hitting an obstacle/border
    def Is_connectable_to(self, Other_waypoint, Obstacles, Border_1):

        # This stores whether a collision with an obstacle/border was detected
        Collision = False

        # check if there is a collision with some obstacle
        for Obstacle_i in Obstacles:
            # check if the segment connecting the two waypoints intersects with the Obstacle
            if Geometrical_functions.Segment_to_circle_intersection(self.Point_2d(),
                                                                    Other_waypoint.Point_2d(),
                                                                    Obstacle_i.Point_2d(),
                                                                    Obstacle_i.r + OBSTACLE_DISTANCE_FOR_VALID_PASS):
                Collision = True

        # check if there is a collision with some border line
        for Border_vertex_index in range(len(Border_1.Border_vertex_list)):
            Border_vertex_1 = Border_1.Border_vertex_list[Border_vertex_index]
            Border_vertex_2 = Border_1.Border_vertex_list[(Border_vertex_index + 1) % len(Border_1.Border_vertex_list)]
            # check if the segment connecting the two waypoints intersects with the border line
            if Geometrical_functions.Segment_to_segment_intersection_with_safe_distance(self.Point_2d(),
                                                                                        Other_waypoint.Point_2d(),
                                                                                        Border_vertex_1.Point_2d(),
                                                                                        Border_vertex_2.Point_2d(),
                                                                                        BORDER_DISTANCE_FOR_VALID_PASS):
                Collision = True

        return not Collision

    # return 2d coordinates
    def Point_2d(self):
        return self.x, self.y

    # Check if waypoint coincides with another waypoint
    def Coincides_with(self, Waypoint_1):
        return Geometrical_functions.Float_eq_2d(self.Point_2d(), Waypoint_1.Point_2d())

    # Check if path is going backwards
    def Is_going_backwards(self, Waypoint_1, Waypoint_2):
        Last_to_current = Geometrical_functions.Sub_vectors(self.Point_2d(), Waypoint_1.Point_2d())
        Current_to_Next = Geometrical_functions.Sub_vectors(Waypoint_2.Point_2d(), self.Point_2d())
        if Geometrical_functions.Distinct_points_3(Last_to_current, Current_to_Next, (0, 0)):
            return abs(Geometrical_functions.Find_angle(Last_to_current, Current_to_Next)) > math.pi / 2
        else:
            return True

    # Check if path through node is not crossing over obstacle or border (bouncing off it instead)
    def Is_bouncing(self, Waypoint_1, Waypoint_2):
        # cannot bounce without a node created on a map object
        if self.Parent_obstacle is None and self.Parent_border_vertex is None:
            return False
        else:
            if self.Parent_obstacle is not None:
                return not Geometrical_functions.Is_crossing_over_edge(Waypoint_1.Point_2d(), Waypoint_2.Point_2d(),
                                                                       self.Point_2d(), self.Parent_obstacle.Point_2d())
            elif self.Parent_border_vertex is not None:
                return not Geometrical_functions.Is_crossing_over_edge(Waypoint_1.Point_2d(), Waypoint_2.Point_2d(),
                                                                       self.Point_2d(),
                                                                       self.Parent_border_vertex.Point_2d())


# path made from multiple waypoints
class Path:
    def __init__(self, Waypoint_list):
        self.Waypoint_list = Waypoint_list
        self.Simple_distance_2d = None
        self.Full_distance_3d = None
        self.Distance_2d_update()

    # returns the path itself
    def Get_Path(self):
        return self

    # Returns list of edges of the path without alleviation waypoints
    def Compute_simple_edges(self):
        Edges_list = list()
        if len(self.Waypoint_list) > 1:
            for i in range(len(self.Waypoint_list) - 1):
                Edges_list.append((self.Waypoint_list[i], self.Waypoint_list[i + 1]))
        return Edges_list

    # Returns list of edges of the path while alleviation waypoints
    def Compute_full_edges(self):
        Edges_list = list()
        if len(self.Waypoint_list) > 1:
            # Add alleviation waypoint of first waypoint in list if there is one
            if self.Waypoint_list[0].Alleviation_waypoint is not None:
                Edges_list.append((self.Waypoint_list[0].Alleviation_waypoint, self.Waypoint_list[0]))
            # Add edges from Waypoint to waypoint with inclusion if middle alleviation waypoint if there is one
            for i in range(len(self.Waypoint_list) - 1):
                if self.Waypoint_list[i + 1].Alleviation_waypoint is None:
                    Edges_list.append((self.Waypoint_list[i], self.Waypoint_list[i + 1]))
                else:
                    Edges_list.append((self.Waypoint_list[i], self.Waypoint_list[i + 1].Alleviation_waypoint))
                    Edges_list.append((self.Waypoint_list[i + 1].Alleviation_waypoint, self.Waypoint_list[i + 1]))
        return Edges_list

    # computes 2D distance of the path without considering altitude or alleviation waypoints
    def Compute_simple_distance_2d(self):
        return sum(Edge[0].Distance_2d_to(Edge[1]) for Edge in self.Compute_simple_edges())

    # computes 3D distance of the path while considering altitude and alleviation waypoints
    def Compute_full_distance_3d(self):
        return sum(Edge[0].Distance_3d_to(Edge[1]) for Edge in self.Compute_full_edges())

    # updates 2D distance
    def Distance_2d_update(self):
        self.Simple_distance_2d = self.Compute_simple_distance_2d()

    # updates 3D distance
    def Distance_3d_update(self):
        self.Full_distance_3d = self.Compute_full_distance_3d()

    # Check if waypoint is part of path
    def Contains_waypoint(self, Waypoint_1):
        Found = False
        for Waypoint_i in self.Waypoint_list:
            if Geometrical_functions.Distance_2d(Waypoint_i.Point_2d(), Waypoint_1.Point_2d()) < NODE_MIN_DISTANCE:
                Found = True
        return Found

    # find total distance of path plus an extra waypoint
    def Path_distance_to_waypoint(self, Next_waypoint):
        Last_node = self.Waypoint_list[len(self.Waypoint_list) - 1]
        return self.Simple_distance_2d + Geometrical_functions.Distance_2d(Last_node.Point_2d(),
                                                                           Next_waypoint.Point_2d())
