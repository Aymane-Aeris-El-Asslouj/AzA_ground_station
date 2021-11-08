import math
import parameters as Para

FLOAT_DIFFERENCE_FOR_EQUALITY = Para.FLOAT_DIFFERENCE_FOR_EQUALITY


"""Check float equality"""
def Float_eq(f1, f2):
    return abs(f2 - f1) < FLOAT_DIFFERENCE_FOR_EQUALITY


"""Check 2d_float equality"""
def Float_eq_2d(p1, p2):
    return math.hypot(p2[1] - p1[1], p2[0] - p1[0]) < FLOAT_DIFFERENCE_FOR_EQUALITY


"""distance between two 2D points"""
def Distance_2d(P1, P2):
    return math.hypot(P2[0] - P1[0], P2[1] - P1[1])


"""gives center of two points"""
def Center_2d(P1, P2):
    return (P1[0] + P2[0]) / 2, (P1[1] + P2[1]) / 2


"""Get norm of vector"""
def Norm(Vector):
    return math.hypot(Vector[0], Vector[1])


"""gives unit vector from vector"""
def Unit_vector(Vector):
    if Float_eq_2d(Vector, (0, 0)):
        return None
    else:
        return Vector[0] / Distance_2d(Vector, (0, 0)), Vector[1] / Distance_2d(Vector, (0, 0))


"""add vectors:"""
def Add_vectors(Vector_1, Vector_2):
    return Vector_1[0] + Vector_2[0], Vector_1[1] + Vector_2[1]


"""add vectors:"""
def Sub_vectors(Vector_1, Vector_2):
    return Vector_1[0] - Vector_2[0], Vector_1[1] - Vector_2[1]


"""scale vector"""
def Scale_vector(Vector, Scalar):
    return Scalar * Vector[0], Scalar * Vector[1]


"""Mirror a vector's x value"""
def Mirror_vector_x(Vector):
    return -Vector[0], Vector[1]


"""Mirror a vector's y value"""
def Mirror_vector_y(Vector):
    return Vector[0], -Vector[1]


"""rotates vector"""
def Rotate_vector(Vector, Angle):
    New_x = Vector[0] * math.cos(Angle) - Vector[1] * math.sin(Angle)
    New_y = Vector[0] * math.sin(Angle) + Vector[1] * math.cos(Angle)
    return New_x, New_y


"""Rotates a vector with respect to some center"""
def Rotate_vector_with_center(Vector, Center, Angle):
    return Add_vectors(Center, Rotate_vector(Sub_vectors(Vector, Center), Angle))


"""Pushes some vector away so it is distance Factor away"""
def Homothety_unit(Vector, Center, Factor):
    return Add_vectors(Center, Scale_vector(Unit_vector(Sub_vectors(Vector, Center)), Factor))


"""Gives average of a list of vectors"""
def Vector_average(Vector_list):
    if len(Vector_list) == 0:
        return None
    else:
        Vec_x = sum(list(Vector_i[0] for Vector_i in Vector_list)) / len(Vector_list)
        Vec_y = sum(list(Vector_i[1] for Vector_i in Vector_list)) / len(Vector_list)
        return Vec_x, Vec_y


"""Angle clamp to (-pi,pi]"""
def Clamp_angle(Angle):
    while Angle <= - math.pi:
        Angle += 2 * math.pi
    while Angle > math.pi:
        Angle -= 2 * math.pi
    return Angle


"""Find angle between two vectors Angle(Vector_2)-Angle(Vector_1)"""
def Find_angle(Vector_1, Vector_2):
    if Norm(Vector_1) == 0 or Norm(Vector_2) == 0:
        return None
    else:
        return Clamp_angle(math.atan2(Vector_2[1], Vector_2[0]) - math.atan2(Vector_1[1], Vector_1[0]))


"""give the equation of a line from two points"""
def Line_from_points(Point_1, Point_2):
    x_1, y_1 = Point_1
    x_2, y_2 = Point_2
    return -(y_2 - y_1), (x_2 - x_1), x_2 * (y_2 - y_1) - y_2 * (x_2 - x_1)


"""Get unit normal vectors to a line"""
def Unit_normal_vectors_to_line(Point_1, Point_2):
    if Float_eq_2d(Point_1, Point_2):
        return None
    else:
        a, b, c = Line_from_points(Point_1, Point_2)
        return Unit_vector((a, b)), Unit_vector((-a, -b))


"""Check that 2 points are distinct"""
def Distinct_points_2(Point_1, Point_2):
    return not Float_eq_2d(Point_2, Point_1)


"""Check that 3 points are distinct"""
def Distinct_points_3(Point_1, Point_2, Point_3):
    D_1 = Distinct_points_2(Point_1, Point_2)
    D_2 = Distinct_points_2(Point_2, Point_3)
    D_3 = Distinct_points_2(Point_1, Point_3)
    return D_1 and D_2 and D_3


"""Check that 4 points are distinct"""
def Distinct_points_4(Point_1, Point_2, Point_3, Point_4):
    D_1 = Distinct_points_3(Point_1, Point_2, Point_3)
    D_2 = Distinct_points_3(Point_1, Point_2, Point_4)
    D_3 = Distinct_points_3(Point_1, Point_4, Point_3)
    D_4 = Distinct_points_3(Point_4, Point_2, Point_3)
    return D_1 and D_2 and D_3 and D_4


"""find distance between point 3 and line of Point_1 and Point_2"""
def Point_to_line_distance(Point_3, Point_1, Point_2):
    # if the two points do not form a line, get distance from point 3 to the two points
    if Float_eq_2d(Point_1, Point_2):
        return Distance_2d(Point_3, Point_2)
    else:
        # get equation of line between the two points:
        x_3, y_3 = Point_3
        a, b, c = Line_from_points(Point_1, Point_2)
        # get distance between line and center of circle
        return abs(a * x_3 + b * y_3 + c) / (math.hypot(a, b))


"""checks if a line intersects with a circle"""
def Line_to_circle_intersection(Point_1, Point_2, Circle_center, Circle_radius):
    # if the two points do not form a line, verify that they are not inside the obstacle
    if Float_eq_2d(Point_1, Point_2):
        return Distance_2d(Point_1, Circle_center) < Circle_radius
    else:
        # get distance between line and center of circle
        Line_to_center_distance = Point_to_line_distance(Circle_center, Point_1, Point_2)
        return Line_to_center_distance < Circle_radius


"""check if two circles intersect"""
def Circle_to_circle_intersection(Circle_center_1, Circle_radius_1, Circle_center_2, Circle_radius_2):
    return Distance_2d(Circle_center_1, Circle_center_2) < Circle_radius_1 + Circle_radius_2


"""Check if a circle intersects with a segment"""
def Segment_to_disk_intersection(Point_1, Point_2, Circle_center, Circle_radius):
    if Float_eq_2d(Point_1, Point_2):
        return Distance_2d(Point_1, Circle_center) < Circle_radius
    else:
        # check if circle center is inside pill area around segment of radius circle radius
        # Move reference point to Point_1
        Point_2_new = Sub_vectors(Point_2, Point_1)
        Circle_center_new = Sub_vectors(Circle_center, Point_1)

        # rotate points around reference Point 1 to make the segment horizontal
        Angle_of_rotation = -Find_angle((1, 0), Point_2_new)
        Circle_center_new = Rotate_vector(Circle_center_new, Angle_of_rotation)

        # Check if circle center is inside a pill around the two points of radius the circle's radius
        if abs(Circle_center_new[1]) < Circle_radius:
            if 0 < Circle_center_new[0] < Norm(Sub_vectors(Point_2, Point_1)):
                return True
            elif Norm(Circle_center_new) < Circle_radius:
                return True
            elif Norm(Sub_vectors(Circle_center, Point_2)) < Circle_radius:
                return True
            else:
                return False
        else:
            return False


"""Check if point is inside circle"""
def Point_inside_Circle(Point, Circle_center, Circle_radius):
    return Distance_2d(Point, Circle_center) < Circle_radius


"""check point 3 is on segment [point 1, point 2]"""
def Point_on_segment(Point_3, Point_1, Point_2):
    Is_on_line = Float_eq(Point_to_line_distance(Point_3, Point_1, Point_2), 0)
    Middle = Center_2d(Point_1, Point_2)
    Half_segment = Distance_2d(Point_1, Point_2) / 2
    Is_inside_range = Point_inside_Circle(Point_3, Middle, Half_segment)
    return Is_on_line and Is_inside_range


"""find the intersection of two lines"""
def Line_intersection(Point_1, Point_2, Point_3, Point_4):
    # if none of the points form lines, check if they are the same
    if Float_eq_2d(Point_1, Point_2) and Float_eq_2d(Point_3, Point_4) and Float_eq_2d(Point_1, Point_3):
        return Point_1
    # if point 1 and point 2 do not form a line, check if they are inside the other segment
    elif Float_eq_2d(Point_1, Point_2) and (not Float_eq_2d(Point_3, Point_4)):
        if Point_on_segment(Point_1, Point_3, Point_4):
            return Point_1
        else:
            return None
    # if point 3 and point 4 do not form a line, check if they are inside the other segment
    elif Float_eq_2d(Point_3, Point_4) and (not Float_eq_2d(Point_1, Point_2)):
        if Point_on_segment(Point_3, Point_1, Point_2):
            return Point_3
        else:
            return None
    # if they do form lines, check if their intersection is on one of the two segments
    else:
        a1, b1, c1 = Line_from_points(Point_1, Point_2)
        a2, b2, c2 = Line_from_points(Point_3, Point_4)

        # Check if they are parallel
        if Float_eq(a1 * b2 - a2 * b1, 0):
            # Check if they are the same line
            if Float_eq(a1 * c2 - a2 * c1, 0):
                return float("+inf"), float("+inf")
            else:
                return None
        else:
            return (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1), (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1)


"""Check if two segments intersect:"""
def Segment_to_segment_intersection(Point_1, Point_2, Point_3, Point_4):
    # if none of the points form lines, check if they are the same
    if Float_eq_2d(Point_1, Point_2) and Float_eq_2d(Point_3, Point_4):
        return Float_eq_2d(Point_1, Point_3)
    # if point 1 and point 2 do not form a line, check if they are inside the other segment
    elif Float_eq_2d(Point_1, Point_2) and (not Float_eq_2d(Point_3, Point_4)):
        return Point_on_segment(Point_1, Point_3, Point_4)
    # if point 3 and point 4 do not form a line, check if they are inside the other segment
    elif Float_eq_2d(Point_3, Point_4) and (not Float_eq_2d(Point_1, Point_2)):
        return Point_on_segment(Point_3, Point_1, Point_2)
    # if they do form lines, check if their intersection is on one of the two segments
    else:
        Intersection = Line_intersection(Point_1, Point_2, Point_3, Point_4)
        if Intersection is None:
            return False
        elif Float_eq_2d(Intersection, (float("+inf"), float("+inf"))):
            return True
        else:
            P_1 = Point_on_segment(Intersection, Point_1, Point_2)
            P_2 = Point_on_segment(Intersection, Point_3, Point_4)
            return P_1 and P_2


"""Check if a segment intersects with a intersect:"""
def Segment_to_line_intersection(Point_1, Point_2, Point_3, Point_4):
    # if none of the points form lines, check if they are the same
    if Float_eq_2d(Point_1, Point_2) and Float_eq_2d(Point_3, Point_4):
        return Float_eq_2d(Point_1, Point_3)
    # if point 1 and point 2 do not form a line, check if they are inside the other segment
    elif Float_eq_2d(Point_1, Point_2) and (not Float_eq_2d(Point_3, Point_4)):
        return Point_on_segment(Point_1, Point_3, Point_4)
    # if point 3 and point 4 do not form a line, check if they are inside the other segment
    elif Float_eq_2d(Point_3, Point_4) and (not Float_eq_2d(Point_1, Point_2)):
        return Point_on_segment(Point_3, Point_1, Point_2)
    # if they do form lines, check if their intersection is on one of the two segments
    else:
        Intersection = Line_intersection(Point_1, Point_2, Point_3, Point_4)
        if Intersection is None:
            return False
        elif Float_eq_2d(Intersection, (float("+inf"), float("+inf"))):
            return True
        else:
            P_1 = Point_on_segment(Intersection, Point_1, Point_2)
            return P_1


"""Check if segment 1 intersects segment 2 in an area close to it by some safety distance"""
def Segment_to_segment_with_safety(Point_1, Point_2, Point_3, Point_4, Safety_distance):
    # Check if points are not intersecting two larger zones around the segment 2
    Middle = Center_2d(Point_3, Point_4)
    Quarter_segment = Distance_2d(Point_3, Point_4) / 4
    Quarter_1 = Center_2d(Middle, Point_3)
    Quarter_2 = Center_2d(Middle, Point_4)
    if not Segment_to_disk_intersection(Point_1, Point_2, Quarter_1, Quarter_segment + Safety_distance):
        if not Segment_to_disk_intersection(Point_1, Point_2, Quarter_2, Quarter_segment + Safety_distance):
            return False

    # if none of the points form lines, check if they are within safety distance of each other
    if Float_eq_2d(Point_1, Point_2) and Float_eq_2d(Point_3, Point_4):
        return Distance_2d(Point_1, Point_3) < Safety_distance
    # if point 1 and point 2 do not form a line, check if they are inside the safety zone of segment 2
    elif Float_eq_2d(Point_1, Point_2) and (not Float_eq_2d(Point_3, Point_4)):
        return Segment_to_disk_intersection(Point_3, Point_4, Point_1, Safety_distance)
    # if point 3 and point 4 do not form a line, check if the first segment intersects their safety zone
    elif Float_eq_2d(Point_3, Point_4) and (not Float_eq_2d(Point_1, Point_2)):
        return Segment_to_disk_intersection(Point_1, Point_2, Point_3, Safety_distance)
    # if they do form lines, check if the segment 1 intersect the pill safety zone of the segment 2
    else:
        # Check if Segment (1-2) does not intersect with circles around Point 3 and 4
        if Segment_to_disk_intersection(Point_1, Point_2, Point_3, Safety_distance):
            return True
        elif Segment_to_disk_intersection(Point_1, Point_2, Point_4, Safety_distance):
            return True
        # Check if Points 1 and 2 and not inside the safety zone of the segment
        elif Segment_to_disk_intersection(Point_3, Point_4, Point_1, Safety_distance):
            return True
        elif Segment_to_disk_intersection(Point_3, Point_4, Point_2, Safety_distance):
            return True
        else:
            # Segment (1-2) does not intersect with safety segments to the side of segment (3-4)
            Offset_normal_1, Offset_normal_2 = Unit_normal_vectors_to_line(Point_3, Point_4)
            Point_3_off_1 = Add_vectors(Point_3, Scale_vector(Offset_normal_1, Safety_distance))
            Point_3_off_2 = Add_vectors(Point_3, Scale_vector(Offset_normal_2, Safety_distance))
            Point_4_off_1 = Add_vectors(Point_4, Scale_vector(Offset_normal_1, Safety_distance))
            Point_4_off_2 = Add_vectors(Point_4, Scale_vector(Offset_normal_2, Safety_distance))
            if Segment_to_segment_intersection(Point_1, Point_2, Point_3_off_1, Point_4_off_1):
                return True
            elif Segment_to_segment_intersection(Point_1, Point_2, Point_3_off_2, Point_4_off_2):
                return True
            else:
                return False


"""Check if path is crossing over an edge in a useful manner"""
def Is_crossing_over_edge(Point_1, Point_2, Crossing_point, Edge_point):
    # Check that all points are distinct
    if Distinct_points_4(Point_1, Point_2, Crossing_point, Edge_point):

        # Put all points in reference frame of Edge_point
        New_point_1 = Sub_vectors(Point_1, Edge_point)
        New_point_2 = Sub_vectors(Point_2, Edge_point)
        New_Crossing_point = Sub_vectors(Crossing_point, Edge_point)

        # Rotate all such that the Crossing point is above the origin
        Rotation_angle = -Find_angle((0, 1), New_Crossing_point)
        New_point_1 = Rotate_vector(New_point_1, Rotation_angle)
        New_point_2 = Rotate_vector(New_point_2, Rotation_angle)
        New_Crossing_point = Rotate_vector(New_Crossing_point, Rotation_angle)

        # Translate all point downwards so the crossing point is at (0,0),
        New_point_1 = Sub_vectors(New_point_1, New_Crossing_point)
        New_point_2 = Sub_vectors(New_point_2, New_Crossing_point)

        # Mirror the start and ending points so that the starting point is to the left
        if New_point_1[0] > 0:
            New_point_1 = Mirror_vector_x(New_point_1)
            New_point_2 = Mirror_vector_x(New_point_2)

        # if waypoints are on the same side, it is not a useful crossing
        if New_point_2[0] <= 0 or Float_eq(New_point_1[0], 0):
            return False
        # Check if the path is crossing over the edge
        else:
            return New_point_2[1] / New_point_2[0] < New_point_1[1] / New_point_1[0]
    else:
        return True


"""Find tangent points to circle passing from other point"""
def Tangent_points(Point, Circle_center, Circle_radius):
    if Distance_2d(Point, Circle_center) <= Circle_radius:
        return None, None
    # Get Axis vector from Waypoint to center of Turn Center
    Axis_vector = Sub_vectors(Circle_center, Point)
    Axis_unit = Unit_vector(Axis_vector)
    Axis_distance = Norm(Axis_vector)
    # Get normal vectors to the line connecting the Waypoint and the center of the obstacle
    N_vector_1, N_vector_2 = Unit_normal_vectors_to_line(Point, Circle_center)
    # distance on axis from Waypoint to Obstacle center for the nodes
    Node_Axis_distance = (Axis_distance ** 2 - Circle_radius ** 2) / Axis_distance

    # distance on normal axis for offset of the new nodes
    Node_normal_distance = (Circle_radius / Axis_distance) * (math.sqrt(Axis_distance ** 2 - Circle_radius ** 2))

    # Scale vectors
    Axis_scaled = Scale_vector(Axis_unit, Node_Axis_distance)
    S_vector_1 = Scale_vector(N_vector_1, Node_normal_distance)
    S_vector_2 = Scale_vector(N_vector_2, Node_normal_distance)

    # compute vectors from waypoint to nodes
    Waypoint_to_node_1 = Add_vectors(Axis_scaled, S_vector_1)
    Waypoint_to_node_2 = Add_vectors(Axis_scaled, S_vector_2)
    # add vectors to Waypoint location type
    return Add_vectors(Point, Waypoint_to_node_1), Add_vectors(Point, Waypoint_to_node_2)


"""Distribute N 1s in an array of M zeros"""
def Distribution_increment(Distribution, Variation_num):
    if Binary_increment(Distribution) == 1:
        Distribution.clear()
        return
    while sum(Distribution) != Variation_num:
        if Binary_increment(Distribution) == 1:
            Distribution.clear()
            break


"""Increment a binary list with returning overflow"""
def Binary_increment(Distribution):
    carry = 1
    for index in range(len(Distribution)):
        Distribution[index] += carry
        if Distribution[index] == 2:
            Distribution[index] = 0
            carry = 1
        else:
            carry = 0
    return carry


"""Increment vector till it has enough variation"""
def Increment_bound_var(Path_vector, Distribution, Max_per_group, Variation_num):
    if Increment_bound(Path_vector, Distribution, Max_per_group) == 1:
        Path_vector.clear()
        return

    while sum(bool(index) for index in Path_vector) != Variation_num:
        if Increment_bound(Path_vector, Distribution, Max_per_group) == 1:
            Path_vector.clear()
            break


"""Increment a vector at the spots indicated by the distribution with max values at each point"""
def Increment_bound(Path_vector, Distribution, Max_per_group):
    carry = 1
    for index in range(len(Path_vector)):
        if Distribution[index] == 1:
            Path_vector[index] += carry
            if Path_vector[index] == Max_per_group[index]:
                Path_vector[index] = 0
                carry = 1
            else:
                carry = 0
    return carry


"""Find angles of arc from point mid to point 2 coming from
point 1 around point center"""
def Find_arc_angles_dash(Point_1, Point_mid, Point_2, Point_center):
    M_Center = Mirror_vector_y(Point_center)
    M_point_1 = Mirror_vector_y(Point_1)
    M_point_mid = Mirror_vector_y(Point_mid)
    M_point_2 = Mirror_vector_y(Point_2)
    # find which angle to start from to get the right arc
    A_1 = Find_angle((1, 0), Sub_vectors(M_point_1, M_Center))
    A_mid = Find_angle((1, 0), Sub_vectors(M_point_mid, M_Center))
    A_2 = Find_angle((1, 0), Sub_vectors(M_point_2, M_Center))
    if -3.14 < A_mid - A_1 < 0 or 3.14 < A_mid - A_1 < 6.30:
        return A_2, A_mid
    else:
        return A_mid, A_2


"""confine a color to RGB spectrum"""
def clamp(v):
    if v < 0:
        return 0
    if v > 255:
        return 255
    return int(v)


"""shifter that changes the hue of a color"""
class RGBRotate(object):
    def __init__(self):
        self.matrix = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]

    def set_hue_rotation(self, degrees):
        cosA = math.cos(math.radians(degrees))
        sinA = math.sin(math.radians(degrees))
        self.matrix[0][0] = cosA + (1.0 - cosA) / 3.0
        self.matrix[0][1] = 1. / 3. * (1.0 - cosA) - math.sqrt(1. / 3.) * sinA
        self.matrix[0][2] = 1. / 3. * (1.0 - cosA) + math.sqrt(1. / 3.) * sinA
        self.matrix[1][0] = 1. / 3. * (1.0 - cosA) + math.sqrt(1. / 3.) * sinA
        self.matrix[1][1] = cosA + 1. / 3. * (1.0 - cosA)
        self.matrix[1][2] = 1. / 3. * (1.0 - cosA) - math.sqrt(1. / 3.) * sinA
        self.matrix[2][0] = 1. / 3. * (1.0 - cosA) - math.sqrt(1. / 3.) * sinA
        self.matrix[2][1] = 1. / 3. * (1.0 - cosA) + math.sqrt(1. / 3.) * sinA
        self.matrix[2][2] = cosA + 1. / 3. * (1.0 - cosA)

    def apply(self, r, g, b):
        rx = r * self.matrix[0][0] + g * self.matrix[0][1] + b * self.matrix[0][2]
        gx = r * self.matrix[1][0] + g * self.matrix[1][1] + b * self.matrix[1][2]
        bx = r * self.matrix[2][0] + g * self.matrix[2][1] + b * self.matrix[2][2]
        return clamp(rx), clamp(gx), clamp(bx)
