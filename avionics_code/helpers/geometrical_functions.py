import math
import avionics_code.helpers.parameters as para

FLOAT_DIFFERENCE_FOR_EQUALITY = para.FLOAT_DIFFERENCE_FOR_EQUALITY


def float_eq(f1, f2):
    """check float equality"""
    
    return abs(f2 - f1) < FLOAT_DIFFERENCE_FOR_EQUALITY


def float_eq_2d(p1, p2):
    """check 2d_float equality"""
    
    return math.hypot(p2[1] - p1[1], p2[0] - p1[0]) < FLOAT_DIFFERENCE_FOR_EQUALITY


def distance_2d(p1, p2):
    """distance between two 2d points"""
    
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def center_2d(p1, p2):
    """gives center of two points"""
    
    return (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2


def norm(vector):
    """get norm of vector"""
    
    return math.hypot(vector[0], vector[1])


def unit_vector(vector):
    """gives unit vector from vector"""
    
    if float_eq_2d(vector, (0, 0)):
        return None
    else:
        return vector[0] / distance_2d(vector, (0, 0)), vector[1] / distance_2d(vector, (0, 0))


def add_vectors(vector_1, vector_2):
    """add vectors:"""
    
    return vector_1[0] + vector_2[0], vector_1[1] + vector_2[1]


def sub_vectors(vector_1, vector_2):
    """add vectors:"""
    
    return vector_1[0] - vector_2[0], vector_1[1] - vector_2[1]


def scale_vector(vector, scalar):
    """scale vector"""
    
    return scalar * vector[0], scalar * vector[1]


def mirror_vector_x(vector):
    """mirror a vector's x value"""
    
    return -vector[0], vector[1]


def mirror_vector_y(vector):
    """mirror a vector's y value"""
    
    return vector[0], -vector[1]


def rotate_vector(vector, angle):
    """rotates vector"""
    
    new_x = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
    new_y = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
    return new_x, new_y


def rotate_vector_with_center(vector, center, angle):
    """rotates a vector with respect to some center"""
    
    return add_vectors(center, rotate_vector(sub_vectors(vector, center), angle))


def homothety_unit(vector, center, factor):
    """pushes some vector away so it is distance factor away"""
    
    return add_vectors(center, scale_vector(unit_vector(sub_vectors(vector, center)), factor))


def vector_average(vector_list):
    """gives average of a list of vectors"""
    
    if len(vector_list) == 0:
        return None
    else:
        vec_x = sum(list(vector_i[0] for vector_i in vector_list)) / len(vector_list)
        vec_y = sum(list(vector_i[1] for vector_i in vector_list)) / len(vector_list)
        return vec_x, vec_y


def clamp_angle(angle):
    """angle clamp to (-pi,pi]"""
    
    while angle <= - math.pi:
        angle += 2 * math.pi
    while angle > math.pi:
        angle -= 2 * math.pi
    return angle


def find_angle(vector_1, vector_2):
    """find angle between two vectors angle(vector_2)-angle(vector_1)"""
    
    if norm(vector_1) == 0 or norm(vector_2) == 0:
        return None
    else:
        angle = math.atan2(vector_2[1], vector_2[0]) - math.atan2(vector_1[1], vector_1[0])
        return clamp_angle(angle)


def line_from_points(point_1, point_2):
    """give the equation of a line from two points"""
    
    x_1, y_1 = point_1
    x_2, y_2 = point_2
    return -(y_2 - y_1), (x_2 - x_1), x_2 * (y_2 - y_1) - y_2 * (x_2 - x_1)


def unit_normal_vectors_to_line(point_1, point_2):
    """get unit normal vectors to a line"""
    
    if float_eq_2d(point_1, point_2):
        return None
    else:
        a, b, c = line_from_points(point_1, point_2)
        return unit_vector((a, b)), unit_vector((-a, -b))

def point_to_line_projection(point, vertex_1, vertex_2):
    """get projection of point on line"""

    if float_eq_2d(vertex_1, vertex_2):
        return None

    normal = unit_normal_vectors_to_line(vertex_1, vertex_2)
    point_2 = add_vectors(point, normal[0])

    return line_intersection(point, point_2, vertex_1, vertex_2)


def distinct_points_2(point_1, point_2):
    """check that 2 points are distinct"""
    
    return not float_eq_2d(point_2, point_1)


def distinct_points_3(point_1, point_2, point_3):
    """check that 3 points are distinct"""
    
    d_1 = distinct_points_2(point_1, point_2)
    d_2 = distinct_points_2(point_2, point_3)
    d_3 = distinct_points_2(point_1, point_3)
    return d_1 and d_2 and d_3


def distinct_points_4(point_1, point_2, point_3, point_4):
    """check that 4 points are distinct"""
    
    d_1 = distinct_points_3(point_1, point_2, point_3)
    d_2 = distinct_points_3(point_1, point_2, point_4)
    d_3 = distinct_points_3(point_1, point_4, point_3)
    d_4 = distinct_points_3(point_4, point_2, point_3)
    return d_1 and d_2 and d_3 and d_4


def point_to_line_distance(point_3, point_1, point_2):
    """find distance between point 3 and line of point_1 and point_2"""
    
    # if the two points do not form a line, get distance from point 3 to the two points
    if float_eq_2d(point_1, point_2):
        return distance_2d(point_3, point_2)
    else:
        # get equation of line between the two points:
        x_3, y_3 = point_3
        a, b, c = line_from_points(point_1, point_2)
        # get distance between line and center of circle
        return abs(a * x_3 + b * y_3 + c) / (math.hypot(a, b))


def line_to_circle_intersection(point_1, point_2, circle_cen, circle_rad):
    """checks if a line intersects with a circle"""
    
    # if the two points do not form a line, verify that they are not inside the obstacle
    if float_eq_2d(point_1, point_2):
        return distance_2d(point_1, circle_cen) < circle_rad
    else:
        # get distance between line and center of circle
        line_to_center_distance = point_to_line_distance(circle_cen, point_1, point_2)
        return line_to_center_distance < circle_rad


def circle_to_circle_intersection(circle_cen_1, circle_rad_1, circle_cen_2, circle_rad_2):
    """check if two circles intersect"""
    
    return distance_2d(circle_cen_1, circle_cen_2) < circle_rad_1 + circle_rad_2


def seg_to_disk_intersection(point_1, point_2, circle_cen, circle_rad):
    """check if a circle intersects with a seg"""

    if float_eq_2d(point_1, point_2):
        return distance_2d(point_1, circle_cen) < circle_rad
    else:
        # check if circle center is inside pill area around seg of radius circle radius
        # move reference point to point_1
        point_2_new = sub_vectors(point_2, point_1)
        circle_cen_new = sub_vectors(circle_cen, point_1)

        # rotate points around reference point 1 to make the seg horizontal
        angle_of_rotation = -find_angle((1, 0), point_2_new)
        circle_cen_new = rotate_vector(circle_cen_new, angle_of_rotation)

        # check if circle center is inside a pill around
        # the two points of radius the circle's radius
        if abs(circle_cen_new[1]) < circle_rad:
            if 0 < circle_cen_new[0] < norm(sub_vectors(point_2, point_1)):
                return True
            elif norm(circle_cen_new) < circle_rad:
                return True
            elif norm(sub_vectors(circle_cen, point_2)) < circle_rad:
                return True
            else:
                return False
        else:
            return False


def point_inside_circle(point, circle_cen, circle_rad):
    """check if point is inside circle"""
    
    return distance_2d(point, circle_cen) < circle_rad


def point_on_seg(point_3, point_1, point_2):
    """check point 3 is on seg [point 1, point 2]"""
    
    is_on_line = float_eq(point_to_line_distance(point_3, point_1, point_2), 0)
    middle = center_2d(point_1, point_2)
    half_seg = distance_2d(point_1, point_2) / 2
    is_inside_range = point_inside_circle(point_3, middle, half_seg)
    return is_on_line and is_inside_range


def line_intersection(point_1, point_2, point_3, point_4):
    """find the intersection of two lines"""
    
    # if None of the points form lines, check if they are the same
    if float_eq_2d(point_1, point_2) and float_eq_2d(point_3, point_4) and float_eq_2d(point_1, point_3):
        return point_1
    # if point 1 and point 2 do not form a line, check if they are inside the other seg
    elif float_eq_2d(point_1, point_2) and (not float_eq_2d(point_3, point_4)):
        if point_on_seg(point_1, point_3, point_4):
            return point_1
        else:
            return None
    # if point 3 and point 4 do not form a line, check if they are inside the other seg
    elif float_eq_2d(point_3, point_4) and (not float_eq_2d(point_1, point_2)):
        if point_on_seg(point_3, point_1, point_2):
            return point_3
        else:
            return None
    # if they do form lines, check if their intersection is on one of the two segments
    else:
        a1, b1, c1 = line_from_points(point_1, point_2)
        a2, b2, c2 = line_from_points(point_3, point_4)

        # check if they are parallel
        if float_eq(a1 * b2 - a2 * b1, 0):
            # check if they are the same line
            if float_eq(a1 * c2 - a2 * c1, 0):
                return float("+inf"), float("+inf")
            else:
                return None
        else:
            x = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1)
            y = (c1 * a2 - c2 * a1) / (a1 * b2 - a2 * b1)
            return x, y


def seg_to_seg_intersection(point_1, point_2, point_3, point_4):
    """check if two segments intersect:"""
    
    # if None of the points form lines, check if they are the same
    if float_eq_2d(point_1, point_2) and float_eq_2d(point_3, point_4):
        return float_eq_2d(point_1, point_3)
    # if point 1 and point 2 do not form a line, check if they are inside the other seg
    elif float_eq_2d(point_1, point_2) and (not float_eq_2d(point_3, point_4)):
        return point_on_seg(point_1, point_3, point_4)
    # if point 3 and point 4 do not form a line, check if they are inside the other seg
    elif float_eq_2d(point_3, point_4) and (not float_eq_2d(point_1, point_2)):
        return point_on_seg(point_3, point_1, point_2)
    # if they do form lines, check if their intersection is on one of the two segments
    else:
        intersection = line_intersection(point_1, point_2, point_3, point_4)
        if intersection is None:
            return False
        elif float_eq_2d(intersection, (float("+inf"), float("+inf"))):
            return True
        else:
            p_1 = point_on_seg(intersection, point_1, point_2)
            p_2 = point_on_seg(intersection, point_3, point_4)
            return p_1 and p_2


def seg_to_line_intersection(point_1, point_2, point_3, point_4):
    """check if a seg intersects with a intersect:"""
    
    # if None of the points form lines, check if they are the same
    if float_eq_2d(point_1, point_2) and float_eq_2d(point_3, point_4):
        return float_eq_2d(point_1, point_3)
    # if point 1 and point 2 do not form a line, check if they are inside the other seg
    elif float_eq_2d(point_1, point_2) and (not float_eq_2d(point_3, point_4)):
        return point_on_seg(point_1, point_3, point_4)
    # if point 3 and point 4 do not form a line, check if they are inside the other seg
    elif float_eq_2d(point_3, point_4) and (not float_eq_2d(point_1, point_2)):
        return point_on_seg(point_3, point_1, point_2)
    # if they do form lines, check if their intersection is on one of the two segments
    else:
        intersection = line_intersection(point_1, point_2, point_3, point_4)
        if intersection is None:
            return False
        elif float_eq_2d(intersection, (float("+inf"), float("+inf"))):
            return True
        else:
            p_1 = point_on_seg(intersection, point_1, point_2)
            return p_1


def seg_to_seg_with_safety(point_1, point_2, point_3, point_4, safety_distance):
    """check if seg 1 intersects seg 2 in an
    area close to it by some safety distance"""
    
    s_d = safety_distance
    
    # check if points are not intersecting two larger zones around the seg 2
    middle = center_2d(point_3, point_4)
    quarter_seg = distance_2d(point_3, point_4) / 4
    quarter_1 = center_2d(middle, point_3)
    quarter_2 = center_2d(middle, point_4)
    dis = quarter_seg + s_d
    if not seg_to_disk_intersection(point_1, point_2, quarter_1, dis):
        if not seg_to_disk_intersection(point_1, point_2, quarter_2, dis):
            return False

    # if None of the points form lines,
    # check if they are within safety distance of each other
    if float_eq_2d(point_1, point_2) and float_eq_2d(point_3, point_4):
        return distance_2d(point_1, point_3) < s_d
    # if point 1 and point 2 do not form a line,
    # check if they are inside the safety zone of seg 2
    elif float_eq_2d(point_1, point_2) and (not float_eq_2d(point_3, point_4)):
        return seg_to_disk_intersection(point_3, point_4, point_1, s_d)
    # if point 3 and point 4 do not form a line,
    # check if the first seg intersects their safety zone
    elif float_eq_2d(point_3, point_4) and (not float_eq_2d(point_1, point_2)):
        return seg_to_disk_intersection(point_1, point_2, point_3, s_d)
    # if they do form lines,
    # check if the seg 1intersect the pill safety zone of the seg 2
    else:
        # check if seg (1-2) does not intersect with circles around point 3 and 4
        if seg_to_disk_intersection(point_1, point_2, point_3, s_d):
            return True
        elif seg_to_disk_intersection(point_1, point_2, point_4, s_d):
            return True
        # check if points 1 and 2 and not inside the safety zone of the seg
        elif seg_to_disk_intersection(point_3, point_4, point_1, s_d):
            return True
        elif seg_to_disk_intersection(point_3, point_4, point_2, s_d):
            return True
        else:
            # seg (1-2) does not intersect with safety segments to the side of seg (3-4)
            offset_normal_1, offset_normal_2 = unit_normal_vectors_to_line(point_3, point_4)
            point_3_off_1 = add_vectors(point_3, scale_vector(offset_normal_1, s_d))
            point_3_off_2 = add_vectors(point_3, scale_vector(offset_normal_2, s_d))
            point_4_off_1 = add_vectors(point_4, scale_vector(offset_normal_1, s_d))
            point_4_off_2 = add_vectors(point_4, scale_vector(offset_normal_2, s_d))
            if seg_to_seg_intersection(point_1, point_2, point_3_off_1, point_4_off_1):
                return True
            elif seg_to_seg_intersection(point_1, point_2, point_3_off_2, point_4_off_2):
                return True
            else:
                return False


def is_crossing_over_edge(point_1, point_2, crossing_point, edge_point):
    """check if path is crossing over an edge in a useful manner"""
    
    # check that all points are distinct
    if distinct_points_4(point_1, point_2, crossing_point, edge_point):

        # put all points in reference frame of edge_point
        new_point_1 = sub_vectors(point_1, edge_point)
        new_point_2 = sub_vectors(point_2, edge_point)
        new_crossing_point = sub_vectors(crossing_point, edge_point)

        # rotate all such that the crossing point is above the origin
        rotation_angle = -find_angle((0, 1), new_crossing_point)
        new_point_1 = rotate_vector(new_point_1, rotation_angle)
        new_point_2 = rotate_vector(new_point_2, rotation_angle)
        new_crossing_point = rotate_vector(new_crossing_point, rotation_angle)

        # translate all point downwards so the crossing point is at (0,0),
        new_point_1 = sub_vectors(new_point_1, new_crossing_point)
        new_point_2 = sub_vectors(new_point_2, new_crossing_point)

        # mirror the start and ending points
        # so that the starting point is to the left
        if new_point_1[0] > 0:
            new_point_1 = mirror_vector_x(new_point_1)
            new_point_2 = mirror_vector_x(new_point_2)

        # if waypoints are on the same side, it is not a useful crossing
        if new_point_2[0] <= 0 or float_eq(new_point_1[0], 0):
            return False
        # check if the path is crossing over the edge
        else:
            return new_point_2[1] / new_point_2[0] < new_point_1[1] / new_point_1[0]
    else:
        return True


def tangent_points(point, circle_cen, circle_rad):
    """find tangent points to circle passing from other point"""
    
    if distance_2d(point, circle_cen) <= circle_rad:
        return None, None
    # get axis vector from waypoint to center of turn center
    axis_vector = sub_vectors(circle_cen, point)
    axis_unit = unit_vector(axis_vector)
    axis_distance = norm(axis_vector)
    # get normal vectors to the line connecting the waypoint and the center of the obstacle
    n_vector_1, n_vector_2 = unit_normal_vectors_to_line(point, circle_cen)
    # distance on axis from waypoint to obstacle center for the nodes
    node_axis_distance = (axis_distance ** 2 - circle_rad ** 2) / axis_distance

    # distance on normal axis for offset of the new nodes
    a = (circle_rad / axis_distance)
    b = (math.sqrt(axis_distance ** 2 - circle_rad ** 2))
    node_normal_distance = a * b

    # scale vectors
    axis_scaled = scale_vector(axis_unit, node_axis_distance)
    s_vector_1 = scale_vector(n_vector_1, node_normal_distance)
    s_vector_2 = scale_vector(n_vector_2, node_normal_distance)

    # compute vectors from waypoint to nodes
    way_to_node_1 = add_vectors(axis_scaled, s_vector_1)
    way_to_node_2 = add_vectors(axis_scaled, s_vector_2)
    # add vectors to waypoint location type
    return add_vectors(point, way_to_node_1), add_vectors(point, way_to_node_2)


def find_arc_angles_dash(point_1, point_mid, point_2, point_center):
    """find angles of arc from point mid to point 2 coming from
    point 1 around point center"""
    
    m_center = mirror_vector_y(point_center)
    m_point_1 = mirror_vector_y(point_1)
    m_point_mid = mirror_vector_y(point_mid)
    m_point_2 = mirror_vector_y(point_2)
    # find which angle to start from to get the right arc
    a_1 = find_angle((1, 0), sub_vectors(m_point_1, m_center))
    a_mid = find_angle((1, 0), sub_vectors(m_point_mid, m_center))
    a_2 = find_angle((1, 0), sub_vectors(m_point_2, m_center))
    if -3.14 < a_mid - a_1 < 0 or 3.14 < a_mid - a_1 < 6.30:
        return a_2, a_mid
    else:
        return a_mid, a_2


def point_inside_polygon(point, polygon):
    """check if a point is inside a polygon"""
    
    if len(polygon) < 3:
        return None
    
    # count number of intersections of a half life going to the right
    # from the point with the edges of the polygon
    count = 0
    for index in range(len(polygon)):
        cur_vertex = polygon[index]
        next_vertex = polygon[(index + 1) % len(polygon)]
        
        # point to the right of the source point
        side_point = add_vectors(point, (1, 0))
        
        # intersection between a horizontal bidirectional
        # ray from the source point with the edge
        inter = line_intersection(cur_vertex, next_vertex, point, side_point)

        # check that an intersection happened
        if inter is not None:
            if inter != (float("+inf"), float("+inf")):
                # check that the intersection to the right of the
                # source point (on the right ray)
                if inter[0] >= point[0]:
                    # check that the intersection is on the edge
                    if point_on_seg(inter, cur_vertex, next_vertex):
                        count += 1
            else:
                # check if one of the vertices is to the right
                # of the source point
                if cur_vertex[0] >= inter[0] or next_vertex[0] >= inter[0]:
                    count += 1
        
    return count % 2 == 1


def point_to_seg_distance(point, vertex_1, vertex_2):
    """returns distance from a line to a segment"""

    return distance_2d(point, point_to_seg_projection(point, vertex_1, vertex_2))


def point_to_seg_projection(point, vertex_1, vertex_2):
    """finds closest point on segment to main point"""

    if float_eq_2d(vertex_1, vertex_2):
        return vertex_1

    proj = point_to_line_projection(point, vertex_1, vertex_2)

    # if orthogonal projection of point on segment
    # is inside it, that is the closest point
    if point_on_seg(proj, vertex_1, vertex_2):
        return proj

    # if not, return closest vertex
    if distance_2d(point, vertex_1) < distance_2d(point, vertex_2):
        return vertex_1
    else:
        return vertex_2


