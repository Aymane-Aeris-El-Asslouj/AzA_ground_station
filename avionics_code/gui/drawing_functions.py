from avionics_code.helpers import geometrical_functions as g_f, parameters as para
from avionics_code.helpers import global_variables as g_v
import pygame
import math

DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
ARROW_HEAD_SIZE = para.ARROW_HEAD_SIZE
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS


def draw_centered_text(surface, font, text, percentage_pos, color=(0, 0, 0)):
    """draws text centered on surface"""

    D_S = DASHBOARD_SIZE
    label_x = font.render(text, True, color)
    rect_x = label_x.get_rect(center=(D_S * percentage_pos[0], percentage_pos[1] * D_S / 15))
    surface.blit(label_x, rect_x)


def draw_rescaled_points(surface, map_object_list, selected, color, style=0):
    """draw a list of map objects with the one
    selected and the previous one being bigger"""

    for i in range(len(map_object_list)):
        # check if the waypoint is one of the selected waypoints
        is_selected_1 = selected % max(1, len(map_object_list)) == i
        is_selected_2 = (selected + 1) % max(1, len(map_object_list)) == i
        # if the waypoint is selected, make it bigger
        if is_selected_1 or is_selected_2:
            waypoint_size = WAYPOINT_SIZE * 1.5
        else:
            waypoint_size = WAYPOINT_SIZE
        vertex_dash = g_v.gui.dashboard_projection(map_object_list[i])
        # style 0 means draw circle waypoints
        if style == 0:
            pygame.draw.circle(surface, color, vertex_dash, waypoint_size)
            # style 1 means draw diamond waypoint
        elif style == 1:
            w_s = waypoint_size
            pairs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            # get edges of diamond
            points = [(vertex_dash[0] + w_s * pair[0], vertex_dash[1] + w_s * pair[1]) for pair in pairs]
            pygame.draw.polygon(surface, color, points)


def draw_curved_edge(surface, way_1, way_2, color):
    """draw edge between two waypoints along with turning points"""

    # Get previous vertex or its off shoot waypoint if it exists
    vertex_1 = g_v.gui.dashboard_projection(way_1)
    pygame.draw.circle(surface, (255, 255, 0), vertex_1, WAYPOINT_SIZE)
    # Get next vertex
    vertex_2 = g_v.gui.dashboard_projection(way_2)
    pygame.draw.circle(surface, (255, 255, 0), vertex_2, WAYPOINT_SIZE)

    # turn waypoint after vertex 1
    if way_1.post_turn_waypoint is not None:
        after_turn = g_v.gui.dashboard_projection(way_1.post_turn_waypoint)
        pygame.draw.circle(surface, (255, 105, 180), after_turn, WAYPOINT_SIZE)
        pygame.draw.line(surface, (255, 105, 180), vertex_1, after_turn)
        vertex_1 = after_turn

    # turn waypoint before vertex 2
    if way_2.pre_turn_waypoint is not None:
        before_turn = g_v.gui.dashboard_projection(way_2.pre_turn_waypoint)
        pygame.draw.circle(surface, (255, 105, 180), before_turn, WAYPOINT_SIZE)
        pygame.draw.line(surface, (255, 105, 180), before_turn, vertex_2)
        vertex_2 = before_turn

    pygame.draw.line(surface, color, vertex_1, vertex_2, width=2)


def draw_triangle(surface, object_origin, angle, size, color):
    """Draw a triangle at the object origin with the angle
    given from the north counterclockwise"""

    origin = object_origin.pos
    angles = [(2/3)*math.pi, 0, -(2/3)*math.pi]
    vecs = [g_f.rotate_vector((0, size), angle_i) for angle_i in angles]
    vecs.append((0, 0))

    rotated_vecs = [g_f.rotate_vector(vec, angle) for vec in vecs]
    vertices = [g_f.add_vectors(vec, origin) for vec in rotated_vecs]
    dash_vertices = [g_v.gui.dashboard_projection_pos(vertex) for vertex in vertices]
    pygame.draw.polygon(surface, color, dash_vertices)


def draw_arrow(surface, object_origin, map_vector, color):
    """Draw an arrow starting at an object origin
    and guided by a map vector"""

    origin = object_origin.pos

    L = ARROW_HEAD_SIZE * (800/DASHBOARD_SIZE)
    h = ARROW_HEAD_SIZE * (800/DASHBOARD_SIZE)

    len_v = g_f.norm(map_vector)

    # check that the arrow size is non-zero
    if len_v > 0:

        # get point to which the arrow head should be pointing
        map_vector = g_f.scale_vector(map_vector, 1 - ARROW_HEAD_SIZE / len_v)

        # get unit vectors in the direction and perpendicular to the arrow
        unit = g_f.unit_vector(map_vector)
        perp = g_f.rotate_vector(unit, math.pi/2)

        # scale those unit vectors by arrow dimensions
        new_unit = g_f.scale_vector(unit, L)
        new_perp = g_f.scale_vector(perp, h)

        # get endpoint of arrow head
        end = g_f.add_vectors(origin, map_vector)

        # sides of arrow head
        left = g_f.add_vectors(g_f.sub_vectors(end, new_unit), new_perp)
        right = g_f.sub_vectors(g_f.sub_vectors(end, new_unit), new_perp)

        # project all arrow points on dashboard
        origin_dash = g_v.gui.dashboard_projection(object_origin)
        end_dash = g_v.gui.dashboard_projection_pos(end)
        left_dash = g_v.gui.dashboard_projection_pos(left)
        right_dash = g_v.gui.dashboard_projection_pos(right)

        # draw arrow body and head
        pygame.draw.line(surface, color, origin_dash, end_dash, width=1)
        pygame.draw.polygon(surface, color, [left_dash, right_dash, end_dash])
