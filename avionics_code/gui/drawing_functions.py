from avionics_code.helpers import geometrical_functions as g_f, parameters as para
from avionics_code.helpers import global_variables as g_v, geography_functions as gg_f

import pygame
import math

DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
ARROW_HEAD_SIZE = para.ARROW_HEAD_SIZE


def draw_map_marker(zoom, surf, frame_5_coordinates, text):
    """draw map markers like takeoff and landing"""

    land_off_pos = gg_f.geographic_to_cartesian_center_5(frame_5_coordinates)
    dash_pos = g_v.gui.dashboard_projection_pos(land_off_pos)
    pygame.draw.circle(surf, (0, 0, 0), dash_pos, WAYPOINT_SIZE * zoom)

    zoomed_font = pygame.font.SysFont('Arial', int(7 * (DASHBOARD_SIZE / 650) * zoom))
    label_x = zoomed_font.render(text, True, (255, 255, 255))
    rect_x = label_x.get_rect(center=dash_pos)
    surf.blit(label_x, rect_x)


def draw_centered_text(surface, font, text, percentage_pos, color=(0, 0, 0)):
    """draws text centered on surface"""

    D_S = DASHBOARD_SIZE
    label_x = font.render(text, True, color)
    rect_x = label_x.get_rect(center=(D_S * percentage_pos[0], percentage_pos[1] * D_S / 15))
    surface.blit(label_x, rect_x)


def draw_rescaled_points(zoom, surface, map_object_list, selected, color, style=0):
    """draw a list of map objects with the one
    selected and the previous one being bigger"""

    for i in range(len(map_object_list)):
        # check if the waypoint is one of the selected waypoints
        is_selected_1 = selected % max(1, len(map_object_list)) == i
        is_selected_2 = (selected + 1) % max(1, len(map_object_list)) == i
        # if the waypoint is selected, make it bigger
        if is_selected_1 or is_selected_2:
            waypoint_size = WAYPOINT_SIZE * 1.5 * zoom
        else:
            waypoint_size = WAYPOINT_SIZE * zoom
        vertex_dash = g_v.gui.dashboard_projection(map_object_list[i])
        # style 0 means draw circle waypoints
        if style == 0:
            pygame.draw.circle(surface, color, vertex_dash, waypoint_size)
            # style 1 means draw diamond waypoint
        elif style == 1:
            w_s = waypoint_size
            pairs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
            # get edges of diamond
            v_d = vertex_dash
            points = [(v_d[0] + w_s * pair[0], v_d[1] + w_s * pair[1]) for pair in pairs]
            pygame.draw.polygon(surface, color, points)


def draw_triangle(zoom, surface, object_origin, angle, size, color):
    """Draw a triangle at the object origin with the angle
    given from the north counterclockwise"""

    origin = object_origin.pos
    angles = [(2/3)*math.pi, 0, -(2/3)*math.pi]
    vecs = [g_f.rotate_vector((size/(zoom ** 0.5), 0), angle_i) for angle_i in angles]
    vecs.append((0, 0))

    rotated_vecs = [g_f.rotate_vector(vec, angle) for vec in vecs]
    vertices = [g_f.add_vectors(vec, origin) for vec in rotated_vecs]
    dash_vertices = [g_v.gui.dashboard_projection_pos(vertex) for vertex in vertices]
    pygame.draw.polygon(surface, color, dash_vertices)
    pygame.draw.polygon(surface, (0, 0, 0), dash_vertices, width=2)


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
