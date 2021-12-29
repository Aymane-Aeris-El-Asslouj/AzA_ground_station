from avionics_code.references import parameters as para
from avionics_code.utility_functions import geometrical_functions as g_f
from avionics_code.utility_functions import geography_functions as gg_f
from avionics_code.references import global_variables as g_v
from avionics_code.pygui import colors as col

import pygame
import math

DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE
W_S = WAYPOINT_SIZE
ARROW_HEAD_SIZE = para.ARROW_HEAD_SIZE

FONT_TYPE = para.FONT_TYPE
F_T = FONT_TYPE
FONT_SIZE = para.FONT_SIZE
F_S = FONT_SIZE

BLACK = col.BLACK
WHITE = col.WHITE


def draw_map_marker(surf, frame_5_coordinates, text):
    """draw map markers like takeoff and landing"""

    # get coordinates in frame 5
    geo_5 = gg_f.geographic_to_cartesian_center_5
    land_off_pos = geo_5(frame_5_coordinates)
    background = g_v.gui.layers["background"]
    zoom = background.zoom()
    dash_pos = background.dashboard_projection_pos(land_off_pos)

    # draw background black circle
    pygame.draw.circle(surf, BLACK, dash_pos, W_S * zoom)

    # draw text on top of it
    zoomed_font = pygame.font.SysFont(F_T, int(F_S/2 * zoom))
    label_x = zoomed_font.render(text, True, WHITE)
    rect_x = label_x.get_rect(center=dash_pos)
    surf.blit(label_x, rect_x)


def draw_centered_text(surface, font, text,
                       relative_pos, color=BLACK):
    """draws text centered on surface"""

    label_x = font.render(text, True, color)
    rect_x = label_x.get_rect(center=(D_S * relative_pos[0],
                                      relative_pos[1] * D_S / 15))
    surface.blit(label_x, rect_x)


def draw_rescaled_points(surface, map_object_list,
                         selected, color, style=0):
    """draw a list of map objects with the one
    selected and the previous one being bigger"""

    # shortcuts
    background = g_v.gui.layers["background"]
    zoom = background.zoom()
    dash_proj = background.dashboard_projection

    for i in range(len(map_object_list)):
        # check if the waypoint is one of the selected waypoints
        obj_num = max(1, len(map_object_list))
        is_selected_1 = selected % obj_num == i
        is_selected_2 = (selected + 1) % obj_num == i
        # if the waypoint is selected, make it bigger
        if is_selected_1 or is_selected_2:
            w_s = W_S * 1.5 * zoom
        else:
            w_s = W_S * zoom
        vertex_dash = dash_proj(map_object_list[i])
        # style 0 means draw circle waypoints
        if style == 0:
            pygame.draw.circle(surface, color, vertex_dash, w_s)
            # style 1 means draw diamond waypoint
        elif style == 1:
            # get edges of diamond
            w_s = W_S * zoom
            cen = vertex_dash
            corners = g_f.rect_from_cen_size(cen, w_s, w_s)
            rot_cen = g_f.rotate_vector_with_center
            new_corners = [rot_cen(corner, cen, math.pi/4)
                           for corner in corners]
            pygame.draw.polygon(surface, color, new_corners)


def draw_skew_triangle(surface, object_origin, angle,
                       size, color):
    """Draw a triangle at the object origin with the angle
    given from the north counterclockwise"""

    # shortcuts
    background = g_v.gui.layers["background"]
    dash_proj_pos = background.dashboard_projection_pos
    rot = g_f.rotate_vector
    origin = object_origin.pos

    # get edges of skewed triangle
    angles = [(2/3) * math.pi, 0, - (2/3) * math.pi]
    vecs = [rot((size, 0), angle_i) for angle_i in angles]
    vecs.append((0, 0))
    
    # rotate and project on dashboard
    rotated_vecs = [rot(vec, angle) for vec in vecs]
    vertices = [g_f.add_vectors(vec, origin) for vec in rotated_vecs]
    dash_vertices = [dash_proj_pos(vertex) for vertex in vertices]

    # draw skewed triangle
    pygame.draw.polygon(surface, color, dash_vertices)
    pygame.draw.polygon(surface, BLACK, dash_vertices, width=2)


def draw_arrow(surface, object_origin, map_vector, color):
    """Draw an arrow starting at an object origin
    and guided by a map vector"""

    # shortcuts
    origin = object_origin.pos
    scale = g_f.scale_vector
    sub = g_f.sub_vectors
    add = g_f.add_vectors
    background = g_v.gui.layers["background"]
    dash_proj = background.dashboard_projection
    dash_proj_pos = background.dashboard_projection_pos

    L = ARROW_HEAD_SIZE * (800/D_S)
    h = ARROW_HEAD_SIZE * (800/D_S)

    len_v = g_f.norm(map_vector)

    # check that the arrow size is non-zero
    if len_v > 0:

        # get point to which the arrow head should be pointing
        map_vector = scale(map_vector, 1 - ARROW_HEAD_SIZE / len_v)

        # get unit vectors in the direction
        # and perpendicular to the arrow
        unit = g_f.unit_vector(map_vector)
        perp = g_f.rotate_vector(unit, math.pi/2)

        # scale those unit vectors by arrow dimensions
        new_unit = scale(unit, L)
        new_perp = scale(perp, h)

        # get endpoint of arrow head
        end = add(origin, map_vector)

        # sides of arrow head
        left = add(sub(end, new_unit), new_perp)
        right = sub(sub(end, new_unit), new_perp)

        # project all arrow points on dashboard
        origin_dash = dash_proj(object_origin)
        end_dash = dash_proj_pos(end)
        left_dash = dash_proj_pos(left)
        right_dash = dash_proj_pos(right)

        # draw arrow body and head
        pygame.draw.line(surface, color,
                         origin_dash, end_dash, width=1)
        points = [left_dash, right_dash, end_dash]
        pygame.draw.polygon(surface, color, points)
