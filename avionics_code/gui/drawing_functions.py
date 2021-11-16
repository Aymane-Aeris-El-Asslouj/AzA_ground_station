from avionics_code.helpers import geometrical_functions as g_f, parameters as para, global_variables as g_v
import pygame
import math

DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
ARROW_HEAD_SIZE = para.ARROW_HEAD_SIZE
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS


def draw_text_off(position, text, color):
    """draws text near some position"""

    ratio = (DASHBOARD_SIZE / 650)
    font_2 = pygame.font.SysFont('Arial', int(10 * ratio))
    text_1 = font_2.render(text, True, color)
    g_v.gui.screen.blit(text_1, (position[0] + 10 * ratio, position[1] - 10 * ratio))


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
            pygame.draw.polygon(g_v.gui.screen, color, points)

def draw_path_points_straight(path, color):
    """draw waypoints of path"""

    for i in range(len(path.waypoint_list)):
        vertex_dash_1 = g_v.gui.dashboard_projection(path.waypoint_list[i])
        pygame.draw.circle(g_v.gui.screen, color, vertex_dash_1, WAYPOINT_SIZE)

def draw_path_points_curved(path, color, altitude=False):
    """draw waypoints of path including alleviation waypoints, offshoot waypoints
    and optionally altitude"""

    for i in range(len(path.waypoint_list)):
        vertex_dash_1 = g_v.gui.dashboard_projection(path.waypoint_list[i])
        pygame.draw.circle(g_v.gui.screen, color, vertex_dash_1, WAYPOINT_SIZE)
        if altitude:
            g_v.gui.draw_text_off(vertex_dash_1, str(int(path.waypoint_list[i].z)) + " ft")
        vertex_off_w = path.waypoint_list[i].off_waypoint
        # draw offshoot waypoint if there is
        if vertex_off_w is not None:
            if vertex_off_w.pos != (None, None):
                vertex_off = g_v.gui.dashboard_projection(vertex_off_w)
                pygame.draw.circle(g_v.gui.screen, (0, 255, 0), vertex_off, WAYPOINT_SIZE)
        # draw alleviation waypoint if there is
        if path.waypoint_list[i].alleviation_waypoint is not None:
            alleviation_waypoint = path.waypoint_list[i].alleviation_waypoint
            vertex_dash_mid = g_v.gui.dashboard_projection(alleviation_waypoint)
            pygame.draw.circle(g_v.gui.screen, (255, 105, 180), vertex_dash_mid, WAYPOINT_SIZE)
            if altitude:
                pass  # g_v.gui.draw_text_off(vertex_dash_mid, str(int(alleviation_waypoint.z))+" ft")


def draw_edges_alleviated_offset(map_structure, color):
    """draw edges of map structure including alleviation and off shoot waypoints"""

    edges_to_draw = map_structure.compute_simple_edges()
    for index, edge in enumerate(edges_to_draw):
        """This part gets points to draw"""
        # Get previous vertex or its off shoot waypoint if it exists
        vertex_1 = g_v.gui.dashboard_projection(edge[0])
        if edge[0].off_waypoint is not None:
            if edge[0].off_waypoint.pos != (None, None):
                vertex_1 = g_v.gui.dashboard_projection(edge[0].off_waypoint)
        # Get next vertex
        vertex_2 = g_v.gui.dashboard_projection(edge[1])
        # If the next vertex has an off shoot waypoint, use it to find center
        # at that vertex
        if edge[1].off_waypoint is not None:
            if edge[1].off_waypoint.pos != (None, None):
                #####line under this shouldn't exist
                vertex_off = g_v.gui.dashboard_projection(edge[1].off_waypoint)
            # Set zone to draw arcs around next vertex
            center = g_v.gui.dashboard_projection(edge[1].off_waypoint.alleviation_waypoint)
            radius = g_f.distance_2d(center, vertex_2)
            rect = (center[0] - radius, center[1] - radius, 2 * radius, 2 * radius)

        """drawing starts here"""
        if edge[1].alleviation_waypoint is None:
            # draw line from off waypoint_1 to waypoint 2
            """This part draws lines/areas from vertex 1 to vertex 2"""
            if (edge[0].off_waypoint is not None and edge[0].off_waypoint.pos != (None, None)) or index == 0:
                pygame.draw.line(g_v.gui.screen, color, vertex_1, vertex_2, width=2)
            else:
                # In the case where there is no offshoot waypoint, but a danger zone instead
                p_r = PREFERRED_TURN_RADIUS
                off_center = g_f.center_2d(vertex_1, vertex_2)
                pygame.draw.circle(g_v.gui.screen, (0, 0, 0), off_center, 2 * p_r * g_v.gui.map_scaling, width=2)
            # draw arc from waypoint 2 to off waypoint
            """This part draws arcs around vertex 2"""
            if edge[1].off_waypoint is not None:
                if edge[1].off_waypoint.pos != (None, None):
                    a_1, a_2 = g_f.find_arc_angles_dash(vertex_1, vertex_2, vertex_off, center)
                    # only non-zero arcs
                    if abs(a_1 - a_2) > 0.01:
                        pygame.draw.arc(g_v.gui.screen, (0, 255, 0), rect, a_1, a_2, 2)
        else:
            # Get alleviation waypoint
            vertex_mid = g_v.gui.dashboard_projection(edge[1].alleviation_waypoint)
            # draw line from vertex 1 (or its off shoot waypoint) to alleviation waypoint
            """This part draws lines/areas from vertex 1 to vertex 2"""
            if index == 0 or (edge[0].off_waypoint is not None and edge[0].off_waypoint.pos != (None, None)):
                pygame.draw.line(g_v.gui.screen, color, vertex_1, vertex_mid, width=2)
            else:
                # In the case where there is no offshoot waypoint, but a danger zone instead
                p_r = PREFERRED_TURN_RADIUS
                off_center = g_f.center_2d(vertex_1, vertex_mid)
                pygame.draw.circle(g_v.gui.screen, (0, 0, 0), off_center, 2 * p_r * g_v.gui.map_scaling, width=2)
            """This part draws arcs around vertex 2"""
            # draw arc from alleviation waypoint to vertex 2
            a_1, a_2 = g_f.find_arc_angles_dash(vertex_1, vertex_mid, vertex_2, center)
            pygame.draw.arc(g_v.gui.screen, (255, 105, 180), rect, a_1, a_2, 2)
            # draw arc from vertex 2 to off shoot waypoint
            if edge[1].off_waypoint.pos != (None, None):
                a_1, a_2 = g_f.find_arc_angles_dash(vertex_mid, vertex_2, vertex_off, center)
                # only non-zero arcs
                if abs(a_1 - a_2) > 0.01:
                    pygame.draw.arc(g_v.gui.screen, (0, 255, 0), rect, a_1, a_2, 2)


def draw_arrow(surf, object_origin, map_vector, color):
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
        pygame.draw.line(surf, color, origin_dash, end_dash, width=1)
        pygame.draw.polygon(surf, color, [left_dash, right_dash, end_dash])
