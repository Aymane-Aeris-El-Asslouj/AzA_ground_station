from references import global_variables as g_v
from references import parameters as para
from utility_functions import geometrical_functions as g_f
from utility_functions import geography_functions as gg_f
from pygui import layer as g_l
from pygui import colors as col

from avionics_gui import drawing_functions as d_f

import pygame
import math

WAYPOINT_SIZE = para.WAYPOINT_SIZE
W_S = WAYPOINT_SIZE
PATH_COLORING_CYCLE = para.PATH_COLORING_CYCLE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE
TAKEOFF_COORDINATES = para.TAKEOFF_COORDINATES
LANDING_COORDINATES = para.LANDING_COORDINATES
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES
DARKISH_PURPLE = col.DARKISH_PURPLE
LIGHTISH_PURPLE = col.LIGHTISH_PURPLE
BLUE = col.BLUE
BLACK = col.BLACK
WHITE = col.WHITE
GREEN = col.GREEN
NEON_GREEN = col.NEON_GREEN
GREENISH_YELLOW = col.GREENISH_YELLOW
DARKISH_RED = col.DARKISH_RED


class MissionProfileLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S, D_S))

        # drag and drop objects
        drag_and_drop = {
            "drag airdrop": 2.3 * W_S,
            "drag airdrop goal": 1.4 * W_S,
            "drag emergent object": 1.4 * W_S,
            "drag off axis object": 2.2 * W_S,
            "drag lost comms": 1.5 * W_S
        }

        # add drag and drop objects
        for key in drag_and_drop:
            self.layer_objects[key] = MPDragAndDrop(self, key,
                                                    drag_and_drop[key])

    def redraw(self):
        """Displays information of the mission profile
        border, obstacles, mission waypoints, etc"""

        # shortcuts
        settings = self.g_u_i.settings
        user_interface = self.g_u_i.layers["user interface"]
        background = self.g_u_i.layers["background"]
        dash_proj = background.dashboard_projection
        dash_proj_pos = background.dashboard_projection_pos
        zoom = background.zoom()
        aalines = pygame.draw.aalines
        cir = pygame.draw.circle

        # draw mission profile elements
        # if the mission profile was retrieved
        if g_v.get_success(g_v.sc, "retrieval_status"):
            # load info from Mission profile
            mission_ways = g_v.mp.mission_waypoints
            obstacles = g_v.mp.obstacles
            border = g_v.mp.border
            search_area = g_v.mp.search_area
            ugv_area = g_v.mp.ugv_area
            mapping_area = g_v.mp.mapping_area

            # draw object search area
            if settings["button input search"]["activated"]:

                # draw polygon of search area
                surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
                surf.set_alpha(100)
                points = list(dash_proj(v) for v in search_area.vertices)
                if len(search_area.vertices) > 2:
                    pygame.draw.polygon(surf, LIGHTISH_PURPLE, points, width=0)
                elif len(search_area.vertices) > 1:
                    aalines(surf, BLUE, closed=False, points=points)
                self.surface.blit(surf, (0, 0))

                # draw corners of search area
                surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
                surf.set_alpha(150)
                points = search_area.vertices
                d_r_p = d_f.draw_rescaled_points
                sel_vertices = user_interface.selection["search"]
                d_r_p(surf, points, sel_vertices, color=LIGHTISH_PURPLE)
                self.surface.blit(surf, (0, 0))

            # airdrop boundary area
            surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
            surf.set_alpha(100)
            polygon = list(dash_proj(v) for v in ugv_area.vertices)
            pygame.draw.polygon(surf, DARKISH_PURPLE, polygon, width=0)
            self.surface.blit(surf, (0, 0))

            # mapping area
            if settings["button input mapping"]["activated"]:
                surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
                surf.set_alpha(50)  # alpha level
                if len(mapping_area.vertices) > 2:
                    polygon = list(dash_proj(v) for v in mapping_area.vertices)
                    pygame.draw.polygon(surf, GREEN, polygon, width=0)
                self.surface.blit(surf, (0, 0))

            # draw the border (edges of border and border vertexes
            # with the ones selected being bigger)
            if settings["button input border"]["activated"]:
                if len(border.vertices) > 1:
                    points = list(dash_proj(v) for v in border.vertices)
                    aalines(self.surface, DARKISH_RED, closed=True, points=points)
                points = border.vertices
                d_r_p = d_f.draw_rescaled_points
                sel_vertices = user_interface.selection["border"]
                d_r_p(self.surface, points, sel_vertices, color=DARKISH_RED)

            # draw obstacles in yellow
            if settings["button input obstacles"]["activated"]:
                for obs_i in obstacles:
                    dash_xy = dash_proj(obs_i)
                    dash_r = obs_i.r * background.map_scaling
                    cir(self.surface, GREENISH_YELLOW, dash_xy, dash_r)

            # draw mission waypoints on top of Final path in black
            if settings["button input waypoints"]["activated"]:
                d_r_p = d_f.draw_rescaled_points
                sel_vertices = user_interface.selection["waypoints"]
                d_r_p(self.surface, mission_ways, sel_vertices, BLUE, 1)

        # map markers
        d_m_m = d_f.draw_map_marker
        d_m_m(self.surface, TAKEOFF_COORDINATES, "T")
        d_m_m(self.surface, LANDING_COORDINATES, "L")
        d_m_m(self.surface, LANDING_LOITER_COORDINATES, "L")

        # draw landing loiter circle
        g_t_c_5 = gg_f.geographic_to_cartesian_center_5
        loiter_cen = g_t_c_5(LANDING_LOITER_COORDINATES)
        dash_loiter_cen = dash_proj_pos(loiter_cen)
        l_rad = LANDING_LOITER_COORDINATES["radius"]
        rad = abs(l_rad) * background.map_scaling
        cir(self.surface, BLACK, dash_loiter_cen, rad, width=int(zoom))

        # draw orientation of landing loiter
        cs = math.copysign
        WS_zoom = W_S * zoom

        # draw upper orientation triangle
        orientation_dash = g_f.add_vectors(dash_loiter_cen, (0, -rad))
        o_dash = orientation_dash
        up_vertex = g_f.add_vectors(o_dash, (0, WS_zoom))
        down_vertex = g_f.add_vectors(o_dash, (0, -WS_zoom))
        side_vertex = g_f.add_vectors(o_dash, (cs(WS_zoom, l_rad), 0))
        poly = [up_vertex, down_vertex, side_vertex]
        pygame.draw.polygon(self.surface, BLACK, poly)

        # draw lower orientation triangle
        orientation_dash = g_f.add_vectors(dash_loiter_cen, (0, rad))
        o_dash = orientation_dash
        up_vertex = g_f.add_vectors(o_dash, (0, WS_zoom))
        down_vertex = g_f.add_vectors(o_dash, (0, -WS_zoom))
        side_vertex = g_f.add_vectors(o_dash, (cs(WS_zoom, -l_rad), 0))
        poly = [up_vertex, down_vertex, side_vertex]
        pygame.draw.polygon(self.surface, BLACK, poly)


class MPDragAndDrop(g_l.DragAndDrop):
    """drag and drop objects specific to the mission profile"""

    def __init__(self, layer, key, size):
        super().__init__(layer, BLACK, (-500000, -500000), size, "")

        # store key of object
        self.key = key

        # references to mission profile objects
        self.mp_objects = {
            "drag airdrop": lambda: g_v.mp.airdrop_object,
            "drag airdrop goal": lambda: g_v.mp.ugv_goal_object,
            "drag emergent object": lambda: g_v.mp.emergent_object,
            "drag lost comms": lambda: g_v.mp.lost_comms_object,
            "drag off axis object": lambda: g_v.mp.off_axis_object
        }

        # functions to set an object position
        # in the mission profile
        self.release_sets = {
            "drag airdrop": lambda x: g_v.mp.set_airdrop(x),
            "drag airdrop goal": lambda x: g_v.mp.set_airdrop_goal(x),
            "drag emergent object": lambda x: g_v.mp.set_emergent_obj(x),
            "drag lost comms": lambda x: g_v.mp.set_lostcomms(x),
            "drag off axis object": lambda x: g_v.mp.set_offaxis_obj(x)
        }

        # functions to draw objects
        self.draws = {
            "drag airdrop": self.draw_airdrop,
            "drag airdrop goal": self.draw_airdrop_goal,
            "drag emergent object": self.draw_emergent_object,
            "drag lost comms": self.draw_lost_comms,
            "drag off axis object": self.draw_off_axis_object
        }

        # names of settings for objects
        self.settings_name = {
            "drag airdrop": "button input airdrop",
            "drag airdrop goal": "button input airdrop goal",
            "drag emergent object": "button input emergent",
            "drag lost comms": "button input lost comms",
            "drag off axis object": "button input off axis"
        }

    def mouse_button_down(self, event, cur_pos):
        """react to mouse button down event"""

        # check activation status
        settings = self.layer.g_u_i.settings
        if not settings[self.settings_name[self.key]]["activated"]:
            return

        # shortcuts
        zoom, obj_dash = self.get_zoom_obj()

        # set position to current position of the object
        self.pos = obj_dash

        # start being dragged if clicked on considering zoom
        if event.button == 1:
            if g_f.distance_2d(cur_pos, self.pos) < self.size * zoom:

                # check if controller is busy
                if not self.layer.g_u_i.is_controller_busy():

                    # check if plane on ground
                    if g_v.th is None:
                        return
                    if not g_v.th.in_air.data_received():
                        return
                    if g_v.th.in_air.data["in air"]:
                        return

                    self.shine = 5
                    self.dragged = True
                    self.initial_cur_pos = cur_pos
                    self.initial_pos = self.pos

    def mouse_button_up(self, event, cur_pos):
        """react to button up event"""

        # check activation status
        settings = self.layer.g_u_i.settings
        if not settings[self.settings_name[self.key]]["activated"]:
            return

        # update position of mission profile object
        if event.button == 1 and self.dragged:
            background = self.layer.g_u_i.layers["background"]
            map_pos = background.map_projection(self.pos)
            self.release_sets[self.key](map_pos)

        super().mouse_button_up(event, cur_pos)

    def redraw(self):
        """react to redraw event"""

        # check activation status
        settings = self.layer.g_u_i.settings
        if not settings[self.settings_name[self.key]]["activated"]:
            return

        # shortcuts
        zoom, obj_dash = self.get_zoom_obj()

        # call hover shine with the right zoom and position
        self.size *= zoom
        if not self.dragged:
            self.hover_shine(obj_dash)
        else:
            self.hover_shine(self.pos)
        self.size /= zoom

        # draw direct position during dragging
        # and true position at rest
        if self.dragged:
            pos = self.pos
        else:
            pos = obj_dash

        # draw the mission profile object
        self.draws[self.key](pos, zoom)

    def tick_event(self, cur_pos):
        """react to tick event"""

        # check activation status
        settings = self.layer.g_u_i.settings
        if not settings[self.settings_name[self.key]]["activated"]:
            return

        # shortcuts
        zoom, obj_dash = self.get_zoom_obj()

        # call the shine decay and hover detection with
        # the right zoom and position
        self.size *= zoom
        self.pos = obj_dash
        super().tick_event(cur_pos)
        self.size /= zoom

    def get_zoom_obj(self):
        """get zoom and mission profile object
        position on dashboard"""

        # shortcuts
        g_u_i = self.layer.g_u_i
        background = g_u_i.layers["background"]
        zoom = background.zoom()
        dash_proj = background.dashboard_projection
        obj_dash = dash_proj(self.mp_objects[self.key]())

        return zoom, obj_dash

    def draw_airdrop(self, pos, zoom):
        """draw airdrop"""

        cir = pygame.draw.circle

        # draw waypoint with a certain size and width considering the zoom
        def way_cir(in_surf, color, in_pos, size,
                    in_zoom, in_width):
            cir(in_surf, color, in_pos, W_S * size * zoom,
                width=int(in_width * in_zoom))

        way_cir(self.layer.surface, BLACK, pos, 2.3, zoom, 0)
        way_cir(self.layer.surface, WHITE, pos, 2, zoom, 1)
        way_cir(self.layer.surface, WHITE, pos, 1, zoom, 1)

    def draw_airdrop_goal(self, pos, zoom):
        """draw airdrop"""

        cir = pygame.draw.circle
        cir(self.layer.surface, BLACK, pos, W_S * 1.4 * zoom)
        cir(self.layer.surface, DARKISH_PURPLE, pos, W_S * 1 * zoom)

    def draw_off_axis_object(self, pos, zoom):
        """draw off axis object"""

        # draw waypoint with a certain size and width considering the zoom
        def way_cir(in_surf, color, in_pos, size,
                    in_zoom, in_width):
            pygame.draw.circle(in_surf, color,
                               in_pos, W_S * size * zoom,
                               width=int(in_width * in_zoom))

        way_cir(self.layer.surface, BLACK, pos, 2.3, zoom, 4)
        way_cir(self.layer.surface, WHITE, pos, 2, zoom, 1)
        way_cir(self.layer.surface, BLACK, pos, 1.8, zoom, 2)

    def draw_emergent_object(self, pos, zoom):
        """draw emergent object"""

        cir = pygame.draw.circle
        cir(self.layer.surface, BLACK, pos, W_S * 1.4 * zoom)
        cir(self.layer.surface, NEON_GREEN, pos, W_S * 1 * zoom)

    def draw_lost_comms(self, pos, zoom):
        """draw lost comms"""

        # draw rectangle with a certain width considering zoom
        def rect(in_surf, color, rectangle,
                 in_width, in_zoom):
            pygame.draw.rect(in_surf, color,
                             rectangle, width=int(in_width * in_zoom))

        # get pygame rectangle from center and side
        def get_rect(in_pos, in_side):
            i_s = in_side
            return pygame.Rect(in_pos[0] - i_s / 2, in_pos[1] - i_s / 2, i_s, i_s)

        lost_rect = get_rect(pos, W_S * 2.5 * zoom)
        rect(self.layer.surface, BLACK, lost_rect, 2, zoom)
        lost_rect = get_rect(pos, W_S * 2 * zoom)
        rect(self.layer.surface, WHITE, lost_rect, 2, zoom)
        lost_rect = get_rect(pos, W_S * 1.5 * zoom)
        rect(self.layer.surface, BLACK, lost_rect, 2, zoom)
