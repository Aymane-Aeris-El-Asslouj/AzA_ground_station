from references import global_variables as g_v
from references import parameters as para
from utility_functions import geometrical_functions as g_f
from utility_functions import algebraic_functions as a_f
from pygui import layer as g_l
from pygui import colors as col

from avionics_gui import drawing_functions as d_f

import pygame
import math

WAYPOINT_SIZE = para.WAYPOINT_SIZE
PATH_COLORING_CYCLE = para.PATH_COLORING_CYCLE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE

PINK = col.PINK
RED = col.RED
GREEN = col.GREEN
YELLOW = col.YELLOW
BLACK = col.BLACK


class PathLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S, D_S))

    def redraw(self):
        """draw current path to be exported or already
        exported to the plane"""

        if not g_v.get_success(g_v.mc, "path_computation_status"):
            return
        
        # shortcuts
        settings = self.g_u_i.settings
        turn_display = settings["button toggle turn waypoints"]["on"]
        inactive_waypoints = settings["button toggle inactive points"]["on"]
        background = self.g_u_i.layers["background"]
        dash_proj = background.dashboard_projection
        cir = pygame.draw.circle
        line = pygame.draw.line

        # check if the path we got to is active
        active_reached = False

        if settings["button toggle computed path"]["on"]:
            # draw the chosen path with a gradient of color
            rotator = a_f.RGBRotate()
            pre_way = None

            # run over all waypoints of the curved path
            chosen_path = g_v.mc.chosen_path.waypoint_list
            for index, way in enumerate(chosen_path):

                # check when the first active waypoint is reached
                act = way.is_mission == g_v.Activity.ACTIVE
                if act and not active_reached:
                    active_reached = True

                MT = g_v.MissionType
                trans = {
                    MT.PLANE: True,
                    MT.TAKEOFF: True,
                    MT.WAYPOINTS: settings["button toggle waypoints"]["on"],
                    MT.IMAGING: settings["button toggle imaging"]["on"],
                    MT.OFF_AXIS: settings["button toggle off axis"]["on"],
                    MT.LANDING: True,
                }

                # draw waypoints if they are active
                # or if crossed ones are allowed
                if inactive_waypoints or active_reached:

                    # draw the waypoint if its mission type is displayed
                    if trans[way.mission_type] and pre_way is not None:

                        # get the color of the waypoint
                        color_factor = index / PATH_COLORING_CYCLE
                        rotator.set_hue_rotation(color_factor * 360)
                        color = rotator.apply(*RED)

                        # get dashboard position of previous
                        # and current waypoints
                        vertex_1 = dash_proj(pre_way)
                        vertex_2 = dash_proj(way)

                        # draw inactive waypoints in green
                        if active_reached == 1:
                            way_color = YELLOW
                        else:
                            way_color = GREEN

                        # draw the waypoint
                        W_S = WAYPOINT_SIZE * 0.75 * math.sqrt(background.zoom())
                        cir(self.surface, way_color, vertex_2, W_S)

                        # turn waypoint after vertex 1
                        p_t_w = pre_way.post_turn_waypoint
                        if p_t_w is not None and turn_display:
                            after_turn = dash_proj(p_t_w)
                            cir(self.surface, PINK, after_turn, W_S)
                            line(self.surface, PINK, vertex_1, after_turn)
                            pre_way = p_t_w

                        # turn waypoint before vertex 2
                        p_t_w = way.pre_turn_waypoint
                        if p_t_w is not None and turn_display:
                            before_turn = dash_proj(p_t_w)
                            cir(self.surface, BLACK, before_turn, W_S)
                            line(self.surface, PINK, before_turn, vertex_2)
                            way = p_t_w

                        # draw arrow of inactive waypoints in green
                        if active_reached == 1:
                            color = color
                        else:
                            color = GREEN

                        # draw middle arrow between waypoints
                        map_vector = g_f.sub_vectors(way.pos, pre_way.pos)
                        d_f.draw_arrow(self.surface, pre_way, map_vector, color)

                # store this waypoint as the previous one for the next waypoint
                pre_way = way
