from references import global_variables as g_v
from references import parameters as para
from utility_functions import geometrical_functions as g_f
from pygui import layer as g_l
from pygui import colors as col

from avionics_gui import drawing_functions as d_f

import pygame
import math

WAYPOINT_SIZE = para.WAYPOINT_SIZE
W_S = WAYPOINT_SIZE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE

BLACK = col.BLACK
RED = col.RED
CYAN = col.CYAN
GREEN = col.GREEN


class MissionStateLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S, D_S))

    def redraw(self):
        """Displays information of the mission state: way list"""

        if not g_v.get_success(g_v.ms, "generation_status"):
            return

        # shortcuts
        background = self.g_u_i.layers["background"]
        zoom = background.zoom()
        settings = self.g_u_i.settings
        inactive_display = settings["button toggle inactive points"]["on"]

        if settings["button toggle mission state"]["on"]:
            surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
            surf.set_alpha(150)

            # draw way list in cyan
            prev_way_active = False
            waypoint_list = g_v.ms.waypoint_list
            for index, way in enumerate(waypoint_list):

                # draw active ways in cyan and those done in green
                if way.is_mission == g_v.Activity.ACTIVE:
                    # if this is the first active way, make it bigger
                    if not prev_way_active:
                        size = W_S * 0.75 * math.sqrt(zoom)
                        color = RED
                    else:
                        size = W_S * 0.25 * math.sqrt(zoom)
                        color = CYAN
                    prev_way_active = True
                else:
                    size = W_S * 0.25 * math.sqrt(zoom)
                    prev_way_active = False
                    color = GREEN

                if inactive_display or way.is_mission == g_v.Activity.ACTIVE:
                    # find the mission index of the way
                    mission_type = way.mission_type

                    MT = g_v.MissionType
                    trans = {
                        MT.TAKEOFF: True,
                        MT.WAYPOINTS: settings["button toggle waypoints"]["on"],
                        MT.IMAGING: settings["button toggle imaging"]["on"],
                        MT.OFF_AXIS: settings["button toggle off axis"]["on"],
                        MT.LANDING: True,
                    }

                    # draw the way with its target and arrow from previous way
                    if trans[mission_type]:

                        # draw the way
                        center = background.dashboard_projection(way)
                        s = size * 2
                        points = g_f.rect_from_cen_size(center, s, s)
                        pygame.draw.polygon(surf, color, points)

                        # draw arrow from previous way to the current one
                        if index > 0:
                            pre_way = waypoint_list[index - 1]
                            delta_pos = g_f.sub_vectors(way.pos, pre_way.pos)
                            d_f.draw_arrow(surf, pre_way, delta_pos, color)

                        # draw target of the way
                        if way.target is not None:
                            delta_pos = g_f.sub_vectors(way.target, way.pos)
                            d_f.draw_arrow(surf, way, delta_pos, BLACK)

            self.surface.blit(surf, (0, 0))
