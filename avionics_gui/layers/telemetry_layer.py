from references import global_variables as g_v
from references import parameters as para
from pygui import layer as g_l
from pygui import colors as col
from utility_functions import geometrical_functions as g_f

from avionics_gui import drawing_functions as d_f

import pygame
import time
import math

KNOTS_PER_FT_PER_S = para.KNOTS_PER_FT_PER_S
RF_TELEMETRY_ALLOWED_DELAY = para.RF_TELEMETRY_ALLOWED_DELAY
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE
PLANE_SIZE = para.PLANE_SIZE
RED = col.RED
PINK = col.PINK
GREY = col.GREY
BLACK = col.BLACK


class TelemetryLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S * 2, D_S))

        # list of plane position
        self.position_history = list()

        # testing
        self.minims = [1000000]*500

    def redraw(self):
        """Displays information of the flight profile
        ugv/plane position, time, etc"""
        
        # don't draw telemetry without the telemetry obj
        if g_v.th is None:
            return

        # shortcuts
        th = g_v.th
        settings = self.g_u_i.settings
        background = self.g_u_i.layers["background"]
        zoom = background.zoom()
        dash_proj = background.dashboard_projection
        d_c_t = d_f.draw_centered_text

        # shortcut for drawing on telemetry column
        def draw_tele(text, y_coordinate, color, under=False):
            pos = (1.55, y_coordinate)
            if under:
                font = self.font_u
            else:
                font = self.font
            d_c_t(self.surface, font, text, pos, color)

        # function for drawing telemetry data on telemetry column
        def draw_tele_data(tele_obj, converter, y):

            # get data and its converted state
            c_d = converter(tele_obj.data)

            # check if data was received
            if tele_obj.data_received():
                tele_rate = str(round(tele_obj.telemetry_rate))

                # check if data is not outdated
                delta_t = time.time() - tele_obj.telemetry_time
                if delta_t < RF_TELEMETRY_ALLOWED_DELAY:
                    color = BLACK
                else:
                    color = GREY
            else:
                tele_rate = "Nan"
                color = GREY

            # draw all converted telemetry
            for i in range(len(c_d)):
                draw_tele(str(c_d[i]) + ", " + tele_rate + " Hz",
                          y + 0.5 * i, color)

        # display telemetry on telemetry column
        draw_tele("Telemetry:", 0.5, BLACK, under=True)
        d_t_d = draw_tele_data
        d_t_d(th.position, self.position_converter, 1.0)
        d_t_d(th.velocity, self.velocity_converter, 1.5)
        d_t_d(th.armed, self.armed_converter, 2.0)
        d_t_d(th.in_air, self.in_air_converter, 2.5)
        d_t_d(th.landed, self.landed_converter, 3.0)
        d_t_d(th.flight_mode, self.flight_converter, 3.5)
        d_t_d(th.status_text, self.status_converter, 4.0)
        d_t_d(th.mission_progress, self.progress_converter, 5.0)

        # get next waypoint position
        active_way = g_v.ms.active_waypoint()
        if active_way is not None:
            way_z_text = str(round(active_way.z))
            way_index_text = str(active_way.mission_type.name)
        else:
            way_z_text = "NaN"
            way_index_text = "NaN"

        # get distance to next waypoint
        if active_way is not None and th.position.data_received():
            plane_obj = th.position.data["flight object"]
            d_xy_text = str(round(active_way.distance_2d_to(plane_obj)))
            d_z_text = str(round(active_way.z - plane_obj.z))

        else:
            d_xy_text = "NaN"
            d_z_text = "NaN"

        # draw next waypoint position and distance to
        draw_tele("Next waypoint:", 6.25, BLACK, under=True)
        draw_tele("way type: " + way_index_text, 6.75, BLACK)
        draw_tele("way z: " + way_z_text + " ft", 7.25, BLACK)
        draw_tele("d_xy: " + d_xy_text + " ft", 7.75, BLACK)
        draw_tele("d_z: " + d_z_text + " ft", 8.25, BLACK)

        # draw plane position history
        surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        if settings["button toggle trajectory"]["on"]:
            for index in range(len(self.position_history) - 1):
                pos_1 = dash_proj(self.position_history[index])
                pos_2 = dash_proj(self.position_history[index + 1])
                pygame.draw.line(surf, RED, pos_1, pos_2, width=2)

        # check if telemetry was received
        if th.position.data_received() and th.heading.data_received():

            # get data
            plane_obj = th.position.data["flight object"]
            angle = th.heading.data["heading"]

            # draw plane
            size = PLANE_SIZE / math.sqrt(zoom)
            d_f.draw_skew_triangle(surf, plane_obj,
                                   angle, size, PINK)

            # add position to position history
            self.position_history.append(plane_obj)

            # for testing
            if len(self.position_history) > 1:
                for index, way in enumerate(g_v.ms.waypoint_list):
                    dis = g_f.point_to_seg_distance(way.pos, plane_obj.pos, self.position_history[-2].pos)
                    self.minims[index] = min(self.minims[index], dis)
                print(self.minims)

        self.surface.blit(surf, (0, 0))

    @staticmethod
    def position_converter(telemetry_data):
        if telemetry_data is not None:
            text = str(round(telemetry_data["flight object"].z))
        else:
            text = "NaN"
        return ["AMSL z: " + text + " ft"]

    @staticmethod
    def velocity_converter(telemetry_data):
        if telemetry_data is not None:
            K_S = KNOTS_PER_FT_PER_S
            text = str(round(telemetry_data["flight object"].v * K_S))
        else:
            text = "NaN"
        return ["ground_s: " + text + " knots"]

    def armed_converter(self, telemetry_data):
        text = self.tele_get(telemetry_data, "armed")
        return ["armed: " + text]

    def in_air_converter(self, telemetry_data):
        text = self.tele_get(telemetry_data, "in air")
        return ["in air: " + text]

    def landed_converter(self, telemetry_data):
        text = self.tele_get(telemetry_data, "landed")
        return ["landed: " + text]

    def flight_converter(self, telemetry_data):
        text = self.tele_get(telemetry_data, "flight mode")
        return ["flight mode: " + text]

    def status_converter(self, telemetry_data):
        text_1 = self.tele_get(telemetry_data, "type")
        text_2 = self.tele_get(telemetry_data, "text")
        return ["msg type: " + text_1, text_2]

    def progress_converter(self, telemetry_data):
        text_1 = self.tele_get(telemetry_data, "current")
        text_2 = self.tele_get(telemetry_data, "total")
        return ["current: " + text_1, "total: " + text_2]

    @staticmethod
    def tele_get(telemetry_data, name):
        if telemetry_data is not None:
            return str(telemetry_data[name])
        else:
            return "NaN"
