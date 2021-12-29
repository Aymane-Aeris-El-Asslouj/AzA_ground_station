from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.pygui import layer as g_l
from avionics_code.pygui import colors as col

from .. import drawing_functions as d_f

import pygame
import time
import math

KNOTS_PER_FT_PER_S = para.KNOTS_PER_FT_PER_S
RF_TELEMETRY_ALLOWED_DELAY = para.RF_TELEMETRY_ALLOWED_DELAY
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE
RED = col.RED
PINK = col.PINK
GREY = col.GREY
BLACK = col.BLACK


class TelemetryLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S * 2, D_S))

        # list of plane position
        self.position_history = list()

    def redraw(self):
        """Displays information of the flight profile
        ugv/plane position, time, etc"""

        # shortcuts
        settings = self.g_u_i.settings
        background = self.g_u_i.layers["background"]
        zoom = background.zoom()
        dash_proj = background.dashboard_projection

        # shortcut for drawing on telemetry column
        def draw_tele(text, y_coordinate, color):
            d_f.draw_centered_text(self.surface, self.font, text, (1.55, y_coordinate), color)

        # shortcut for drawing on system column
        def draw_tele_2(text, y_coordinate, color):
            d_f.draw_centered_text(self.surface, self.font, text, (1.85, y_coordinate), color)

        # function for drawing telemetry data on telemetry column
        def draw_telemetry(telemetry_object, converter, y):
            telemetry_data = telemetry_object.data
            c_d = converter(telemetry_data)
            if telemetry_data is not None:
                telemetry_rate = str(round(telemetry_object.telemetry_rate))
                # check if data is not outdated
                if (time.time() - telemetry_object.telemetry_time) < RF_TELEMETRY_ALLOWED_DELAY:
                    color = BLACK
                else:
                    color = GREY
            else:
                telemetry_rate = "Nan"
                color = GREY

            # draw all converted telemetry
            for i in range(len(c_d)):
                draw_tele(str(c_d[i]) + ", " + telemetry_rate + " Hz", y + 0.5 * i, color)

        def position_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                plane_obj_data = telemetry_data["flight object"]
                c_d.append("AMSL z: " + str(round(plane_obj_data.z)) + " ft")
            else:
                c_d.append("AMSL z: " + "NaN" + " ft")
            return c_d

        def velocity_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                plane_obj_2_data_tele = telemetry_data["flight object"]
                K_S = KNOTS_PER_FT_PER_S
                c_d.append("ground_s: " + str(round(plane_obj_2_data_tele.v * K_S)) + " knots")

            else:
                c_d.append("ground_s: " + "NaN" + " knots")
            return c_d

        def armed_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("armed: " + str(telemetry_data["armed"]))
            else:
                c_d.append("armed: " + "NaN")
            return c_d

        def in_air_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("in air: " + str(telemetry_data["in air"]))
            else:
                c_d.append("in air: " + "NaN")
            return c_d

        def landed_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("landed: " + str(telemetry_data["landed"]))
            else:
                c_d.append("landed: " + "NaN")
            return c_d

        def flight_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("flight mode: " + str(telemetry_data["flight mode"]))
            else:
                c_d.append("flight mode: " + "NaN")
            return c_d

        def status_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("msg type: " + str(telemetry_data["type"]))
                c_d.append(str(telemetry_data["text"]))
            else:
                c_d.append("msg type: " + "NaN")
                c_d.append("NaN")
            return c_d

        def mission_progress_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("current: " + str(telemetry_data["current"]))
                c_d.append("total: " + str(telemetry_data["total"]))
            else:
                c_d.append("current: " + "NaN")
                c_d.append("total: " + "NaN")
            return c_d

        # get next waypoint position
        active_waypoint = g_v.ms.active_waypoint()
        if active_waypoint is not None:
            way_x_text = str(round(active_waypoint.pos[0]))
            way_y_text = str(round(active_waypoint.pos[1]))
            way_z_text = str(round(active_waypoint.z))
            way_index_text = str(active_waypoint.mission_index)
        else:
            way_x_text = "NaN"
            way_y_text = "NaN"
            way_z_text = "NaN"
            way_index_text = "NaN"

        # get distance to next waypoint
        if active_waypoint is not None and g_v.th.position.data is not None:
            plane_obj = g_v.th.position.data["flight object"]
            d_xy_text = str(round(active_waypoint.distance_2d_to(plane_obj)))
            d_z_text = str(round(active_waypoint.z - plane_obj.z))
        else:
            d_xy_text = "NaN"
            d_z_text = "NaN"

        # display telemetry on telemetry column
        draw_tele("Telemetry", 0.5, BLACK)
        draw_telemetry(g_v.th.position, position_converter, 1.0)
        draw_telemetry(g_v.th.velocity, velocity_converter, 1.5)
        draw_telemetry(g_v.th.armed, armed_converter, 2.0)
        draw_telemetry(g_v.th.in_air, in_air_converter, 2.5)
        draw_telemetry(g_v.th.landed, landed_converter, 3.0)
        draw_telemetry(g_v.th.flight_mode, flight_converter, 3.5)
        draw_telemetry(g_v.th.status_text, status_converter, 4.0)
        draw_telemetry(g_v.th.mission_progress, mission_progress_converter, 5.0)

        # draw next waypoint position
        draw_tele_2("way x: " + way_x_text + " ft", 8.0, BLACK)
        draw_tele_2("way y: " + way_y_text + " ft", 8.5, BLACK)
        draw_tele_2("way z: " + way_z_text + " ft", 9.0, BLACK)
        draw_tele_2("way index: " + way_index_text, 9.5, BLACK)

        # draw distance to next waypoint
        draw_tele_2("d_xy: " + d_xy_text + " ft", 10.0, BLACK)
        draw_tele_2("d_z: " + d_z_text + " ft", 10.5, BLACK)

        # draw map telemetry
        screen_surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)

        # draw plane position history
        if settings["button toggle trajectory"]["on"]:
            for index in range(len(self.position_history) - 1):
                pos_1 = dash_proj(self.position_history[index])
                pos_2 = dash_proj(self.position_history[index + 1])
                pygame.draw.line(screen_surf, RED, pos_1, pos_2, width=2)

        # check if telemetry was received
        if (g_v.th.position.data is not None) and (g_v.th.heading.data is not None):
            plane_obj = g_v.th.position.data["flight object"]
            angle = g_v.th.heading.data["heading"]
            # add position to position history
            size = 100 / math.sqrt(zoom)
            d_f.draw_skew_triangle(screen_surf, plane_obj, angle, size, PINK)
            if g_v.th.velocity.data is not None:
                plane_obj_2_data = g_v.th.velocity.data["flight object"]
                if plane_obj_2_data.v != 0:
                    self.position_history.append(plane_obj)
        self.surface.blit(screen_surf, (0, 0))
