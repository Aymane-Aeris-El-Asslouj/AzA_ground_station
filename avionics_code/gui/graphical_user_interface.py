from avionics_code.helpers import geometrical_functions as g_f, parameters as para
from avionics_code.helpers import algebraic_functions as a_f, global_variables as g_v
from avionics_code.helpers import geography_functions as gg_f
from avionics_code.gui import drawing_functions as d_f, gui_input_manager as g_i_m

import pygame
import threading
import os
import math
import time

DEFAULT_MAP_SIZE = para.DEFAULT_MAP_SIZE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
PATH_COLORING_CYCLE = para.PATH_COLORING_CYCLE
KNOTS_PER_FT_PER_S = para.KNOTS_PER_FT_PER_S
RF_TELEMETRY_ALLOWED_DELAY = para.RF_TELEMETRY_ALLOWED_DELAY
REGULAR_UPDATE_PER_SECOND = para.REGULAR_UPDATE_PER_SECOND
MESSAGE_DISPLAY_PERIOD = para.MESSAGE_DISPLAY_PERIOD
TAKEOFF_COORDINATES = para.TAKEOFF_COORDINATES
LANDING_COORDINATES = para.LANDING_COORDINATES
LANDING_LOITER_COORDINATES = para.LANDING_LOITER_COORDINATES


class GUI:
    """Handles the dashboard display"""

    def __init__(self):
        # dashboard variables that define what is displayed, selected, and inputted
        # (0:waypoints, 1:obstacles, 2:border vertices, etc)
        self.input_type = -1
        # whether there is a current input being done (probably obstacle or plane input)
        self.inputing = 0
        # position in which a current input is being done (probably obstacle or plane input)
        self.input_position = (0, 0)
        # altitude input and altitude typing box
        self.altitude_box = 200.0
        self.input_altitude = 0.0
        # selected map objects
        self.selection = [0]*5
        # displayed map objects
        self.is_displayed = [1]*12
        # parts of the mission state to show
        self.mission_state_display = [1]*10
        # message on screen
        self.message_1 = ""
        self.message_2 = ""
        self.message_color = 0, 0, 0
        self.message_time = 0
        # list of plane position
        self.position_history = list()
        # computation percentages
        self.path_search_percentage = 0
        self.cover_percentage = 0

        # load or create display settings
        if os.path.exists("extra files/settings.txt"):
            with open("extra files/settings.txt", "r") as file:
                display_setting = file.read()
                for i in range(len(self.mission_state_display)):
                    self.mission_state_display[i] = int(display_setting[i])
        else:
            with open("extra files/settings.txt", "w") as file:
                for i in range(len(self.mission_state_display)):
                    file.write(str(self.mission_state_display[i]))

        # set the map_scaling to fit it
        self.default_map_scaling = DASHBOARD_SIZE / DEFAULT_MAP_SIZE
        self.map_scaling = self.default_map_scaling
        D_S = DASHBOARD_SIZE
        self.screen_center = D_S/2, D_S/2
        self.moving_map = False

        # dashboard initialization
        pygame.init()
        pygame.display.set_caption('GUI')
        self.screen = pygame.display.set_mode((D_S * 2.0, D_S))
        self.background = pygame.image.load('extra files/background.jpg').convert()

        # drawing surfaces
        self.to_draw = {
            "user interface": False,
            "system status": False,
            "mission profile": False,
            "mission state": False,
            "telemetry": False,
            "path": False,
        }
        self.layer_background = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        self.layer_user_interface = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        self.layer_system_status = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        self.layer_mission_profile = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        self.layer_mission_state = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        self.layer_telemetry = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        self.layer_path = pygame.Surface((D_S, D_S), pygame.SRCALPHA)

        # initializes the selection menu
        pygame.font.init()
        self.font = None
        self.buttons = None
        self.input_type_dict = None
        self.texts = None
        self.mission_state_display_dict = None
        self.interface_init()

        self.close_request = False

    def activate(self):
        """Starts the interaction loop"""

        # gui input manager thread
        threading.Thread(target=g_i_m.gui_input_manager_loop, args=(self,)).start()
        self.to_draw["user interface"] = True
        self.to_draw["system status"] = True
        self.to_draw["telemetry"] = True

        # regular display update thread
        def display_full_update(g_u_i):
            while True:
                for key in g_u_i.to_draw:
                    g_u_i.to_draw[key] = True
                time.sleep(1/REGULAR_UPDATE_PER_SECOND)
                if g_u_i.close_request:
                    break
        threading.Thread(target=display_full_update, args=(self,)).start()

    def display_message(self, message_1, message_2, level):
        """display a message on screen with a certain priority level"""

        self.message_time = time.time()

        self.message_1 = message_1
        self.message_2 = message_2
        if level == 0:
            self.message_color = 0, 0, 0
        elif level == 1:
            self.message_color = 255, 0, 0

        self.to_draw["system status"] = True

    def display_update(self):
        """Updates the display"""

        # fill background
        self.screen.fill((205, 133, 63))
        D_S = DASHBOARD_SIZE
        self.layer_background = pygame.Surface((D_S, D_S), pygame.SRCALPHA)

        scaling_ratio = self.zoom()
        new_map_size = DASHBOARD_SIZE * scaling_ratio

        new_background = pygame.transform.scale(self.background, (new_map_size, new_map_size))
        corner_x = self.screen_center[0] - new_map_size/2
        corner_y = self.screen_center[1] - new_map_size/2

        self.layer_background.fill((0, 0, 0))
        self.layer_background.blit(new_background, (corner_x, corner_y))

        self.screen.blit(self.layer_background, (0, 0))
        self.screen.blit(self.layer_user_interface, (0, 0))
        self.screen.blit(self.layer_system_status, (0, 0))
        self.screen.blit(self.layer_mission_profile, (0, 0))
        self.screen.blit(self.layer_mission_state, (0, 0))
        self.screen.blit(self.layer_path, (0, 0))
        self.screen.blit(self.layer_telemetry, (0, 0))

    def draw_user_interface(self):
        """draws control menu"""

        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_user_interface = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        layer = self.layer_user_interface

        RED = (255, 0, 0)
        GREEN = (0, 255, 0)

        # change size of selected buttons
        for key in self.input_type_dict:
            if self.input_type == key:
                B = self.buttons[self.input_type_dict[key]]
                self.buttons[self.input_type_dict[key]] = (B[0], B[1], B[2], 3)
            else:
                B = self.buttons[self.input_type_dict[key]]
                self.buttons[self.input_type_dict[key]] = (B[0], B[1], B[2], 2)

        # change color of mission state buttons
        for key in self.mission_state_display_dict:
            if self.mission_state_display[key] == 1:
                B = self.buttons[self.mission_state_display_dict[key]]
                self.buttons[self.mission_state_display_dict[key]] = (GREEN, B[1], B[2], B[3])
            else:
                B = self.buttons[self.mission_state_display_dict[key]]
                self.buttons[self.mission_state_display_dict[key]] = (RED, B[1], B[2], B[3])

        # for displaying buttons
        for key in self.buttons:
            B = self.buttons[key]
            W_S = WAYPOINT_SIZE
            pygame.draw.circle(layer, B[0], (D_S * B[1], B[2] * D_S / 15), B[3] * W_S)

        # for displaying labels
        for key in self.texts:
            label = self.font.render(key, True, self.texts[key][0])
            center = (D_S * self.texts[key][1], self.texts[key][2] * D_S / 15)
            label_pos = label.get_rect(center=center)
            layer.blit(label, label_pos)

        # for displaying altitude input
        label = self.font.render("altitude: " + str(self.altitude_box) + "ft", True, (0, 0, 0))
        label_pos = label.get_rect(center=(D_S * 1.1, 14.5 * D_S / 15))
        pygame.draw.rect(layer, (0, 0, 0), label_pos, width=1)
        layer.blit(label, label_pos)
        label = self.font.render("altitude: " + str(self.altitude_box) + "ft", True, (0, 0, 0))
        label_pos = label.get_rect(center=(D_S * 1.3, 11.5 * D_S / 15))
        pygame.draw.rect(layer, (0, 0, 0), label_pos, width=1)
        layer.blit(label, label_pos)

        # draw line separating two side of the menu
        D_S = DASHBOARD_SIZE
        pygame.draw.line(layer, (0, 0, 0), (D_S*1.2, 0), (D_S*1.2, D_S))
        pygame.draw.line(layer, (0, 0, 0), (D_S*1.4, 0), (D_S*1.4, D_S))
        pygame.draw.line(layer, (0, 0, 0), (D_S*1.7, 0), (D_S*1.7, D_S))

    def draw_system_status(self):
        """draws system_status"""
        
        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_system_status = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        layer = self.layer_system_status

        def draw_status_enum(element, status_var, name, y):
            if element is not None:
                d_f.draw_centered_text(layer, self.font, name + " " + str(status_var.name), (1.85, y))

        def draw_status(element, status_var, name, state_list, y):
            if element is not None:
                for index, state in enumerate(state_list):
                    if status_var == index:
                        d_f.draw_centered_text(layer, self.font, name + " " + state, (1.85, y))

        draw_status_enum(g_v.sc, g_v.sc.connection_status, "Fake Server connection:", 0.5)
        draw_status_enum(g_v.mp, g_v.sc.retrieval_status, "Map download:", 1.0)
        draw_status_enum(g_v.ms, g_v.ms.generation_status, "Mission generation:", 1.5)

        states = ["disconnected", "connected"]
        draw_status(g_v.rf, g_v.rf.connection_status, "Pixhawk:", states, 2.0)

        draw_status_enum(g_v.rf, g_v.rf.parameters_status, "Parameters upload:", 2.5)
        draw_status_enum(g_v.rf, g_v.rf.geofence_status, "Geofence upload:", 3.0)
        draw_status_enum(g_v.rf, g_v.rf.download_status, "Mission download:", 3.5)
        draw_status_enum(g_v.mc, g_v.mc.path_computation_status, "Path computation:", 4.0)
        draw_status_enum(g_v.rf, g_v.rf.upload_status, "Mission upload:", 4.5)
        draw_status_enum(g_v.rf, g_v.rf.mission_start, "Mission start:", 5.0)
        draw_status_enum(g_v.rf, g_v.rf.mission_pause, "Mission pause:", 5.5)
        draw_status_enum(g_v.rf, g_v.rf.go_to_status, "Go orbit position:", 6.0)

        # draw message on screen
        M_D_P = MESSAGE_DISPLAY_PERIOD
        if time.time() - self.message_time < M_D_P or self.message_color == (255, 0, 0):
            color = self.message_color
        else:
            color = (60, 60, 60)

        d_f.draw_centered_text(layer, self.font, str(g_v.mc.action.name), (1.85, 12.5), (0, 0, 0))
        text_path = "path search: " + str(round(self.path_search_percentage)) + " %"
        d_f.draw_centered_text(layer, self.font, text_path, (1.85, 13.0), (0, 0, 0))
        text_path = "cover search: " + str(round(self.cover_percentage)) + " %"
        d_f.draw_centered_text(layer, self.font, text_path, (1.85, 13.5), (0, 0, 0))

        d_f.draw_centered_text(layer, self.font, self.message_1, (1.85, 14.0), color)
        d_f.draw_centered_text(layer, self.font, self.message_2, (1.85, 14.5), color)

    def draw_mission_profile(self):
        """Displays information of the mission profile
        border, obstacles, mission waypoints, etc"""

        # reset layer
        D_S = DASHBOARD_SIZE
        W_S = WAYPOINT_SIZE
        self.layer_mission_profile = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        layer = self.layer_mission_profile

        if g_v.mp is not None and g_v.sc.retrieval_status == g_v.StandardStatus.SUCCESS:
            # load info from Mission profile
            mission_waypoints = g_v.mp.mission_waypoints
            obstacles = g_v.mp.obstacles
            border = g_v.mp.border
            lost_comms_object = g_v.mp.lost_comms_object
            search_area = g_v.mp.search_area
            off_axis_object = g_v.mp.off_axis_object
            emergent_object = g_v.mp.emergent_object
            ugv_area = g_v.mp.ugv_area
            airdrop_object = g_v.mp.airdrop_object
            ugv_goal_object = g_v.mp.ugv_goal_object
            mapping_area = g_v.mp.mapping_area

            # object search area
            if self.is_displayed[3] == 1:
                surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
                surf.set_alpha(100)  # alpha level
                points = list(self.dashboard_projection(v) for v in search_area.vertices)
                if len(search_area.vertices) > 2:
                    pygame.draw.polygon(surf, (50, 50, 255), points, width=0)
                elif len(search_area.vertices) > 1:
                    pygame.draw.aalines(surf, (0, 0, 255), closed=False, points=points)

                layer.blit(surf, (0, 0))
                surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
                surf.set_alpha(150)  # alpha level
                points = search_area.vertices
                d_r_p = d_f.draw_rescaled_points
                d_r_p(self.zoom(), surf, points, self.selection[3], color=(50, 50, 255))
                layer.blit(surf, (0, 0))

            # airdrop boundary area
            surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
            surf.set_alpha(100)  # alpha level
            polygon = list(self.dashboard_projection(v) for v in ugv_area.vertices)
            pygame.draw.polygon(surf, (139, 0, 139), polygon, width=0)
            layer.blit(surf, (0, 0))

            # mapping area
            if self.is_displayed[9] == 1:
                surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
                surf.set_alpha(50)  # alpha level
                if len(mapping_area.vertices) > 2:
                    polygon = list(self.dashboard_projection(v) for v in mapping_area.vertices)
                    pygame.draw.polygon(surf, (0, 255, 0), polygon, width=0)
                layer.blit(surf, (0, 0))

            # draw the border (edges of border and border vertexes
            # with the ones selected being bigger)
            if self.is_displayed[2] == 1:
                if len(border.vertices) > 1:
                    points = list(self.dashboard_projection(v) for v in border.vertices)
                    pygame.draw.aalines(layer, (200, 0, 0), closed=True, points=points)
                points = border.vertices
                d_r_p = d_f.draw_rescaled_points
                d_r_p(self.zoom(), layer, points, self.selection[2], color=(200, 0, 0))

            # draw obstacles in yellow
            if self.is_displayed[1] == 1:
                for obstacle_i in obstacles:
                    obstacle_i_dash_xy = self.dashboard_projection(obstacle_i)
                    obstacle_i_dash_r = obstacle_i.r * self.map_scaling
                    pygame.draw.circle(layer, (200, 200, 0), obstacle_i_dash_xy, obstacle_i_dash_r)

                # draw the obstacle that is being inputted if there is one
                if self.inputing == 1 and self.input_type == 1:
                    inputing_radius = g_f.distance_2d(self.input_position, pygame.mouse.get_pos())
                    pygame.draw.circle(layer, (200, 200, 0), self.input_position, inputing_radius)

            # draw off-axis object,
            if self.is_displayed[7] == 1:
                off_obj_pos = self.dashboard_projection(off_axis_object)
                cir = pygame.draw.circle
                cir(layer, (0, 0, 0), off_obj_pos, W_S * 2.3 * self.zoom(), width=int(4 * self.zoom()))
                width = int(2 * self.zoom())
                cir(layer, (255, 255, 255), off_obj_pos, W_S * 2 * self.zoom(), width=width)
                cir(layer, (0, 0, 0), off_obj_pos, W_S * 1.8 * self.zoom(), width=int(2 * self.zoom()))

            # airdrop pos
            if self.is_displayed[4] == 1:
                airdrop_object_pos = self.dashboard_projection(airdrop_object)
                cir = pygame.draw.circle
                cir(layer, (0, 0, 0), airdrop_object_pos, W_S * 2.3 * self.zoom())
                width = int(2 * self.zoom())
                cir(layer, (255, 255, 255), airdrop_object_pos, W_S * 2 * self.zoom(), width=width)
                cir(layer, (255, 255, 255), airdrop_object_pos, W_S * 1 * self.zoom(), width=width)

            # emergent obj
            if self.is_displayed[8] == 1:
                emergent_object_pos = self.dashboard_projection(emergent_object)
                pygame.draw.circle(layer, (0, 0, 0), emergent_object_pos, W_S * 1.4 * self.zoom())
                pygame.draw.circle(layer, (100, 255, 100), emergent_object_pos, W_S * 1 * self.zoom())

            # lost_comms_object
            if self.is_displayed[6] == 1:
                lost_pos = self.dashboard_projection(lost_comms_object)
                side = W_S * 2.5 * self.zoom()
                lost_rect = pygame.Rect(lost_pos[0] - side / 2, lost_pos[1] - side / 2, side, side)
                pygame.draw.rect(layer, (0, 0, 0), lost_rect, width=int(2 * self.zoom()))
                side = W_S * 2 * self.zoom()
                lost_rect = pygame.Rect(lost_pos[0] - side / 2, lost_pos[1] - side / 2, side, side)
                pygame.draw.rect(layer, (255, 255, 255), lost_rect, width=int(2 * self.zoom()))
                side = W_S * 1.5 * self.zoom()
                lost_rect = pygame.Rect(lost_pos[0] - side / 2, lost_pos[1] - side / 2, side, side)
                pygame.draw.rect(layer, (0, 0, 0), lost_rect, width=int(2 * self.zoom()))

            # ugv goal obj
            if self.is_displayed[5] == 1:
                ugv_goal_object_pos = self.dashboard_projection(ugv_goal_object)
                pygame.draw.circle(layer, (0, 0, 0), ugv_goal_object_pos, W_S * 1.4 * self.zoom())
                pygame.draw.circle(layer, (139, 0, 139), ugv_goal_object_pos, W_S * 1 * self.zoom())

            # draw mission waypoints on top of Final path in black
            if self.is_displayed[0] == 1:
                BLUE = (0, 0, 255)
                d_r_p = d_f.draw_rescaled_points
                d_r_p(self.zoom(), layer, mission_waypoints, self.selection[0], BLUE, 1)

        # map markers
        d_f.draw_map_marker(self.zoom(), layer, TAKEOFF_COORDINATES, "T")
        d_f.draw_map_marker(self.zoom(), layer, LANDING_COORDINATES, "L")
        d_f.draw_map_marker(self.zoom(), layer, LANDING_LOITER_COORDINATES, "L")

        # draw landing radius
        loiter_cen = gg_f.geographic_to_cartesian_center_5(LANDING_LOITER_COORDINATES)
        dash_loiter_cen = self.dashboard_projection_pos(loiter_cen)
        l_rad = LANDING_LOITER_COORDINATES["radius"]
        rad = abs(l_rad) * self.map_scaling
        pygame.draw.circle(layer, (0, 0, 0), dash_loiter_cen, rad, width=int(self.zoom()))

        # draw orientation of landing loiter
        cs = math.copysign
        orientation_dash = g_f.add_vectors(dash_loiter_cen, (0, -rad))
        up_or = g_f.add_vectors(orientation_dash, (0, W_S * self.zoom()))
        down_or = g_f.add_vectors(orientation_dash, (0, -W_S * self.zoom()))
        edge_or = g_f.add_vectors(orientation_dash, (cs(W_S * self.zoom(), l_rad), 0))
        pygame.draw.polygon(layer, (0, 0, 0), [up_or, down_or, edge_or])
        orientation_dash = g_f.add_vectors(dash_loiter_cen, (0, rad))
        up_or = g_f.add_vectors(orientation_dash, (0, W_S * self.zoom()))
        down_or = g_f.add_vectors(orientation_dash, (0, -W_S * self.zoom()))
        edge_or = g_f.add_vectors(orientation_dash, (cs(W_S * self.zoom(), -l_rad), 0))
        pygame.draw.polygon(layer, (0, 0, 0), [up_or, down_or, edge_or])

    def draw_mission_state(self):
        """Displays information of the mission state: waypoint list"""

        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_mission_state = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        layer = self.layer_mission_state

        if self.mission_state_display[5] == 1:
            surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
            surf.set_alpha(150)  # alpha level

            # draw waypoint list in cyan
            for index, waypoint in enumerate(g_v.ms.waypoint_list):

                # stop drawing if a computation started
                if g_v.ms.generation_status == g_v.StandardStatus.STARTED:
                    break

                # draw active waypoints in cyan and those done in green
                if waypoint.is_mission == 1:
                    color = (0, 255, 255)
                else:
                    color = (0, 255, 0)

                if self.mission_state_display[8] == 1 or waypoint.is_mission == 1:
                    # find the mission index of the waypoint
                    mission_index = waypoint.mission_index
                    # draw the waypoint with an arrow coming from the previous
                    # waypoint if the mission index is to be displayed
                    if self.mission_state_display[mission_index] == 1:
                        w_s = WAYPOINT_SIZE * 0.25 * math.sqrt(self.zoom())
                        vertex_dash = self.dashboard_projection(waypoint)
                        pairs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
                        v_x, v_y = vertex_dash
                        points = [(v_x + w_s * pair[0], v_y + w_s * pair[1]) for pair in pairs]
                        pygame.draw.polygon(surf, color, points)

                        if index > 0:
                            pre_waypoint = g_v.ms.waypoint_list[index - 1]
                            delta_pos = g_f.sub_vectors(waypoint.pos, pre_waypoint.pos)
                            d_f.draw_arrow(surf, pre_waypoint, delta_pos, color)
                            if pre_waypoint.target is not None:
                                delta_pos = g_f.sub_vectors(pre_waypoint.target, pre_waypoint.pos)
                                d_f.draw_arrow(surf, pre_waypoint, delta_pos, (0, 0, 0))

            layer.blit(surf, (0, 0))

    def draw_telemetry(self):
        """Displays information of the flight profile
        ugv/plane position, time, etc"""

        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_telemetry = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        layer = self.layer_telemetry

        # shortcut for drawing on telemetry column
        def draw_tele(text, y_coordinate, color):
            d_f.draw_centered_text(layer, self.font, text, (1.55, y_coordinate), color)

        # shortcut for drawing on system column
        def draw_tele_2(text, y_coordinate, color):
            d_f.draw_centered_text(layer, self.font, text, (1.85, y_coordinate), color)

        GREY = (60, 60, 60)
        BLACK = (0, 0, 0)

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
                draw_tele(str(c_d[i]) + ", " + telemetry_rate + " Hz", y + 0.5*i, color)

        def position_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                plane_obj_data = telemetry_data["flight object"]
                c_d.append("plane x: " + str(round(plane_obj_data.pos[0])) + " ft")
                c_d.append("plane y: " + str(round(plane_obj_data.pos[1])) + " ft")
                c_d.append("AMSL z: " + str(round(plane_obj_data.z)) + " ft")
            else:
                c_d.append("plane x: " + "NaN" + " ft")
                c_d.append("plane y: " + "NaN" + " ft")
                c_d.append("AMSL z: " + "NaN" + " ft")
            return c_d

        def velocity_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                plane_obj_2_data_tele = telemetry_data["flight object"]
                c_d.append("north_s: " + str(round(plane_obj_2_data_tele.vel[0])) + " ft/s")
                c_d.append("east_s: " + str(round(plane_obj_2_data_tele.vel[1])) + " ft/s")
                c_d.append("down_s: " + str(round(plane_obj_2_data_tele.v_z)) + " ft/s")
                c_d.append("ground_s: " + str(round(plane_obj_2_data_tele.v)) + " ft/s")
                K_S = KNOTS_PER_FT_PER_S
                c_d.append("ground_s: " + str(round(plane_obj_2_data_tele.v * K_S)) + " knots")
                ang = plane_obj_2_data_tele.v_ang
                if ang is not None:
                    c_d.append("g_s_ang: " + str(round(math.degrees(ang))) + " deg")
                else:
                    c_d.append("g_s_ang: " + "undef" + " deg")

            else:
                c_d.append("north_s: " + "NaN" + " ft/s")
                c_d.append("east_s: " + "NaN" + " ft/s")
                c_d.append("down_s: " + "NaN" + " ft/s")
                c_d.append("ground_s: " + "NaN" + " ft/s")
                c_d.append("ground_s: " + "NaN" + " knots")
                c_d.append("g_s_ang: " + "NaN" + " deg")
            return c_d

        def heading_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                heading_angle = math.degrees(telemetry_data["heading"])
                c_d.append("heading: " + str(round(heading_angle)) + " deg")
            else:
                c_d.append("heading: " + "NaN" + " deg")
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
        draw_tele("Telemetry", 0.5, (0, 0, 0))
        draw_telemetry(g_v.th.position, position_converter, 1.5)
        draw_telemetry(g_v.th.velocity, velocity_converter, 3.5)
        draw_telemetry(g_v.th.heading, heading_converter, 7.0)
        draw_telemetry(g_v.th.armed, armed_converter, 10.0)
        draw_telemetry(g_v.th.in_air, in_air_converter, 10.5)
        draw_telemetry(g_v.th.landed, landed_converter, 11.0)
        draw_telemetry(g_v.th.flight_mode, flight_converter, 11.5)
        draw_telemetry(g_v.th.status_text, status_converter, 12.0)
        draw_telemetry(g_v.th.mission_progress, mission_progress_converter, 14.0)

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
        if self.mission_state_display[9] == 1:
            for index in range(len(self.position_history)-1):
                pos_1 = self.dashboard_projection(self.position_history[index])
                pos_2 = self.dashboard_projection(self.position_history[index+1])
                pygame.draw.line(screen_surf, (255, 0, 0), pos_1, pos_2, width=2)

        # check if telemetry was received
        if (g_v.th.position.data is not None) and (g_v.th.heading.data is not None):
            plane_obj = g_v.th.position.data["flight object"]
            angle = g_v.th.heading.data["heading"]
            # add position to position history
            d_f.draw_triangle(self.zoom(), screen_surf, plane_obj, angle, 100, (255, 20, 147))
            if g_v.th.velocity.data is not None:
                plane_obj_2_data = g_v.th.velocity.data["flight object"]
                if plane_obj_2_data.v != 0:
                    self.position_history.append(plane_obj)
        layer.blit(screen_surf, (0, 0))

    def draw_path(self):
        """draw current path to be exported or already
        exported to the plane"""

        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_path = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
        layer = self.layer_path

        # check if the path we got to is active
        active_reached = 0

        if self.mission_state_display[6] == 1:
            chosen_path = g_v.mc.chosen_path
            if chosen_path is not None:
                # run over all waypoints of the curved path
                rotator = a_f.RGBRotate()
                pre_way = None
                for index, way in enumerate(chosen_path.waypoint_list):

                    # stop drawing if a computation started
                    if g_v.mc.path_computation_status == g_v.StandardStatus.STARTED:
                        break

                    if way.is_mission == 1 and active_reached == 0:
                        active_reached = 1

                    if self.mission_state_display[8] == 1 or active_reached == 1:
                        if self.mission_state_display[way.mission_index] == 1:
                            # draw the selected path from the path path_group
                            if pre_way is not None:
                                color_factor = index/PATH_COLORING_CYCLE
                                rotator.set_hue_rotation(color_factor * 360)
                                color = rotator.apply(255, 0, 0)
                                show_turns = bool(self.mission_state_display[7])

                                # Get previous vertex or its off shoot waypoint if it exists
                                # Get next vertex
                                vertex_1 = self.dashboard_projection(pre_way)
                                vertex_2 = g_v.gui.dashboard_projection(way)
                                if active_reached == 1:
                                    way_color = (255, 255, 0)
                                else:
                                    way_color = (0, 255, 0)
                                W_S = WAYPOINT_SIZE * 0.75 * math.sqrt(self.zoom())
                                pygame.draw.circle(layer, way_color, vertex_2, W_S)

                                # turn waypoint after vertex 1
                                if pre_way.post_turn_waypoint is not None and show_turns:
                                    after_turn = self.dashboard_projection(pre_way.post_turn_waypoint)
                                    pygame.draw.circle(layer, (255, 105, 180), after_turn, W_S)
                                    pygame.draw.line(layer, (255, 105, 180), vertex_1, after_turn)
                                    pre_way = pre_way.post_turn_waypoint

                                # turn waypoint before vertex 2
                                if way.pre_turn_waypoint is not None and show_turns:
                                    before_turn = self.dashboard_projection(way.pre_turn_waypoint)
                                    pygame.draw.circle(layer, (0, 0, 0), before_turn, W_S)
                                    pygame.draw.line(layer, (255, 105, 180), before_turn, vertex_2)
                                    way = way.pre_turn_waypoint

                                if active_reached == 1:
                                    color = color
                                else:
                                    color = (0, 255, 0)

                                map_vector = g_f.sub_vectors(way.pos, pre_way.pos)
                                d_f.draw_arrow(layer, pre_way, map_vector, color)

                    pre_way = chosen_path.waypoint_list[index]

    def interface_init(self):
        """Defines dictionaries for the interface"""

        WHITE = (255, 255, 255)
        BLACK = (0, 0, 0)
        BLUE = (0, 0, 255)
        RED = (255, 0, 0)
        DARK_GREEN = (0, 140, 0)
        YELLOW = (255, 255, 0)
        LOW_RED = (200, 0, 0)
        PURPLE = (148, 0, 211)

        self.font = pygame.font.SysFont('Arial', int(14 * (DASHBOARD_SIZE / 650)))

        # default dictionary of control menu buttons
        self.buttons = {
            "Waypoint button": (BLUE, 1.1, 1.0, 2),
            "Obstacle button": (YELLOW, 1.1, 2.0, 2),
            "Border button": (LOW_RED, 1.1, 3.0, 2),
            "Search area button": (BLUE, 1.1, 4.0, 2),
            "Airdrop button": (PURPLE, 1.1, 5.0, 2),
            "Airdrop goal button": (PURPLE, 1.1, 6.0, 2),
            "lost comms button": (WHITE, 1.1, 7.0, 2),
            "Off axis obj button": (WHITE, 1.1, 8.0, 2),
            "Emergent obj button": (DARK_GREEN, 1.1, 9.0, 2),
            "Mapping area button": (DARK_GREEN, 1.1, 10.0, 2),
            "waypoint mission button": (RED, 1.22, 1.0, 2),
            "airdrop mission button": (RED, 1.22, 1.5, 2),
            "scouting mission button": (RED, 1.22, 2.0, 2),
            "off axis scout mission button": (RED, 1.22, 2.5, 2),
            "landing loiter mission button": (RED, 1.22, 3.0, 2),
            "mission state button": (RED, 1.22, 4.0, 2),
            "computed path button": (RED, 1.22, 4.5, 2),
            "turn waypoints button": (RED, 1.22, 5.0, 2),
            "crossed waypoints button": (RED, 1.22, 6.0, 2),
            "trajectory button": (RED, 1.22, 6.5, 2),
            "Generate button": (BLACK, 1.1, 12.0, 2),
            "Compute button": (BLACK, 1.1, 13.0, 2),
            "Upload mission button": (BLACK, 1.1, 14.0, 2),
            "Start mission button": (BLACK, 1.3, 8.0, 2),
            "Pause mission button": (BLACK, 1.3, 9.0, 2),
            "land button": (BLACK, 1.3, 10.0, 2),
            "go to position button": (BLACK, 1.3, 11.0, 2),
            "drop authorization button": (BLACK, 1.3, 14.0, 2)
        }

        # input type of each button
        self.input_type_dict = {
            0: "Waypoint button",
            1: "Obstacle button",
            2: "Border button",
            3: "Search area button",
            4: "Airdrop button",
            5: "Airdrop goal button",
            6: "lost comms button",
            7: "Off axis obj button",
            8: "Emergent obj button",
            9: "Mapping area button",
            10: "go to position button"
        }

        # status type of each button
        self.mission_state_display_dict = {
            0: "waypoint mission button",
            1: "airdrop mission button",
            2: "scouting mission button",
            3: "off axis scout mission button",
            4: "landing loiter mission button",
            5: "mission state button",
            6: "computed path button",
            7: "turn waypoints button",
            8: "crossed waypoints button",
            9: "trajectory button"
        }

        # Text dictionary
        self.texts = {
            'Waypoints': (BLUE, 1.1, 0.5),
            'Obstacles': (YELLOW, 1.1, 1.5),
            'Border': (LOW_RED, 1.1, 2.5),
            'Search area': (BLUE, 1.1, 3.5),
            'Airdrop': (PURPLE, 1.1, 4.5),
            'Airdrop Goal': (PURPLE, 1.1, 5.5),
            'Lost comms': (WHITE, 1.1, 6.5),
            'Off axis object': (WHITE, 1.1, 7.5),
            'Emergent object': (DARK_GREEN, 1.1, 8.5),
            'Mapping area': (DARK_GREEN, 1.1, 9.5),
            'path display:': (WHITE, 1.3, 0.5),
            'waypoints': (WHITE, 1.31, 1.0),
            'airdrop': (WHITE, 1.31, 1.5),
            'scouting': (WHITE, 1.31, 2.0),
            'off axis scout': (WHITE, 1.31, 2.5),
            'landing loiter': (WHITE, 1.31, 3.0),
            'mission state': (WHITE, 1.31, 4.0),
            'computed path': (WHITE, 1.31, 4.5),
            'turn waypoints': (WHITE, 1.31, 5.0),
            'crossed points': (WHITE, 1.31, 6.0),
            'trajectory': (WHITE, 1.31, 6.5),
            'Generate mission': (BLACK, 1.1, 11.5),
            'Compute path': (BLACK, 1.1, 12.5),
            'Upload mission': (BLACK, 1.1, 13.5),
            'Start mission': (BLACK, 1.3, 7.5),
            'Pause mission': (BLACK, 1.3, 8.5),
            'End mission': (BLACK, 1.3, 9.5),
            'orbit position': (BLACK, 1.3, 10.5),
            'authorize airdrop': (BLACK, 1.3, 13.5),
        }

    def dashboard_projection(self, map_object):
        """project map object's position on the dashboard screen
        centered in the middle with proper scale"""

        return self.dashboard_projection_pos(map_object.pos)

    def dashboard_projection_pos(self, map_object):
        """project map position on the dashboard screen
        centered in the middle with proper scale"""

        new_x = self.screen_center[0] + map_object[0] * self.map_scaling
        new_y = self.screen_center[1] - map_object[1] * self.map_scaling
        return new_x, new_y

    def map_projection(self, dash_board_position):
        """project dashboard position on the map with proper scale"""
        inv_x = (dash_board_position[0] - self.screen_center[0]) / self.map_scaling
        inv_y = (self.screen_center[1] - dash_board_position[1]) / self.map_scaling
        return inv_x, inv_y

    def zoom(self):
        """returns screen zoom ratio"""

        return self.map_scaling / self.default_map_scaling
