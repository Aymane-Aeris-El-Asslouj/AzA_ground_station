import pygame
import threading
import os
import math
import time

from avionics_code.helpers import geometrical_functions as g_f, parameters as para
from avionics_code.helpers import algebraic_functions as a_f, global_variables as g_v
from avionics_code.gui import drawing_functions as d_f, gui_input_manager as g_i_m


DEFAULT_MAP_SIZE = para.DEFAULT_MAP_SIZE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
PATH_COLORING_CYCLE = para.PATH_COLORING_CYCLE
KNOTS_PER_FT_PER_S = para.KNOTS_PER_FT_PER_S
RF_TELEMETRY_ALLOWED_DELAY = para.RF_TELEMETRY_ALLOWED_DELAY
REGULAR_UPDATE_PER_SECOND = para.REGULAR_UPDATE_PER_SECOND
MESSAGE_DISPLAY_PERIOD = para.MESSAGE_DISPLAY_PERIOD


class GUI:
    """Handles the dashboard display"""

    def __init__(self):
        # dashboard variables that define what is displayed, selected, and inputted
        # (0:waypoints, 1:obstacles, 2:border vertices, etc)
        self.input_type = 0
        # whether there is a current input being done (probably obstacle or plane input)
        self.inputing = 0
        # position in which a current input is being done (probably obstacle or plane input)
        self.input_position = (0, 0)
        # altitude input and altitude typing box
        self.altitude_box = 0.0
        self.input_altitude = 0.0
        # selected map objects
        self.selection = [0]*5
        # displayed map objects
        self.is_displayed = [1]*12
        # parts of the mission state to show
        self.mission_state_display = [1]*8
        # message on screen
        self.message_1 = ""
        self.message_2 = ""
        self.message_color = 0, 0, 0
        self.message_time = 0

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
        self.map_scaling = DASHBOARD_SIZE / DEFAULT_MAP_SIZE
        D_S = DASHBOARD_SIZE

        # dashboard initialization
        pygame.init()
        pygame.display.set_caption('GUI')
        self.screen = pygame.display.set_mode((D_S * 2.0, D_S))
        self.background = pygame.image.load('extra files/background.png').convert()
        self.background = pygame.transform.scale(self.background, (D_S, D_S))

        # drawing surfaces
        self.to_draw = {
            "user interface": False,
            "system status": False,
            "mission profile": False,
            "mission state": False,
            "telemetry": False,
            "path": False,
        }
        self.layer_user_interface = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        self.layer_system_status = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        self.layer_mission_profile = pygame.Surface((D_S * 1.2, D_S), pygame.SRCALPHA)
        self.layer_mission_state = pygame.Surface((D_S * 1.2, D_S), pygame.SRCALPHA)
        self.layer_telemetry = pygame.Surface((D_S * 2.0, D_S), pygame.SRCALPHA)
        self.layer_path = pygame.Surface((D_S * 1.2, D_S), pygame.SRCALPHA)

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
        self.screen.blit(self.background, (0, 0))

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
        label_pos = label.get_rect(center=(D_S * 1.1, 14 * D_S / 15))
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

        def draw_status(element, status_var, name, state_list, y):
            if element is not None:
                for index, state in enumerate(state_list):
                    if status_var == index:
                        d_f.draw_centered_text(layer, self.font, name + " " + state, (1.85, y))

        states = ["disconnected", "connecting...", "connected"]
        draw_status(g_v.sc, g_v.sc.connection_status, "Fake Server:", states, 0.5)
        states = ["not retrieved", "retrieving...", "retrieved"]
        draw_status(g_v.mp, g_v.mp.retrieval_status, "Mission profile:", states, 1.0)
        states = ["not generated", "generating...", "generated"]
        draw_status(g_v.ms, g_v.ms.generation_status, "Mission state:", states, 1.5)
        states = ["disconnected", "connecting...", "connected"]
        draw_status(g_v.rf, g_v.rf.connection_status, "Pixhawk:", states, 2.0)
        states = ["not set", "setting...", "set"]
        draw_status(g_v.rf, g_v.rf.parameters_status, "Parameters:", states, 2.5)
        states = ["not uploaded", "uploading...", "uploaded"]
        draw_status(g_v.rf, g_v.rf.geofence_status, "Geofence:", states, 3.0)
        states = ["not computed", "computing...", "computed"]
        draw_status(g_v.mc, g_v.mc.path_computation_status, "Path:", states, 3.5)
        states = ["not uploaded", "uploading...", "uploaded"]
        draw_status(g_v.rf, g_v.rf.mission_upload_status, "Mission:", states, 4.0)
        states = ["not started", "arming...", "starting...", "started", "pausing...", "paused"]
        draw_status(g_v.rf, g_v.rf.mission_status, "Mission:", states, 4.5)

        # draw message on screen
        if time.time() - self.message_time < MESSAGE_DISPLAY_PERIOD:
            color = self.message_color
        else:
            color = (60, 60, 60)
        d_f.draw_centered_text(layer, self.font, self.message_1, (1.85, 14.0), color)
        d_f.draw_centered_text(layer, self.font, self.message_2, (1.85, 14.5), color)

    def draw_mission_profile(self):
        """Displays information of the mission profile
        border, obstacles, mission waypoints, etc"""

        # reset layer
        D_S = DASHBOARD_SIZE
        W_S = WAYPOINT_SIZE
        self.layer_mission_profile = pygame.Surface((D_S * 1.2, D_S), pygame.SRCALPHA)
        layer = self.layer_mission_profile

        if g_v.mp is None:
            return

        if g_v.mp.retrieval_status != 2:
            return

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
            d_f.draw_rescaled_points(surf, points, self.selection[3], color=(50, 50, 255))
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
            d_f.draw_rescaled_points(layer, points, self.selection[2], color=(200, 0, 0))

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
            pygame.draw.circle(layer, (0, 0, 0), off_obj_pos, W_S * 2.3, width=4)
            pygame.draw.circle(layer, (255, 255, 255), off_obj_pos, W_S * 2, width=2)
            pygame.draw.circle(layer, (0, 0, 0), off_obj_pos, W_S * 1.8, width=2)

        # airdrop pos
        if self.is_displayed[4] == 1:
            airdrop_object_pos = self.dashboard_projection(airdrop_object)
            pygame.draw.circle(layer, (0, 0, 0), airdrop_object_pos, W_S * 2.3)
            pygame.draw.circle(layer, (255, 255, 255), airdrop_object_pos, W_S * 2, width=2)
            pygame.draw.circle(layer, (255, 255, 255), airdrop_object_pos, W_S * 1, width=2)

        # emergent obj
        if self.is_displayed[8] == 1:
            emergent_object_pos = self.dashboard_projection(emergent_object)
            pygame.draw.circle(layer, (0, 0, 0), emergent_object_pos, W_S * 1.4)
            pygame.draw.circle(layer, (100, 255, 100), emergent_object_pos, W_S * 1)

        # lost_comms_object
        if self.is_displayed[6] == 1:
            lost_pos = self.dashboard_projection(lost_comms_object)
            side = W_S * 2.5
            lost_rect = pygame.Rect(lost_pos[0] - side / 2, lost_pos[1] - side / 2, side, side)
            pygame.draw.rect(layer, (0, 0, 0), lost_rect, width=2)
            side = W_S * 2
            lost_rect = pygame.Rect(lost_pos[0] - side / 2, lost_pos[1] - side / 2, side, side)
            pygame.draw.rect(layer, (255, 255, 255), lost_rect, width=2)
            side = W_S * 1.5
            lost_rect = pygame.Rect(lost_pos[0] - side / 2, lost_pos[1] - side / 2, side, side)
            pygame.draw.rect(layer, (0, 0, 0), lost_rect, width=2)

        # ugv goal obj
        if self.is_displayed[5] == 1:
            ugv_goal_object_pos = self.dashboard_projection(ugv_goal_object)
            pygame.draw.circle(layer, (0, 0, 0), ugv_goal_object_pos, W_S * 1.4)
            pygame.draw.circle(layer, (139, 0, 139), ugv_goal_object_pos, W_S * 1)

        # draw mission waypoints on top of Final path in black
        if self.is_displayed[0] == 1:
            BLUE = (0, 0, 255)
            d_f.draw_rescaled_points(layer, mission_waypoints, self.selection[0], BLUE, 1)

    def draw_mission_state(self):
        """Displays information of the mission state: waypoint list"""

        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_mission_state = pygame.Surface((D_S * 1.2, D_S), pygame.SRCALPHA)
        layer = self.layer_mission_state

        if self.mission_state_display[6] == 1:
            surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
            surf.set_alpha(150)  # alpha level

            # draw waypoint list in cyan
            for index, waypoint in enumerate(g_v.ms.waypoint_list):
                # find the mission index of the waypoint
                mission_index = waypoint.mission_index
                # draw the waypoint with an arrow coming from the previous
                # waypoint if the mission index is to be displayed
                if self.mission_state_display[mission_index] == 1:
                    w_s = WAYPOINT_SIZE * 0.5
                    vertex_dash = self.dashboard_projection(waypoint)
                    pairs = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
                    v_x, v_y = vertex_dash
                    points = [(v_x + w_s * pair[0], v_y + w_s * pair[1]) for pair in pairs]
                    pygame.draw.polygon(surf, (0, 255, 255), points)

                    if index > 0:
                        pre_waypoint = g_v.ms.waypoint_list[index - 1]
                        delta_pos = g_f.sub_vectors(waypoint.pos, pre_waypoint.pos)
                        d_f.draw_arrow(surf, pre_waypoint, delta_pos, (0, 255, 255))
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

        # round to 3 significant digits
        def round_3(x):
            return x if x == 0 else round(x, -int(math.floor(math.log10(abs(x)))) + (3 - 1))

        def connection_converter(telemetry_data):
            c_d = list()
            if telemetry_data is not None:
                c_d.append("Telemetry con: " + str(telemetry_data["connection state"]))
            else:
                c_d.append("Telemetry con: " + "NaN")
            return c_d

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
                plane_obj_2_data = telemetry_data["flight object"]
                c_d.append("north_s: " + str(round(plane_obj_2_data.vel[0])) + " ft/s")
                c_d.append("east_s: " + str(round(plane_obj_2_data.vel[1])) + " ft/s")
                c_d.append("down_s: " + str(round(plane_obj_2_data.v_z)) + " ft/s")
                c_d.append("ground_s: " + str(round(plane_obj_2_data.v)) + " ft/s")
                K_S = KNOTS_PER_FT_PER_S
                c_d.append("ground_s: " + str(round(plane_obj_2_data.v * K_S)) + " knots")
                ang = plane_obj_2_data.v_ang
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
                c_d.append("heading: " + str(round(telemetry_data["heading"])) + " deg")
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
        if len(g_v.ms.waypoint_list) != 0:
            next_waypoint = g_v.ms.waypoint_list[0]
            way_x_text = str(round(next_waypoint.pos[0]))
            way_y_text = str(round(next_waypoint.pos[1]))
            way_z_text = str(round(next_waypoint.z))
        else:
            way_x_text = "NaN"
            way_y_text = "NaN"
            way_z_text = "NaN"

        # get distance to next waypoint
        if len(g_v.ms.waypoint_list) != 0 and g_v.th.position.data is not None:
            plane_obj = g_v.th.position.data["flight object"]
            d_xy_text = str(round(next_waypoint.distance_2d_to(plane_obj)))
            d_z_text = str(round(next_waypoint.z - plane_obj.z))
        else:
            d_xy_text = "NaN"
            d_z_text = "NaN"

        # display telemetry on telemetry column
        draw_telemetry(g_v.th.connection, connection_converter, 0.5)
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
        draw_tele_2("way x: " + way_x_text + " ft", 6.0, BLACK)
        draw_tele_2("way y: " + way_y_text + " ft", 6.5, BLACK)
        draw_tele_2("way z: " + way_z_text + " ft", 7.0, BLACK)

        # draw distance to next waypoint
        draw_tele_2("d_xy: " + d_xy_text + " ft", 8.0, BLACK)
        draw_tele_2("d_z: " + d_z_text + " ft", 8.5, BLACK)

        # draw plane
        if (g_v.th.position.data is not None) and (g_v.th.heading.data is not None):
            plane_obj = g_v.th.position.data["flight object"]
            screen_plane_angle = -math.radians(g_v.th.heading.data["heading"])
            d_f.draw_triangle(layer, plane_obj, screen_plane_angle, 100, (255, 20, 147))

        #ACRO = 12
        #ALTCTL = 10
        #FOLLOW_ME = 8
        #HOLD = 3
        #LAND = 6
        #MANUAL = 9
        #MISSION = 4
        #OFFBOARD = 7
        #POSCTL = 11
        #RATTITUDE = 14
        #READY = 1
        #RETURN_TO_LAUNCH = 5
        #STABILIZED = 13
        #TAKEOFF = 2
        #NaN = 0

        #ALERT = 6
        #CRITICAL = 5
        #DEBUG = 0
        #EMERGENCY = 7
        #ERROR = 4
        #INFO = 1
        #NOTICE = 2
        #WARNING = 3

    def draw_path(self):
        """draw current path to be exported or already
        exported to the plane"""

        # reset layer
        D_S = DASHBOARD_SIZE
        self.layer_path = pygame.Surface((D_S * 1.2, D_S), pygame.SRCALPHA)
        layer = self.layer_path

        if self.mission_state_display[7] == 1:
            chosen_path = g_v.mc.chosen_path
            if chosen_path is not None:
                # run over all waypoints of the curved path
                rotator = a_f.RGBRotate()
                pre_way = None
                for index, way in enumerate(chosen_path.waypoint_list):
                    if self.mission_state_display[way.mission_index] == 1:
                        # draw the selected path from the path path_group
                        if pre_way is not None:
                            color_factor = index/PATH_COLORING_CYCLE
                            rotator.set_hue_rotation(color_factor * 360)
                            color = rotator.apply(255, 0, 0)
                            d_f.draw_curved_edge(layer, pre_way, way, color)
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
            "Reload button": (BLACK, 1.06, 11.0, 2),
            "Clear button": (BLACK, 1.14, 11.0, 2),
            "waypoint mission button": (RED, 1.22, 1.0, 2),
            "airdrop mission button": (RED, 1.22, 1.5, 2),
            "scouting mission button": (RED, 1.22, 2.0, 2),
            "off axis scout mission button": (RED, 1.22, 2.5, 2),
            "requested scout mission button": (RED, 1.22, 3.0, 2),
            "landing loiter mission button": (RED, 1.22, 3.5, 2),
            "mission state button": (RED, 1.22, 4.5, 2),
            "computed path button": (RED, 1.22, 5.0, 2),
            "Generate button": (BLACK, 1.1, 12.0, 2),
            "Compute button": (BLACK, 1.3, 6.5, 2),
            "Upload mission button": (BLACK, 1.3, 7.5, 2),
            "Start mission button": (BLACK, 1.3, 8.5, 2),
            "Pause mission button": (BLACK, 1.3, 9.5, 2),
            "land button": (BLACK, 1.3, 12.0, 2),
            "ending mission button": (RED, 1.3, 13.0, 2),
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
        }

        # status type of each button
        self.mission_state_display_dict = {
            0: "waypoint mission button",
            1: "airdrop mission button",
            2: "scouting mission button",
            3: "off axis scout mission button",
            4: "requested scout mission button",
            5: "landing loiter mission button",
            6: "mission state button",
            7: "computed path button",
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
            'Reload': (BLACK, 1.06, 10.5),
            'Clear': (BLACK, 1.14, 10.5),
            'path display:': (WHITE, 1.3, 0.5),
            'waypoints': (WHITE, 1.31, 1.0),
            'airdrop': (WHITE, 1.31, 1.5),
            'scouting': (WHITE, 1.31, 2.0),
            'off axis scout': (WHITE, 1.31, 2.5),
            'requested scout': (WHITE, 1.31, 3.0),
            'landing loiter': (WHITE, 1.31, 3.5),
            'mission state': (WHITE, 1.31, 4.5),
            'computed path': (WHITE, 1.31, 5.0),
            'Generate mission': (BLACK, 1.1, 11.5),
            'Compute path': (BLACK, 1.3, 6.0),
            'Upload mission': (BLACK, 1.3, 7.0),
            'Start mission': (BLACK, 1.3, 8.0),
            'Pause mission': (BLACK, 1.3, 9.0),
            'land': (BLACK, 1.3, 11.5),
            'ending mission': (RED, 1.3, 12.5),
            'authorize airdrop': (BLACK, 1.3, 13.5),
        }

    def dashboard_projection(self, map_object):
        """project map object's position on the dashboard screen
        centered in the middle with proper scale"""

        return self.dashboard_projection_pos(map_object.pos)

    def dashboard_projection_pos(self, map_object):
        """project map position on the dashboard screen
        centered in the middle with proper scale"""

        new_x = DASHBOARD_SIZE / 2 + map_object[0] * self.map_scaling
        new_y = DASHBOARD_SIZE / 2 - map_object[1] * self.map_scaling
        return new_x, new_y

    def map_projection(self, dash_board_position):
        """project dashboard position on the map with proper scale"""
        inv_x = (dash_board_position[0] - DASHBOARD_SIZE / 2) / self.map_scaling
        inv_y = (DASHBOARD_SIZE / 2 - dash_board_position[1]) / self.map_scaling
        return inv_x, inv_y
