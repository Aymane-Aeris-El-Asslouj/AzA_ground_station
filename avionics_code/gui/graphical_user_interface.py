import pygame
import time

from avionics_code.flight import flight_profile as f_p
from avionics_code.helpers import geometrical_functions as g_f, parameters as para, global_variables as g_v
from avionics_code.gui import drawing_functions as d_f


COLOR_OFFSET_FOR_PATH = para.COLOR_OFFSET_FOR_PATH
DEFAULT_MAP_SIZE = para.DEFAULT_MAP_SIZE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS
TIME_CUT_OFF_FOR_FLIGHT_STATUS = para.TIME_CUT_OFF_FOR_FLIGHT_STATUS


class GUI:
    """Handles the dashboard display"""

    def __init__(self):
        # dashboard variables that define what is displayed, selected, and inputted
        # (0: straight paths, 1: curved paths, 2: 3d paths, 3: selected path)
        self.path_display_mode = 0
        # (0:waypoints, 1:obstacles, 2:border vertices, etc)
        self.input_type = 0
        # whether there is a current input being done (probably obstacle input)
        self.inputing = 0
        # position in which a current input is being done (probably obstacle input)
        self.input_position = (0, 0)
        # selected map objects
        self.selection = [0]*9
        # path to show
        self.selected_path = 0

        # set the map_scaling to fit it
        self.map_scaling = DASHBOARD_SIZE / DEFAULT_MAP_SIZE

        # dashboard initialization
        pygame.init()
        pygame.display.set_caption('GUI')
        self.screen = pygame.display.set_mode((DASHBOARD_SIZE * 1.4, DASHBOARD_SIZE))
        self.background = pygame.image.load('extra files/background.png').convert()
        self.background = pygame.transform.scale(self.background, (DASHBOARD_SIZE, DASHBOARD_SIZE))

        # initializes the selection menu
        pygame.font.init()
        self.font = None
        self.buttons = None
        self.input_type_dict = None
        self.texts = None
        self.status_type_dict = None
        self.interface_init()

    def display_update(self):
        """Update the display"""

        # fill background
        self.screen.fill((205, 133, 63))
        self.screen.blit(self.background, (0, 0))

        # draw control menu
        self.draw_control_menu()

        # draw mission profile info (map)
        self.draw_mission_profile()

        # draw flight profile history (plane)
        self.draw_flight_profile()

        # draw paths depending on the display mode
        # ## self.draw_paths()

        # draw starting position on top of Mission waypoints in grey
        # ##Current_position_dash = self.dashboard_projection(Current_position)
        # ##pygame.draw.circle(self.screen, (50, 50, 50), Current_position_dash, 1.5 * WAYPOINT_SIZE)

    def draw_control_menu(self):
        """draws control menu"""

        d_s = DASHBOARD_SIZE
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

        # change color of selected buttons
        for key in self.status_type_dict:
            if self.path_display_mode == key:
                B = self.buttons[self.status_type_dict[key]]
                self.buttons[self.status_type_dict[key]] = (GREEN, B[1], B[2], B[3])
            else:
                B = self.buttons[self.status_type_dict[key]]
                self.buttons[self.status_type_dict[key]] = (RED, B[1], B[2], B[3])

        # for displaying buttons
        for key in self.buttons:
            B = self.buttons[key]
            pygame.draw.circle(self.screen, B[0], (d_s * B[1], B[2] * d_s / 15), B[3] * WAYPOINT_SIZE)

        # for displaying labels
        for key in self.texts:
            label = self.font.render(key, True, self.texts[key][0])
            label_pos = label.get_rect(center=(d_s * self.texts[key][1], self.texts[key][2] * d_s / 15))
            self.screen.blit(label, label_pos)

        # draw line separating two side of the menu
        D_S = DASHBOARD_SIZE
        pygame.draw.line(self.screen, (0, 0, 0), (D_S*1.2, 0), (D_S*1.2, D_S))

    def draw_mission_profile(self):
        """Displays information of the mission profile
        border, obstacles, mission waypoints, etc"""

        # load info from Mission profile
        mission_waypoints = g_v.mp.mission_waypoints
        obstacles = g_v.mp.obstacles
        border = g_v.mp.border
        lost_comms = g_v.mp.lost_comms
        search_boundary = g_v.mp.search_boundary
        off_axis_object = g_v.mp.off_axis_object
        emergent_object = g_v.mp.emergent_object
        ugv_boundary = g_v.mp.ugv_boundary
        airdrop_obj = g_v.mp.airdrop_obj
        ugv_goal = g_v.mp.ugv_goal
        mapping_area = g_v.mp.mapping_area

        # object search area
        surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
        surf.set_alpha(100)  # alpha level
        polygon = list(self.dashboard_projection(v) for v in search_boundary.vertices)
        if len(search_boundary.vertices) > 2:
            pygame.draw.polygon(surf, (50, 50, 255), polygon, width=0)
        else:
            d_f.draw_edges(search_boundary, color=(75, 75, 255))
        self.screen.blit(surf, (0, 0))
        surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
        surf.set_alpha(150)  # alpha level
        d_f.draw_rescaled_points(surf, search_boundary.vertices, self.selection[3], color=(50, 50, 255))
        self.screen.blit(surf, (0, 0))

        # airdrop boundary area
        surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
        surf.set_alpha(100)  # alpha level
        polygon = list(self.dashboard_projection(v) for v in ugv_boundary.vertices)
        pygame.draw.polygon(surf, (139, 0, 139), polygon, width=0)
        self.screen.blit(surf, (0, 0))

        # mapping area
        surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
        surf.set_alpha(50)  # alpha level
        if len(mapping_area.vertices) > 2:
            polygon = list(self.dashboard_projection(v) for v in mapping_area.vertices)
        pygame.draw.polygon(surf, (0, 255, 0), polygon, width=0)
        self.screen.blit(surf, (0, 0))

        # draw the border (edges of border and border vertexes with the ones selected being bigger)
        d_f.draw_edges(border, color=(200, 0, 0))
        d_f.draw_rescaled_points(self.screen, border.vertices, self.selection[2], color=(200, 0, 0))

        # draw obstacles in white
        for obstacle_i in obstacles:
            obstacle_i_dash_xy = self.dashboard_projection(obstacle_i)
            obstacle_i_dash_r = obstacle_i.r * self.map_scaling
            pygame.draw.circle(self.screen, (200, 200, 0), obstacle_i_dash_xy, obstacle_i_dash_r)

        # draw the obstacle that is being inputted if there is one
        if self.inputing == 1 and self.input_type == 1:
            inputing_radius = g_f.distance_2d(self.input_position, pygame.mouse.get_pos())
            pygame.draw.circle(self.screen, (200, 200, 0), self.input_position, inputing_radius)

        # draw off-axis object
        off_obj_pos = self.dashboard_projection(off_axis_object)
        pygame.draw.circle(self.screen, (0, 0, 0), off_obj_pos, WAYPOINT_SIZE * 2.3, width=4)
        pygame.draw.circle(self.screen, (255, 255, 255), off_obj_pos, WAYPOINT_SIZE * 2, width=2)
        pygame.draw.circle(self.screen, (0, 0, 0), off_obj_pos, WAYPOINT_SIZE * 1.8, width=2)

        # airdrop pos
        airdrop_obj_pos = self.dashboard_projection(airdrop_obj)
        pygame.draw.circle(self.screen, (0, 0, 0), airdrop_obj_pos, WAYPOINT_SIZE * 2.3)
        pygame.draw.circle(self.screen, (255, 255, 255), airdrop_obj_pos, WAYPOINT_SIZE * 2, width=2)
        pygame.draw.circle(self.screen, (255, 255, 255), airdrop_obj_pos, WAYPOINT_SIZE * 1, width=2)

        # emergent obj
        emergent_object_pos = self.dashboard_projection(emergent_object)
        pygame.draw.circle(self.screen, (0, 0, 0), emergent_object_pos, WAYPOINT_SIZE * 1.4)
        pygame.draw.circle(self.screen, (100, 255, 100), emergent_object_pos, WAYPOINT_SIZE * 1)

        # lost_comms
        lost_comms_pos = self.dashboard_projection(lost_comms)
        side = WAYPOINT_SIZE * 2.5
        lost_rect = pygame.Rect(lost_comms_pos[0] - side / 2, lost_comms_pos[1] - side / 2, side, side)
        pygame.draw.rect(self.screen, (0, 0, 0), lost_rect, width=2)
        side = WAYPOINT_SIZE * 2
        lost_rect = pygame.Rect(lost_comms_pos[0] - side / 2, lost_comms_pos[1] - side / 2, side, side)
        pygame.draw.rect(self.screen, (255, 255, 255), lost_rect, width=2)
        side = WAYPOINT_SIZE * 1.5
        lost_rect = pygame.Rect(lost_comms_pos[0] - side / 2, lost_comms_pos[1] - side / 2, side, side)
        pygame.draw.rect(self.screen, (0, 0, 0), lost_rect, width=2)

        # ugv goal obj
        ugv_goal_pos = self.dashboard_projection(ugv_goal)
        pygame.draw.circle(self.screen, (0, 0, 0), ugv_goal_pos, WAYPOINT_SIZE * 1.4)
        pygame.draw.circle(self.screen, (139, 0, 139), ugv_goal_pos, WAYPOINT_SIZE * 1)

        # draw mission waypoints on top of Final path in black
        d_f.draw_rescaled_points(self.screen, mission_waypoints, self.selection[0], (0, 0, 255), 1)

    def draw_flight_profile(self):
        """Displays information of the flight profile
        ugv/plane position, time, etc"""

        # draw all flight profiles
        f_p_list = g_v.th.flight_profile_list
        for index, flight_profile in enumerate(f_p_list):

            # time difference for the profile
            abs_time_delta = time.time() - flight_profile.time_int
            plane_obj = flight_profile.plane_obj
            ugv_obj = flight_profile.ugv_obj

            # alpha surface to draw the flight profile on
            # with alpha based on time passed
            surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
            surf.set_alpha(255 / (abs_time_delta + 1))  # alpha level
            plane_pos = self.dashboard_projection(plane_obj)
            ugv_pos = self.dashboard_projection(ugv_obj)

            if len(f_p_list) > index + 1:

                # get velocity vector from current position to next one
                plane_obj_2 = f_p_list[index + 1].plane_obj
                time_delta = f_p_list[index + 1].time_int - flight_profile.time_int
                pos_delta = g_f.sub_vectors(plane_obj_2.pos, plane_obj.pos)
                velocity_delta = g_f.scale_vector(pos_delta, 1/max(0.0001, time_delta))
                d_f.draw_arrow(surf, plane_obj, velocity_delta, (255, 255, 255))

                # get velocity vector from current position to next one
                ugv_obj_2 = f_p_list[index + 1].ugv_obj
                time_delta = f_p_list[index + 1].time_int - flight_profile.time_int
                pos_delta = g_f.sub_vectors(ugv_obj_2.pos, ugv_obj.pos)
                velocity_delta = g_f.scale_vector(pos_delta, 1 / max(0.0001, time_delta))
                d_f.draw_arrow(surf, ugv_obj, velocity_delta, (0, 0, 0))

            # draw plane and ugv positions fading based on time delta
            pygame.draw.circle(surf, (255, 255, 255), plane_pos, WAYPOINT_SIZE)
            pygame.draw.circle(surf, (0, 0, 0), ugv_pos, WAYPOINT_SIZE)
            self.screen.blit(surf, (0, 0))

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
            "Plane/UGV button": (WHITE, 1.1, 12.0, 2),
            "Plane button": (WHITE, 1.1, 13.0, 2),
            "Reload 2 button": (BLACK, 1.05, 14.0, 2),
            "Clear 2 button": (BLACK, 1.15, 14.0, 2),
            "Straight paths button": (RED, 1.22, 1.0, 2),
            "Curved paths button": (RED, 1.22, 1.5, 2),
            "3D paths button": (RED, 1.22, 2.0, 2),
            "Chosen path button": (RED, 1.22, 2.5, 2),
            "Path <- button": (RED, 1.28, 3.5, 2),
            "Path -> button": (RED, 1.32, 3.5, 2),
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
            10: "Plane/UGV button",
            11: "Plane button"
        }

        # status type of each button
        self.status_type_dict = {
            0: "Straight paths button",
            1: "Curved paths button",
            2: "3D paths button",
            3: "Chosen path button"
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
            'Plane/UGV position': (WHITE, 1.1, 11.5),
            'Plane position': (WHITE, 1.1, 12.5),
            'Reload 2': (BLACK, 1.05, 13.5),
            'Clear 2': (BLACK, 1.15, 13.5),
            'display Mode:': (WHITE, 1.3, 0.5),
            'straight_paths': (WHITE, 1.31, 1),
            'curved_paths': (WHITE, 1.31, 1.5),
            '3d_paths': (WHITE, 1.31, 2.0),
            'chosen_path': (WHITE, 1.31, 2.5),
            'selected path:': (WHITE, 1.3, 3.0),
            '<--             -->': (WHITE, 1.3, 3.5),
        }

    def dashboard_projection(self, map_object):
        """project map object's position on the dashboard screen
        centered in the middle with proper scale"""

        new_x = DASHBOARD_SIZE / 2 + map_object.pos[0] * self.map_scaling
        new_y = DASHBOARD_SIZE / 2 - map_object.pos[1] * self.map_scaling
        return new_x, new_y

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

    def clamp_display_mode(self):
        """Limit selected path to the number of paths available"""

        straight_2d_paths_list = g_v.mp.straight_2d_paths_list
        curved_2d_paths_list = g_v.mp.curved_2d_paths_list
        curved_3d_paths_list = g_v.mp.curved_3d_paths_list
        if self.path_display_mode == 0:
            if len(straight_2d_paths_list) > 0:
                max_size = max(0, len(straight_2d_paths_list[0]) - 1)
            else:
                max_size = 0
        elif self.path_display_mode == 1:
            max_size = max(0, len(curved_2d_paths_list) - 1)
        elif self.path_display_mode == 2:
            max_size = max(0, len(curved_3d_paths_list) - 1)
        elif self.path_display_mode == 3:
            max_size = 0
        self.selected_path = min(self.selected_path, max_size)
