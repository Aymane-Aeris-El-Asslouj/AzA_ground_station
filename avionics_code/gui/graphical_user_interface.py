import pygame

from avionics_code.helpers import geometrical_functions as g_f, parameters as para, global_variables as g_v

COLOR_OFFSET_FOR_PATH = para.COLOR_OFFSET_FOR_PATH
DEFAULT_MAP_SIZE = para.DEFAULT_MAP_SIZE
DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)
PREFERRED_TURN_RADIUS = para.PREFERRED_TURN_RADIUS


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
        self.screen = pygame.display.set_mode((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE))
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

        # draw paths depending on the display mode
        # ## self.draw_paths()

        # draw starting position on top of Mission waypoints in grey
        # ##Current_position_dash = self.dashboard_projection(Current_position)
        # ##pygame.draw.circle(self.screen, (50, 50, 50), Current_position_dash, 1.5 * WAYPOINT_SIZE)

    def draw_rescaled_points(self, surface, map_object_list, selected, color, type=0):
        """draw a list of map objects with the one
        selected and the previous one being bigger"""

        for i in range(len(map_object_list)):
            is_selected_1 = selected % max(1, len(map_object_list)) == i
            is_selected_2 = (selected + 1) % max(1, len(map_object_list)) == i
            if is_selected_1 or is_selected_2:
                waypoint_size = WAYPOINT_SIZE * 1.5
            else:
                waypoint_size = WAYPOINT_SIZE
            vertex_dash = self.dashboard_projection(map_object_list[i])
            if type == 0:
                pygame.draw.circle(surface, color, vertex_dash, waypoint_size)
            elif type == 1:
                w_s = waypoint_size
                pairs = [(1, 0), (0, 1), (-1, 0), (0, -1)]
                points = [(vertex_dash[0] + w_s * pair[0], vertex_dash[1] + w_s * pair[1]) for pair in pairs]
                pygame.draw.polygon(self.screen, color, points)

    def draw_path_points(self, path, color, altitude=False):
        """draw waypoints of path including alleviation waypoints, offshoot waypoints
        and optionally altitude"""

        for i in range(len(path.waypoint_list)):
            vertex_dash_1 = self.dashboard_projection(path.waypoint_list[i])
            pygame.draw.circle(self.screen, color, vertex_dash_1, WAYPOINT_SIZE)
            if altitude:
                self.draw_text_off(vertex_dash_1, str(int(path.waypoint_list[i].z)) + " ft")
            vertex_off_w = path.waypoint_list[i].off_waypoint
            # draw offshoot waypoint if there is
            if vertex_off_w is not None:
                if vertex_off_w.x is not None:
                    vertex_off = self.dashboard_projection(vertex_off_w)
                    pygame.draw.circle(self.screen, (0, 255, 0), vertex_off, WAYPOINT_SIZE)
            # draw alleviation waypoint if there is
            if path.waypoint_list[i].alleviation_waypoint is not None:
                alleviation_waypoint = path.waypoint_list[i].alleviation_waypoint
                vertex_dash_mid = self.dashboard_projection(alleviation_waypoint)
                pygame.draw.circle(self.screen, (255, 105, 180), vertex_dash_mid, WAYPOINT_SIZE)
                if altitude:
                    pass  # self.draw_text_off(vertex_dash_mid, str(int(alleviation_waypoint.z))+" ft")

    def draw_edges(self, map_structure, color):
        """draw edges of map structure"""

        edges_to_draw = map_structure.compute_simple_edges()
        for edge in edges_to_draw:
            vertex_1_dash = self.dashboard_projection(edge[0])
            vertex_2_dash = self.dashboard_projection(edge[1])
            pygame.draw.line(self.screen, color, vertex_1_dash, vertex_2_dash, width=2)

    def draw_edges_alleviated_offset(self, map_structure, color):
        """draw edges of map structure including alleviation and off shoot waypoints"""

        edges_to_draw = map_structure.compute_simple_edges()
        for index, edge in enumerate(edges_to_draw):
            """This part gets points to draw"""
            # Get previous vertex or its off shoot waypoint if it exists
            vertex_1 = self.dashboard_projection(edge[0])
            if edge[0].off_waypoint is not None:
                if edge[0].off_waypoint.x is not None:
                    vertex_1 = self.dashboard_projection(edge[0].off_waypoint)
            # Get next vertex
            vertex_2 = self.dashboard_projection(edge[1])
            # If the next vertex has an off shoot waypoint, use it to find center
            # at that vertex
            if edge[1].off_waypoint is not None:
                if edge[1].off_waypoint.x is not None:
                    vertex_off = self.dashboard_projection(edge[1].off_waypoint)
                # Set zone to draw arcs around next vertex
                center = self.dashboard_projection(edge[1].off_waypoint.alleviation_waypoint)
                radius = g_f.distance_2d(center, vertex_2)
                rect = (center[0] - radius, center[1] - radius, 2 * radius, 2 * radius)

            """drawing starts here"""
            if edge[1].alleviation_waypoint is None:
                # draw line from off waypoint_1 to waypoint 2
                """This part draws lines/areas from vertex 1 to vertex 2"""
                if (edge[0].off_waypoint is not None and edge[0].off_waypoint.x is not None) or index == 0:
                    pygame.draw.line(self.screen, color, vertex_1, vertex_2, width=2)
                else:
                    # In the case where there is no offshoot waypoint, but a danger zone instead
                    p_r = PREFERRED_TURN_RADIUS
                    off_center = g_f.center_2d(vertex_1, vertex_2)
                    pygame.draw.circle(self.screen, (0, 0, 0), off_center, 2 * p_r * self.map_scaling, width=2)
                # draw arc from waypoint 2 to off waypoint
                """This part draws arcs around vertex 2"""
                if edge[1].off_waypoint is not None:
                    if edge[1].off_waypoint.x is not None:
                        a_1, a_2 = g_f.Find_arc_angles_dash(vertex_1, vertex_2, vertex_off, center)
                        # only non-zero arcs
                        if abs(a_1 - a_2) > 0.01:
                            pygame.draw.arc(self.screen, (0, 255, 0), rect, a_1, a_2, 2)
            else:
                # Get alleviation waypoint
                vertex_mid = self.dashboard_projection(edge[1].alleviation_waypoint)
                # draw line from vertex 1 (or its off shoot waypoint) to alleviation waypoint
                """This part draws lines/areas from vertex 1 to vertex 2"""
                if index == 0 or (edge[0].off_waypoint is not None and edge[0].off_waypoint.x is not None):
                    pygame.draw.line(self.screen, color, vertex_1, vertex_mid, width=2)
                else:
                    # In the case where there is no offshoot waypoint, but a danger zone instead
                    p_r = PREFERRED_TURN_RADIUS
                    off_center = g_f.center_2d(vertex_1, vertex_mid)
                    pygame.draw.circle(self.screen, (0, 0, 0), off_center, 2 * p_r * self.map_scaling, width=2)
                """This part draws arcs around vertex 2"""
                # draw arc from alleviation waypoint to vertex 2
                a_1, a_2 = g_f.Find_arc_angles_dash(vertex_1, vertex_mid, vertex_2, center)
                pygame.draw.arc(self.screen, (255, 105, 180), rect, a_1, a_2, 2)
                # draw arc from vertex 2 to off shoot waypoint
                if edge[1].off_waypoint.x is not None:
                    a_1, a_2 = g_f.Find_arc_angles_dash(vertex_mid, vertex_2, vertex_off, center)
                    # only non-zero arcs
                    if abs(a_1 - a_2) > 0.01:
                        pygame.draw.arc(self.screen, (0, 255, 0), rect, a_1, a_2, 2)

    def dashboard_projection(self, map_object):
        """project map object's position on the dashboard screen
        centered in the middle with proper scale"""

        new_x = DASHBOARD_SIZE / 2 + map_object.pos[0] * self.map_scaling
        new_y = DASHBOARD_SIZE / 2 - map_object.pos[1] * self.map_scaling
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
            self.draw_edges(search_boundary, color=(75, 75, 255))
        self.screen.blit(surf, (0, 0))
        surf = pygame.Surface((DASHBOARD_SIZE * 1.2, DASHBOARD_SIZE), pygame.SRCALPHA)
        surf.set_alpha(150)  # alpha level
        self.draw_rescaled_points(surf, search_boundary.vertices, self.selection[3], color=(50, 50, 255))
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
        self.draw_edges(border, color=(200, 0, 0))
        self.draw_rescaled_points(self.screen, border.vertices, self.selection[2], color=(200, 0, 0))

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
        self.draw_rescaled_points(self.screen, mission_waypoints, self.selection[0], (0, 0, 255), 1)

    """draw paths of a certain type depending on the path display mode"""
    def draw_paths(self):

        straight_2d_paths_list = None
        curved_2d_paths_list = None
        curved_3d_paths_list = None
        chosen_3d_path = None

        """straight line paths"""
        if self.path_display_mode == 0:
            rotator_2 = g_f.RGBRotate()
            # draw each path in a different color that shifts between mission points
            # pick path number path_index from each path group to draw with alternating colors
            for group_index, group_i in enumerate(straight_2d_paths_list):
                if self.selected_path < len(group_i):
                    rotator_2.set_hue_rotation((group_index / len(straight_2d_paths_list)) * 360)
                    path_to_draw = group_i[self.selected_path]
                    self.draw_edges(path_to_draw, color=rotator_2.apply(204, 0, 0))
                    self.draw_path_points(path_to_draw, rotator_2.apply(204, 0, 0))

            """draw curved paths"""
        elif self.path_display_mode == 1:
            # draw path
            if self.selected_path < len(curved_2d_paths_list):
                path_to_draw = curved_2d_paths_list[self.selected_path]
                # pick path number path_index from each path group to draw with alternating colors
                self.draw_edges_alleviated_offset(path_to_draw, color=(204, 204, 0))
                self.draw_path_points(path_to_draw, (204, 204, 0))

        elif self.path_display_mode == 2:
            # draw path
            if self.selected_path < len(curved_3d_paths_list):
                path_to_draw = curved_3d_paths_list[self.selected_path]
                # pick path number path_index from each path group to draw with alternating colors
                self.draw_edges_alleviated_offset(path_to_draw, color=(204, 204, 0))
                self.draw_path_points(path_to_draw, (204, 204, 0), altitude=True)

        elif self.path_display_mode == 3:
            # draw chosen path
            self.draw_edges_alleviated_offset(chosen_3d_path, color=(204, 204, 0))
            self.draw_path_points(chosen_3d_path, (204, 204, 0), altitude=True)

    """draws text near some position"""
    def draw_text_off(self, position, text):
        ratio = (DASHBOARD_SIZE / 650)
        font_2 = pygame.font.Sysfont('Arial', int(10 * ratio))
        text_1 = font_2.render(text, True, (0, 0, 0))
        self.screen.blit(text_1, (position[0] + 10 * ratio, position[1] - 10 * ratio))

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
            "Straight paths button": (RED, 1.02, 12.0, 2),
            "Curved paths button": (RED, 1.02, 12.5, 2),
            "3D paths button": (RED, 1.02, 13.0, 2),
            "Chosen path button": (RED, 1.02, 13.5, 2),
            "Path <- button": (RED, 1.08, 14.5, 2),
            "Path -> button": (RED, 1.12, 14.5, 2),
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
            9: "Mapping area button"
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
            'display Mode:': (WHITE, 1.1, 11.5),
            'straight_paths': (WHITE, 1.11, 12),
            'curved_paths': (WHITE, 1.11, 12.5),
            '3d_paths': (WHITE, 1.11, 13.0),
            'chosen_path': (WHITE, 1.11, 13.5),
            'selected path:': (WHITE, 1.1, 14.0),
            '<--             -->': (WHITE, 1.1, 14.5),
        }
