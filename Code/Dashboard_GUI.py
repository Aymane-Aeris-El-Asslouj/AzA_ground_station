import pygame
import sys

import Geometrical_functions as G_f
import Parameters as Para


COLOR_OFFSET_FOR_PATH = Para.COLOR_OFFSET_FOR_PATH
DEFAULT_MAP_SIZE = Para.DEFAULT_MAP_SIZE
DASHBOARD_SIZE = Para.DASHBOARD_SIZE
WAYPOINT_SIZE = Para.WAYPOINT_SIZE*(DASHBOARD_SIZE/650)
SELECTION_MENU_POSITION = Para.SELECTION_MENU_POSITION*(DASHBOARD_SIZE/650)
PREFERRED_TURN_RADIUS = Para.PREFERRED_TURN_RADIUS

'''Dashboard window: initialize > update > start events
Shows map data, flight data, and current path
************GUI instructions************
• Click on the dots in the selection menu up-left to go into
mission_waypoint(blue)/obstacle(white)/border(yellow) input mode
• For waypoints and border vertices, click once to place them. Click on a
waypoint/border_vertex to select it for placing a waypoint/border_vertex after it
• For Obstacles, click at where you want the obstacle to be, then drag to select the radius
• To delete any object (waypoint/obstacle/border_vertex), right click on it
• To delete all objects of a certain type, right click on the dot in the
selection menu that corresponds to it
• The current algorithm will show two possible paths for each two waypoints
in different colors. That color itself alternates for the same path between waypoints
• Parameters for the GUI are in the Parameters module'''


class Dashboard:

    """Initialize Dashboard"""
    def __init__(self, Mission_profile):
        # Dashboard variables that define what is displayed, selected, and inputted
        self.Path_display_mode = 0  # (0: straight paths, 1: curved paths, 2: 3D paths, 3: selected path)
        self.Input_type = 0  # (0:waypoints, 1:obstacles, 2:border vertices
        self.Inputing = 0  # whether there is a current input being done (probably obstacle input)
        self.Input_position = (0, 0)  # position in which a current input is being done (probably obstacle input)
        self.Selected_waypoint = 0  # selected waypoint
        self.Selected_vertex = 0  # selected border vertex
        self.Selected_path = 0  # path to show

        # Make a reference to mission profile and to obstacles/Border/Mission_list
        self.Mission_profile = Mission_profile

        # set the map_scaling to fit it
        self.Map_scaling = float('+inf')

        # Dashboard initialization
        pygame.init()
        pygame.display.set_caption('Dashboard')
        self.screen = pygame.display.set_mode((DASHBOARD_SIZE*1.2, DASHBOARD_SIZE))
        self.background = pygame.image.load('Background.png').convert()

        # text of selection menu
        pygame.font.init()
        self.Font = pygame.font.SysFont('Arial', int(14*(DASHBOARD_SIZE/650)))
        self.Text_waypoint = self.Font.render('Waypoint', True, (0, 0, 255))
        self.Text_obstacle = self.Font.render('Obstacle', True, (255, 255, 255))
        self.Text_border = self.Font.render('Border', True, (200, 0, 0))
        self.Text_display = self.Font.render('Display Mode:', True, (255, 255, 255))
        self.Text_mode_1 = self.Font.render('Straight_paths', True, (255, 255, 255))
        self.Text_mode_2 = self.Font.render('Curved_paths', True, (255, 255, 255))
        self.Text_mode_3 = self.Font.render('3D_paths', True, (255, 255, 255))
        self.Text_mode_4 = self.Font.render('Chosen_path', True, (255, 255, 255))
        self.Text_path = self.Font.render('Selected path:', True, (255, 255, 255))
        self.Text_select = self.Font.render('<--             -->', True, (255, 255, 255))

    """Check for interface inputs"""
    def GUI_input_manager_start(self):
        while True:

            # Check if some input that would require a screen update was done
            Input_done = False

            # to check if path computation is needed
            Compute = False

            # check interface inputs
            for event in pygame.event.get():

                # if the user starts clicking, add the object or delete
                # the object they want, or record their input
                if event.type == pygame.MOUSEBUTTONDOWN:

                    # get position of the cursor
                    Cursor_position = pygame.mouse.get_pos()
                    # Check if the user is selecting a certain object type
                    # to input (left click) or trying to delete all instances
                    # of the object type (right click), or changing display mode
                    D_S = DASHBOARD_SIZE
                    if Cursor_position[0] > D_S:
                        if G_f.Distance_2d(Cursor_position, (D_S * 1.1, 1.0 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Input_type = 0
                                Input_done = True
                            else:
                                self.Mission_profile.Clear_waypoints()
                                Compute = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.1, 2.0 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Input_type = 1
                                Input_done = True
                            else:
                                self.Mission_profile.Clear_obstacles()
                                Compute = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.1, 3.0 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Input_type = 2
                                Input_done = True
                            else:
                                self.Mission_profile.Clear_border()
                                Compute = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.02, 5.0 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Path_display_mode = 0
                                self.clamp_display_mode()
                                Input_done = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.02, 5.5 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Path_display_mode = 1
                                self.clamp_display_mode()
                                Input_done = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.02, 6.0 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Path_display_mode = 2
                                self.clamp_display_mode()
                                Input_done = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.02, 6.5 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Path_display_mode = 3
                                self.clamp_display_mode()
                                Input_done = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.08, 8.5 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                if self.Selected_path != 0:
                                    self.Selected_path -= 1
                                Input_done = True

                        elif G_f.Distance_2d(Cursor_position, (D_S * 1.12, 8.5 * D_S / 15)) < (2 * WAYPOINT_SIZE):
                            if pygame.mouse.get_pressed()[0]:
                                self.Selected_path += 1
                                self.clamp_display_mode()
                                Input_done = True

                    # if the user is not interacting with the input menu, then add whatever they want to input
                    # (left click) or delete whatever they are trying to delete (right click)
                    else:
                        # to check if the user is click on the map with nothing on its way (need nothing in the way
                        # to create an obstacle/waypoint/border vertex)
                        contact = False

                        # delete any obstacle they are trying to delete (right click on it)
                        Obstacles = self.Mission_profile.Obstacles
                        i = 0
                        while i < len(Obstacles):
                            Obstacle_i_dash_xy = self.Dashboard_projection(Obstacles[i])
                            Cursor_to_obstacle = G_f.Distance_2d(Cursor_position, Obstacle_i_dash_xy)
                            Obstacle_i_dash_r = Obstacles[i].r * self.Map_scaling
                            if Cursor_to_obstacle < Obstacle_i_dash_r:
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    i += 1
                                else:
                                    self.Mission_profile.Delete_obstacle(i)
                                    Compute = True
                            else:
                                i += 1

                        # delete any border vertex they are trying to delete (right click on it) or select one (left
                        # click)
                        Vertex_list = self.Mission_profile.Border.Vertex_list
                        i = 0
                        while i < len(Vertex_list):
                            Vertex_dash_xy = self.Dashboard_projection(Vertex_list[i])
                            Cursor_to_vertex = G_f.Distance_2d(Cursor_position, Vertex_dash_xy)
                            if Cursor_to_vertex < WAYPOINT_SIZE:
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    self.Selected_vertex = i % max(len(Vertex_list), 1)
                                    i += 1
                                    self.Input_type = 2
                                    Input_done = True
                                else:
                                    self.Mission_profile.Delete_vertex(i)
                                    if i <= self.Selected_vertex:
                                        self.Selected_vertex -= 1
                                        self.Selected_vertex %= max(len(Vertex_list), 1)
                                    Compute = True
                            else:
                                i += 1

                        # delete any waypoint they are trying to delete
                        # (right click on it) or select one (left click)
                        Mission_list = self.Mission_profile.Mission_waypoint_list
                        i = 0
                        while i < len(Mission_list):
                            Waypoint_dash_xy = self.Dashboard_projection(Mission_list[i])
                            Cursor_to_waypoint = G_f.Distance_2d(Cursor_position, Waypoint_dash_xy)
                            if Cursor_to_waypoint < WAYPOINT_SIZE:
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    self.Selected_waypoint = i % max(len(Mission_list), 1)
                                    i += 1
                                    self.Input_type = 0
                                    Input_done = True
                                else:
                                    self.Mission_profile.Delete_waypoint(i)
                                    if i <= self.Selected_waypoint and len(Mission_list) > 0:
                                        self.Selected_waypoint -= 1
                                        self.Selected_waypoint %= max(1, len(Mission_list))
                                    Compute = True
                            else:
                                i += 1

                        # if the user is click on the map, add the object they want to add
                        if not contact and pygame.mouse.get_pressed()[0]:
                            if self.Input_type == 0:
                                Mission_list = self.Mission_profile.Mission_waypoint_list
                                Cursor_on_map = self.Map_projection(Cursor_position)
                                self.Mission_profile.Add_waypoint(self.Selected_waypoint + 1, Cursor_on_map)
                                self.Selected_waypoint += 1
                                self.Selected_waypoint %= max(1, len(Mission_list))
                                Compute = True
                                if len(Mission_list) == 2:
                                    self.Selected_waypoint = 1

                            elif self.Input_type == 1:
                                self.Inputing = 1
                                self.Input_position = Cursor_position
                            elif self.Input_type == 2:
                                Vertex_list = self.Mission_profile.Border.Vertex_list
                                Cursor_on_map = self.Map_projection(Cursor_position)
                                self.Mission_profile.Add_vertex(self.Selected_vertex + 1, Cursor_on_map)
                                self.Selected_vertex += 1
                                self.Selected_vertex %= max(1, len(Vertex_list))
                                Compute = True
                                if len(Vertex_list) == 2:
                                    self.Selected_vertex = 1

                # When user releases click, add the input made to map info if it was an obstacle
                if event.type == pygame.MOUSEBUTTONUP:

                    # get cursor position
                    Cursor_position = pygame.mouse.get_pos()

                    # if inputing obstacle, add it
                    if self.Inputing == 1:
                        self.Inputing = 0
                        Input_position_to_cursor_distance = G_f.Distance_2d(
                            self.Map_projection(self.Input_position),
                            self.Map_projection(Cursor_position))
                        Input_on_map = self.Map_projection(self.Input_position)
                        if Input_position_to_cursor_distance > 10:
                            self.Mission_profile.Add_obstacle((Input_on_map, Input_position_to_cursor_distance))
                        Compute = True

                # Close Dashboard if prompted to, and quit the program
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # if some interface input that changed mission info was made, update the path
            if Compute:
                self.Mission_profile.Compute_path()
                self.Display_update()

            # if some interface input was made that changed the path, update the display
            if (self.Inputing == 1) or Input_done and (not Compute):
                self.Display_update()
            pygame.display.update()

    """Update the display"""
    def Display_update(self):

        # load info from Mission profile
        Current_position = self.Mission_profile.Current_position
        Mission_waypoint_list = self.Mission_profile.Mission_waypoint_list
        Obstacles = self.Mission_profile.Obstacles
        Border = self.Mission_profile.Border

        # rescale map
        self.Rescale()
        self.screen.fill((110, 0, 200))
        self.screen.blit(self.background, (0, 0))

        # draw control menu
        self.Draw_control_menu()

        # draw the border (edges of border and border vertexes with the ones selected being bigger)
        self.Draw_Edges(Border, Color=(200, 0, 0))
        self.Draw_rescaled_points(Border.Vertex_list, self.Selected_vertex, Color=(200, 0, 0))

        # draw obstacles in white
        for Obstacle_i in Obstacles:
            Obstacle_i_dash_xy = self.Dashboard_projection(Obstacle_i)
            Obstacle_i_dash_r = Obstacle_i.r * self.Map_scaling
            pygame.draw.circle(self.screen, (255, 255, 255), Obstacle_i_dash_xy, Obstacle_i_dash_r)

        # draw the obstacle that is being inputted if there is one
        if self.Inputing == 1:
            Inputing_radius = G_f.Distance_2d(self.Input_position, pygame.mouse.get_pos())
            pygame.draw.circle(self.screen, (255, 255, 255), self.Input_position, Inputing_radius)

        # Draw paths depending on the display mode
        self.Draw_paths()

        # draw mission waypoints on top of Final path in black
        self.Draw_rescaled_points(Mission_waypoint_list, self.Selected_waypoint, (0, 0, 255))

        # draw starting position on top of Mission waypoints in grey
        Current_position_dash = self.Dashboard_projection(Current_position)
        pygame.draw.circle(self.screen, (50, 50, 50), Current_position_dash, 1.5 * WAYPOINT_SIZE)

    """draw a list of map objects with the one selected and the previous one being bigger"""
    def Draw_rescaled_points(self, Map_object_list, Selected, Color):
        for i in range(len(Map_object_list)):
            Is_selected_1 = Selected % max(1, len(Map_object_list)) == i
            Is_selected_2 = (Selected + 1) % max(1, len(Map_object_list)) == i
            if Is_selected_1 or Is_selected_2:
                Waypoint_size = WAYPOINT_SIZE * 1.5
            else:
                Waypoint_size = WAYPOINT_SIZE
            Vertex_dash = self.Dashboard_projection(Map_object_list[i])
            pygame.draw.circle(self.screen, Color, Vertex_dash, Waypoint_size)

    """draw waypoints of path including alleviation waypoints, offshoot waypoints
    and optionally altitude"""
    def Draw_path_points(self, Path, Color, Altitude=False):
        for i in range(len(Path.Waypoint_list)):
            Vertex_dash_1 = self.Dashboard_projection(Path.Waypoint_list[i])
            pygame.draw.circle(self.screen, Color, Vertex_dash_1, WAYPOINT_SIZE)
            if Altitude:
                self.Draw_text_off(Vertex_dash_1, str(int(Path.Waypoint_list[i].z))+" ft")
            Vertex_off_w = Path.Waypoint_list[i].Off_waypoint
            # draw offshoot waypoint if there is
            if Vertex_off_w is not None:
                if Vertex_off_w.x is not None:
                    Vertex_off = self.Dashboard_projection(Vertex_off_w)
                    pygame.draw.circle(self.screen, (0, 255, 0), Vertex_off, WAYPOINT_SIZE)
            # draw alleviation waypoint if there is
            if Path.Waypoint_list[i].Alleviation_waypoint is not None:
                Alleviation_waypoint = Path.Waypoint_list[i].Alleviation_waypoint
                Vertex_dash_mid = self.Dashboard_projection(Alleviation_waypoint)
                pygame.draw.circle(self.screen, (255, 105, 180), Vertex_dash_mid, WAYPOINT_SIZE)
                if Altitude:
                    pass #  self.Draw_text_off(Vertex_dash_mid, str(int(Alleviation_waypoint.z))+" ft")

    """draw edges of map structure"""
    def Draw_Edges(self, Map_structure, Color):
        Edges_to_draw = Map_structure.Compute_simple_edges()
        for Edge in Edges_to_draw:
            Vertex_1_dash = self.Dashboard_projection(Edge[0])
            Vertex_2_dash = self.Dashboard_projection(Edge[1])
            pygame.draw.line(self.screen, Color, Vertex_1_dash, Vertex_2_dash, width=2)

    """draw edges of map structure including alleviation and off shoot waypoints"""
    def Draw_Edges_alleviated_offset(self, Map_structure, Color):
        Edges_to_draw = Map_structure.Compute_simple_edges()
        for index, Edge in enumerate(Edges_to_draw):
            """This part gets points to draw"""
            # Get previous vertex or its off shoot waypoint if it exists
            Vertex_1 = self.Dashboard_projection(Edge[0])
            if Edge[0].Off_waypoint is not None:
                if Edge[0].Off_waypoint.x is not None:
                    Vertex_1 = self.Dashboard_projection(Edge[0].Off_waypoint)
            # Get next vertex
            Vertex_2 = self.Dashboard_projection(Edge[1])
            # If the next vertex has an off shoot waypoint, use it to find center
            # at that vertex
            if Edge[1].Off_waypoint is not None:
                if Edge[1].Off_waypoint.x is not None:
                    Vertex_off = self.Dashboard_projection(Edge[1].Off_waypoint)
                # Set zone to draw arcs around next vertex
                Center = self.Dashboard_projection(Edge[1].Off_waypoint.Alleviation_waypoint)
                Radius = G_f.Distance_2d(Center, Vertex_2)
                Rect = (Center[0] - Radius, Center[1] - Radius, 2 * Radius, 2 * Radius)

            """Drawing starts here"""
            if Edge[1].Alleviation_waypoint is None:
                # draw line from off waypoint_1 to waypoint 2
                """This part draws lines/areas from vertex 1 to vertex 2"""
                if (Edge[0].Off_waypoint is not None and Edge[0].Off_waypoint.x is not None) or index == 0:
                    pygame.draw.line(self.screen, Color, Vertex_1, Vertex_2, width=2)
                else:
                    # In the case where there is no offshoot waypoint, but a danger zone instead
                    P_R = PREFERRED_TURN_RADIUS
                    Off_center = G_f.Center_2d(Vertex_1, Vertex_2)
                    pygame.draw.circle(self.screen, (0, 0, 0), Off_center, 2*P_R * self.Map_scaling, width=2)
                # draw arc from waypoint 2 to off waypoint
                """This part draws arcs around vertex 2"""
                if Edge[1].Off_waypoint is not None:
                    if Edge[1].Off_waypoint.x is not None:
                        A_1, A_2 = G_f.Find_arc_angles_dash(Vertex_1, Vertex_2, Vertex_off, Center)
                        # only non-zero arcs
                        if abs(A_1-A_2) > 0.01:
                            pygame.draw.arc(self.screen, (0, 255, 0), Rect, A_1, A_2, 2)
            else:
                # Get alleviation waypoint
                Vertex_mid = self.Dashboard_projection(Edge[1].Alleviation_waypoint)
                # draw line from vertex 1 (or its off shoot waypoint) to alleviation waypoint
                """This part draws lines/areas from vertex 1 to vertex 2"""
                if index == 0 or (Edge[0].Off_waypoint is not None and Edge[0].Off_waypoint.x is not None):
                    pygame.draw.line(self.screen, Color, Vertex_1, Vertex_mid, width=2)
                else:
                    # In the case where there is no offshoot waypoint, but a danger zone instead
                    P_R = PREFERRED_TURN_RADIUS
                    Off_center = G_f.Center_2d(Vertex_1, Vertex_mid)
                    pygame.draw.circle(self.screen, (0, 0, 0), Off_center, 2*P_R * self.Map_scaling, width=2)
                """This part draws arcs around vertex 2"""
                # draw arc from alleviation waypoint to vertex 2
                A_1, A_2 = G_f.Find_arc_angles_dash(Vertex_1, Vertex_mid, Vertex_2, Center)
                pygame.draw.arc(self.screen, (255, 105, 180), Rect, A_1, A_2, 2)
                # draw arc from vertex 2 to off shoot waypoint
                if Edge[1].Off_waypoint.x is not None:
                    A_1, A_2 = G_f.Find_arc_angles_dash(Vertex_mid, Vertex_2, Vertex_off, Center)
                    # only non-zero arcs
                    if abs(A_1-A_2) > 0.01:
                        pygame.draw.arc(self.screen, (0, 255, 0), Rect, A_1, A_2, 2)

    """project map object's position on the dashboard screen centered in the middle with proper scale"""
    def Dashboard_projection(self, Map_object):
        new_x = DASHBOARD_SIZE / 2 + Map_object.x * self.Map_scaling
        new_y = DASHBOARD_SIZE / 2 - Map_object.y * self.Map_scaling
        return new_x, new_y

    """project dashboard position on the map with proper scale"""
    def Map_projection(self, Dash_board_position):
        inv_x = (Dash_board_position[0] - DASHBOARD_SIZE / 2) / self.Map_scaling
        inv_y = (DASHBOARD_SIZE / 2 - Dash_board_position[1]) / self.Map_scaling
        return inv_x, inv_y

    """change the scaling of the map to fit the dashboard window"""
    def Rescale(self):
        Map_obj_list_1 = self.Mission_profile.Obstacles
        Map_obj_list_2 = self.Mission_profile.Border.Vertex_list
        Map_obj_list_3 = self.Mission_profile.Mission_waypoint_list
        Map_objects_all = Map_obj_list_1 + Map_obj_list_2 + Map_obj_list_3
        if len(Map_objects_all) == 0:
            self.Map_scaling = (DASHBOARD_SIZE / DEFAULT_MAP_SIZE) * 0.90
        else:
            Map_objects_x = list(Map_object_i.x for Map_object_i in Map_objects_all)
            Map_objects_y = list(Map_object_i.y for Map_object_i in Map_objects_all)
            New_map_size = max(2 * abs(min(Map_objects_x)), 2 * abs(max(Map_objects_x)), 2 * abs(min(Map_objects_y)),
                               2 * abs(max(Map_objects_y)))
            self.Map_scaling = min((DASHBOARD_SIZE / (New_map_size + 1)) * 0.90,
                                   (DASHBOARD_SIZE / DEFAULT_MAP_SIZE) * 0.90)

    """Limit selected path to the number of paths available"""
    def clamp_display_mode(self):
        Straight_2D_paths_list = self.Mission_profile.Straight_2D_paths_list
        Curved_2D_paths_list = self.Mission_profile.Curved_2D_paths_list
        Curved_3D_paths_list = self.Mission_profile.Curved_3D_paths_list
        if self.Path_display_mode == 0:
            if len(Straight_2D_paths_list) > 0:
                Max = max(0, len(Straight_2D_paths_list[0])-1)
            else:
                Max = 0
        elif self.Path_display_mode == 1:
            Max = max(0, len(Curved_2D_paths_list)-1)
        elif self.Path_display_mode == 2:
            Max = max(0, len(Curved_3D_paths_list)-1)
        elif self.Path_display_mode == 3:
            Max = 0
        self.Selected_path = min(self.Selected_path, Max)

    """Draws control menu"""
    def Draw_control_menu(self):
        D_S = DASHBOARD_SIZE

        pygame.draw.circle(self.screen, (0, 0, 255), (D_S * 1.1, 1.0 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 255, 255), (D_S * 1.1, 2.0 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (200, 0, 0), (D_S * 1.1, 3.0 * D_S / 15), 2 * WAYPOINT_SIZE)

        if self.Input_type == 0:
            pygame.draw.circle(self.screen, (0, 0, 255), (D_S * 1.1, 1.0 * D_S / 15), 3 * WAYPOINT_SIZE)
        elif self.Input_type == 1:
            pygame.draw.circle(self.screen, (255, 255, 255), (D_S * 1.1, 2.0 * D_S / 15), 3 * WAYPOINT_SIZE)
        elif self.Input_type == 2:
            pygame.draw.circle(self.screen, (200, 0, 0), (D_S * 1.1, 3.0 * D_S / 15), 3 * WAYPOINT_SIZE)

        pygame.draw.circle(self.screen, (255, 0, 0), (D_S * 1.02, 5.0 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 0, 0), (D_S * 1.02, 5.5 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 0, 0), (D_S * 1.02, 6.0 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 0, 0), (D_S * 1.02, 6.5 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 0, 0), (D_S * 1.08, 8.5 * D_S / 15), 2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 0, 0), (D_S * 1.12, 8.5 * D_S / 15), 2 * WAYPOINT_SIZE)

        if self.Path_display_mode == 0:
            pygame.draw.circle(self.screen, (0, 255, 0), (D_S * 1.02, 5.0 * D_S / 15), 2 * WAYPOINT_SIZE)
        elif self.Path_display_mode == 1:
            pygame.draw.circle(self.screen, (0, 255, 0), (D_S * 1.02, 5.5 * D_S / 15), 2 * WAYPOINT_SIZE)
        elif self.Path_display_mode == 2:
            pygame.draw.circle(self.screen, (0, 255, 0), (D_S * 1.02, 6.0 * D_S / 15), 2 * WAYPOINT_SIZE)
        elif self.Path_display_mode == 3:
            pygame.draw.circle(self.screen, (0, 255, 0), (D_S * 1.02, 6.5 * D_S / 15), 2 * WAYPOINT_SIZE)

        Text_pos_1 = self.Text_waypoint.get_rect(center=(D_S * 1.1, 0.5 * D_S / 15))
        Text_pos_2 = self.Text_obstacle.get_rect(center=(D_S * 1.1, 1.5 * D_S / 15))
        Text_pos_3 = self.Text_border.get_rect(center=(D_S * 1.1, 2.5 * D_S / 15))
        Text_pos_4 = self.Text_display.get_rect(center=(D_S * 1.1, 4.5 * D_S / 15))
        Text_pos_5 = self.Text_display.get_rect(center=(D_S * 1.1, 8.0 * D_S / 15))
        Text_pos_6 = self.Text_display.get_rect(center=(D_S * 1.1, 8.5 * D_S / 15))

        self.screen.blit(self.Text_waypoint, Text_pos_1)
        self.screen.blit(self.Text_obstacle, Text_pos_2)
        self.screen.blit(self.Text_border, Text_pos_3)
        self.screen.blit(self.Text_display, Text_pos_4)
        self.screen.blit(self.Text_mode_1, (D_S * 1.04, 4.85 * D_S / 15))
        self.screen.blit(self.Text_mode_2, (D_S * 1.04, 5.35 * D_S / 15))
        self.screen.blit(self.Text_mode_3, (D_S * 1.04, 5.85 * D_S / 15))
        self.screen.blit(self.Text_mode_4, (D_S * 1.04, 6.35 * D_S / 15))
        self.screen.blit(self.Text_path, Text_pos_5)
        self.screen.blit(self.Text_select, Text_pos_6)

    """Draw paths of a certain type depending on the path display mode"""
    def Draw_paths(self):
        Straight_2D_paths_list = self.Mission_profile.Straight_2D_paths_list
        Curved_2D_paths_list = self.Mission_profile.Curved_2D_paths_list
        Curved_3D_paths_list = self.Mission_profile.Curved_3D_paths_list
        Chosen_3D_path = self.Mission_profile.Chosen_3D_path

        """straight line paths"""
        if self.Path_display_mode == 0:
            Rotator_2 = G_f.RGBRotate()
            # draw each path in a different color that shifts between mission points
            # pick path number Path_index from each path group to draw with alternating colors
            for Group_index, Group_i in enumerate(Straight_2D_paths_list):
                if self.Selected_path < len(Group_i):
                    Rotator_2.set_hue_rotation((Group_index / len(Straight_2D_paths_list)) * 360)
                    Path_to_draw = Group_i[self.Selected_path]
                    self.Draw_Edges(Path_to_draw, Color=Rotator_2.apply(204, 0, 0))
                    self.Draw_path_points(Path_to_draw, Rotator_2.apply(204, 0, 0))

            """Draw curved paths"""
        elif self.Path_display_mode == 1:
            # draw path
            if self.Selected_path < len(Curved_2D_paths_list):
                Path_to_draw = Curved_2D_paths_list[self.Selected_path]
                # pick path number Path_index from each path group to draw with alternating colors
                self.Draw_Edges_alleviated_offset(Path_to_draw, Color=(204, 204, 0))
                self.Draw_path_points(Path_to_draw, (204, 204, 0))

        elif self.Path_display_mode == 2:
            # draw path
            if self.Selected_path < len(Curved_3D_paths_list):
                Path_to_draw = Curved_3D_paths_list[self.Selected_path]
                # pick path number Path_index from each path group to draw with alternating colors
                self.Draw_Edges_alleviated_offset(Path_to_draw, Color=(204, 204, 0))
                self.Draw_path_points(Path_to_draw, (204, 204, 0), Altitude=True)

        elif self.Path_display_mode == 3:
            # draw chosen path
            self.Draw_Edges_alleviated_offset(Chosen_3D_path, Color=(204, 204, 0))
            self.Draw_path_points(Chosen_3D_path, (204, 204, 0), Altitude=True)

    """Draws text near some position"""
    def Draw_text_off(self, Position, Text):
        Ratio = (DASHBOARD_SIZE/650)
        Font_2 = pygame.font.SysFont('Arial', int(10*Ratio))
        Text_1 = Font_2.render(Text, True, (0, 0, 0))
        self.screen.blit(Text_1, (Position[0]+10*Ratio, Position[1]-10*Ratio))
