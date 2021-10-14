import pygame as pygame
import sys

import Path_finder
import Path_objects
import Geometrical_functions
import Parameters

WAYPOINT_SIZE = Parameters.WAYPOINT_SIZE
SELECTION_MENU_POSITION = Parameters.SELECTION_MENU_POSITION
NUMBER_OF_PATH_TO_SHOW = Parameters.NUMBER_OF_PATH_TO_SHOW
SHIFT_FOR_PATHS_TO_SHOW = Parameters.SHIFT_FOR_PATHS_TO_SHOW
COLOR_OFFSET_FOR_PATH = Parameters.COLOR_OFFSET_FOR_PATH
DEFAULT_MAP_SIZE = Parameters.DEFAULT_MAP_SIZE
DASHBOARD_SIZE = Parameters.DASHBOARD_SIZE


class Dashboard:
    def __init__(self, Map_center, Start_position, Mission_waypoint_list, Obstacles,
                 Border):

        # mouse variables that define what is selected and what is being inputted
        self.Input_type = 0  # input type (0:waypoints, 1:obstacles, 2:border vertices
        self.Inputing = 0  # whether there is a current input being done (probably obstacle input)
        self.Input_position = (0, 0)  # position in which a current input is being done (probably obstacle input)
        self.Selected_waypoint = len(Mission_waypoint_list) - 1  # selected waypoint
        self.Selected_border_vertex = len(Border.Border_vertex_list) - 1  # selected border vertex

        # load mission and map info
        self.Final_path = None
        self.Start_position = Start_position
        self.Mission_waypoint_list = Mission_waypoint_list
        self.Obstacles = Obstacles
        self.Border = Border
        self.Map_center = Map_center

        # Initial Path computation
        self.Compute_path()
        # For QGroundControl plan exportation: generate_plan(self.b_l, self.scaled_path, self.center)

        # set the dashboard size and map_scaling to fit it
        self.Map_scaling = float('+inf')
        self.Dashboard_size = DASHBOARD_SIZE
        self.Rescale()

        # Dashboard initialization
        pygame.init()
        pygame.display.set_caption('Dashboard')
        self.screen = pygame.display.set_mode((self.Dashboard_size, self.Dashboard_size))
        self.background = pygame.image.load('Background.jpg').convert()

        # text of selection menu
        pygame.font.init()
        self.Font = pygame.font.SysFont('Comic Sans MS', 20)
        self.Text_waypoint = self.Font.render('Waypoint', True, (0, 255, 255))
        self.Text_obstacle = self.Font.render('Obstacle', True, (255, 255, 255))
        self.Text_border = self.Font.render('Border', True, (255, 255, 0))

    # change the scaling of the map to fit the dashboard window
    def Rescale(self):
        Map_points = sum(list(Path_i.Waypoint_list for Path_i in self.Final_path),
                         []) + self.Mission_waypoint_list + self.Obstacles + self.Border.Border_vertex_list
        if len(Map_points) == 0:
            self.Map_scaling = (self.Dashboard_size / DEFAULT_MAP_SIZE) * 0.90
        else:
            Map_points_x = list(Map_point.x for Map_point in Map_points)
            Map_points_y = list(Map_point.y for Map_point in Map_points)
            New_map_size = max(2 * abs(min(Map_points_x)), 2 * abs(max(Map_points_x)), 2 * abs(min(Map_points_y)),
                               2 * abs(max(Map_points_y)))
            self.Map_scaling = min((self.Dashboard_size / (New_map_size + 1)) * 0.90,
                                   (self.Dashboard_size / DEFAULT_MAP_SIZE) * 0.90)

    # call compute path if new mission/map info is given through the interface for testing purposes
    def Compute_path(self):
        self.Final_path = Path_finder.Final_path_finder(self.Start_position, self.Mission_waypoint_list, self.Obstacles,
                                                        self.Border)
        # For QGroundControl plan exportation: generate_plan(self.b_l, self.scaled_path, self.center)

    # update the display
    def Update(self):
        # rescale map
        self.Rescale()
        self.screen.blit(self.background, (0, 0))

        # draw control menu (in order, blue waypoints, white obstacles, yellow border)
        pygame.draw.circle(self.screen, (0, 255, 255), (SELECTION_MENU_POSITION, 1 * SELECTION_MENU_POSITION),
                           2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 255, 255), (SELECTION_MENU_POSITION, 2.5 * SELECTION_MENU_POSITION),
                           2 * WAYPOINT_SIZE)
        pygame.draw.circle(self.screen, (255, 255, 0), (SELECTION_MENU_POSITION, 4 * SELECTION_MENU_POSITION),
                           2 * WAYPOINT_SIZE)
        self.screen.blit(self.Text_waypoint, (0.3 * SELECTION_MENU_POSITION, 0.3 * SELECTION_MENU_POSITION))
        self.screen.blit(self.Text_obstacle, (0.4 * SELECTION_MENU_POSITION, 1.8 * SELECTION_MENU_POSITION))
        self.screen.blit(self.Text_border, (0.5 * SELECTION_MENU_POSITION, 3.3 * SELECTION_MENU_POSITION))

        # draw the border in yellow (edges of border and border vertexes with the ones selected being bigger)
        self.Draw_Edges(self.Border, Color=(255, 255, 0))
        self.Draw_rescaled_points(self.Border.Border_vertex_list, self.Selected_border_vertex, Color=(255, 255, 0))

        # draw obstacles in white
        for Obstacle_i in self.Obstacles:
            pygame.draw.circle(self.screen, (255, 255, 255),
                               self.Dashboard_projection(Obstacle_i), Obstacle_i.r * self.Map_scaling)

        # draw the obstacle that is being inputted if there is one
        if self.Inputing == 1:
            pygame.draw.circle(self.screen, (255, 255, 255), self.Input_position,
                               Geometrical_functions.Distance_2d(self.Input_position, pygame.mouse.get_pos()))

        # draw waypoints of paths and paths themselves in different colors
        if len(sum(list(Path_i.Waypoint_list for Path_i in self.Final_path), [])) > 0:
            Rotator = Geometrical_functions.RGBRotate()
            Rotator_2 = Geometrical_functions.RGBRotate()
            Rotator_3 = Geometrical_functions.RGBRotate()
            Paths_to_show = min(len(self.Final_path), NUMBER_OF_PATH_TO_SHOW)
            for Path_index in range(Paths_to_show):
                # draw each path in a different color that shifts between mission points
                Rotator.set_hue_rotation(Path_index * (360 / Paths_to_show))
                Rotator_2.set_hue_rotation(
                    (Path_index + COLOR_OFFSET_FOR_PATH / Paths_to_show) * (360 / Paths_to_show))
                Color_1 = Rotator.apply(255, 0, 0)
                Color_2 = Rotator_2.apply(255, 0, 0)
                self.Draw_Edges(self.Final_path[Path_index], Color=Color_1, Shift=Path_index * SHIFT_FOR_PATHS_TO_SHOW,
                                Color_2=Color_2)

                # draw waypoints of path in a slightly different color that shifts between mission points
                Rotator_3.set_hue_rotation(
                    (0.5 * COLOR_OFFSET_FOR_PATH / (Paths_to_show + 1)) * (360 / Paths_to_show))
                Colors = Rotator_3.apply(Color_1[0], Color_1[1], Color_1[2]), Rotator_3.apply(Color_2[0], Color_2[1],
                                                                                              Color_2[2])
                Color_index = 0
                for Waypoint_i in self.Final_path[Path_index].Waypoint_list:
                    if self.Is_a_mission_waypoint(Waypoint_i):
                        Color_index += 1
                    Shift = Path_index * SHIFT_FOR_PATHS_TO_SHOW
                    pygame.draw.circle(self.screen, Colors[Color_index % 2],
                                       self.Dashboard_projection(Waypoint_i, Shift=Shift),
                                       WAYPOINT_SIZE)

        # draw mission waypoints on top of Final path in blue
        self.Draw_rescaled_points(self.Mission_waypoint_list, self.Selected_waypoint, (0, 0, 0))

        # draw starting position on top of Mission waypoints in orange
        pygame.draw.circle(self.screen, (50, 50, 50), self.Dashboard_projection(self.Start_position),
                           1.5 * WAYPOINT_SIZE)

    # draw Points with the one selected and the previous one being bigger
    def Draw_rescaled_points(self, Points, Selected, Color):
        for i in range(len(Points)):
            if Selected % len(Points) == i or (Selected + 1) % max(1,len(Points)) == i:
                Waypoint_size = WAYPOINT_SIZE * 1.5
            else:
                Waypoint_size = WAYPOINT_SIZE
            pygame.draw.circle(self.screen, Color, self.Dashboard_projection(Points[i]),
                               Waypoint_size)

    # Check if a waypoint is part of the mission waypoints
    def Is_a_mission_waypoint(self, Waypoint_1):
        Reached_new_mission_point = False
        for Mission_point in self.Mission_waypoint_list:
            if Geometrical_functions.Float_eq_2d(Waypoint_1.Point_2d(), Mission_point.Point_2d()):
                Reached_new_mission_point = True
        return Reached_new_mission_point

    # draw edges of path or border with varying color between each two mission waypoints
    def Draw_Edges(self, Object_with_edges, Color, Shift=0, Color_2=None):
        Edges_to_draw = Object_with_edges.Compute_simple_edges()
        if Color_2 is None:
            Color_2 = Color
        Colors = Color, Color_2
        Color_index = 0
        for Edge in Edges_to_draw:
            if self.Is_a_mission_waypoint(Edge[0]):
                Color_index += 1
            pygame.draw.line(self.screen, Colors[Color_index % 2], self.Dashboard_projection(Edge[0], Shift=Shift),
                             self.Dashboard_projection(Edge[1], Shift=Shift), width=2)

    # project map object's position on the dashboard screen centered in the middle with proper scale
    def Dashboard_projection(self, Map_object, Shift=0):
        new_x = self.Dashboard_size / 2 + Map_object.x * self.Map_scaling + Shift
        new_y = self.Dashboard_size / 2 - Map_object.y * self.Map_scaling + Shift
        return new_x, new_y

    # project dashboard position on the map with proper scale
    def Map_projection(self, Dash_board_position):
        inv_x = (Dash_board_position[0] - self.Dashboard_size / 2) / self.Map_scaling
        inv_y = (self.Dashboard_size / 2 - Dash_board_position[1]) / self.Map_scaling
        return inv_x, inv_y

    # GUI window update display and check for interface inputs
    def Events(self):
        while True:

            # Check if some input that would require a screen update was done
            Input_done = False

            # to check if path computation is needed
            Compute = False

            # check interface inputs
            for event in pygame.event.get():

                # if the user starts clicking, add the object or delete the object they want, or record their input
                if event.type == pygame.MOUSEBUTTONDOWN:

                    # get position of the cursor
                    Cursor_position = pygame.mouse.get_pos()

                    # Check if the user is selecting a certain object type to input or (left click) or trying to
                    # delete all instances of the object type (right click)
                    if Geometrical_functions.Distance_2d(Cursor_position,
                                                         (SELECTION_MENU_POSITION, SELECTION_MENU_POSITION)) < (
                            2 * WAYPOINT_SIZE):
                        if pygame.mouse.get_pressed()[0]:
                            self.Input_type = 0
                        else:
                            self.Mission_waypoint_list.clear()
                            Compute = True

                    elif Geometrical_functions.Distance_2d(Cursor_position,
                                                           (SELECTION_MENU_POSITION, 2.5 * SELECTION_MENU_POSITION)) < (
                            2 * WAYPOINT_SIZE):
                        if pygame.mouse.get_pressed()[0]:
                            self.Input_type = 1
                        else:
                            self.Obstacles.clear()
                            Compute = True

                    elif Geometrical_functions.Distance_2d(Cursor_position,
                                                           (SELECTION_MENU_POSITION, 4 * SELECTION_MENU_POSITION)) < (
                            2 * WAYPOINT_SIZE):
                        if pygame.mouse.get_pressed()[0]:
                            self.Input_type = 2
                        else:
                            self.Border.Border_vertex_list.clear()
                            Compute = True

                    # if the user is not interacting with the input menu, then add whatever they want to input
                    # (left click) or delete whatever they are trying to delete (right click)
                    else:

                        # to check if the user is click on the map with nothing on its way (need nothing in the way
                        # to create an obstacle/waypoint/border vertex)
                        contact = False

                        # delete any obstacle they are trying to delete (right click on it)
                        i = 0
                        while i < len(self.Obstacles):
                            if (Geometrical_functions.Distance_2d(Cursor_position,
                                                                  self.Dashboard_projection(self.Obstacles[i])) <
                                    self.Obstacles[i].r * self.Map_scaling):
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    i += 1
                                else:
                                    del self.Obstacles[i]
                                    Compute = True
                            else:
                                i += 1

                        # delete any border vertex they are trying to delete (right click on it) or select one (left
                        # click)
                        i = 0
                        while i < len(self.Border.Border_vertex_list):
                            if (Geometrical_functions.Distance_2d(Cursor_position,
                                                                  self.Dashboard_projection(
                                                                      self.Border.Border_vertex_list[i])) <
                                    WAYPOINT_SIZE):
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    self.Selected_border_vertex = i % max(len(self.Border.Border_vertex_list),1)
                                    i += 1
                                    self.Input_type = 2
                                    Input_done = True
                                else:
                                    del self.Border.Border_vertex_list[i]
                                    if i <= self.Selected_border_vertex:
                                        self.Selected_border_vertex -= 1
                                        self.Selected_border_vertex %= max(len(self.Border.Border_vertex_list),1)
                                    Compute = True
                            else:
                                i += 1

                        # delete any waypoint they are trying to delete (right click on it) or select one (left click)
                        i = 0
                        while i < len(self.Mission_waypoint_list):
                            if (Geometrical_functions.Distance_2d(Cursor_position, self.Dashboard_projection(
                                    self.Mission_waypoint_list[i])) <
                                    WAYPOINT_SIZE):
                                contact = True
                                if pygame.mouse.get_pressed()[0]:
                                    self.Selected_waypoint = i % max(len(self.Mission_waypoint_list)+1)
                                    i += 1
                                    self.Input_type = 0
                                    Input_done = True
                                else:
                                    del self.Mission_waypoint_list[i]
                                    if i <= self.Selected_waypoint and len(self.Mission_waypoint_list) > 0:
                                        self.Selected_waypoint -= 1
                                        self.Selected_waypoint %= max(1,len(self.Mission_waypoint_list))
                                    Compute = True
                            else:
                                i += 1

                        # if the user is click on the map, add the object they want to add
                        if not contact and pygame.mouse.get_pressed()[0]:
                            if self.Input_type == 0:
                                self.Mission_waypoint_list.insert(self.Selected_waypoint + 1, Path_objects.Waypoint(
                                    self.Map_projection(Cursor_position)))
                                self.Selected_waypoint += 1
                                self.Selected_waypoint %= max(1,len(self.Mission_waypoint_list))
                                Compute = True
                                if len(self.Mission_waypoint_list) == 2:
                                    self.Selected_waypoint = 1

                            elif self.Input_type == 1:
                                self.Inputing = 1
                                self.Input_position = Cursor_position
                            elif self.Input_type == 2:
                                self.Border.Border_vertex_list.insert(self.Selected_border_vertex + 1,
                                                                      Path_objects.Border_vertex(
                                                                          self.Map_projection(Cursor_position)))
                                self.Selected_border_vertex += 1
                                self.Selected_border_vertex %= max(1,len(self.Border.Border_vertex_list))
                                Compute = True
                                if len(self.Border.Border_vertex_list) == 2:
                                    self.Selected_border_vertex = 1

                # When user releases click, add the input made to map info if it was an obstacle
                if event.type == pygame.MOUSEBUTTONUP:

                    # get cursor position
                    Cursor_position = pygame.mouse.get_pos()

                    # if inputing obstacle, add it
                    if self.Inputing == 1:
                        self.Inputing = 0
                        Input_position_to_cursor_distance = Geometrical_functions.Distance_2d(
                            self.Map_projection(self.Input_position),
                            self.Map_projection(Cursor_position))
                        self.Obstacles.append(Path_objects.Obstacle(self.Map_projection(self.Input_position),
                                                                    Input_position_to_cursor_distance))
                        Compute = True

                # Close Dashboard if prompted to, and quit the program
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            # if some interface input that changed mission info was made, update the path
            if Compute:
                if len(self.Mission_waypoint_list) > 0:
                    self.Compute_path()
                else:
                    self.Final_path = [Path_objects.Path(list())]
                self.Update()

            # if some interface input was made that changed the path, update the display
            if (self.Inputing == 1) or Input_done:
                self.Update()
            pygame.display.update()
