from pygui import layer as g_l
from pygui import colors as col
from references import parameters as para
from references import global_variables as g_v
from utility_functions import geometrical_functions as g_f

import pygame

DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE
W_S = WAYPOINT_SIZE
ORBIT_RADIUS = para.ORBIT_RADIUS
O_R = ORBIT_RADIUS

CS = g_v.ControllerStatus
MT = g_v.MessageType

WHITE = col.WHITE
BLACK = col.BLACK
BLUE = col.BLUE
GREEN = col.GREEN
RED = col.RED
DARK_CYAN = col.DARK_CYAN
DARK_GREEN = col.DARK_GREEN
YELLOW = col.YELLOW
LOW_RED = col.LOW_RED
PURPLE = col.PURPLE


def u_i_pos(a, b):
    return D_S * a, D_S / 15 * b


class UserInterfaceLayer(g_l.Layer):
    """Layer handling the input menu"""

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S * 2, D_S))

        # Text dictionary
        labels = {
        }

        # add menu labels
        for key in labels:
            label_i = labels[key]
            pos = (D_S * label_i[1], D_S / 15 * label_i[2])
            label_obj = g_l.Label(self, label_i[3], label_i[0], pos)
            self.layer_objects[key] = label_obj

        # function that returns the appropriate mission profile function
        def mp_func(name_i):

            # check if controller is busy
            if self.g_u_i.is_controller_busy():
                return

            mp_funcs = {
                "button input waypoints": g_v.mp.clear_waypoints,
                "button input obstacles": g_v.mp.clear_obstacles,
                "button input border": g_v.mp.clear_border,
                "button input search": g_v.mp.clear_search,
                "button input airdrop": lambda: None,
                "button input airdrop goal": lambda: None,
                "button input lost comms": lambda: None,
                "button input off axis": lambda: None,
                "button input emergent": lambda: None,
                "button input mapping": g_v.mp.clear_mapping_area,
            }
            mp_funcs[name_i]()

        # input buttons with color, position, size, and text
        setting_input_buttons = {
            "waypoints": (BLUE, 1.03, 0.5, 2, "Waypoint"),
            "obstacles": (YELLOW, 1.03, 1.0, 2, "Obstacles"),
            "border": (LOW_RED, 1.03, 1.5, 2, "Border"),
            "search": (BLUE, 1.03, 2.0, 2, "Search area"),
            "mapping": (DARK_GREEN, 1.03, 2.5, 2, "Mapping area"),
            "off axis": (WHITE, 1.03, 3.0, 2, "Off axis obj"),
            "airdrop": (PURPLE, 1.03, 3.5, 2, "Airdrop"),
            "airdrop goal": (PURPLE, 1.03, 4.0, 2, "Airdrop Goal"),
            "lost comms": (WHITE, 1.03, 4.5, 2, "Lost comms"),
            "emergent": (DARK_GREEN, 1.03, 5.0, 2, "Emergent obj"),
            "go to": (BLACK, 1.03, 13.0, 2, "go to position")
        }

        # middle click actions
        def m():
            self.g_u_i.layers["mission profile"].to_draw = True

        radio_input = []
        for key_i in setting_input_buttons:
            b_i = setting_input_buttons[key_i]
            key_i = "button input " + key_i
            pos = (D_S * b_i[1], D_S / 15 * b_i[2])
            # give the appropriate action based on the button type
            SIB = g_l.SettingInputButton
            button_obj = SIB(self, b_i[0], pos, b_i[3] * W_S,
                             b_i[4], key_i, middle=(m, ()),
                             right=(mp_func, (key_i,)))

            # add button to list of references
            # for other input buttons
            self.layer_objects[key_i] = button_obj
            radio_input.append(button_obj)
            button_obj.radio = radio_input

        # left click actions
        def l_1():
            self.g_u_i.layers["mission state"].to_draw = True

        def l_2():
            self.g_u_i.layers["path"].to_draw = True

        def l_3():
            self.g_u_i.layers["mission state"].to_draw = True
            self.g_u_i.layers["path"].to_draw = True

        def l_4():
            self.g_u_i.layers["telemetry"].to_draw = True

        # toggle buttons
        toggle_buttons = {
            "mission state": (WHITE, 1.03, 7.5, 2, "Mission state", l_1),
            "computed path": (WHITE, 1.03, 8.0, 2, "Computed path", l_2),
            "turn waypoints": (WHITE, 1.03, 8.5, 2, "Turn waypoints", l_2),
            "inactive points": (WHITE, 1.03, 9.0, 2, "Inactive points", l_3),
            "trajectory": (WHITE, 1.03, 9.5, 2, "Trajectory", l_4),
            "waypoints": (DARK_CYAN, 1.03, 10.0, 2, "Waypoints", l_3),
            "imaging": (DARK_CYAN, 1.03, 10.5, 2, "Imaging", l_3),
            "off axis": (DARK_CYAN, 1.03, 11.0, 2, "Off axis", l_3),
        }

        # add toggle menu buttons
        for key in toggle_buttons:
            b_i = toggle_buttons[key]
            key = "button toggle " + key
            pos = (D_S * b_i[1], D_S / 15 * b_i[2])
            STB = g_l.SettingToggleButton
            button_obj = STB(self, b_i[0], pos, b_i[3] * W_S,
                             b_i[4], key, (b_i[5], ()))
            self.layer_objects[key] = button_obj

        # button actions
        def l_action(name_i):

            left_click_funcs = {
                "button action generate": CS.GENERATE_MISSION,
                "button action compute": CS.COMPUTE_PATH,
                "button action upload": CS.UPLOAD_MISSION,
                "button action start": CS.START_MISSION,
                "button action pause": CS.PAUSE_MISSION,
                "button action land": CS.LAND,
                "button action camera gimbal": CS.CAMERA_GIMBAL_COMMAND
                #"button action drop authorization": (BLACK, 1.03, 14.0, 2, "drop UGV")
            }

            # mission generation is disabled during flights
            if (name_i == "button action generate"
                    or name_i == "button action compute"
                    or name_i == "button action upload"):

                # check if plane on ground
                if g_v.th is None:
                    return
                if not g_v.th.in_air.data_received():
                    return
                if g_v.th.in_air.data["in air"]:
                    self.g_u_i.display_message("not available mid-air",
                                               "", MT.CONTROLLER)
                    return

            g_v.mc.queue.append(left_click_funcs[name_i])

        # default dictionary of control menu buttons
        buttons = {
            "generate": (BLACK, 1.03, 5.75, 2, "Generate state"),
            "compute": (BLACK, 1.03, 6.25, 2, "Compute path"),
            "upload": (BLACK, 1.03, 6.75, 2, "Upload mission"),
            "start": (BLACK, 1.03, 12.0, 2, "Start mission"),
            "pause": (BLACK, 1.03, 12.5, 2, "Pause mission"),
            "land": (BLACK, 1.03, 13.5, 2, "land"),
            #"camera gimbal": (BLACK, 1.03, 14.0, 2, "camera gimbal")
            #"drop authorization": (BLACK, 1.03, 14.0, 2, "drop UGV")
        }

        # add input menu buttons
        for key in buttons:
            b_i = buttons[key]
            key = "button action " + key
            pos = (D_S * b_i[1], D_S / 15 * b_i[2])
            button_obj = g_l.Button(self, b_i[0], pos, b_i[3] * W_S,
                                    b_i[4], (l_action, (key,)))
            self.layer_objects[key] = button_obj

        # selected map objects
        self.selection = {
            "waypoints": 0,
            "obstacles": 0,
            "border": 0,
            "search": 0,
            "mission state": 0
        }

        # for inputing obstacles
        self.obstacle_inputing = False
        self.obstacle_input_pos = None

        # for inputing mapping area
        self.mapping_inputing = False
        self.mapping_input_pos = None

    # altitude input and altitude typing box
    altitude_box = 200.0
    input_altitude = 0.0

    def redraw(self):
        """draws control menu"""

        # shortcuts
        line = pygame.draw.line
        background = self.g_u_i.layers["background"]

        # for displaying altitude input
        text = "altitude: " + str(self.altitude_box) + "ft"
        label = self.font.render(text, True, BLACK)
        label_pos = label.get_rect(center=(D_S * 1.1, 14.5 * D_S / 15))
        pygame.draw.rect(self.surface, BLACK, label_pos, width=1)
        self.surface.blit(label, label_pos)

        # draw the places to add waypoints

        # draw the obstacle that is being inputted if there is one
        if self.obstacle_inputing:
            surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
            o_i_p = self.obstacle_input_pos
            input_rad = g_f.distance_2d(o_i_p, pygame.mouse.get_pos())
            pygame.draw.circle(surf, YELLOW, o_i_p, input_rad)
            self.surface.blit(surf, (0, 0))

        # draw line separating parts of the user interface
        line(self.surface, BLACK, (D_S * 1.2, 0), (D_S * 1.2, D_S))
        line(self.surface, BLACK, (D_S * 1.4, 0), (D_S * 1.4, D_S))
        line(self.surface, BLACK, (D_S * 1.7, 0), (D_S * 1.7, D_S))

        # draw cursor for rotation
        if self.layer_objects["button input go to"].on:
            surf = pygame.Surface((D_S, D_S), pygame.SRCALPHA)
            cur_pos = pygame.mouse.get_pos()
            rad = background.map_scaling * O_R
            pygame.draw.circle(surf, BLACK, cur_pos, rad, width=3)
            self.surface.blit(surf, (0, 0))

    def mouse_button_down(self, event, cur_pos):
        """reacts to mouse button down event"""

        # shortcuts
        background = self.g_u_i.layers["background"]
        dash_proj = background.dashboard_projection
        zoom = background.zoom()
        map_scaling = background.map_scaling
        mp_layer = self.g_u_i.layers["mission profile"]
        cur_on_map = background.map_projection(cur_pos)

        # check that the user is clicking on the map
        if cur_pos[0] < D_S:

            # check if there is contact with any map object
            contact = False

            # dictionary of map objects that can be interacted with
            map_objs = {
                "waypoints": g_v.mp.mission_waypoints,
                "obstacles": g_v.mp.obstacles,
                "border": g_v.mp.border.vertices,
                "search": g_v.mp.search_area.vertices
            }

            gen1 = range(len(g_v.mp.mission_waypoints))
            gen2 = range(len(g_v.mp.border.vertices))
            gen3 = range(len(g_v.mp.search_area.vertices))

            def scale(sel_name, ind_i, gen):
                sel_index = self.selection[sel_name]
                if sel_index == ind_i or sel_index == (ind_i - 1) % len(gen):
                    return W_S * zoom * 1.5
                else:
                    return W_S * zoom

            map_objs_size = {
                "waypoints": [scale("waypoints", i, gen1) for i in gen1],
                "obstacles": [o.r * map_scaling for o in g_v.mp.obstacles],
                "border": [scale("border", i, gen2) for i in gen2],
                "search": [scale("search", i, gen3) for i in gen3]
            }

            # dictionary of functions to delete those map_objects
            map_objs_del = {
                "waypoints": lambda x: g_v.mp.delete_waypoint(x),
                "obstacles": lambda x: g_v.mp.delete_obstacle(x),
                "border": lambda x: g_v.mp.delete_border_vertex(x),
                "search": lambda x: g_v.mp.delete_search_vertex(x)
            }

            # dictionary of input buttons
            input_button_dict = {
                "waypoints": self.layer_objects["button input waypoints"],
                "obstacles": self.layer_objects["button input obstacles"],
                "border": self.layer_objects["button input border"],
                "search": self.layer_objects["button input search"]
            }

            # go over all map objects that can be interacted with
            for delete_type in map_objs:
                # get the list of map objects
                d_t = delete_type
                map_objects = map_objs[d_t]
                i = 0
                # iterate over list of map objects
                while i < len(map_objects):
                    dash_xy = dash_proj(map_objects[i])
                    cursor_to_obj = g_f.distance_2d(cur_pos, dash_xy)
                    # check if cursor is inside map object
                    if cursor_to_obj < map_objs_size[d_t][i]:
                        contact = True
                        # select object
                        obj_num = max(len(map_objects), 1)
                        if pygame.mouse.get_pressed()[0]:
                            if input_button_dict[d_t].on:
                                self.selection[d_t] = i % obj_num
                                mp_layer.to_draw = True
                            i += 1
                        # delete object
                        elif pygame.mouse.get_pressed()[2]:
                            if input_button_dict[d_t].on:
                                map_objs_del[d_t](i)
                                self.selection[d_t] -= 1
                                self.selection[d_t] %= obj_num
                                mp_layer.to_draw = True
                            else:
                                i += 1
                        else:
                            i += 1
                    else:
                        i += 1

            # if the user is click on the map,
            # add the object they want to add
            if pygame.mouse.get_pressed()[0]:

                # change location of emergent object last known position
                if self.layer_objects["button input go to"].on:
                    g_v.mc.queue.append(CS.GO_TO)
                    g_v.mc.queue.append((cur_on_map, self.altitude_box))
                    return

                if not contact:

                    # check if controller is busy
                    if self.g_u_i.is_controller_busy():
                        return

                    # check if plane on ground
                    if g_v.th is None:
                        return
                    if not g_v.th.in_air.data_received():
                        return
                    if g_v.th.in_air.data["in air"]:
                        self.g_u_i.display_message("not available mid-air",
                                                   "", MT.CONTROLLER)
                        return

                    # add waypoint
                    if self.layer_objects["button input waypoints"].on:
                        mission_waypoints = g_v.mp.mission_waypoints
                        a_b = self.altitude_box
                        sel_way = self.selection["waypoints"]
                        g_v.mp.add_waypoint(sel_way + 1, cur_on_map, a_b)
                        self.selection["waypoints"] += 1
                        if len(mission_waypoints) == 2:
                            self.selection["waypoints"] = 1

                    # start inputing of obstacle
                    elif self.layer_objects["button input obstacles"].on:
                        self.obstacle_inputing = True
                        self.obstacle_input_pos = cur_pos

                    # add border vertex
                    elif self.layer_objects["button input border"].on:
                        sel_border = self.selection["border"]
                        g_v.mp.add_border_vertex(sel_border + 1, cur_on_map)
                        self.selection["border"] += 1

                    # add search border vertex
                    elif self.layer_objects["button input search"].on:
                        sel_search = self.selection["search"]
                        g_v.mp.add_search_vertex(sel_search + 1, cur_on_map)
                        self.selection["search"] += 1

                    # change location of airdrop position
                    elif self.layer_objects["button input airdrop"].on:
                        g_v.mp.set_airdrop(cur_on_map)

                    # change location of airdrop goal
                    elif self.layer_objects["button input airdrop goal"].on:
                        g_v.mp.set_airdrop_goal(cur_on_map)

                    # change location of lost comms position
                    elif self.layer_objects["button input lost comms"].on:
                        g_v.mp.set_lostcomms(cur_on_map)

                    # change location of off axis object position
                    elif self.layer_objects["button input off axis"].on:
                        g_v.mp.set_offaxis_obj(cur_on_map)

                    # change location of emergent object last known position
                    elif self.layer_objects["button input emergent"].on:
                        g_v.mp.set_emergent_obj(cur_on_map)

                    # change map area by inputing the top and bottom point
                    elif self.layer_objects["button input mapping"].on:
                        if not self.mapping_inputing:
                            self.mapping_inputing = True
                            self.mapping_input_pos = cur_on_map
                        else:
                            self.mapping_inputing = False
                            set_map = g_v.mp.set_mapping_area
                            set_map(self.mapping_input_pos, cur_on_map)
    
    def mouse_button_up(self, event, cur_pos):
        """react to mouse button up event"""

        # shortcut for functions
        background = self.g_u_i.layers["background"]
        proj = background.map_projection
        # get cursor position
        cur_pos = pygame.mouse.get_pos()

        # if inputing obstacle, add it
        if self.obstacle_inputing:
            self.obstacle_inputing = False
            obs_input = proj(self.obstacle_input_pos)
            dis = g_f.distance_2d(obs_input, proj(cur_pos))
            if dis > 10:
                g_v.mp.add_obstacle((obs_input, dis))
                
    def key_down(self, event, cur_pos):
        """react to key down event"""

        if event.key == pygame.K_BACKSPACE:
            self.altitude_box = float(int(self.altitude_box / 10))
            self.g_u_i.to_draw("user interface")
        else:
            if event.unicode.isdigit():
                self.altitude_box *= 10
                self.altitude_box += int(event.unicode)
                self.g_u_i.to_draw("user interface")

    def tick(self, cur_pos):
        """react to tick event"""

        # keep updating window if inputing obstacle
        if (self.obstacle_inputing
                or self.layer_objects["button input go to"].on):
            self.to_draw = True
        