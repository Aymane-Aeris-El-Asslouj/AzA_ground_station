from avionics_code.helpers import geometrical_functions as g_f, parameters as para, global_variables as g_v
import pygame
import sys
import time

DASHBOARD_SIZE = para.DASHBOARD_SIZE
WAYPOINT_SIZE = para.WAYPOINT_SIZE * (DASHBOARD_SIZE / 650)


def gui_input_manager_start():
    """Check for interface inputs and update the mission profile"""

    while True:

        # Check if some input that would require a screen update was done
        input_done = False

        # check interface inputs
        for event in pygame.event.get():

            # if the user starts clicking, add the object or delete
            # the object they want, or record their input
            if event.type == pygame.MOUSEBUTTONDOWN:

                # get position of the cursor
                cur_pos = pygame.mouse.get_pos()
                # Check if the user is selecting a certain object type
                # to input (left click) or trying to delete all instances
                # of the object type (right click), or changing display mode
                d_s = DASHBOARD_SIZE
                dict1 = g_v.gui.input_type_dict
                dict3 = g_v.gui.mission_state_display_dict
                # reversed dictionaries to check which buttons has what type or what status
                input_type_dict_rev = {dict1[key]: key for key in dict1}
                mission_state_display_dict_rev = {dict3[key]: key for key in dict3}

                def action_1():
                    g_v.mp = g_v.sc.get_mission()

                def action_2():
                    g_v.th.clear()
                    g_v.rf.fetch_plane_status()

                # left click actions for buttons
                left_button_actions = {
                    "Reload button": action_1,
                    "Clear button": g_v.mp.clear_all,
                    "Reload 2 button": action_2,
                    "Clear 2 button": g_v.th.clear,
                    "Generate button": g_v.ms.generate,
                    "Compute button": g_v.mc.compute_path,
                    "Export button": g_v.mc.export_path,
                    "land button": g_v.ms.land,
                    "ending mission button": g_v.rf.end_mission,
                    "drop authorization button": g_v.rf.authorize_airdrop
                }

                # right click actions for buttons
                right_button_actions = {
                    "Waypoint button": g_v.mp.clear_waypoints,
                    "Obstacle button": g_v.mp.clear_obstacles,
                    "Border button": g_v.mp.clear_border,
                    "Search area button": g_v.mp.clear_search,
                    "Airdrop button": g_v.mp.clear_airdrop,
                    "Airdrop goal button": g_v.mp.clear_airdrop_goal,
                    "lost comms button": g_v.mp.clear_lostcomms,
                    "Off axis obj button": g_v.mp.clear_offaxis_obj,
                    "Emergent obj button": g_v.mp.clear_emergent_obj,
                    "Mapping area button": g_v.mp.clear_mapping_area,
                    "Plane button": g_v.mp.clear_emergent_obj,
                    "UGV button": g_v.mp.clear_mapping_area,
                }

                # Check that cursor in in the control panel
                if cur_pos[0] > d_s:
                    g_v.gui.inputing = 0

                    # run over all buttons to check if one was click
                    for key in g_v.gui.buttons:
                        button = g_v.gui.buttons[key]
                        button_pos = (d_s * button[1], button[2] * d_s / 15)
                        # check if the cursor is inside the button
                        if g_f.distance_2d(cur_pos, button_pos) < (button[3] * WAYPOINT_SIZE):
                            # check for left click
                            if pygame.mouse.get_pressed()[0]:
                                # change input type if the button does that
                                if key in input_type_dict_rev.keys():
                                    g_v.gui.input_type = input_type_dict_rev[key]
                                    input_done = True
                                # change mission state input type if there is one
                                if key in mission_state_display_dict_rev.keys():
                                    index = mission_state_display_dict_rev[key]
                                    var = g_v.gui.mission_state_display[index]
                                    g_v.gui.mission_state_display[index] = (var + 1) % 2
                                    input_done = True
                                # do a left click actions if the button has one
                                if key in left_button_actions.keys():
                                    left_button_actions[key]()
                                    input_done = True
                            # or else right click
                            elif pygame.mouse.get_pressed()[2]:
                                # do a right click actions if the button has one
                                if key in right_button_actions.keys():
                                    if key in input_type_dict_rev.keys():
                                        if g_v.gui.is_displayed[input_type_dict_rev[key]] == 1:
                                            right_button_actions[key]()
                                            input_done = True
                                    else:
                                        right_button_actions[key]()
                                        input_done = True
                            # or middle click
                            elif pygame.mouse.get_pressed()[1]:
                                # change input type if the button does that
                                if key in input_type_dict_rev.keys():
                                    g_v.gui.is_displayed[input_type_dict_rev[key]] += 1
                                    g_v.gui.is_displayed[input_type_dict_rev[key]] %= 2
                                    input_done = True

                # if the user is not interacting with the input menu, then add whatever they want to input
                # (left click) or delete whatever they are trying to delete (right click)
                else:
                    # to check if the user is clicking on the map with nothing
                    # on its way (need nothing in the way
                    # to create an obstacle/waypoint/border vertex)
                    contact = False

                    # dictionary of map_objects that can be interacted with in the map
                    map_objects_dict = {
                        0: g_v.mp.mission_waypoints,
                        1: g_v.mp.obstacles,
                        2: g_v.mp.border.vertices,
                        3: g_v.mp.search_area.vertices
                    }
                    W_S = WAYPOINT_SIZE
                    s = g_v.gui.selection
                    gen1 = range(len(g_v.mp.mission_waypoints))
                    gen2 = range(len(g_v.mp.border.vertices))
                    gen3 = range(len(g_v.mp.search_area.vertices))
                    map_objects_size_dict = {
                        0: [W_S * (1.5 if (s[0] == i or s[0] == (i - 1) % len(gen1)) else 1) for i in gen1],
                        1: [o.r * g_v.gui.map_scaling for o in g_v.mp.obstacles],
                        2: [W_S * (1.5 if (s[2] == i or s[2] == (i - 1) % len(gen2)) else 1) for i in gen2],
                        3: [W_S * (1.5 if (s[2] == i or s[2] == (i - 1) % len(gen3)) else 1) for i in gen3]
                    }
                    # dictionary of functions to delete those map_objects
                    delete_func_dict = {
                        0: lambda x: g_v.mp.delete_waypoint(x),
                        1: lambda x: g_v.mp.delete_obstacle(x),
                        2: lambda x: g_v.mp.delete_border_vertex(x),
                        3: lambda x: g_v.mp.delete_search_vertex(x)
                    }

                    # go over all map objects that can be interacted with
                    for delete_type in range(4):
                        # get the list of map objects
                        map_objects = map_objects_dict[delete_type]
                        i = 0
                        # iterate over list of map objects
                        while i < len(map_objects):
                            map_object_dash_xy = g_v.gui.dashboard_projection(map_objects[i])
                            cursor_to_map_object = g_f.distance_2d(cur_pos, map_object_dash_xy)
                            # check if cursor is inside map object
                            if cursor_to_map_object < map_objects_size_dict[delete_type][i]:
                                contact = True
                                # select object
                                if pygame.mouse.get_pressed()[0]:
                                    if g_v.gui.input_type == delete_type and g_v.gui.is_displayed[delete_type] == 1:
                                        g_v.gui.selection[delete_type] = i % max(len(map_objects), 1)
                                        input_done = True
                                    i += 1
                                # delete object
                                else:
                                    if g_v.gui.input_type == delete_type and g_v.gui.is_displayed[delete_type] == 1:
                                        delete_func_dict[delete_type](i)
                                        g_v.gui.selection[delete_type] -= 1
                                        g_v.gui.selection[delete_type] %= max(len(map_objects), 1)
                                        input_done = True
                                    else:
                                        i += 1
                            else:
                                i += 1

                    # if the user is click on the map, add the object they want to add
                    if not contact and pygame.mouse.get_pressed()[0]:

                        # add waypoint
                        if g_v.gui.input_type == 0 and g_v.gui.is_displayed[0] == 1:
                            mission_waypoints = g_v.mp.mission_waypoints
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.add_waypoint(g_v.gui.selection[0] + 1, cursor_on_map, g_v.gui.altitude_box)
                            g_v.gui.selection[0] += 1
                            g_v.gui.selection[0] %= max(1, len(mission_waypoints))
                            input_done = True
                            if len(mission_waypoints) == 2:
                                g_v.gui.selection[0] = 1

                        # start inputing of obstacle
                        elif g_v.gui.input_type == 1 and g_v.gui.is_displayed[1] == 1:
                            g_v.gui.inputing = 1
                            g_v.gui.input_position = cur_pos

                        # add border vertex
                        elif g_v.gui.input_type == 2 and g_v.gui.is_displayed[2] == 1:
                            vertices = g_v.mp.border.vertices
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.add_border_vertex(g_v.gui.selection[2] + 1, cursor_on_map)
                            g_v.gui.selection[2] += 1
                            g_v.gui.selection[2] %= max(1, len(vertices))
                            input_done = True
                            if len(vertices) == 2:
                                g_v.gui.selection[2] = 1

                        # add search border vertex
                        elif g_v.gui.input_type == 3 and g_v.gui.is_displayed[3] == 1:
                            vertices = g_v.mp.search_area.vertices
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.add_search_vertex(g_v.gui.selection[3] + 1, cursor_on_map)
                            g_v.gui.selection[3] += 1
                            g_v.gui.selection[3] %= max(1, len(vertices))
                            input_done = True
                            if len(vertices) == 2:
                                g_v.gui.selection[3] = 1

                        # change location of airdrop position
                        elif g_v.gui.input_type == 4 and g_v.gui.is_displayed[4] == 1:
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.set_airdrop(cursor_on_map)
                            input_done = True

                        # change location of airdrop goal
                        elif g_v.gui.input_type == 5 and g_v.gui.is_displayed[5] == 1:
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.set_airdrop_goal(cursor_on_map)
                            input_done = True

                        # change location of lost comms position
                        elif g_v.gui.input_type == 6 and g_v.gui.is_displayed[6] == 1:
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.set_lostcomms(cursor_on_map)
                            input_done = True

                        # change location of off axis object position
                        elif g_v.gui.input_type == 7 and g_v.gui.is_displayed[7] == 1:
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.set_offaxis_obj(cursor_on_map)
                            input_done = True

                        # change location of emergent object last known position
                        elif g_v.gui.input_type == 8 and g_v.gui.is_displayed[8] == 1:
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            g_v.mp.set_emergent_obj(cursor_on_map)
                            input_done = True

                        # change map area by inputing the top and bottom point
                        elif g_v.gui.input_type == 9 and g_v.gui.is_displayed[9] == 1:
                            if g_v.gui.inputing == 0:
                                g_v.gui.inputing = 1
                                g_v.gui.input_position = g_v.gui.map_projection(cur_pos)
                            else:
                                g_v.gui.inputing = 0
                                cursor_on_map = g_v.gui.map_projection(cur_pos)
                                g_v.mp.set_mapping_area(g_v.gui.input_position, cursor_on_map)
                                input_done = True

                        # change plane and ugv position
                        elif g_v.gui.input_type == 10:
                            if g_v.gui.inputing == 0:
                                g_v.gui.inputing = 1
                                g_v.gui.input_position = g_v.gui.map_projection(cur_pos)
                                g_v.gui.input_altitude = g_v.gui.altitude_box
                            else:
                                g_v.gui.inputing = 0
                                plane_pos = g_v.gui.input_position
                                plane_z = g_v.gui.input_altitude
                                ugv_pos = g_v.gui.map_projection(cur_pos)
                                ugv_z = g_v.gui.altitude_box
                                g_v.th.add_flight_profile(plane_pos, plane_z, ugv_pos, ugv_z, time.time())
                                input_done = True

                        # change ugv position
                        elif g_v.gui.input_type == 11:
                            cursor_on_map = g_v.gui.map_projection(cur_pos)
                            plane_z = g_v.gui.altitude_box
                            if g_v.th.is_empty():
                                ugv_pos = (0, 0)
                                ugv_z = 0
                            else:
                                ugv_pos = g_v.th.last_flight_profile().ugv_obj.pos
                                ugv_z = g_v.th.last_flight_profile().ugv_obj.z
                            g_v.th.add_flight_profile(cursor_on_map, plane_z, ugv_pos, ugv_z, time.time())
                            input_done = True

            # When user releases click, add the input made to map info if it was an obstacle
            if event.type == pygame.MOUSEBUTTONUP:
                # shortcut for functions
                proj = g_v.gui.map_projection
                # get cursor position
                cur_pos = pygame.mouse.get_pos()

                # if inputing obstacle, add it
                if g_v.gui.inputing == 1 and g_v.gui.input_type == 1:
                    g_v.gui.inputing = 0
                    Dis = g_f.distance_2d(proj(g_v.gui.input_position), proj(cur_pos))
                    if Dis > 10:
                        g_v.mp.add_obstacle((proj(g_v.gui.input_position), Dis))
                    input_done = True

            # When user types altitude
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_BACKSPACE:
                    g_v.gui.altitude_box = float(int(g_v.gui.altitude_box/10))
                    input_done = True
                else:
                    if event.unicode.isdigit():
                        g_v.gui.altitude_box = 10 * g_v.gui.altitude_box + int(event.unicode)
                        input_done = True

            # Close dashboard if prompted to, and quit the program
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # if some interface input was made that changed the path, update the display
        if (g_v.gui.inputing == 1 and g_v.gui.input_type == 1) or input_done:
            g_v.gui.display_update()
        pygame.display.update()
