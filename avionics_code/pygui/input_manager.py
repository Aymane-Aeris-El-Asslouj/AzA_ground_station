from avionics_code.references import global_variables as g_v

import pygame
import os
import time
import threading
import json


def gui_input_manager_loop(g_u_i, fps):
    """Check for interface inputs and update the display"""

    last_time = time.time()
    while True:
        # keep the frame rate under or equal to FRAMES_PER_SECOND
        new_time = time.time()
        time.sleep(abs(1/fps - (new_time - last_time)))
        last_time = time.time()

        with threading.Lock():

            # get cursor position
            cur_pos = pygame.mouse.get_pos()

            for layer_key in g_u_i.layers_order:
                g_u_i.layers[layer_key].tick_event(cur_pos)

            # check window events
            for event in pygame.event.get():

                # transfer window events to the screen layers
                for layer_key in g_u_i.layers_order:
                    g_u_i.layers[layer_key].event(cur_pos, event)

                # if the window is being closed, register a close request
                if event.type == pygame.QUIT:
                    g_v.close_request = True

                    def closing():
                        time.sleep(1)
                        os._exit(1)

                    threading.Thread(target=closing).start()

            # close and save the settings when requested
            if g_v.close_request:
                file_name = g_u_i.settings_file_name
                if file_name is not None:
                    with open(g_u_i.settings_file_name, "w") as file:
                        json.dump(g_u_i.settings, file)
                pygame.quit()
                break

            # draw the layers that have requested a draw
            drawing_needed = False
            for key in g_u_i.layers_order:
                if g_u_i.layers[key].to_draw:
                    g_u_i.layers[key].to_draw = False
                    g_u_i.layers[key].update()
                    drawing_needed = True

            # if layers were redrawn, update display
            if drawing_needed:
                # add all layers to the screen
                for layer_key in g_u_i.layers_order:
                    g_u_i.screen.blit(g_u_i.layers[layer_key].surface, (0, 0))

            # update the window display
            pygame.display.update()
