import pygame
import time
import threading
import _thread


def activate(g_u_i):
    """Starts the interaction loop of the window"""

    # start input manager thread
    manager_loop = gui_input_manager_loop
    threading.Thread(target=manager_loop, args=(g_u_i, g_u_i.fps)).start()
    g_u_i.to_draw_all()

    # start regular display update thread
    def display_full_update():
        while True:
            # request redrawing of all layers
            g_u_i.to_draw_all()
            time.sleep(1 / g_u_i.rus)

    threading.Thread(target=display_full_update).start()


def gui_input_manager_loop(g_u_i, fps):
    """Check for interface inputs and update the display"""

    try:
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
                        _thread.interrupt_main()
                        return

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
    except pygame.error:
        pass
