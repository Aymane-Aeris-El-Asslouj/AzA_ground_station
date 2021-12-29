from avionics_code.references import global_variables as g_v

from . import input_manager as g_i_m

import pygame
import threading
import os
import time
import json


class GUI:
    """Handles the dashboard display"""

    def __init__(self, name, size, fps, rus):

        # dashboard initialization
        pygame.init()
        pygame.display.set_caption(name)
        self.size = size
        self.screen = pygame.display.set_mode(self.size)

        # frames per second and regular updates per second
        self.fps = fps
        self.rus = rus

        # window layers
        self.layers = {}
        self.layers_order = []

        # default font type and size
        self.font_size = None
        self.font_type = None
        self.font = None

        # stored settings
        self.settings = {}
        self.settings_file_name = None

    def load_settings(self, settings_file_name, default_settings):

        # load or create display settings
        self.settings_file_name = settings_file_name
        if os.path.exists(settings_file_name):
            with open(settings_file_name, "r") as file:
                self.settings = json.load(file)
        else:
            with open(settings_file_name, "w") as file:
                json.dump(default_settings, file)
                self.settings = default_settings

    def set_font(self, font_type, font_size):
        """sets the font type and size"""

        # font for drawing
        pygame.font.init()
        self.font_size = font_size
        self.font_type = font_type
        self.font = pygame.font.SysFont(self.font_type, self.font_size)

    def add_layer(self, layer_name, layer):
        """add gui layer to the window"""

        self.layers[layer_name] = layer
        self.layers_order.append(layer_name)

    def activate(self):
        """Starts the interaction loop of the window"""

        # start input manager thread
        manager_loop = g_i_m.gui_input_manager_loop
        threading.Thread(target=manager_loop, args=(self, self.fps)).start()
        self.to_draw_all()

        # start regular display update thread
        def display_full_update():
            while True:
                # request redrawing of all layers
                self.to_draw_all()
                time.sleep(1/self.rus)
                if g_v.close_request:
                    break
        threading.Thread(target=display_full_update).start()

    def to_draw(self, layer_name):
        """requests a layer to be redrawn"""

        self.layers[layer_name].to_draw = True

    def to_draw_all(self):
        """request all layers to be redrawn"""

        for key in self.layers:
            self.to_draw(key)
