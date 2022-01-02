import pygame
import os
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
        self.font_u = None

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
        SF = pygame.font.SysFont
        self.font = SF(self.font_type, self.font_size)
        self.font_u = SF(self.font_type, self.font_size)
        self.font_u.set_underline(True)

    def add_layer(self, layer_name, layer):
        """add gui layer to the window"""

        self.layers[layer_name] = layer
        self.layers_order.append(layer_name)

    def to_draw(self, layer_name):
        """requests a layer to be redrawn"""

        self.layers[layer_name].to_draw = True

    def to_draw_all(self):
        """request all layers to be redrawn"""

        for key in self.layers:
            self.to_draw(key)

    def save_settings(self):
        """closes the gui and saves settings"""

        file_name = self.settings_file_name
        if file_name is not None:
            with open(self.settings_file_name, "w") as file:
                json.dump(self.settings, file)
        pygame.quit()
