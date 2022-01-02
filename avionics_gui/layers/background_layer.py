from references import parameters as para
from utility_functions import geometrical_functions as g_f
from pygui import layer as g_l
from pygui import colors as col

from pathlib import Path
import pygame

DASHBOARD_SIZE = para.DASHBOARD_SIZE
DEFAULT_MAP_SIZE = para.DEFAULT_MAP_SIZE
D_S = DASHBOARD_SIZE
ZOOM_RATIO = para.ZOOM_RATIO
Z_R = ZOOM_RATIO
ZOOM_MAX = para.ZOOM_MAX
Z_M = ZOOM_MAX

BLACK = col.BLACK
BROWNISH_ORANGE = col.BROWNISH_ORANGE


class BackgroundLayer(g_l.Layer):

    def __init__(self, g_u_i):

        super().__init__(g_u_i, (D_S * 2, D_S))

        # set the map_scaling to fit it
        self.default_map_scaling = DASHBOARD_SIZE / DEFAULT_MAP_SIZE
        self.map_scaling = self.default_map_scaling
        self.screen_center = D_S / 2, D_S / 2
        self.moving_map = False

        # get background image
        pygame.init()
        pygame.display.set_mode((D_S * 2.0, D_S))
        dir_path = Path(__file__).parents[1]
        background_path = (dir_path / './resources/background.jpg').resolve()
        self.background = pygame.image.load(background_path).convert()

        # variables for moving and zooming on map
        self.move_old_cur_position = None
    
    def redraw(self):
        """draws background (picture of flight area with zoom)"""

        # get current map size
        zoom_ratio = self.zoom()
        new_map_size = DASHBOARD_SIZE * zoom_ratio

        # generate the new scaled map and get its corners
        new_background = pygame.transform.scale(self.background, (new_map_size, new_map_size))
        corner_x = self.screen_center[0] - new_map_size / 2
        corner_y = self.screen_center[1] - new_map_size / 2

        # draw map on screen with appropriate backgrounds
        self.surface.fill(BLACK)
        self.surface.blit(new_background, (corner_x, corner_y))
        pygame.draw.rect(self.surface, BROWNISH_ORANGE,
                         (D_S, 0, 2 * D_S, D_S))
        
    def mouse_button_up(self, event, cur_pos):
        """reacts to mouse button up"""

        d_m_s = self.default_map_scaling
        cur_on_map = self.map_projection(cur_pos)

        # check if the user is zooming on the map
        if cur_pos[0] < D_S:

            # wheel up
            old_scale = self.map_scaling
            if event.button == 4:
                # raise the map scaling up to 5 zooms in
                self.map_scaling *= Z_R
                self.map_scaling = min(self.map_scaling,
                                       d_m_s * (Z_R ** Z_M))
            # wheel down
            elif event.button == 5:
                # lower the map scaling down to default zoom
                self.map_scaling *= 1/Z_R
                self.map_scaling = max(self.map_scaling, d_m_s)

            # if a zoom was performed change the map center
            if event.button == 4 or event.button == 5:
                new_scale = self.map_scaling
                diff = old_scale - new_scale

                # get new map center that allows to zoom on the selected point
                new_cen_x = self.screen_center[0] + cur_on_map[0] * diff
                new_cen_y = self.screen_center[1] - cur_on_map[1] * diff

                # get new map size
                scaling_ratio = self.zoom()
                new_map_size = DASHBOARD_SIZE * scaling_ratio

                # bound the map to the edges of the background layer
                new_x = max(new_cen_x, DASHBOARD_SIZE - new_map_size / 2)
                new_x = min(new_x, new_map_size / 2)
                new_y = max(new_cen_y, DASHBOARD_SIZE - new_map_size / 2)
                new_y = min(new_y, new_map_size / 2)

                # update the map center
                self.screen_center = (new_x, new_y)

                # ask for a redrawn of all layers with the new zoom
                self.g_u_i.to_draw_all()

        # left button released
        if event.button == 2:
            # stop moving map
            if self.moving_map:
                self.moving_map = False

    def mouse_button_down(self, event, cur_pos):
        """rect to mouse button down event"""

        # middle button down
        if event.button == 2:

            # start moving map if not moving and no active input
            if not self.moving_map:
                self.moving_map = True
                self.move_old_cur_position = cur_pos

    def tick(self, cur_pos):
        """react to window regular tick"""

        # check if map is being moved
        if self.moving_map:
            # get the current change in cursor position from the click to now
            cur_pos = pygame.mouse.get_pos()
            diff = g_f.sub_vectors(cur_pos, self.move_old_cur_position)

            # check if the cursor was moved
            if g_f.non_zero_2d_vec(diff):
                # get the new screen center
                new_x = self.screen_center[0] + diff[0]
                new_y = self.screen_center[1] + diff[1]

                # bound the map to the map part of the dashboard
                scaling_ratio = self.zoom()
                new_map_size = DASHBOARD_SIZE * scaling_ratio
                new_x = max(new_x, DASHBOARD_SIZE - new_map_size / 2)
                new_x = min(new_x, + new_map_size / 2)
                new_y = max(new_y, DASHBOARD_SIZE - new_map_size / 2)
                new_y = min(new_y, + new_map_size / 2)

                # update the screen center and ask for a total redraw
                self.move_old_cur_position = cur_pos
                self.screen_center = (new_x, new_y)
                self.g_u_i.to_draw_all()

    def dashboard_projection_pos(self, map_position):
        """project map position on the dashboard screen
        centered in the middle with proper scale"""

        new_x = self.screen_center[0] + map_position[0] * self.map_scaling
        new_y = self.screen_center[1] - map_position[1] * self.map_scaling
        return new_x, new_y

    def dashboard_projection(self, map_object):
        """project map object on the dashboard screen
        centered in the middle with proper scale"""

        return self.dashboard_projection_pos(map_object.pos)

    def map_projection(self, dash_board_position):
        """project dashboard position on the map with proper scale"""
        m_s = self.map_scaling
        inv_x = (dash_board_position[0] - self.screen_center[0]) / m_s
        inv_y = (self.screen_center[1] - dash_board_position[1]) / m_s
        return inv_x, inv_y

    def zoom(self):
        """returns screen zoom ratio"""

        return self.map_scaling / self.default_map_scaling
