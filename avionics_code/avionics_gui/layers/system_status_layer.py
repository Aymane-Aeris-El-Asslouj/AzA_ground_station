from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.pygui import layer as g_l

from .. import drawing_functions as d_f

import time

MESSAGE_DISPLAY_PERIOD = para.MESSAGE_DISPLAY_PERIOD
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE


class SystemStatusLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S * 2, D_S))

        # message on screen
        self.message_1 = ""
        self.message_2 = ""
        self.message_color = 0, 0, 0
        self.message_time = 0

        # computation percentages
        self.path_search_percentage = 0
        self.cover_percentage = 0

    def display_message(self, message_1, message_2, level):
        """display a message on screen with a certain priority level"""

        self.message_time = time.time()

        self.message_1 = message_1
        self.message_2 = message_2
        if level == 0:
            self.message_color = 0, 0, 0
        elif level == 1:
            self.message_color = 255, 0, 0

        self.to_draw = True

    def redraw(self):
        """draws system_status"""

        def draw_status_enum(surf, element, status_var, name, y):
            if element is not None:
                d_f.draw_centered_text(surf, self.font, name + " " + str(status_var.name), (1.85, y))

        def draw_status(surf, element, status_var, name, state_list, y):
            if element is not None:
                for index, state in enumerate(state_list):
                    if status_var == index:
                        d_f.draw_centered_text(surf, self.font, name + " " + state, (1.85, y))

        draw_status_enum(self.surface, g_v.sc, g_v.sc.connection_status, "Fake Server connection:", 0.5)
        draw_status_enum(self.surface, g_v.mp, g_v.sc.retrieval_status, "Map download:", 1.0)

        states = ["disconnected", "connected"]
        draw_status(self.surface, g_v.rf, g_v.rf.connection_status, "Pixhawk:", states, 1.75)
        draw_status_enum(self.surface, g_v.rf, g_v.rf.parameters_status, "Parameters upload:", 2.25)
        draw_status_enum(self.surface, g_v.rf, g_v.rf.download_status, "Mission download:", 2.75)

        draw_status_enum(self.surface, g_v.ms, g_v.ms.generation_status, "Mission generation:", 3.5)
        draw_status_enum(self.surface, g_v.mc, g_v.mc.path_computation_status, "Path computation:", 4.0)
        draw_status_enum(self.surface, g_v.rf, g_v.rf.upload_status, "Mission upload:", 4.5)

        draw_status_enum(self.surface, g_v.rf, g_v.rf.mission_start, "Mission start:", 5.25)
        draw_status_enum(self.surface, g_v.rf, g_v.rf.mission_pause, "Mission pause:", 5.75)
        draw_status_enum(self.surface, g_v.rf, g_v.rf.go_to_status, "Go orbit position:", 6.25)

        # draw message on screen
        M_D_P = MESSAGE_DISPLAY_PERIOD
        if time.time() - self.message_time < M_D_P or self.message_color == (255, 0, 0):
            color = self.message_color
        else:
            color = (60, 60, 60)

        d_f.draw_centered_text(self.surface, self.font, str(g_v.mc.action.name), (1.85, 12.5), (0, 0, 0))
        text_path = "path search: " + str(round(self.path_search_percentage)) + " %"
        d_f.draw_centered_text(self.surface, self.font, text_path, (1.85, 13.0), (0, 0, 0))
        text_path = "cover search: " + str(round(self.cover_percentage)) + " %"
        d_f.draw_centered_text(self.surface, self.font, text_path, (1.85, 13.5), (0, 0, 0))

        d_f.draw_centered_text(self.surface, self.font, self.message_1, (1.85, 14.0), color)
        d_f.draw_centered_text(self.surface, self.font, self.message_2, (1.85, 14.5), color)
