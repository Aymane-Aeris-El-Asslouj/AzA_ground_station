from avionics_code.references import global_variables as g_v
from avionics_code.references import parameters as para
from avionics_code.pygui import layer as g_l
from avionics_code.pygui import colors as col

from avionics_code.avionics_gui import drawing_functions as d_f

import time

MESSAGE_DISPLAY_PERIOD = para.MESSAGE_DISPLAY_PERIOD
DASHBOARD_SIZE = para.DASHBOARD_SIZE
D_S = DASHBOARD_SIZE

MT = g_v.MessageType

RED = col.RED
BLACK = col.BLACK
GREY = col.GREY
YELLOW = col.YELLOW
BLUE = col.BLUE
GREEN = col.GREEN


class SystemStatusLayer(g_l.Layer):

    def __init__(self, g_u_i):
        super().__init__(g_u_i, (D_S * 2, D_S))

        # message on screen
        self.message_time = 0
        self.messages = []

        # computation percentages
        self.path_search_percentage = 0
        self.cover_percentage = 0

    def display_message(self, message_1, message_2, level):
        """display a message on screen with
        a certain priority level"""

        # record message time if first
        if len(self.messages) == 0:
            self.message_time = time.time()

        # store the message time
        self.messages.append({"message 1": message_1,
                              "message 2": message_2,
                              "importance level": level})

        # redraw status layer
        self.to_draw = True

    def redraw(self):
        """draws system_status"""
        
        d_c_t = d_f.draw_centered_text

        # draw enum in system status column
        def draw_status_enum(element, status_var, in_name, y):
            
            if element is not None:
                status = getattr(element, status_var)
                in_text = in_name + " " + status.name
                d_c_t(self.surface, self.font, in_text, (1.85, y))

        # draw in system status column
        def draw_status(in_text, y, in_color=BLACK, under=False):

            if under:
                font = self.font_u
            else:
                font = self.font

            d_c_t(self.surface, font, in_text, (1.85, y), in_color)

        draw_status("System Status:", 0.5, under=True)

        d_s_e = draw_status_enum
        d_s_e(g_v.sc, "connection_status", "Server connect:", 1.25)
        d_s_e(g_v.sc, "retrieval_status", "Map download:", 1.75)

        d_s_e(g_v.rf, "connection_status", "Pixhawk connect:", 2.5)
        d_s_e(g_v.rf, "download_status", "Mission download:", 3.0)

        d_s_e(g_v.ms, "generation_status", "Mission generate:", 3.75)
        d_s_e(g_v.mc, "path_computation_status", "Path compute:", 4.25)
        d_s_e(g_v.rf, "upload_status", "Mission upload:", 4.75)

        d_s_e(g_v.rf, "mission_start", "Mission start:", 5.50)
        d_s_e(g_v.rf, "mission_pause", "Mission pause:", 6.0)
        d_s_e(g_v.rf, "go_to_status", "Go orbit position:", 6.5)
        d_s_e(g_v.rf, "camera_gimbal", "Camera Gimbal:", 7.0)

        # draw path search percentages
        draw_status("Search percentages:", 7.5, under=True)
        text_path = str(round(self.path_search_percentage))
        draw_status("Path search: " + text_path + " %", 8.0)
        text_path = str(round(self.cover_percentage))
        draw_status("Cover search: " + text_path + " %", 8.5)

        # draw screen messages
        draw_status("Controller messages:", 9.25, under=True)

        if len(self.messages) > 0:

            # get color of message
            message = self.messages[0]
            if message["importance level"] == MT.DEFAULT:
                color = BLACK
            elif message["importance level"] == MT.ERROR:
                color = RED
            elif message["importance level"] == MT.COMMS:
                color = YELLOW
            elif message["importance level"] == MT.CONTROLLER:
                color = BLUE
            elif message["importance level"] == MT.MISSION_STATE:
                color = GREEN
            else:
                color = GREY

            # draw messages
            draw_status(message["message 1"], 9.75, color)
            draw_status(message["message 2"], 10.25, color)

            # delete message if outdated
            M_D_P = MESSAGE_DISPLAY_PERIOD
            if time.time() - self.message_time > M_D_P:
                self.message_time = time.time()
                # pop all messages if repeated
                while self.messages[0] == message:
                    self.messages.pop(0)
                    if len(self.messages) == 0:
                        break
        else:
            draw_status("//no message", 9.75, GREY)
            draw_status("//no message", 10.25, GREY)

        # draw controller queue
        draw_status("Controller queue:", 11.0, under=True)

        if len(g_v.mc.queue) == 0:
            draw_status("//nothing", 11.5, in_color=GREY)

        off_set = 0
        for index, state in enumerate(g_v.mc.queue):
            # only print states
            if type(state) == g_v.ControllerStatus:

                # print special states with args
                if state == g_v.ControllerStatus.GO_TO:
                    args = g_v.mc.queue[index+1]
                    name = str(state.name)
                    t_x = str(round(args[0][0]))
                    t_y = str(round(args[0][1]))
                    t_z = str(round(args[1]))
                    text = name + ": " + t_x + ", " + t_y + ", " + t_z
                    draw_status(text, 11.5 + off_set * 0.5)
                else:
                    draw_status(str(state.name), 11.5 + off_set * 0.5)

                off_set += 1

    def tick_event(self, cur_pos):
        """react to tick event"""

        self.to_draw = True
