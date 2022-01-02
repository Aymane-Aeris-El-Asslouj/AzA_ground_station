from utility_functions import geometrical_functions as g_f
from utility_functions import algebraic_functions as a_f

import pygame


def emp():
    """empty function"""
    
    pass


GREY = (100, 100, 100)
EMP = (emp, ())


def call(func_with_args):
    """call function with arguments"""

    func_with_args[0](*func_with_args[1])


class Layer:
    """parent class for GUI layers, contains
    a surface that can be redrawn"""

    def __init__(self, g_u_i, size):

        """defines if the layer only takes up
        half the window"""
        self.size = size

        # initialize the layer's surface
        self.surface = None
        self.reset()

        # store reference to GUI window
        self.g_u_i = g_u_i
        self.font = g_u_i.font
        self.font_u = g_u_i.font_u

        # stores whether the surface needs to be redrawn
        self.to_draw = False

        """list of objects stored inside the layer"""
        self.layer_objects = {}

    def reset(self):
        """resets the layer's surface"""

        self.surface = pygame.Surface(self.size, pygame.SRCALPHA)

    def update(self):
        """update the current layer resetting
        it and redrawing it"""

        # reset surface
        self.reset()

        # redraw surface
        self.redraw()

        # redraw objects of layer and add blit their surface
        for object_key in self.layer_objects:
            layer_obj = self.layer_objects[object_key]
            layer_obj.redraw()

    def redraw(self):
        """redraws the layer's surface"""

        pass

    def tick_event(self, cur_pos):
        """react to window regular tick"""

        self.tick(cur_pos)

        # transfer tick event to objects
        for object_key in self.layer_objects:
            self.layer_objects[object_key].tick_event(cur_pos)

    def tick(self, cur_pos):
        """react to tick event"""

        pass

    def event(self, cur_pos, event):
        """reacts to a certain window event"""

        if event.type == pygame.MOUSEBUTTONDOWN:
            self.mouse_button_down(event, cur_pos)

        elif event.type == pygame.MOUSEBUTTONUP:
            self.mouse_button_up(event, cur_pos)

        elif event.type == pygame.KEYDOWN:
            self.key_down(event, cur_pos)

        # transfer events to objects
        for object_key in self.layer_objects:
            self.layer_objects[object_key].event(event)

    def mouse_button_down(self, event, cur_pos):
        """reacts to mouse button down event"""

        pass

    def mouse_button_up(self, event, cur_pos):
        """reacts to mouse button up event"""

        pass

    def key_down(self, event, cur_pos):
        """reacts to key down event"""

        pass


class LayerObject:
    """parent class for layer objects,
    draws on a layer surface"""

    def __init__(self, layer):

        # store reference to GUI window
        self.layer = layer
        self.font = layer.font

    def redraw(self):
        """redraws the layer's surface"""

        pass

    def tick_event(self, cur_pos):
        """react to window regular tick"""

        pass

    def event(self, event):
        """reacts to a certain window event"""

        # get cursor position on screen and on map
        cur_pos = pygame.mouse.get_pos()

        if event.type == pygame.MOUSEBUTTONDOWN:
            self.mouse_button_down(event, cur_pos)

        elif event.type == pygame.MOUSEBUTTONUP:
            self.mouse_button_up(event, cur_pos)

        elif event.type == pygame.KEYDOWN:
            self.key_down(event, cur_pos)

    def mouse_button_down(self, event, cur_pos):
        """reacts to mouse button down event"""

        pass

    def mouse_button_up(self, event, cur_pos):
        """reacts to mouse button up event"""

        pass

    def key_down(self, event, cur_pos):
        """reacts to key down event"""

        pass


class Label(LayerObject):
    """label draws text on layer"""

    def __init__(self, layer, text, color, pos):
        super().__init__(layer)
        self.text = text
        self.color = color
        self.pos = pos

    def redraw(self):
        """draw text on layer"""

        label = self.font.render(self.text, True, self.color)
        label_pos = label.get_rect(midleft=self.pos)
        self.layer.surface.blit(label, label_pos)


class Button(LayerObject):
    """draws circle on layer and has 3 click functions"""

    def __init__(self, layer, color, pos, size, text,
                 left=EMP, middle=EMP, right=EMP):
        super().__init__(layer)

        # button parameters
        self.color = color
        self.pos = pos
        self.size = size
        self.text = text

        # change color when hovering
        self.hover = False

        # for shining after click
        self.shine = False

        # click actions
        self.left_action = lambda: call(left)
        self.middle_action = lambda: call(middle)
        self.right_action = lambda: call(right)

    def mouse_button_down(self, event, cur_pos):
        """react to mouse button down event"""

        # check if cursor is inside button
        if g_f.distance_2d(cur_pos, self.pos) < self.size:

            # left click
            if event.button == 1:
                self.left_action()
                self.shine = 5

            # middle click
            if event.button == 2:
                self.middle_action()
                self.shine = 5

            # right click
            if event.button == 3:
                self.right_action()
                self.shine = 5

    def hover_shine(self, pos):
        """draws hover and click circles"""

        hover_shine_size = self.size * 1.2

        if self.shine == 0:
            # draw hover circle if hovering
            if self.hover:
                pygame.draw.circle(self.layer.surface,
                                   a_f.inv_rgb(self.color),
                                   pos, hover_shine_size)
        else:
            # draw shine circle after click
            surf = pygame.Surface(self.layer.size, pygame.SRCALPHA)
            surf.set_alpha(int(255/self.shine))
            pygame.draw.circle(surf, a_f.inv_rgb(self.color),
                               pos, hover_shine_size)
            self.layer.surface.blit(surf, (0, 0))

    def redraw(self):
        """draw circle on layer"""

        # draw hover and shine circles
        self.hover_shine(self.pos)

        # draw button circle
        pygame.draw.circle(self.layer.surface, self.color,
                           self.pos, self.size)

        # draw text on layer
        label = self.font.render(self.text, True,
                                 self.color)
        pos = g_f.add_vectors(self.pos, (self.size * 2, 0))
        label_pos = label.get_rect(midleft=pos)
        self.layer.surface.blit(label, label_pos)

    def tick_event(self, cur_pos):
        """react to window tick event"""

        # change shine status
        if self.shine > 0:
            self.shine -= 1
            self.layer.to_draw = True

        # change hover status when cursor inside
        # or outside button
        if g_f.distance_2d(cur_pos, self.pos) < self.size:
            if not self.hover:
                self.hover = True
                self.layer.to_draw = True
        else:
            if self.hover:
                self.hover = False
                self.layer.to_draw = True


class InputButton(Button):
    """button with radio size toggle function, middle deactivation,
    and right action with arguments"""

    def __init__(self, layer, color, pos, size, text,
                 left=EMP, middle=EMP, right=EMP):

        """list of other buttons in the same radio group,
        such that only one can be on at the same time"""
        self.radio = []

        self.on = False
        self.activated = True

        # if active, switch its on state
        def l_action():
            if self.activated:
                if self.on:
                    self.turn_off()
                else:
                    self.turn_on()
                self.layer.to_draw = True
            call(left)

        # switch activation state
        def m_action():
            if self.activated:
                self.deactivate()
            else:
                self.activate()
            self.layer.to_draw = True
            call(middle)

        # call button constructor
        super().__init__(layer, color, pos, size, text,
                         (l_action, ()), (m_action, ()), right)

    def turn_on(self):
        """switch mode to on and others' to off"""

        if not self.on:
            self.on = True
            self.size *= 5/4

            for button in self.radio:
                if button is not self:
                    button.turn_off()

    def turn_off(self):
        """switch mode to off"""

        if self.on:
            self.on = False
            self.size *= 4/5

    def activate(self):
        """activates button"""

        if not self.activated:
            self.activated = True

    def deactivate(self):
        """activates button"""

        if self.activated:
            self.activated = False
            self.turn_off()

    def redraw(self):
        """draw self with text in color if active
        or grey otherwise"""

        # draw hover and shine circles
        self.hover_shine(self.pos)

        # draw button and create label for text
        if self.activated:
            color = self.color
        else:
            color = GREY
        pygame.draw.circle(self.layer.surface, color,
                           self.pos, self.size)
        label = self.font.render(self.text, True, color)

        # draw text on layer
        pos = g_f.add_vectors(self.pos, (self.size * 2, 0))
        label_pos = label.get_rect(midleft=pos)
        self.layer.surface.blit(label, label_pos)


class ToggleButton(Button):
    """button with color toggle function"""

    def __init__(self, layer, color, pos, size, text,
                 left=EMP, middle=EMP, right=EMP):

        self.on = True

        # if active, switch its on state
        def l_action():
            if self.on:
                self.turn_off()
            else:
                self.turn_on()
            self.layer.to_draw = True
            call(left)

        # call button constructor
        super().__init__(layer, color, pos, size, text,
                         (l_action, ()), middle, right)

    def turn_on(self):
        """switch mode to on and others' to off"""

        if not self.on:
            self.on = True

    def turn_off(self):
        """switch mode to off"""

        if self.on:
            self.on = False

    def redraw(self):
        """draw self with text in color if active
        or grey otherwise"""

        # draw hover and shine circles
        self.hover_shine(self.pos)

        # draw hover circle if hovering
        if self.hover:
            pygame.draw.circle(self.layer.surface,
                               a_f.inv_rgb(self.color),
                               self.pos, self.size*1.2)

        # draw button and create label for text
        if self.on:
            color = self.color
        else:
            color = GREY
        pygame.draw.circle(self.layer.surface, color,
                           self.pos, self.size)
        label = self.font.render(self.text, True, color)

        # draw text on layer
        pos = g_f.add_vectors(self.pos, (self.size * 2, 0))
        label_pos = label.get_rect(midleft=pos)
        self.layer.surface.blit(label, label_pos)


class SettingInputButton(InputButton):
    """input button that saves state to settings"""

    def __init__(self, layer, color, pos, size, text, setting,
                 left=EMP, middle=EMP, right=EMP):
        super().__init__(layer, color, pos, size,
                         text, left, middle, right)

        # store setting name
        self.setting = setting

        # create or load setting
        settings = self.layer.g_u_i.settings
        if self.setting not in settings:
            settings[self.setting] = {
                "on": False,
                "activated": True
            }
        if settings[self.setting]["on"]:
            self.turn_on()
        if not settings[self.setting]["activated"]:
            self.deactivate()

    def turn_on(self):
        """switch mode to on and others' to off"""

        if not self.on:
            self.on = True
            self.size *= 5/4

            for button in self.radio:
                if button is not self:
                    button.turn_off()

        self.layer.g_u_i.settings[self.setting]["on"] = self.on

    def turn_off(self):
        """switch mode to off"""

        if self.on:
            self.on = False
            self.size *= 4/5

        self.layer.g_u_i.settings[self.setting]["on"] = self.on

    def activate(self):
        """activates button"""

        if not self.activated:
            self.activated = True

        self.layer.g_u_i.settings[self.setting]["activated"] = self.activated

    def deactivate(self):
        """activates button"""

        if self.activated:
            self.activated = False
            self.turn_off()

        self.layer.g_u_i.settings[self.setting]["activated"] = self.activated


class SettingToggleButton(ToggleButton):
    """toggle button that saves state to settings"""

    def __init__(self, layer, color, pos, size, text, setting,
                 left=EMP, middle=EMP, right=EMP):
        super().__init__(layer, color, pos, size,
                         text, left, middle, right)

        # store setting name
        self.setting = setting

        # load or create setting
        settings = self.layer.g_u_i.settings
        if self.setting not in settings:
            settings[self.setting] = {
                "on": True,
            }
        if not settings[self.setting]["on"]:
            self.turn_off()

    def turn_on(self):
        """switch mode to on and others' to off"""

        if not self.on:
            self.on = True

        self.layer.g_u_i.settings[self.setting]["on"] = self.on

    def turn_off(self):
        """switch mode to off"""

        if self.on:
            self.on = False

        self.layer.g_u_i.settings[self.setting]["on"] = self.on


class DragAndDrop(Button):
    """Object that can be dragged around"""

    def __init__(self, layer, color, pos, size, text):

        # store position
        super().__init__(layer, color, pos, size, text)

        # variables for dragging
        self.initial_cur_pos = None
        self.initial_pos = None
        self.dragged = False

    def mouse_button_down(self, event, cur_pos):
        """reacts to mouse button down event"""

        super().mouse_button_down(event, cur_pos)

        # start dragging if clicked on
        if event.button == 1:
            if g_f.distance_2d(cur_pos, self.pos) < self.size:
                self.dragged = True
                self.initial_cur_pos = cur_pos
                self.initial_pos = self.pos

    def mouse_button_up(self, event, cur_pos):
        """react to mouse button up event"""

        super().mouse_button_up(event, cur_pos)

        # stop dragging if click released
        if event.button == 1:
            self.dragged = False

    def tick_event(self, cur_pos):
        """react to tick event"""

        super().tick_event(cur_pos)

        # update position and shine during drag
        if self.dragged:
            self.shine = 5
            cur_diff = g_f.sub_vectors(cur_pos, self.initial_cur_pos)
            self.pos = g_f.add_vectors(self.initial_pos, cur_diff)
