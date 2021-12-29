import math


def inv_rgb(color):
    """inverts RGB color"""

    return 255-color[0], 255-color[1], 255-color[2]


def clamp(v):
    """confine a color to rgb spectrum"""

    if v < 0:
        return 0
    if v > 255:
        return 255
    return int(v)


class RGBRotate(object):
    """shifter that changes the hue of a color"""

    def __init__(self):
        self.matrix = [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]

    def set_hue_rotation(self, degrees):
        cos_a = math.cos(math.radians(degrees))
        sin_a = math.sin(math.radians(degrees))
        self.matrix[0][0] = cos_a + (1.0 - cos_a) / 3.0
        self.matrix[0][1] = 1. / 3. * (1.0 - cos_a) - math.sqrt(1. / 3.) * sin_a
        self.matrix[0][2] = 1. / 3. * (1.0 - cos_a) + math.sqrt(1. / 3.) * sin_a
        self.matrix[1][0] = 1. / 3. * (1.0 - cos_a) + math.sqrt(1. / 3.) * sin_a
        self.matrix[1][1] = cos_a + 1. / 3. * (1.0 - cos_a)
        self.matrix[1][2] = 1. / 3. * (1.0 - cos_a) - math.sqrt(1. / 3.) * sin_a
        self.matrix[2][0] = 1. / 3. * (1.0 - cos_a) - math.sqrt(1. / 3.) * sin_a
        self.matrix[2][1] = 1. / 3. * (1.0 - cos_a) + math.sqrt(1. / 3.) * sin_a
        self.matrix[2][2] = cos_a + 1. / 3. * (1.0 - cos_a)

    def apply(self, r, g, b):
        rx = r * self.matrix[0][0] + g * self.matrix[0][1] + b * self.matrix[0][2]
        gx = r * self.matrix[1][0] + g * self.matrix[1][1] + b * self.matrix[1][2]
        bx = r * self.matrix[2][0] + g * self.matrix[2][1] + b * self.matrix[2][2]
        return clamp(rx), clamp(gx), clamp(bx)
