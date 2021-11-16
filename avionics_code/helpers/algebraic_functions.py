import math


def distribution_increment(distribution, variation_num):
    """distribute n 1s in an array of m zeros"""

    if binary_increment(distribution) == 1:
        distribution.clear()
        return
    while sum(distribution) != variation_num:
        print(distribution)
        if binary_increment(distribution) == 1:
            distribution.clear()
            break


def binary_increment(distribution):
    """increment a binary list with returning overflow"""

    carry = 1
    for index in range(len(distribution)):
        distribution[index] += carry
        if distribution[index] == 2:
            distribution[index] = 0
            carry = 1
        else:
            carry = 0
    return carry


def increment_bound_var(path_vector, distribution, max_per_group, variation_num):
    """increment vector till it has enough variation"""

    if increment_bound(path_vector, distribution, max_per_group) == 1:
        path_vector.clear()
        return

    while sum(bool(index) for index in path_vector) != variation_num:
        if increment_bound(path_vector, distribution, max_per_group) == 1:
            path_vector.clear()
            break


def increment_bound(path_vector, distribution, max_per_group):
    """increment a vector at the spots indicated by the distribution with max values at each point"""

    carry = 1
    for index in range(len(path_vector)):
        if distribution[index] == 1:
            path_vector[index] += carry
            if path_vector[index] == max_per_group[index]:
                path_vector[index] = 0
                carry = 1
            else:
                carry = 0
    return carry


def clamp(v):
    """confine a color to rgb spectrum"""

    if v < 0:
        return 0
    if v > 255:
        return 255
    return int(v)


class rgb_rotate(object):
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
