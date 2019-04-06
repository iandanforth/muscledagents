import numpy as np
from math import ceil
from .viridis import viridis # NoQA
from .plasma import plasma # NoQA
from .inferno import inferno # NoQA
from .magma import magma # NoQA


def rgba_to_decimal(r, g, b, a):
    dec = rgb_to_decimal(r, g, b)
    dec.append(a / 255)
    return dec


def rgb_to_decimal(r, g, b):
    assert (np.array([r, g, b]) <= 255).all()
    dec = [r / 255, g / 255, b / 255]
    return dec


def get_rgb(colorscale, val):
    assert 0 <= val and val <= 1
    num_colors = len(colorscale)

    index = int(ceil(val * num_colors)) - 1

    return colorscale[index]
