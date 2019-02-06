from mjcf import elements as e
from random import random, uniform
from colors import get_rgb, viridis


def get_cubes():
    square_count = 10
    colorscale = viridis
    colorscale.reverse()
    cubes = []
    for i in range(square_count):
        for j in range(square_count):
            x = i + random()
            y = j + random()
            min_side = 0.1
            max_side = 0.5
            side_range = max_side - min_side
            side = uniform(min_side, max_side)
            z = side * 2
            color_point = (side - min_side) / side_range
            rgb = get_rgb(colorscale, color_point)
            alpha = 1 - (color_point / 10)
            rgba = rgb + [alpha]
            cube = get_cube(x, y, z, side, rgba)
            cubes.append(cube)

    return cubes


def get_cube(x=0, y=0, z=1, size=0.2, rgba=[0.5, 0.5, 0.5, 1]):

    body = e.Body(
        pos=[x, y, z]
    )
    freejoint = e.Freejoint()
    geom = e.Geom(
        type="box",
        size=[size, size, size],
        rgba=rgba
    )

    body.add_children([
        freejoint,
        geom
    ])

    return body
