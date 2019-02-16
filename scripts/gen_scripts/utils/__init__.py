import os
from .colors import rgb_to_decimal
from mjcf import elements as e


def add_axes(worldbody, axis_length=1.2, axis_radius=0.05):
    """
    Adds a set of axis at the world origin. Helpful for debugging.
    """
    x_axis = e.Geom(
        conaffinity=0,
        name="x_axis",
        pos=[0, 0, 0],
        rgba=[1, 0, 0, 0.5],
        type="cylinder",
        size=axis_radius,
        fromto=[0, 0, 0, axis_length, 0, 0]
    )
    y_axis = e.Geom(
        conaffinity=0,
        name="y_axis",
        pos=[0, 0, 0],
        rgba=[0, 1, 0, 0.5],
        type="cylinder",
        size=axis_radius,
        fromto=[0, 0, 0, 0, axis_length, 0]
    )
    z_axis = e.Geom(
        conaffinity=0,
        name="z_axis",
        pos=[0, 0, 0],
        rgba=[0, 0, 1, 0.5],
        type="cylinder",
        size=axis_radius,
        fromto=[0, 0, 0, 0, 0, axis_length]
    )
    worldbody.add_children([
        x_axis,
        y_axis,
        z_axis
    ])


def populated_ma_asset(asset):
    """
    Adds standard MuscledAgent environment assets into the provided
    Asset() object
    """
    skybox_texture = e.Texture(
        type="skybox",
        fileright="sunny-right.png",
        fileleft="sunny-left.png",
        fileup="sunny-up.png",
        filedown="sunny-down.png",
        filefront="sunny-front.png",
        fileback="sunny-back.png"
    )

    tex2 = e.Texture(
        builtin="flat",
        height=1278,
        mark="cross",
        markrgb=[1, 1, 1],
        name="texgeom",
        random=0.01,
        rgb1=rgb_to_decimal(220, 227, 233),
        rgb2=rgb_to_decimal(231, 235, 246),
        type="cube",
        width=127
    )
    tex3 = e.Texture(
        builtin="checker",
        height=[100],
        name="texplane",
        rgb1=rgb_to_decimal(210, 207, 241),
        rgb2=rgb_to_decimal(110, 163, 230),
        type="2d",
        width=100
    )
    mat1 = e.Material(
        name="MatPlane",
        reflectance=0.5,
        shininess=1,
        specular=1,
        texrepeat=[60, 60],
        texture="texplane"
    )
    mat2 = e.Material(
        name="geom",
        texture="texgeom",
        texuniform=True
    )

    asset.add_children([
        skybox_texture,
        tex2,
        tex3,
        mat1,
        mat2,
    ])


def populate_ma_worldbody(worldbody):
    """
    Adds a standard MuscledAgents floor and global light to the provided
    WorldBody() object.
    """

    light = e.Light(
        cutoff=100,
        diffuse=[1, 1, 1],
        dir=[-0, 0, -1.3],
        directional=True,
        exponent=1,
        pos=[0, 0, 1.3],
        specular=[.1, .1, .1]
    )

    floor_geom = e.Geom(
        conaffinity=1,
        condim=3,
        material="MatPlane",
        name="floor",
        pos=[0, 0, 0],
        rgba=[0.8, 0.9, 0.8, 1],
        size=[40, 40, 40],
        type="plane"
    )

    worldbody.add_children([
        light,
        floor_geom,
    ])


def splitall(path):
    """
    Split a path into a list of all its components

    Credit: Trent Mick
    https://www.oreilly.com/library/view/python-cookbook/0596001673/ch04s16.html
    """
    allparts = []
    while 1:
        parts = os.path.split(path)
        if parts[0] == path:  # sentinel for absolute paths
            allparts.insert(0, parts[0])
            break
        elif parts[1] == path:  # sentinel for relative paths
            allparts.insert(0, parts[1])
            break
        else:
            path = parts[0]
            allparts.insert(0, parts[1])
    return allparts


def get_ma_abspath():
    """
    Returns the absolute path to the muscled agents module
    dir.
    """
    this_abspath = os.path.abspath(__file__)
    parts = splitall(this_abspath)
    parts.reverse()
    i = 0
    # Walk backward through the path to the first instance
    # of 'muscledagents'
    for p in parts:
        if p == "muscledagents":
            break
        i += 1

    # Flip the list again and grab the bits we want
    parts.reverse()
    last_part_ind = len(parts) - i
    desired_pieces = parts[:last_part_ind]
    abspath = os.path.join(*desired_pieces)
    return abspath


def save_model(model_xml, filename):

    basepath = get_ma_abspath()
    out_path = os.path.join(
        basepath,
        "muscledagents",  # Actual module dir under repo dir
        "envs",
        "mujoco",
        "assets",
        filename
    )
    with open(out_path, 'w') as fh:
        fh.write(model_xml)
