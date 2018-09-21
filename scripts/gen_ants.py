import os
from mjcf import elements as e
from gen_ant import get_ant


def main():
    #########################
    # Level 1
    mujoco = e.Mujoco(
        model="ant-army"
    )

    #########################
    # Level 2
    compiler = e.Compiler(
        angle="degree",
        coordinate="local",
        inertiafromgeom=True
    )
    option = e.Option(
        integrator="RK4",
        timestep=0.01
    )
    size = e.Size(
        njmax=1000,
        nconmax=500,
    )
    custom = e.Custom()
    default = e.Default()
    asset = e.Asset()
    worldbody = e.Worldbody()
    tendon = e.Tendon()
    actuator = e.Actuator()

    mujoco.add_children([
        compiler,
        option,
        size,
        custom,
        default,
        asset,
        worldbody,
        tendon,
        actuator
    ])

    ######################
    # Level 3

    # Custom
    numeric = e.Numeric(
        name="init_qpos",
        data="0.0 0.0 0.55 1.0 0.0 0.0 0.0 0.0 1.0 0.0 -1.0 0.0 -1.0 0.0 1.0",
    )
    custom.add_child(numeric)

    # Default
    d_joint = e.Joint(
        armature=1,
        damping=1,
        limited=True
    )
    d_geom = e.Geom(
        conaffinity=0,
        condim=3,
        density=5.0,
        friction=[1, 0.5, 0.5],
        margin=0.01,
        rgba=[0.8, 0.6, 0.4, 1]
    )
    default.add_children([d_joint, d_geom])

    # Asset
    tex1 = e.Texture(
        builtin="gradient",
        height=100,
        rgb1=[1, 1, 1],
        rgb2=[0, 0, 0],
        type="skybox",
        width=100
    )
    tex2 = e.Texture(
        builtin="flat",
        height=1278,
        mark="cross",
        markrgb=[1, 1, 1],
        name="texgeom",
        random=0.01,
        rgb1=[0.8, 0.6, 0.4],
        rgb2=[0.8, 0.6, 0.4],
        type="cube",
        width=127
    )
    tex3 = e.Texture(
        builtin="checker",
        height=[100],
        name="texplane",
        rgb1=[0, 0, 0],
        rgb2=[0.8, 0.8, 0.8],
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
        tex1,
        tex2,
        tex3,
        mat1,
        mat2,
    ])

    # Worldbody
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
        floor_geom
    ])

    ant_bodies = []
    ant_tendons = []
    ant_actuators = []
    square_side = 3
    for i in range(square_side):
        for j in range(square_side):
            name = "ant_{}_{}".format(i, j)
            y_offset = 2.5 * i
            x_offset = 2.5 * j
            body, tendons, actuators = get_ant(name=name, location=[x_offset, y_offset, 0.75])
            ant_bodies.append(body)
            ant_tendons.extend(tendons)
            ant_actuators.extend(actuators)

    worldbody.add_children(ant_bodies)

    # Tendon
    tendon.add_children(ant_tendons)

    # Actuator
    actuator.add_children(ant_actuators)

    model_xml = mujoco.xml()

    # Output
    out_path = os.path.join(
        "..",
        "muscledagents",
        "envs",
        "mujoco",
        "assets",
        "muscled-ants.xml"
    )
    with open(out_path, 'w') as fh:
        fh.write(model_xml)


if __name__ == '__main__':
    main()
