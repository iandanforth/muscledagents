import os
import utils
from mjcf import elements as e


def main():

    mujoco = e.Mujoco(
        model="arm",
    )
    compiler = e.Compiler(
        angle="degree",
        coordinate="local",
        inertiafromgeom="true",
    )
    default = e.Default(
    )
    option = e.Option(
        integrator="RK4",
        timestep="0.002",
    )
    visual = e.Visual(
    )
    worldbody = e.Worldbody(
    )
    tendon = e.Tendon(
    )
    sensor = e.Sensor(
    )
    actuator = e.Actuator(
    )
    asset = e.Asset(
    )
    mujoco.add_children([
        compiler,
        default,
        option,
        visual,
        worldbody,
        tendon,
        sensor,
        actuator,
        asset,
    ])

    # Standard assets
    utils.populated_ma_asset(asset)

    # Standard floor and lighting
    utils.populate_ma_worldbody(worldbody)


    geom = e.Geom(
        conaffinity="1",
        condim="1",
        contype="1",
        margin="0.001",
        material="geom",
        rgba="0.8 0.6 .4 1",
        solimp=".8 .8 .01",
        solref=".02 1",
    )
    motor = e.Motor(
        ctrllimited="true",
        ctrlrange="-.4 .4",
    )
    default.add_children([
        geom,
        motor,
    ])
    map = e.visual.Map(
        znear="0.02",
    )
    visual.add_children([
        map,
    ])
    # light = e.Light(
    #     cutoff="100",
    #     diffuse="1 1 1",
    #     dir="-0 0 -1.3",
    #     directional="true",
    #     exponent="1",
    #     pos="0 0 1.3",
    #     specular=".1 .1 .1",
    # )
    # floor = e.Geom(
    #     conaffinity="1",
    #     condim="3",
    #     material="MatPlane",
    #     name="floor",
    #     pos="0 0 0",
    #     rgba="0.8 0.9 0.8 1",
    #     size="20 20 .125",
    #     type="plane",
    # )
    block_body = e.Body(
        name="block_body",
        pos="0.0 0.0 3.0",
    )
    worldbody.add_children([
        # light,
        # floor,
        block_body,
    ])
    string = e.Spatial(
        name="string",
        width="0.02",
        rgba=".95 .3 .3 1",
        limited="false",
        range="0.5 2",
        stiffness="3700.0",
        damping="100.0",
    )
    tendon.add_children([
        string,
    ])
    ball_sensor = e.sensor.Force(
        name="ball_sensor",
        site="ball_site",
    )
    sensor.add_children([
        ball_sensor,
    ])
    string_actuator = e.General(
        name="string_actuator",
        tendon="string",
        ctrllimited="true",
        gainprm="2000",
        ctrlrange="-2 0",
    )
    actuator.add_children([
        string_actuator,
    ])
    # texture = e.Texture(
    #     builtin="gradient",
    #     height="100",
    #     rgb1=".4 .5 .6",
    #     rgb2="0 0 0",
    #     type="skybox",
    #     width="100",
    # )
    # texgeom = e.Texture(
    #     builtin="flat",
    #     height="1278",
    #     mark="cross",
    #     markrgb="1 1 1",
    #     name="texgeom",
    #     random="0.01",
    #     rgb1="0.8 0.6 0.4",
    #     rgb2="0.8 0.6 0.4",
    #     type="cube",
    #     width="127",
    # )
    # texplane = e.Texture(
    #     builtin="checker",
    #     height="100",
    #     name="texplane",
    #     rgb1="0 0 0",
    #     rgb2="0.8 0.8 0.8",
    #     type="2d",
    #     width="100",
    # )
    # MatPlane = e.Material(
    #     name="MatPlane",
    #     reflectance="0.5",
    #     shininess="1",
    #     specular="1",
    #     texrepeat="60 60",
    #     texture="texplane",
    # )
    # geom_1 = e.Material(
    #     name="geom",
    #     texture="texgeom",
    #     texuniform="true",
    # )
    # asset.add_children([
    #     texture,
    #     texgeom,
    #     texplane,
    #     MatPlane,
    #     geom_1,
    # ])
    block_geom = e.Geom(
        name="block_geom",
        size="0.2 0.2 0.2",
        type="box",
    )
    box_site = e.Site(
        name="box_site",
        pos="0.0 0.0 -0.2",
        size="0.05",
    )
    ball = e.Body(
        name="ball",
        pos="0.0 0.0 -1.0",
    )
    block_body.add_children([
        block_geom,
        box_site,
        ball,
    ])
    site = e.spatial.Site(
        site="box_site",
    )
    site_1 = e.spatial.Site(
        site="ball_site",
    )
    string.add_children([
        site,
        site_1,
    ])
    ball_joint = e.Joint(
        name="ball_joint",
        pos="0.0 0.0 -0.2",
        type="ball",
        damping="2.0",
    )
    joint = e.Joint(
        type="slide",
        axis="0 0 1",
        damping="2.0",
    )
    joint_1 = e.Joint(
        type="slide",
        axis="0 1 0",
        damping="2.0",
    )
    joint_2 = e.Joint(
        type="slide",
        axis="1 0 0",
        damping="2.0",
    )
    geom_2 = e.Geom(
        pos="0.0 0.0 0.0",
        size="0.2",
        type="sphere",
    )
    ball_site = e.Site(
        name="ball_site",
        pos="0.0 0.0 0.2",
        size="0.05",
    )
    ball.add_children([
        ball_joint,
        joint,
        joint_1,
        joint_2,
        geom_2,
        ball_site,
    ])

    model_xml = mujoco.xml()

    # Output
    out_path = os.path.join(
        "..",
        "muscledagents",
        "envs",
        "mujoco",
        "assets",
        "ballonstring.xml"
    )
    with open(out_path, 'w') as fh:
        fh.write(model_xml)


if __name__ == '__main__':
    main()