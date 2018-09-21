import os
from mjcf import elements as e


def get_ant(name="ant1", location=[0, 0, 0.75]):

    if name != "":
        name = "_{}".format(name)
    
    torso = e.Body(
        name="torso"+name,
        pos=location,
    )

    # Torso
    camera = e.Camera(
        name="track"+name,
        mode="trackcom",
        pos=[0, -3, 0.3],
        xyaxes=[1, 0, 0, 0, 0, 1]
    )
    torso_geom = e.Geom(
        name="torso_geom"+name,
        pos=[0, 0, 0],
        size=0.25,
        type="sphere",
        rgba=None
    )
    joint = e.Joint(
        armature=0,
        damping=0,
        limited=False,
        margin=0.01,
        name="root"+name,
        pos=[0, 0, 0],
        type="free"
    )
    front_right_leg, fr_tendons, fr_actuators = get_leg(
        "front_right_leg"+name
    )
    front_left_leg, fl_tendons, fl_actuators = get_leg(
        "front_left_leg"+name,
        hip_angle=90,
    )
    back_left_leg, bl_tendons, bl_actuators = get_leg(
        "back_left_leg"+name,
        hip_angle=180,
    )
    back_right_leg, br_tendons, br_actuators = get_leg(
        "back_right_leg"+name,
        hip_angle=270,
    )
    torso.add_children([
        camera,
        torso_geom,
        joint,
        front_right_leg,
        front_left_leg,
        back_left_leg,
        back_right_leg
    ])

    tendons = fr_tendons + fl_tendons + bl_tendons + br_tendons
    actuators = fr_actuators + fl_actuators + bl_actuators + br_actuators

    return torso, tendons, actuators


def get_leg(
    name,
    hip_width=0.2,
    hip_angle=0.0,
    thigh_length=0.4,
    shin_length=0.6,
    leg_radius=0.08
):

    joint_geom_radius = leg_radius * 1.1  # For tendons
    joint_geom_color = [0.3, 0.9, 0.3, 0.4]  # Translucent green

    hip = e.Body(
        name=name,
        pos=[0, 0, 0],
        euler=[0, 0, hip_angle]
    )
    hip_geom = e.Geom(
        fromto=[0, 0, 0, hip_width, 0, 0],
        name="hip_geom"+name,
        size=leg_radius,
        type="capsule"
    )
    hip_insertion_x = hip_width * 0.1
    left_flex_hip_insertion = e.Site(
        name="left_flex_hip_insertion"+name,
        pos=[hip_insertion_x, leg_radius, 0.0],
        size=leg_radius / 2
    )
    right_flex_hip_insertion = e.Site(
        name="right_flex_hip_insertion"+name,
        pos=[hip_insertion_x, -leg_radius, 0.0],
        size=leg_radius / 2
    )
    hip_joint_geom = e.Geom(
        name="hip_joint_geom"+name,
        fromto=[hip_width, 0.0, leg_radius, hip_width, 0.0, -leg_radius],
        type="cylinder",
        size=joint_geom_radius,
        rgba=joint_geom_color
    )
    thigh_body = e.Body(
        name="thigh_body"+name,
        pos=[hip_width, 0, 0]
    )
    hip.add_children([
        hip_geom,
        left_flex_hip_insertion,
        right_flex_hip_insertion,
        hip_joint_geom,
        thigh_body
    ])

    # thigh_body
    hip_joint = e.Joint(
        axis=[0, 0, 1],
        name="hip_joint"+name,
        pos=[0.0, 0.0, 0.0],
        range=[-30, 30],
        type="hinge"
    )
    thigh_geom = e.Geom(
        fromto=[0.0, 0.0, 0.0, thigh_length, 0.0, 0.0],
        name="thigh_geom"+name,
        size=leg_radius,
        type="capsule"
    )
    upper_insertion_x = thigh_length * 0.3
    flex_thigh_insert_x = thigh_length * 0.8
    quad_u_insertion = e.Site(
        name="quad_u_insertion"+name,
        pos=[upper_insertion_x, 0.0, leg_radius],
        size=leg_radius / 2
    )
    hamstring_u_insertion = e.Site(
        name="hamstring_u_insertion"+name,
        pos=[upper_insertion_x, 0.0, -leg_radius],
        size=leg_radius / 2
    )
    left_flex_thigh_insertion = e.Site(
        name="left_flex_thigh_insertion"+name,
        pos=[flex_thigh_insert_x, leg_radius, 0.0],
        size=leg_radius / 2
    )
    right_flex_thigh_insertion = e.Site(
        name="right_flex_thigh_insertion"+name,
        pos=[flex_thigh_insert_x, -leg_radius, 0.0],
        size=leg_radius / 2
    )
    knee_geom = e.Geom(
        name="knee_geom"+name,
        fromto=[thigh_length, leg_radius, 0.0, thigh_length, -leg_radius, 0.0],
        type="cylinder",
        size=joint_geom_radius,
        rgba=joint_geom_color
    )
    knee_top_side = e.Site(
        name="knee_top_side"+name,
        pos=[thigh_length, 0.0, leg_radius * 2],
        size=leg_radius / 2
    )
    shin_body = e.Body(
        name="shin_body"+name,
        pos=[thigh_length, 0.0, 0]
    )
    thigh_body.add_children([
        hip_joint,
        thigh_geom,
        left_flex_thigh_insertion,
        right_flex_thigh_insertion,
        quad_u_insertion,
        hamstring_u_insertion,
        knee_geom,
        knee_top_side,
        shin_body
    ])

    # shin_body
    knee_joint = e.Joint(
        axis=[0, 1, 0],
        name="knee_joint"+name,
        pos=[0.0, 0.0, 0.0],
        range=[30, 70],
        type="hinge"
    )
    shin_geom = e.Geom(
        fromto=[0.0, 0.0, 0.0, shin_length, 0.0, 0.0],
        name="shin_geom"+name,
        size=leg_radius,
        type="capsule"
    )
    lower_insertion_x = shin_length * 0.2
    quad_l_insertion = e.Site(
        name="quad_l_insertion"+name,
        pos=[lower_insertion_x, 0.0, leg_radius],
        size=leg_radius / 2
    )
    hamstring_l_insertion = e.Site(
        name="hamstring_l_insertion"+name,
        pos=[lower_insertion_x, 0.0, -leg_radius],
        size=leg_radius / 2
    )
    shin_body.add_children([
        knee_joint,
        shin_geom,
        quad_l_insertion,
        hamstring_l_insertion
    ])

    # Tendons
    tendons = []
    tendon_width = 0.04
    tendon_color = [0.95, 0.3, 0.3, 1]
    tendon_stiffness = 1000
    left_hip_flex = e.Spatial(
        name="left_hip_flex"+name,
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    lhs1 = e.spatial.Site(site=left_flex_hip_insertion.name)
    lh_geom = e.spatial.Geom(geom=hip_joint_geom.name)
    lhs2 = e.spatial.Site(site=left_flex_thigh_insertion.name)
    left_hip_flex.add_children([
        lhs1,
        lh_geom,
        lhs2
    ])
    right_hip_flex = e.Spatial(
        name="right_hip_flex"+name,
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    rhs1 = e.spatial.Site(site=right_flex_hip_insertion.name)
    rh_geom = e.spatial.Geom(geom=hip_joint_geom.name)
    rhs2 = e.spatial.Site(site=right_flex_thigh_insertion.name)
    right_hip_flex.add_children([
        rhs1,
        rh_geom,
        rhs2
    ])
    quad = e.Spatial(
        name="quad"+name,
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    qs1 = e.spatial.Site(site=quad_u_insertion.name)
    q_geom = e.spatial.Geom(
        geom=knee_geom.name,
        sidesite=knee_top_side.name
    )
    qs2 = e.spatial.Site(site=quad_l_insertion.name)
    quad.add_children([
        qs1,
        q_geom,
        qs2
    ])
    hamstring = e.Spatial(
        name="hamstring"+name,
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    hs1 = e.spatial.Site(site=hamstring_u_insertion.name)
    h_geom = e.spatial.Geom(geom=knee_geom.name)
    hs2 = e.spatial.Site(site=hamstring_l_insertion.name)
    hamstring.add_children([
        hs1,
        h_geom,
        hs2
    ])
    tendons.extend([
        left_hip_flex,
        right_hip_flex,
        quad,
        hamstring
    ])

    # Actuators
    # TODO: Replace with Muscles once 2.0 is released
    actuators = []
    quad_act = e.General(
        name="quad_act"+name,
        tendon=quad.name,
        ctrllimited="true",
        gainprm="50",
        ctrlrange="0 1"
    )
    hamstring_act = e.General(
        name="hamstring_act"+name,
        tendon=hamstring.name,
        ctrllimited="true",
        gainprm="50",
        ctrlrange="0 1"
    )
    right_hip_flex_act = e.General(
        name="right_hip_flex_act"+name,
        tendon=right_hip_flex.name,
        ctrllimited="true",
        gainprm="50",
        ctrlrange="0 1"
    )
    left_hip_flex_act = e.General(
        name="left_hip_flex_act"+name,
        tendon=left_hip_flex.name,
        ctrllimited="true",
        gainprm="50",
        ctrlrange="0 1"
    )
    actuators.extend([
        quad_act,
        hamstring_act,
        right_hip_flex_act,
        left_hip_flex_act
    ])

    return hip, tendons, actuators


def main():
    #########################
    # Level 1
    mujoco = e.Mujoco(
        model="ant"
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
    body, tendons, actuators = get_ant(name="")
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
        "muscled-ant.xml"
    )
    with open(out_path, 'w') as fh:
        fh.write(model_xml)


if __name__ == '__main__':
    main()
