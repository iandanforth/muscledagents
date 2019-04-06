import utils
from utils.colors import rgba_to_decimal
from itertools import chain
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
        mode="track",
        pos=[0, -6, 0.6],
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
        "front_right_leg"+name,
        hip_angle=-45
    )
    front_left_leg, fl_tendons, fl_actuators = get_leg(
        "front_left_leg"+name,
        hip_angle=45,
    )
    back_right_leg, br_tendons, br_actuators = get_leg(
        "back_right_leg"+name,
        hip_angle=-135,
    )
    back_left_leg, bl_tendons, bl_actuators = get_leg(
        "back_left_leg"+name,
        hip_angle=135,
    )

    torso.add_children([
        camera,
        torso_geom,
        joint,
        front_right_leg,
        front_left_leg,
        back_right_leg,
        back_left_leg,
    ])

    # chain() is used for easy commenting out
    tendons = list(chain(
        fr_tendons,
        fl_tendons,
        br_tendons,
        bl_tendons,
    ))
    actuators = list(chain(
        fr_actuators,
        fl_actuators,
        br_actuators,
        bl_actuators,
    ))

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
        name="hip_geom_"+name,
        size=leg_radius,
        type="capsule"
    )
    hip_insertion_x = hip_width * 0.1
    left_flex_hip_insertion = e.Site(
        name="left_flex_hip_insertion_"+name,
        pos=[hip_insertion_x, leg_radius, 0.0],
        size=leg_radius / 2
    )
    right_flex_hip_insertion = e.Site(
        name="right_flex_hip_insertion_"+name,
        pos=[hip_insertion_x, -leg_radius, 0.0],
        size=leg_radius / 2
    )
    hip_joint_geom = e.Geom(
        name="hip_joint_geom_"+name,
        fromto=[hip_width, 0.0, leg_radius, hip_width, 0.0, -leg_radius],
        type="cylinder",
        size=joint_geom_radius,
        rgba=joint_geom_color
    )
    thigh_body = e.Body(
        name="thigh_body_"+name,
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
        name="hip_joint_"+name,
        pos=[0.0, 0.0, 0.0],
        range=[-30, 30],
        type="hinge"
    )
    thigh_geom = e.Geom(
        fromto=[0.0, 0.0, 0.0, thigh_length, 0.0, 0.0],
        name="thigh_geom_"+name,
        size=leg_radius,
        type="capsule"
    )
    upper_insertion_x = thigh_length * 0.3
    flex_thigh_insert_x = thigh_length * 0.8
    quad_u_insertion = e.Site(
        name="quad_u_insertion_"+name,
        pos=[upper_insertion_x, 0.0, leg_radius],
        size=leg_radius / 2
    )
    hamstring_u_insertion = e.Site(
        name="hamstring_u_insertion_"+name,
        pos=[upper_insertion_x, 0.0, -leg_radius],
        size=leg_radius / 2
    )
    left_flex_thigh_insertion = e.Site(
        name="left_flex_thigh_insertion_"+name,
        pos=[flex_thigh_insert_x, leg_radius, 0.0],
        size=leg_radius / 2
    )
    right_flex_thigh_insertion = e.Site(
        name="right_flex_thigh_insertion_"+name,
        pos=[flex_thigh_insert_x, -leg_radius, 0.0],
        size=leg_radius / 2
    )
    knee_geom = e.Geom(
        name="knee_geom_"+name,
        fromto=[thigh_length, leg_radius, 0.0, thigh_length, -leg_radius, 0.0],
        type="cylinder",
        size=joint_geom_radius,
        rgba=joint_geom_color
    )
    knee_top_side = e.Site(
        name="knee_top_side_"+name,
        pos=[thigh_length, 0.0, leg_radius * 2],
        rgba=[0, 0, 0, 0],
        size=leg_radius / 8
    )
    shin_body = e.Body(
        name="shin_body_"+name,
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
        name="knee_joint_"+name,
        pos=[0.0, 0.0, 0.0],
        range=[30, 70],
        type="hinge"
    )
    shin_geom = e.Geom(
        fromto=[0.0, 0.0, 0.0, shin_length, 0.0, 0.0],
        name="shin_geom_"+name,
        size=leg_radius,
        type="capsule"
    )
    lower_insertion_x = shin_length * 0.2
    quad_l_insertion = e.Site(
        name="quad_l_insertion_"+name,
        pos=[lower_insertion_x, 0.0, leg_radius],
        size=leg_radius / 2
    )
    hamstring_l_insertion = e.Site(
        name="hamstring_l_insertion_"+name,
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
    tendon_stiffness = 100
    left_hip_flex = e.Spatial(
        name="left_hip_flex_"+name,
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
        name="right_hip_flex_"+name,
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
        name="quad_"+name,
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
        name="hamstring_"+name,
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
    act_gain = 50  # Max 100N output
    act_ctrl_range = [-1.0, 0.0]
    ctrllimited = "true"
    quad_act = e.General(
        name="quad_act_"+name,
        tendon=quad.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )
    hamstring_act = e.General(
        name="hamstring_act_"+name,
        tendon=hamstring.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )
    right_hip_flex_act = e.General(
        name="right_hip_flex_act_"+name,
        tendon=right_hip_flex.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )
    left_hip_flex_act = e.General(
        name="left_hip_flex_act_"+name,
        tendon=left_hip_flex.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )
    actuators.extend([
        quad_act,
        hamstring_act,
        right_hip_flex_act,
        left_hip_flex_act
    ])

    return hip, tendons, actuators


def main():
    mujoco = e.Mujoco(
        model="ant"
    )

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

    # Standard assets
    utils.populated_ma_asset(asset)

    # Standard floor and lighting
    utils.populate_ma_worldbody(worldbody)

    # Add axes. Useful for debugging
    # utils.add_axes(worldbody)

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
    filename = "muscled-ant.xml"
    utils.save_model(model_xml, filename)


if __name__ == '__main__':
    main()
