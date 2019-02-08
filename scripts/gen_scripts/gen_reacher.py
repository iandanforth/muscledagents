import utils
from mjcf import elements as e


def get_side(fromto, name):
    side_color = [0.3, 0.1, 0.3, 1]
    side = e.Geom(
        conaffinity="0",
        fromto=fromto,
        name=name,
        rgba=side_color,
        size=0.2,
        type="capsule",
    )
    return side


def get_sides():
    # Half Side Length
    hsl = 3
    side_height = 0.1

    sideS = get_side(
        fromto=[-hsl, -hsl, side_height, hsl, -hsl, side_height],
        name="sideS",
    )
    sideE = get_side(
        fromto=[hsl, -hsl, side_height, hsl, hsl, side_height],
        name="sideE",
    )
    sideN = get_side(
        fromto=[-hsl, hsl, side_height, hsl, hsl, side_height],
        name="sideN",
    )
    sideW = get_side(
        fromto=[-hsl, -hsl, side_height, -hsl, hsl, side_height],
        name="sideW",
    )

    return sideS, sideE, sideN, sideW


def main():

    mujoco = e.Mujoco(
        model="reacher",
    )
    compiler = e.Compiler(
        angle="degree",
        inertiafromgeom=True,
    )
    default = e.Default()
    option = e.Option(
        gravity="0 0 -9.81",
        integrator="RK4",
        timestep="0.01",
    )
    worldbody = e.Worldbody()
    tendon = e.Tendon()
    actuator = e.Actuator()
    asset = e.Asset()
    mujoco.add_children([
        compiler,
        default,
        option,
        worldbody,
        tendon,
        actuator,
        asset
    ])

    # Standard assets
    utils.populated_ma_asset(asset)

    # Standard floor and lighting
    utils.populate_ma_worldbody(worldbody)

    default_joint = e.Joint(
        armature=1,
        damping=1,
        limited=True,
    )

    default_geom = e.Geom(
        contype="0",
        friction="1 0.1 0.1",
        rgba="0.7 0.7 0 1",
    )
    default.add_children([
        default_joint,
        default_geom,
    ])

    sideS, sideE, sideN, sideW = get_sides()

    shoulder_body = e.Body(
        name="shoulder_body",
        pos=[0, 0, 0]
    )

    target_body = e.Body(
        name="target_body",
        pos=[0, 0, .1],
    )

    worldbody.add_children([
        sideS,
        sideE,
        sideN,
        sideW,
        shoulder_body,
        target_body,
    ])

    # ARM

    # Shoulder (central pole)
    pole_radius = 0.18
    pole_height = 0.2
    shoulder_geom = e.Geom(
        conaffinity="0",
        contype="0",
        fromto=[0, 0, 0, 0, 0, pole_height],
        name="shoulder_geom",
        rgba="0.9 0.4 0.6 1",
        size=pole_radius,
        type="cylinder",
    )

    upper_arm_radius = 0.05
    side_site_radius = 0.005
    pole_site_offset = pole_radius * 1.2
    left_flex_shoulder_insertion = e.Site(
        name="left_flex_shoulder_insertion",
        pos=[-pole_radius, pole_site_offset, pole_height / 2],
        size=upper_arm_radius / 2
    )
    right_flex_shoulder_insertion = e.Site(
        name="right_flex_shoulder_insertion",
        pos=[-pole_radius, -pole_site_offset, pole_height / 2],
        size=upper_arm_radius / 2
    )

    left_flex_shoulder_side = e.Site(
        name="left_flex_shoulder_side",
        pos=[0.0, pole_site_offset, pole_height / 2],
        size=side_site_radius
    )
    right_flex_shoulder_side = e.Site(
        name="right_flex_shoulder_side",
        pos=[0.0, -pole_site_offset, pole_height / 2],
        size=side_site_radius
    )

    upper_arm_body = e.Body(
        name="upper_arm_body",
        pos="0 0 .1",
    )

    shoulder_body.add_children([
        shoulder_geom,
        left_flex_shoulder_insertion,
        left_flex_shoulder_side,
        right_flex_shoulder_insertion,
        right_flex_shoulder_side,
        upper_arm_body
    ])

    upper_arm_len = 1.0
    upper_arm_geom = e.Geom(
        fromto=[0, 0, 0, upper_arm_len, 0, 0],
        name="upper_arm_geom",
        rgba="0.0 0.2 0.4 1",
        size=upper_arm_radius,
        type="capsule",
    )

    shoulder_joint = e.Joint(
        axis=[0, 0, 1],
        limited=True,
        range=[-100, 100],
        name="shoulder_joint",
        pos=[0, 0, 0],
        type="hinge",
    )

    left_flex_ua_insertion = e.Site(
        name="left_flex_ua_insertion",
        pos=[upper_arm_len / 4, upper_arm_radius, 0.0],
        size=upper_arm_radius / 2
    )
    right_flex_ua_insertion = e.Site(
        name="right_flex_ua_insertion",
        pos=[upper_arm_len / 4, -upper_arm_radius, 0.0],
        size=upper_arm_radius / 2
    )

    left_curl_ua_insertion = e.Site(
        name="left_curl_ua_insertion",
        pos=[upper_arm_len / 2, upper_arm_radius, 0.0],
        size=upper_arm_radius / 2
    )
    right_curl_ua_insertion = e.Site(
        name="right_curl_ua_insertion",
        pos=[upper_arm_len / 2, -upper_arm_radius, 0.0],
        size=upper_arm_radius / 2
    )

    elbow_body = e.Body(
        name="elbow_body",
        pos=[upper_arm_len, 0, 0]
    )

    lower_arm_radius = 0.05
    lower_arm = e.Body(
        name="lower_arm",
        pos="1 0 0",
    )

    upper_arm_body.add_children([
        upper_arm_geom,
        shoulder_joint,
        left_flex_ua_insertion,
        right_flex_ua_insertion,
        left_curl_ua_insertion,
        right_curl_ua_insertion,
        elbow_body,
        lower_arm,
    ])

    joint_geom_radius = upper_arm_radius * 1.1
    joint_geom_color = [0.3, 0.9, 0.3, 0.4]  # Translucent green
    elbow_geom = e.Geom(
        conaffinity="0",
        contype="0",
        fromto=[0, 0, -upper_arm_radius, 0, 0, upper_arm_radius],
        name="elbow_geom",
        rgba=joint_geom_color,
        size=joint_geom_radius,
        type="cylinder",
    )

    elbow_left_side = e.Site(
        name="elbow_left_side",
        pos=[0, joint_geom_radius * 1.1, 0.0],
        size=side_site_radius
    )
    elbow_right_side = e.Site(
        name="elbow_right_side",
        pos=[0, -joint_geom_radius * 1.1, 0.0],
        size=side_site_radius
    )

    elbow_body.add_children([
        elbow_geom,
        elbow_left_side,
        elbow_right_side,
    ])

    elbow_joint = e.Joint(
        axis="0 0 1",
        limited="true",
        name="elbow_joint",
        pos="0 0 0",
        range=[-172, 172],
        type="hinge",
    )

    lower_arm_len = 1.0
    lower_arm_geom = e.Geom(
        fromto=[0, 0, 0, lower_arm_len, 0, 0],
        name="lower_arm_geom",
        rgba="0.0 0.4 0.6 1",
        size=lower_arm_radius,
        type="capsule",
    )

    left_curl_la_insertion = e.Site(
        name="left_curl_la_insertion",
        pos=[lower_arm_len / 4, lower_arm_radius, 0.0],
        size=lower_arm_radius / 2
    )
    right_curl_la_insertion = e.Site(
        name="right_curl_la_insertion",
        pos=[lower_arm_len / 4, -lower_arm_radius, 0.0],
        size=lower_arm_radius / 2
    )

    fingertip_body = e.Body(
        name="fingertip_body",
        pos="1.1 0 0",
    )

    lower_arm.add_children([
        elbow_joint,
        lower_arm_geom,
        left_curl_la_insertion,
        right_curl_la_insertion,
        fingertip_body,
    ])

    fingertip_geom = e.Geom(
        contype="0",
        name="fingertip_geom",
        pos="0 0 0",
        rgba="0.0 0.8 0.6 1",
        size=".1",
        type="sphere",
    )

    fingertip_body.add_children([
        fingertip_geom,
    ])

    # Tendons
    tendon_width = 0.02
    tendon_color = [0.95, 0.3, 0.3, 1]
    tendon_stiffness = 100

    left_flex_tendon = e.Spatial(
        name="left_flex_tendon",
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    lfs1 = e.spatial.Site(site=left_flex_shoulder_insertion.name)
    lf_geom = e.spatial.Geom(
        geom=shoulder_geom.name,
        sidesite=left_flex_shoulder_side.name
    )
    lfs2 = e.spatial.Site(site=left_flex_ua_insertion.name)
    left_flex_tendon.add_children([
        lfs1,
        lf_geom,
        lfs2
    ])

    right_flex_tendon = e.Spatial(
        name="right_flex_tendon",
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    rfs1 = e.spatial.Site(site=right_flex_shoulder_insertion.name)
    rf_geom = e.spatial.Geom(
        geom=shoulder_geom.name,
        sidesite=right_flex_shoulder_side.name
    )
    rfs2 = e.spatial.Site(site=right_flex_ua_insertion.name)
    right_flex_tendon.add_children([
        rfs1,
        rf_geom,
        rfs2
    ])

    left_curl_tendon = e.Spatial(
        name="left_curl_tendon",
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    lcs1 = e.spatial.Site(site=left_curl_ua_insertion.name)
    lc_geom = e.spatial.Geom(
        geom=elbow_geom.name,
        sidesite=elbow_left_side.name
    )
    lcs2 = e.spatial.Site(site=left_curl_la_insertion.name)
    left_curl_tendon.add_children([
        lcs1,
        lc_geom,
        lcs2
    ])

    right_curl_tendon = e.Spatial(
        name="right_curl_tendon",
        width=tendon_width,
        rgba=tendon_color,
        stiffness=tendon_stiffness
    )
    rcs1 = e.spatial.Site(site=right_curl_ua_insertion.name)
    rc_geom = e.spatial.Geom(
        geom=elbow_geom.name,
        sidesite=elbow_right_side.name
    )
    rcs2 = e.spatial.Site(site=right_curl_la_insertion.name)
    right_curl_tendon.add_children([
        rcs1,
        rc_geom,
        rcs2
    ])

    tendon.add_children([
        left_flex_tendon,
        right_flex_tendon,
        left_curl_tendon,
        right_curl_tendon,
    ])

    # Target Puck / Ball

    # How is the ball/puck allowed to move?
    target_joint_x = e.Joint(
        armature="0",
        axis="1 0 0",
        damping="0",
        limited=False,
        name="target_joint_x",
        pos="0 0 0",
        stiffness="0",
        type="slide",
    )
    target_joint_y = e.Joint(
        armature="0",
        axis="0 1 0",
        damping="0",
        limited=False,
        name="target_joint_y",
        pos="0 0 0",
        stiffness="0",
        type="slide",
    )
    target_geom = e.Geom(
        conaffinity="0",
        contype="0",
        name="target_geom",
        pos="0 0 0",
        rgba="0.9 0.2 0.2 1",
        size=".09",
        type="sphere",
    )
    target_body.add_children([
        target_joint_x,
        target_joint_y,
        target_geom,
    ])

    # Actuators
    # TODO: Replace with Muscles once 2.0 is released
    act_gain = 100  # Max 100N output
    act_ctrl_range = [-1.0, 0.0]
    ctrllimited = "true"

    left_flex_act = e.General(
        name="left_flex_act",
        tendon=left_flex_tendon.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )
    right_flex_act = e.General(
        name="right_flex_act",
        tendon=right_flex_tendon.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )

    left_curl_act = e.General(
        name="left_curl_act",
        tendon=left_curl_tendon.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )
    right_curl_act = e.General(
        name="right_curl_act",
        tendon=right_curl_tendon.name,
        ctrllimited=ctrllimited,
        gainprm=act_gain,
        ctrlrange=act_ctrl_range
    )

    actuator.add_children([
        left_flex_act,
        right_flex_act,
        left_curl_act,
        right_curl_act,
    ])

    model_xml = mujoco.xml()

    # Output
    filename = "muscled-reacher.xml"
    utils.save_model(model_xml, filename)


if __name__ == '__main__':
    main()
