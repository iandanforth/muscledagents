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
        angle="radian",
        inertiafromgeom="true",
    )
    default = e.Default()
    option = e.Option(
        gravity="0 0 -9.81",
        integrator="RK4",
        timestep="0.01",
    )
    worldbody = e.Worldbody()
    actuator = e.Actuator()
    asset = e.Asset()
    mujoco.add_children([
        compiler,
        default,
        option,
        worldbody,
        actuator,
        asset
    ])

    # Standard assets
    utils.populated_ma_asset(asset)

    # Standard floor and lighting
    utils.populate_ma_worldbody(worldbody)

    default_joint = e.Joint(
        armature="1",
        damping="1",
        limited="true",
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
        pos="1 -1 .1",
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
    pole_radius = 0.1
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

    upper_arm_radius = 0.1
    left_flex_shoulder_insertion = e.Site(
        name="left_flex_shoulder_insertion",
        pos=[0.0, pole_radius, pole_height / 2],
        size=upper_arm_radius / 2
    )
    right_flex_shoulder_insertion = e.Site(
        name="right_flex_shoulder_insertion",
        pos=[0.0, -pole_radius, pole_height / 2],
        size=upper_arm_radius / 2
    )

    upper_arm_body = e.Body(
        name="upper_arm_body",
        pos="0 0 .1",
    )

    shoulder_body.add_children([
        shoulder_geom,
        left_flex_shoulder_insertion,
        right_flex_shoulder_insertion,
        upper_arm_body
    ])

    upper_arm_geom = e.Geom(
        fromto="0 0 0 1 0 0",
        name="upper_arm_geom",
        rgba="0.0 0.4 0.6 1",
        size=".1",
        type="capsule",
    )

    shoulder_joint = e.Joint(
        axis="0 0 1",
        limited="false",
        name="shoulder_joint",
        pos="0 0 0",
        type="hinge",
    )

    lower_arm = e.Body(
        name="lower_arm",
        pos="0.1 0 0",
    )

    upper_arm_body.add_children([
        upper_arm_geom,
        shoulder_joint,
        lower_arm,
    ])

    elbow_joint = e.Joint(
        axis="0 0 1",
        limited="true",
        name="elbow_joint",
        pos="0 0 0",
        range="-3.0 3.0",
        type="hinge",
    )
    lower_arm_geom = e.Geom(
        fromto="0 0 0 1 0 0",
        name="lower_arm_geom",
        rgba="0.0 0.4 0.6 1",
        size=".1",
        type="capsule",
    )
    fingertip_body = e.Body(
        name="fingertip_body",
        pos="0.11 0 0",
    )

    lower_arm.add_children([
        elbow_joint,
        lower_arm_geom,
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

    # Target Puck / Ball

    # How is the ball/puck allowed to move?
    target_joint_x = e.Joint(
        armature="0",
        axis="1 0 0",
        damping="0",
        limited="true",
        name="target_joint_x",
        pos="0 0 0",
        range="-.27 .27",
        ref=".1",
        stiffness="0",
        type="slide",
    )
    target_joint_y = e.Joint(
        armature="0",
        axis="0 1 0",
        damping="0",
        limited="true",
        name="target_joint_y",
        pos="0 0 0",
        range="-.27 .27",
        ref="-.1",
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

    motor = e.Motor(
        ctrllimited="true",
        ctrlrange="-1.0 1.0",
        gear="200.0",
        joint="shoulder_joint",
    )
    motor_1 = e.Motor(
        ctrllimited="true",
        ctrlrange="-1.0 1.0",
        gear="200.0",
        joint="elbow_joint",
    )
    actuator.add_children([
        motor,
        motor_1,
    ])

    model_xml = mujoco.xml()

    # Output
    filename = "muscled-reacher.xml"
    utils.save_model(model_xml, filename)


if __name__ == '__main__':
    main()
