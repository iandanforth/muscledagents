import utils
from mjcf import elements as e


def main():
    mujoco = e.Mujoco(
        model="empty-world"
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
    default = e.Default()
    asset = e.Asset()
    worldbody = e.Worldbody()

    mujoco.add_children([
        compiler,
        option,
        size,
        default,
        asset,
        worldbody,
    ])

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

    model_xml = mujoco.xml()
    filename = "empty-world.xml"
    utils.save_model(model_xml, filename)


if __name__ == '__main__':
    main()
