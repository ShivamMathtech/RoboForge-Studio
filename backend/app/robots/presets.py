from math import pi

from app.models.robot import DHParameter, Joint, JointLimit, JointType, RobotModel


def _revolute(name: str, a: float, alpha: float = 0.0, d: float = 0.0, mass: float = 1.0) -> Joint:
    return Joint(
        name=name,
        type=JointType.REVOLUTE,
        dh=DHParameter(a=a, alpha=alpha, d=d),
        limit=JointLimit(lower=-pi, upper=pi, velocity=2.5, acceleration=7.0, torque=120.0),
        damping=0.08,
        mass=mass,
    )


PRESETS: dict[str, RobotModel] = {
    "planar-2dof": RobotModel(
        id="planar-2dof",
        name="2-DOF Planar Arm",
        description="Beginner planar arm with two 0.8 m revolute links.",
        payload_kg=1.0,
        joints=[_revolute("Shoulder", 0.8, mass=2.0), _revolute("Elbow", 0.8, mass=1.2)],
    ),
    "articulated-3dof": RobotModel(
        id="articulated-3dof",
        name="3-DOF Articulated Arm",
        description="Spatial educational arm for FK, IK, and Jacobian studies.",
        payload_kg=1.5,
        joints=[
            _revolute("Base", 0.0, pi / 2, 0.45, 3.0),
            _revolute("Shoulder", 0.7, 0.0, 0.0, 2.0),
            _revolute("Elbow", 0.55, 0.0, 0.0, 1.2),
        ],
    ),
    "scara-4dof": RobotModel(
        id="scara-4dof",
        name="4-DOF SCARA",
        description="Educational assembly arm with two rotary axes, a vertical slide, and wrist.",
        payload_kg=4.0,
        joints=[
            _revolute("Shoulder", 0.65, 0.0, 0.45, 3.5),
            _revolute("Elbow", 0.55, pi, 0.0, 2.5),
            Joint(
                name="Vertical slide",
                type=JointType.PRISMATIC,
                dh=DHParameter(a=0.0, alpha=0.0, d=0.15),
                limit=JointLimit(lower=-0.35, upper=0.05, velocity=0.8, acceleration=2.0, torque=180),
                mass=2.0,
            ),
            _revolute("Wrist", 0.16, 0.0, 0.0, 0.8),
        ],
    ),
    "industrial-6dof": RobotModel(
        id="industrial-6dof",
        name="6-DOF Industrial Arm",
        description="General-purpose articulated research preset with a spherical wrist.",
        payload_kg=8.0,
        joints=[
            _revolute("J1 Base", 0.0, pi / 2, 0.42, 8.0),
            _revolute("J2 Shoulder", 0.62, 0.0, 0.0, 6.0),
            _revolute("J3 Elbow", 0.48, 0.0, 0.0, 4.0),
            _revolute("J4 Wrist roll", 0.0, pi / 2, 0.18, 1.5),
            _revolute("J5 Wrist pitch", 0.0, -pi / 2, 0.16, 1.0),
            _revolute("J6 Tool", 0.18, 0.0, 0.10, 0.6),
        ],
    ),
    "redundant-7dof": RobotModel(
        id="redundant-7dof",
        name="7-DOF Redundant Arm",
        description="Research preset for redundant IK and future null-space control.",
        payload_kg=5.0,
        joints=[
            _revolute("J1", 0.0, -pi / 2, 0.34, 5.0),
            _revolute("J2", 0.0, pi / 2, 0.0, 4.5),
            _revolute("J3", 0.0, pi / 2, 0.40, 4.0),
            _revolute("J4", 0.0, -pi / 2, 0.0, 3.0),
            _revolute("J5", 0.0, -pi / 2, 0.40, 2.0),
            _revolute("J6", 0.0, pi / 2, 0.0, 1.2),
            _revolute("J7", 0.18, 0.0, 0.13, 0.7),
        ],
    ),
}

