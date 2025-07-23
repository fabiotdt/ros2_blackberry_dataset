from setuptools import find_packages, setup
import os
from glob import glob

package_name = "ur5e_motion_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fabio_tdt",
    maintainer_email="fabio_tdt@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_executor = ur5e_motion_controller.motion_executor:main",
            "arm_state_publisher = ur5e_motion_controller.arm_state_publisher:main",
            "berry_pose_publisher = ur5e_motion_controller.berry_pose_publisher:main",
        ],
    },
)
