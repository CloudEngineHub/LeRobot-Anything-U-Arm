from setuptools import find_packages, setup

package_name = "openarm_servo_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="OpenArm User",
    maintainer_email="user@example.com",
    description="Servo leader bridge for OpenArm v1 ROS 2 controllers.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["servo_teleop_node = openarm_servo_teleop.servo_teleop_node:main"]},
)
