from setuptools import find_packages, setup

package_name = "pupper_motion"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel Petkevich",
    maintainer_email="daniel.petkevich@gmail.com",
    description="Pupper motion server (closed-loop turns, dead-reckoned moves, safety)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_server = pupper_motion.motion_server:main",
            "calibrate_k = pupper_motion.calibrate_k:main",
            "fake_plant = pupper_motion.fake_plant:main",
        ],
    },
)
