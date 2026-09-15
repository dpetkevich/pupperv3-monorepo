from setuptools import find_packages, setup

package_name = "pupper_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={package_name: ["camera_params.yaml"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["pupper_vision/camera_params.yaml"]),
    ],
    install_requires=["setuptools", "numpy", "opencv-python", "pyyaml"],
    zip_safe=False,
    maintainer="Daniel Petkevich",
    maintainer_email="daniel.petkevich@gmail.com",
    description="Shared vision library for the Pupper brain",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
