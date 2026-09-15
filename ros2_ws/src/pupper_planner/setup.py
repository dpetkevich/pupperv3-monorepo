from setuptools import find_packages, setup

package_name = "pupper_planner"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "jsonschema"],
    zip_safe=True,
    maintainer="Daniel Petkevich",
    maintainer_email="daniel.petkevich@gmail.com",
    description="Plan executor for the Pupper brain",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan_executor = pupper_planner.plan_executor:main",
        ],
    },
)
