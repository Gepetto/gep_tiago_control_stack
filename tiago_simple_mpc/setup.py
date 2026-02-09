from glob import glob
import os
from setuptools import find_packages, setup

package_name = "tiago_simple_mpc"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # copy yaml files to share/tiago_simple_mpc/config
        (
            os.path.join("share", package_name, "config"),
            glob("tiago_simple_mpc/config/*.yaml"),
        ),
    ],
    install_requires=[
        "setuptools",
        "meshcat",
    ],
    zip_safe=True,
    maintainer="cpene",
    maintainer_email="pene.clement@gmail.com",
    description="Simple MPC controller for Tiago using Crocoddyl",
    license="MIT OR Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "cartesian_target_mpc_node = tiago_simple_mpc.nodes.cartesian_target_mpc_node:main",
            "test_cartesian_target_ocp = tiago_simple_mpc.tests.test_cartesian_target_ocp:main",
        ],
    },
)
