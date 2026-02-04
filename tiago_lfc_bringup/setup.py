from glob import glob
import os
from setuptools import find_packages, setup

package_name = "tiago_lfc_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (
            os.path.join("share", package_name, "config/fixed"),
            glob("config/fixed/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config/free_flyer"),
            glob("config/free_flyer/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cpene",
    maintainer_email="pene.clement@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "odom_to_interface = tiago_lfc_bringup.odom_to_interface:main",
        ],
    },
)
