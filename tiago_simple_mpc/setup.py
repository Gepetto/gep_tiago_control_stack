from setuptools import find_packages, setup

package_name = "tiago_simple_mpc"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
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
            "simple_mpc_node = tiago_simple_mpc.simple_mpc_node:main",
            "cartesian_target_mpc_node = tiago_simple_mpc.cartesian_target_mpc_node:main",
            "patch_urdf_effort = tiago_simple_mpc.tools.patch_urdf_effort:main",
        ],
    },
)
