from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rob599_hw2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Colin Mitchell",
    maintainer_email="mitchcol@oregonstate.edu",
    description="ROB 599 Homework 2",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "speed_limiter = rob599_hw2.SpeedLimiter:main",
            "pub_helper = rob599_hw2.PubHelper:main",
            "msg_checker = rob599_hw2.MsgChecker:main",
            "nasa = rob599_hw2.ActionServer:main",
            "flight_control = rob599_hw2.ActionClient:main",
        ],
    },
)
