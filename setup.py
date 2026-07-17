from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "panda_pick_place_planning"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pybullet>=3.2.6,<4"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="韩广志",
    maintainer_email="3136412152@qq.com",
    description="ROS2 + PyBullet Panda 抓取放置与路径规划仿真系统。",
    license="MIT",
    url="https://github.com/Hangz0416/panda_pick_place_planning",
    entry_points={
        "console_scripts": [
            "run_demo = panda_pick_place_planning.core.demo:main",
            "run_benchmark = panda_pick_place_planning.core.benchmark:main",
            "task_generator_node = panda_pick_place_planning.nodes.task_generator_node:main",
            "manipulation_node = panda_pick_place_planning.nodes.manipulation_node:main",
            "evaluator_node = panda_pick_place_planning.nodes.evaluator_node:main",
        ]
    },
)
