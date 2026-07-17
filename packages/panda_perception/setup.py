from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "panda_perception"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML>=5.4"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Han Guangzhi",
    maintainer_email="3136412152@qq.com",
    description="Deterministic RGB-D scene localization for Panda pick and place.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scene_estimator_node = panda_perception.scene_estimator_node:main",
        ]
    },
)
