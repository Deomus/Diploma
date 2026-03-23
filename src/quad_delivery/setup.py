from setuptools import find_packages, setup


package_name = "quad_delivery"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/sim.launch.py"]),
        (f"share/{package_name}/worlds", ["worlds/empty_world.sdf"]),
        (
            f"share/{package_name}/models/delivery_drone",
            ["models/delivery_drone/model.config", "models/delivery_drone/model.sdf"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="smagin",
    maintainer_email="smagin@example.com",
    description="Minimal ROS 2 mover for a delivery drone in Gazebo Harmonic.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_sim = quad_delivery.run_sim:main",
            "simple_mover = quad_delivery.simple_mover:main",
        ],
    },
)
