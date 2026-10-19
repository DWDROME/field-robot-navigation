from setuptools import find_packages, setup
from pathlib import Path
from xml.etree import ElementTree

package_xml = ElementTree.parse(Path(__file__).parent / "package.xml")
package_name = package_xml.find("./name").text
version = package_xml.find("./version").text

setup(
    name=package_name,
    version=version,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/x500_sites.yaml",
        ]),
    ],
    install_requires=["setuptools", "pymavlink==2.4.50"],
    zip_safe=True,
    maintainer="Greenhouse Maintainers",
    maintainer_email="dev@example.com",
    description="PX4 SITL air-vehicle integration for the simulation matrix",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "micro_xrce_agent = greenhouse_sim_air.micro_xrce_agent:main",
            "air_mission = greenhouse_sim_air.air_mission:main",
            "simulation_gcs = greenhouse_sim_air.simulation_gcs:main",
        ],
    },
)
