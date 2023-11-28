from setuptools import find_packages, setup

package_name = "dock"

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
    maintainer="ncolesummers",
    maintainer_email="nsummers@uidaho.edu",
    description="Adapter to slow down a create3 docking status topic",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ['dock = dock.check_sensor:main'],
    },
)
