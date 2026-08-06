from setuptools import find_packages, setup

setup(
    name="pib_motors",
    version="1.0",
    description="motors for pib",
    # Include nested modules such as pib_motors.STservo_sdk.
    packages=find_packages(include=["pib_motors", "pib_motors.*"]),
)
