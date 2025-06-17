from setuptools import setup, find_packages

with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="ability_hand",
    version="0.2.1",
    packages=find_packages(
        include=["ah_wrapper", "ah_examples", "ah_plotting"]
    ),
    install_requires=["pyserial>=3.5", "matplotlib>=3.6.3,<=3.10.0"],
    long_description=long_description,
    long_description_content_type="text/markdown",
    include_package_data=True,
    package_data={
        "ah_plotting": ["touch_sensor_legend_sml.png"],
    },
    entry_points={
        "console_scripts": [
            "hand_wave=ah_examples.hand_wave:main",
            "plot_motors=ah_examples.plot_motors:main",
            "plot_touch_sensors=ah_examples.plot_touch_sensors:main",
            "plot_motors_and_touch=ah_examples.plot_motors_and_touch:main",
            "validate_hand=ah_examples.validate_hand:main",
        ]
    },
    author="Justin Francis",
    author_email="jfrancis@psyonic.io",
    description="Python wrapper for the PSYONIC Ability Hand API",
    url="https://github.com/psyonicinc/ability-hand-api",
    project_urls={
        "PSYONIC Website": "https://psyonic.io",
        "CHANGELOG": "https://github.com/psyonicinc/ability-hand-api/blob/master/python/CHANGELOG.md",
    },
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.10",
)
