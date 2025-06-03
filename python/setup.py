from setuptools import setup, find_packages

setup(
    name="ability_hand",
    version="0.1.0",
    packages=find_packages(
        include=["ah_wrapper", "ah_examples", "ah_plotting"]
    ),
    install_requires=["pyserial>=3.5", "matplotlib>=3.6.3,<=3.10.0"],
    entry_points={
        "console_scripts": [
            "hand_wave=ah_examples.hand_wave:main",
            "plot_motors=ah_examples.plot_motors:main",
            "plot_touch_sensors=ah_examples.plot_touch_sensors:main",
            "validate_hand=ah_examples.validate_hand:main",
        ]
    },
    author="Justin Francis",
    author_email="jfrancis@psyonic.io",
    description="Python wrapper for Ability Hand API",
    url="https://github.com/psyonicinc/ability-hand-api",
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.7",
)
