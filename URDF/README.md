# URDF Description Files

The URDFs describe each different type of ability hand including the locations
of each of the touch sensors.  Since the URDF format cannot simulate a four bar
linkage a linear approximation is used to determine the L2 joint positions.

Inertia is calculated using know weight of individual components and uniform
weight distribution across a simplified mesh.  Collisions are handled using the
simplified mesh.

You can easily view the URDF using our [ROS2 URDF Viewer](https://github.com/psyonicinc/ah-ros2-urdf-viewer)

For Mujoco descriptions and examples on simulating the Ability Hand see [/python/ah_simulators](https://github.com/psyonicinc/ability-hand-api/tree/master/python/ah_simulators).