# This message is part of the multi-contact feasibility module
# This message describes how a keyframe is parameterized in terms of kinematic objectives

# The keyframe kinematics snapshot
toolbox_msgs/KinematicsToolboxInputCollectionMessage kinematics_snapshot

# The output of the IK toolbox
toolbox_msgs/KinematicsToolboxOutputStatus kinematics_solution

# Which rigid bodies are in contact in kinematics_snapshot
bool[<=20] in_contact

# The surface normals of the contacting rigid bodies
geometry_msgs/Vector3[<=20] surface_normals