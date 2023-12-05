# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Name of the frame the this action is expressed in
string object_frame_name

# Transform that expresses the screw axis in the object frame
controller_msgs/RigidBodyTransformMessage screw_axis_transform_to_object

# The pitch
float64 pitch

# The axial torque
float64 axial_torque

# The axial_force
float64 axial_force

# The distance
float64 distance

# Whether maintaining the rigid body controlled in world after the action is complete
bool hold_pose_in_world
