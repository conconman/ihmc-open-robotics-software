# This message is part of the IHMC whole-body controller API.
# This message commands the controller to move an arm in jointspace to the desired joint angles while going through the specified trajectory points.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte REQUESTED_MODE_TORQUE_CONTROL = 0
byte REQUESTED_MODE_POSITION_CONTROL = 1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# When the robot is walking, restrictions on upper-body motion may be applied.
# To by-pass the safety check and force the execution of this message, set this field to true.
bool force_execution false

# Optional field, if REQUESTED_MODE_POSITION_CONTROL and the robot has low-level gains for the arms, this is executed in position control mode.
# If REQUESTED_MODE_TORQUE_CONTROL this is executed in torque control mode. If left unset the existing execution mode is unchanged.
byte requested_mode 255

# Specifies the side of the robot that will execute the trajectory.
byte robot_side 255
# Trajectories for each joint.
# The indexing for the joints goes increasingly from the first shoulder joint to the last arm joint.
JointspaceTrajectoryMessage jointspace_trajectory