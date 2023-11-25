# This message is part of the multi-contact feasibility module
# This message acts as an input to the module and describes the candidate robot state.

# The current keyframe snapshot
toolbox_msgs/MultiContactKinematicsSnapshot snapshot

# The current keyframe snapshot (optional)
toolbox_msgs/MultiContactKinematicsSnapshot previous_snapshot

# Whether previous_snapshot is populated
bool contains_previous_keyframe


