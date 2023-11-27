# This message is part of the multi-contact feasibility module
# This message acts as an output of the module and contains the CoM region of the current keyframe.

# Feasible CoM region of the current keyframe
controller_msgs/Polygon2DMessage com_feasibility_region

# Feasible CoM region for contact removability
controller_msgs/Polygon2DMessage[<=15] com_feasibility_preview_regions
