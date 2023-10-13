package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class WalkActionState extends BehaviorActionState
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final WalkActionDefinition definition;
   private final DetachableReferenceFrame goalFrame;

   public WalkActionState(ROS2SyncedRobotModel syncedRobot, FootstepPlannerParametersBasics footstepPlannerParameters)
   {
      this.syncedRobot = syncedRobot;
      definition = new WalkActionDefinition(footstepPlannerParameters);
      goalFrame = new DetachableReferenceFrame(definition.getGoalToParentTransform());
   }

   @Override
   public void update()
   {
      goalFrame.update(definition.getParentFrameName(), syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      setCanExecute(goalFrame.isChildOfWorld());
   }

   public void toMessage(WalkActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public WalkActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }
}
