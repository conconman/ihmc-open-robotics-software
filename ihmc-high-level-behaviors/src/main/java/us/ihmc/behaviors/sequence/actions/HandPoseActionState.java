package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseActionStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class HandPoseActionState extends BehaviorActionState
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final HandPoseActionDefinition definition = new HandPoseActionDefinition();
   private final DetachableReferenceFrame palmFrame;

   public HandPoseActionState(ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      palmFrame = new DetachableReferenceFrame(definition.getPalmTransformToParent());
   }

   @Override
   public void update()
   {
      palmFrame.update(definition.getPalmParentFrameName(), syncedRobot.getReferenceFrames().getCommonReferenceFrames());
      setCanExecute(palmFrame.isChildOfWorld());
   }

   public void toMessage(HandPoseActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(HandPoseActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public HandPoseActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }
}
