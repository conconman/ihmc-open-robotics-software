package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ChestOrientationActionStateMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class ChestOrientationActionState extends BehaviorActionState
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ChestOrientationActionDefinition definition = new ChestOrientationActionDefinition();
   private final DetachableReferenceFrame chestFrame;

   public ChestOrientationActionState(ROS2SyncedRobotModel syncedRobot)
   {
      this.syncedRobot = syncedRobot;
      chestFrame = new DetachableReferenceFrame(definition.getChestToParentTransform());
   }

   @Override
   public void update()
   {
      chestFrame.update(definition.getParentFrameName(), syncedRobot.getReferenceFrames().getCommonReferenceFrames());

      setCanExecute(chestFrame.isChildOfWorld());
   }

   public void toMessage(ChestOrientationActionStateMessage message)
   {
      super.toMessage(message.getActionState());

      definition.toMessage(message.getDefinition());
   }

   public void fromMessage(ChestOrientationActionStateMessage message)
   {
      super.fromMessage(message.getActionState());

      definition.fromMessage(message.getDefinition());
   }

   @Override
   public ChestOrientationActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }
}
