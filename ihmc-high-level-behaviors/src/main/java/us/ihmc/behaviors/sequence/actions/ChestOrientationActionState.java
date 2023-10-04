package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.BodyPartPoseActionDefinitionMessage;
import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class ChestOrientationActionState extends BehaviorActionState<BodyPartPoseActionDefinitionMessage>
{
   private final ChestOrientationActionDefinition definition = new ChestOrientationActionDefinition();
   private final DetachableReferenceFrame chestFrame = new DetachableReferenceFrame(definition.getTransformToParent());

   @Override
   public void update()
   {
      chestFrame.update(definition.getParentFrameName());
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
