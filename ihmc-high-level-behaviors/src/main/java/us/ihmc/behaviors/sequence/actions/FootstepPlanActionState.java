package us.ihmc.behaviors.sequence.actions;

import us.ihmc.behaviors.sequence.BehaviorActionState;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class FootstepPlanActionState extends BehaviorActionState
{
   private final FootstepPlanActionDefinition definition = new FootstepPlanActionDefinition();
   private final DetachableReferenceFrame soleFrame = new DetachableReferenceFrame(definition.getTransformToParent());

   @Override
   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      soleFrame.update(referenceFrameLibrary, definition.getParentFrameName());
   }

   @Override
   public FootstepPlanActionDefinition getDefinition()
   {
      return definition;
   }

   public DetachableReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }
}
