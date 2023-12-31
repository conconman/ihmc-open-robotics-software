package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.WalkActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class WalkActionState extends ActionNodeState<WalkActionDefinition>
{
   private final CRDTDetachableReferenceFrame goalFrame;
   private final FootstepPlanActionStateBasics footstepPlanStateBasics;

   public WalkActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new WalkActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      goalFrame = new CRDTDetachableReferenceFrame(referenceFrameLibrary,
                                                   getDefinition().getBasics().getCRDTParentFrameName(),
                                                   getDefinition().getGoalToParentTransform());
      footstepPlanStateBasics = new FootstepPlanActionStateBasics(crdtInfo);
   }

   @Override
   public void update()
   {
      goalFrame.update();
   }

   public void toMessage(WalkActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      footstepPlanStateBasics.toMessage(message.getFootstepPlanStateBasics());
   }

   public void fromMessage(WalkActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      footstepPlanStateBasics.fromMessage(message.getFootstepPlanStateBasics());
   }

   public CRDTDetachableReferenceFrame getGoalFrame()
   {
      return goalFrame;
   }

   public FootstepPlanActionStateBasics getBasics()
   {
      return footstepPlanStateBasics;
   }
}
