package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionState extends ActionNodeState<ScrewPrimitiveActionDefinition>
{
   public ScrewPrimitiveActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ScrewPrimitiveActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);
   }

   @Override
   public void update()
   {
   }

   public void toMessage(ScrewPrimitiveActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

   }

   public void fromMessage(ScrewPrimitiveActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

   }
}
