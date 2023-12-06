package us.ihmc.rdx.ui.behavior.actions;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXScrewPrimitiveAction extends RDXActionNode<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo objectFrameComboBox;

   public RDXScrewPrimitiveAction(long id,
                                  CRDTInfo crdtInfo,
                                  WorkspaceResourceDirectory saveFileDirectory,
                                  RDX3DPanel panel3D,
                                  DRCRobotModel robotModel,
                                  FullHumanoidRobotModel syncedFullRobotModel,
                                  RobotCollisionModel selectionCollisionModel,
                                  ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));


      getDefinition().setDescription("Screw primitive");


      objectFrameComboBox = new ImGuiReferenceFrameLibraryCombo("Object frame",
                                                                referenceFrameLibrary,
                                                                getDefinition()::getObjectFrameName,
                                                                getDefinition()::setObjectFrameName);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      objectFrameComboBox.render();
   }

   @Override
   public String getActionTypeTitle()
   {
      return getDefinition().getSide().getPascalCaseName() + " Screw Primitive";
   }
}
