package us.ihmc.rdx.ui.behavior.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.actions.HandPoseActionState;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionDefinition;
import us.ihmc.behaviors.sequence.actions.ScrewPrimitiveActionState;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.behavior.sequence.RDXActionNode;
import us.ihmc.rdx.ui.graphics.RDXTrajectoryGraphic;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class RDXScrewPrimitiveAction extends RDXActionNode<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiReferenceFrameLibraryCombo objectFrameComboBox;
   private final ImDoubleWrapper rotationWidget;
   private final ImDoubleWrapper translationWidget;
   private final RDXTrajectoryGraphic trajectoryGraphic = new RDXTrajectoryGraphic();
   private final RecyclingArrayList<FramePose3D> trajectoryPoses = new RecyclingArrayList<>(FramePose3D::new);

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
      rotationWidget = new ImDoubleWrapper(getDefinition()::getRotation,
                                           getDefinition()::setRotation,
                                           imDouble -> ImGuiTools.sliderDouble(labels.get("Rotation"), imDouble, -2.0 * Math.PI, 2.0 * Math.PI));
      translationWidget = new ImDoubleWrapper(getDefinition()::getTranslation,
                                              getDefinition()::setTranslation,
                                              imDouble -> ImGuiTools.sliderDouble(labels.get("Translation"), imDouble, -0.4, 0.4));
   }

   @Override
   public void update()
   {
      super.update();

      if (getState().getScrewFrame().isChildOfWorld())
      {
         if (getParent().getState() instanceof ActionSequenceState parent)
         {
            HandPoseActionState previousHandPose = parent.findNextPreviousAction(HandPoseActionState.class,
                                                                                 getState().getActionIndex(),
                                                                                 getDefinition().getSide());
            if (previousHandPose != null)
            {
               trajectoryPoses.clear();
               FramePose3D firstPose = trajectoryPoses.add();
               firstPose.setToZero(previousHandPose.getPalmFrame().getReferenceFrame());
               firstPose.changeFrame(ReferenceFrame.getWorldFrame());

               int segments = (int) Math.ceil(Math.abs(getDefinition().getRotation()) / 0.3 + Math.abs(getDefinition().getTranslation()) / 0.02);
               double rotationPerSegment = getDefinition().getRotation() / segments;
               double translationPerSegment = getDefinition().getTranslation() / segments;

               for (int i = 0; i < segments; i++)
               {
                  FramePose3D lastPose = trajectoryPoses.getLast();

                  FramePose3D nextPose = trajectoryPoses.add();
                  nextPose.setIncludingFrame(lastPose);
                  nextPose.changeFrame(getState().getScrewFrame().getReferenceFrame());

                  nextPose.prependRollRotation(rotationPerSegment);
                  nextPose.prependTranslation(translationPerSegment, 0.0, 0.0);

                  nextPose.changeFrame(ReferenceFrame.getWorldFrame());
               }

               double lineWidth = 0.01;
               trajectoryGraphic.update(lineWidth, trajectoryPoses);
            }
         }
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      objectFrameComboBox.render();
      rotationWidget.renderImGuiWidget();
      translationWidget.renderImGuiWidget();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
         trajectoryGraphic.getRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return getDefinition().getSide().getPascalCaseName() + " Screw Primitive";
   }
}
