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
import us.ihmc.euclid.geometry.Pose3D;
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
   private final ImDoubleWrapper pitchWidget;
   private final ImDoubleWrapper distanceWidget;
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
      pitchWidget = new ImDoubleWrapper(getDefinition()::getPitch,
                                        getDefinition()::setPitch,
                                        imDouble -> ImGuiTools.sliderDouble(labels.get("Pitch"), imDouble, 0.0, 2.0));
      distanceWidget = new ImDoubleWrapper(getDefinition()::getDistance,
                                           getDefinition()::setDistance,
                                           imDouble -> ImGui.inputDouble(labels.get("Distance"), imDouble, 0.0, 0.5));
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

               double pitch = getDefinition().getPitch();
               double traversed = 0.0;

//               double translationNormal = pitch / ;
//               double translationPerPoint = pitch / 0.01;

               while (traversed < getDefinition().getDistance() && trajectoryPoses.size() < 100)
               {
                  FramePose3D lastPose = trajectoryPoses.getLast();

                  FramePose3D nextPose = trajectoryPoses.add();
                  nextPose.setIncludingFrame(lastPose);
                  nextPose.changeFrame(getState().getScrewFrame().getReferenceFrame());

                  nextPose.prependRollRotation(0.01);
                  nextPose.appendTranslation(0.0, 0.0, -pitch * 0.01);

                  nextPose.changeFrame(ReferenceFrame.getWorldFrame());

                  traversed += nextPose.getTranslation().distance(lastPose.getTranslation());
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
      pitchWidget.renderImGuiWidget();
      distanceWidget.renderImGuiWidget();
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
