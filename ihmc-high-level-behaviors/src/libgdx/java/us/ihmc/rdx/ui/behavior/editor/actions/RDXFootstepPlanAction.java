package us.ihmc.rdx.ui.behavior.editor.actions;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.sequence.actions.FootstepActionData;
import us.ihmc.behaviors.sequence.actions.FootstepPlanActionDescription;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXFootstepPlanAction extends RDXBehaviorAction
{
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final FootstepPlanActionDescription actionDescription = new FootstepPlanActionDescription();
   private final ImGuiReferenceFrameLibraryCombo referenceFrameLibraryCombo;
   private final RecyclingArrayList<RDXFootstepAction> footsteps;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final TypedNotification<RobotSide> userAddedFootstep = new TypedNotification<>();
   private final Notification userRemovedFootstep = new Notification();
   private final ImDoubleWrapper swingDurationWidget = new ImDoubleWrapper(actionDescription::getSwingDuration,
                                                                           actionDescription::setSwingDuration,
                                                                           imDouble -> ImGui.inputDouble(labels.get("Swing duration"), imDouble));
   private final ImDoubleWrapper transferDurationWidget = new ImDoubleWrapper(actionDescription::getTransferDuration,
                                                                              actionDescription::setTransferDuration,
                                                                              imDouble -> ImGui.inputDouble(labels.get("Transfer duration"), imDouble));

   public RDXFootstepPlanAction(RDXBaseUI baseUI,
                                DRCRobotModel robotModel,
                                ROS2SyncedRobotModel syncedRobot,
                                ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.referenceFrameLibrary = referenceFrameLibrary;

      referenceFrameLibraryCombo = new ImGuiReferenceFrameLibraryCombo(referenceFrameLibrary);

      footsteps = new RecyclingArrayList<>(() -> new RDXFootstepAction(actionDescription, baseUI, robotModel, getSelected()::get));
   }

   @Override
   public void updateAfterLoading()
   {
      referenceFrameLibraryCombo.setSelectedReferenceFrame(actionDescription.getConditionalReferenceFrame());
   }

   public void setToReferenceFrame(ReferenceFrame referenceFrame)
   {
      actionDescription.getConditionalReferenceFrame().setParentFrameName(ReferenceFrame.getWorldFrame().getName());
      actionDescription.setTransformToParent(referenceFrame.getTransformToWorldFrame());
      update();
   }

   @Override
   public void update(boolean concurrencyWithPreviousAction, int indexShiftConcurrentAction)
   {
      actionDescription.update(referenceFrameLibrary);

      // Add a footstep to the action data only
      if (userAddedFootstep.poll())
      {
         RobotSide newSide = userAddedFootstep.read();
         FootstepActionData addedFootstep = actionDescription.getFootsteps().add();
         addedFootstep.setSide(newSide);
         FramePose3D newFootstepPose = new FramePose3D();
         if (!footsteps.isEmpty())
         {
            RDXFootstepAction previousFootstep = footsteps.get(footsteps.size() - 1);
            newFootstepPose.setToZero(previousFootstep.getFootstepFrame());

            if (previousFootstep.getActionData().getSide() != newSide)
            {
               double minStepWidth = robotModel.getWalkingControllerParameters().getSteppingParameters().getMinStepWidth();
               newFootstepPose.getPosition().addY(newSide.negateIfRightSide(minStepWidth));
            }
         }
         else
         {
            newFootstepPose.setToZero(syncedRobot.getReferenceFrames().getSoleFrame(newSide));
         }

         double aLittleInFront = 0.15;
         newFootstepPose.getPosition().addX(aLittleInFront);

         newFootstepPose.changeFrame(actionDescription.getConditionalReferenceFrame().get());
         addedFootstep.getSolePose().set(newFootstepPose);
      }

      if (userRemovedFootstep.poll())
         actionDescription.getFootsteps().remove(actionDescription.getFootsteps().size() - 1);

      // Synchronizes the RDX footsteps to the action data
      footsteps.clear();
      while (footsteps.size() < actionDescription.getFootsteps().size())
         footsteps.add();
      for (int i = 0; i < actionDescription.getFootsteps().size(); i++)
      {
         footsteps.get(i).update(actionDescription.getFootsteps().get(i), i);
      }
   }

   @Override
   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      for (RDXFootstepAction footstep : footsteps)
      {
         footstep.calculate3DViewPick(input);
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      for (RDXFootstepAction footstep : footsteps)
      {
         footstep.process3DViewInput(input);
      }
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      if (referenceFrameLibraryCombo.render())
      {
         actionDescription.getConditionalReferenceFrame().setParentFrameName(referenceFrameLibraryCombo.getSelectedReferenceFrame().getParent().getName());
         update();
      }

      ImGui.pushItemWidth(80.0f);
      swingDurationWidget.renderImGuiWidget();
      transferDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();

      ImGui.text("Number of footsteps: %d".formatted(footsteps.size()));
      ImGui.text("Add:");
      for (RobotSide side : RobotSide.values)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get(side.getPascalCaseName())))
            userAddedFootstep.set(side);
      }
      ImGui.sameLine();
      ImGui.text("Remove:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Last")))
      {
         userRemovedFootstep.set();
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXFootstepAction footstep : footsteps)
      {
         footstep.getVirtualRenderables(renderables, pool);
      }
   }

   @Override
   public String getActionTypeTitle()
   {
      return "Footstep Plan";
   }

   @Override
   public FootstepPlanActionDescription getActionDescription()
   {
      return actionDescription;
   }
}
