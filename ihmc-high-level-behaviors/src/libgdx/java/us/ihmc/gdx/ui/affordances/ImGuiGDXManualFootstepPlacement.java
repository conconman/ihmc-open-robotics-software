package us.ihmc.gdx.ui.affordances;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import imgui.flag.ImGuiMouseButton;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.gdx.imgui.ImGuiLabelMap;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.UUID;

/**
 * Manages and assists with the operator placement of footsteps.
 */
public class ImGuiGDXManualFootstepPlacement implements RenderableProvider
{
   private final ImGuiLabelMap labels = new ImGuiLabelMap();
   private final Pose3D goalPoseForReading = new Pose3D();
   private final ArrayList<ImGuiGDXManuallyPlacedFootstep> footstepArrayList = new ArrayList<>();
   private ImGuiGDXManuallyPlacedFootstep footstepBeingPlaced;
   private int footstepIndex = -1;
   private GDXImGuiBasedUI baseUI;
   private CommunicationHelper communicationHelper;
   private RobotSide currentFootStepSide;
   private ROS2SyncedRobotModel syncedRobot;
   private ImGuiGDXManuallyPlacedFootstepChecker stepChecker;
   private ImGui3DViewInput latestInput;
   private GDX3DPanel primary3DPanel;
   private boolean renderTooltip = false;
   FramePose3D tempFramePose = new FramePose3D();

   public void create(GDXImGuiBasedUI baseUI, CommunicationHelper communicationHelper, ROS2SyncedRobotModel syncedRobot)
   {
      this.baseUI = baseUI;
      this.communicationHelper = communicationHelper;
      this.syncedRobot = syncedRobot;
      primary3DPanel = baseUI.getPrimary3DPanel();
      primary3DPanel.addImGuiOverlayAddition(this::renderTooltips);

      stepChecker = new ImGuiGDXManuallyPlacedFootstepChecker(baseUI, communicationHelper, syncedRobot);
      clear();
   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {
      renderTooltip = false;

      for (ImGuiGDXManuallyPlacedFootstep singleFootstep : footstepArrayList)
      {
         singleFootstep.calculate3DViewPick(input);
      }
      if (footstepBeingPlaced != null)
         footstepBeingPlaced.calculate3DViewPick(input);
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      latestInput = input;
      boolean anyFootstepIsSelected = false;

      for (ImGuiGDXManuallyPlacedFootstep singleFootstep : footstepArrayList)
      {
         singleFootstep.process3DViewInput(input);
      }
      if (footstepBeingPlaced != null)
         footstepBeingPlaced.process3DViewInput(input);

      if (footstepBeingPlaced != null || footstepArrayList.size() > 0)
      {
         if (isCurrentlyPlacingFootstep())
         {
            stepChecker.getInput(input);
            Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();
            renderTooltip = true;

            //Set position of modelInstance, selectablePose3DGizmo, and the sphere used in stepCheckIsPointInsideAlgorithm all to the pointInWorld that the cursor is at
            GDXTools.toGDX(pickPointInWorld, footstepBeingPlaced.getFootstepModelInstance().transform);

            footstepBeingPlaced.setGizmoPose(pickPointInWorld.getX(),
                                             pickPointInWorld.getY(),
                                             pickPointInWorld.getZ(),
                                             footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());

            footstepBeingPlaced.getBoundingSphere().getPosition().set(pickPointInWorld.getX(), pickPointInWorld.getY(), pickPointInWorld.getZ());

            // when left button clicked and released.
            if (input.isWindowHovered() & input.mouseReleasedWithoutDrag(ImGuiMouseButton.Left))
            {
               footstepIndex++;
               footstepArrayList.add(footstepBeingPlaced);

               //Switch sides
               currentFootStepSide = currentFootStepSide.getOppositeSide();
               createNewFootStep(currentFootStepSide);
            }

            if (input.isWindowHovered() && input.mouseReleasedWithoutDrag(ImGuiMouseButton.Right))
            {
               removeFootStep();
            }
         }

         Point3DReadOnly pickPointInWorld = input.getPickPointInWorld();

         // hovering.

         stepChecker.checkValidStepList(footstepArrayList);
         if (isCurrentlyPlacingFootstep())
         {
            stepChecker.checkValidSingleStep(footstepArrayList,
                                             new DiscreteFootstep(pickPointInWorld.getX(),
                                                                  pickPointInWorld.getY(),
                                                                  getFootstepBeingPlacedOrLastFootstepPlaced().getYaw(),
                                                                  currentFootStepSide));
         }

         ArrayList<BipedalFootstepPlannerNodeRejectionReason> temporaryReasons = stepChecker.getReasons();
         if (temporaryReasons.size() > 0)
         {
            for (int i = 0; i < temporaryReasons.size(); i++)
            {
               if (footstepArrayList.size() > i)
                  footstepArrayList.get(i).flashFootstepWhenBadPlacement(temporaryReasons.get(i));
            }
            if (footstepBeingPlaced != null)
               footstepBeingPlaced.flashFootstepWhenBadPlacement(temporaryReasons.get(temporaryReasons.size() - 1));
         }

         if (footstepBeingPlaced == null)
         {
            if (footstepArrayList.size() > 0)
            {
               for (int i = 0; i < footstepArrayList.size(); ++i)
               {
                  if (footstepArrayList.get(i).isPickSelected())
                  {
                     stepChecker.setReasonFrom(i);
                     anyFootstepIsSelected = true;
                     break;
                  }
               }
            }
         }
         else
         {
            anyFootstepIsSelected = true;
         }
      }
      if (anyFootstepIsSelected)
      {
         renderTooltip = true;
         stepChecker.setRenderToolTip(true);
         stepChecker.makeWarnings();
      }
      else {
         stepChecker.setRenderToolTip(false);
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Place footstep:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Left")))
      {
         createNewFootStep(RobotSide.LEFT);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Right")))
      {
         createNewFootStep(RobotSide.RIGHT);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Walk")))
      {
         if (getFootstepArrayList().size() > 0)
         {
            walkFromSteps();
         }
      }

      if (ImGui.isKeyPressed(ImGuiTools.getSpaceKey()))
      {
         if (getFootstepArrayList().size() > 0)
         {
            walkFromSteps();
         }
      }

      if (ImGui.isKeyPressed(ImGuiTools.getDeleteKey()))
      {
         removeFootStep();
      }
   }

   private void renderTooltips()
   {
      if (renderTooltip)
      {
         float offsetX = 10.0f;
         float offsetY = 10.0f;
         float mousePosX = latestInput.getMousePosX();
         float mousePosY = latestInput.getMousePosY();
         float drawStartX = primary3DPanel.getWindowDrawMinX() + mousePosX + offsetX;
         float drawStartY = primary3DPanel.getWindowDrawMinY() + mousePosY + offsetY;

         ImGui.getWindowDrawList()
              .addRectFilled(drawStartX, drawStartY, drawStartX + 150.0f, drawStartY + 21.0f, new Color(0.2f, 0.2f, 0.2f, 0.7f).toIntBits());
         ImGui.getWindowDrawList()
              .addText(ImGuiTools.getSmallFont(),
                       ImGuiTools.getSmallFont().getFontSize(),
                       drawStartX + 5.0f,
                       drawStartY + 2.0f,
                       Color.WHITE.toIntBits(),
                       "Right click to exit");
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

      for (int i = 0; i < footstepArrayList.size(); i++)
      {
         footstepArrayList.get(i).getVirtualRenderables(renderables, pool);
         footstepArrayList.get(i).getFootstepModelInstance().getRenderables(renderables, pool);
      }
      if (footstepBeingPlaced != null)
      {
         footstepBeingPlaced.getVirtualRenderables(renderables, pool);
         footstepBeingPlaced.getFootstepModelInstance().getRenderables(renderables, pool);
      }
   }

   public boolean isPlaced()
   {
      if (footstepArrayList.size() <= 0)
      {
         return false;
      }
      else if (footstepBeingPlaced == null)
      {
         return false;
      }

      return true;
   }

   public void update()
   {
      for (int i = 0; i < footstepArrayList.size(); i++)
      {
         footstepArrayList.get(i).update();
      }
      if (footstepBeingPlaced != null)
         footstepBeingPlaced.update();
   }

   public void clear()
   {

      while (footstepArrayList.size() > 0 || footstepBeingPlaced != null)
      {
         removeFootStep();
      }

      footstepArrayList.clear();
      footstepIndex = -1;
   }

   private void walkFromSteps()
   {
      FootstepDataListMessage messageList = new FootstepDataListMessage();
      for (ImGuiGDXManuallyPlacedFootstep step : footstepArrayList)
      {
         generateFootStepDataMessage(messageList, step);
         messageList.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         messageList.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
      communicationHelper.publishToController(messageList);
      // done walking >>
      // set stance and swing as last two steps of the footstepArrayList (if this list is not empty)
      // delete steps in singleFootStepAffordance.
      stepChecker.clear(footstepArrayList);
      clear();
   }

   private void generateFootStepDataMessage(FootstepDataListMessage messageList, ImGuiGDXManuallyPlacedFootstep step)
   {
      FootstepDataMessage stepMessage = messageList.getFootstepDataList().add();
      stepMessage.setRobotSide(step.getFootstepSide().toByte());
      stepMessage.getLocation().set(new Point3D(step.getSelectablePose3DGizmo().getPoseGizmo().getPose().getPosition()));
      stepMessage.getOrientation().set(step.getPose().getOrientation());
      stepMessage.setSwingDuration(0.7);
      stepMessage.setTransferDuration(0.25);
   }

   public Pose3DReadOnly getGoalPose()
   {
      return goalPoseForReading;
   }

   public void setGoalPoseAndPassOn(Pose3DReadOnly pose)
   {
      setGoalPoseNoCallbacks(pose);
   }

   public void setGoalPoseNoCallbacks(Pose3DReadOnly pose)
   {
      if (pose == null)
      {
         clear();
      }
      else
      {
         GDXTools.toGDX(pose.getPosition(), footstepArrayList.get(footstepIndex).getFootstepModelInstance().transform);
      }
      goalPoseForReading.set(pose);
   }

   public ArrayList<ImGuiGDXManuallyPlacedFootstep> getFootstepArrayList()
   {
      return footstepArrayList;
   }

   public void createNewFootStep(RobotSide footstepSide)
   {
      RigidBodyTransform latestFootstepTransform = getLatestPlacedFootstepTransform(footstepSide.getOppositeSide());
      double latestFootstepYaw = latestFootstepTransform.getRotation().getYaw();
      footstepBeingPlaced = new ImGuiGDXManuallyPlacedFootstep(baseUI, footstepSide, footstepIndex);
      currentFootStepSide = footstepSide;

      tempFramePose.setToZero(ReferenceFrame.getWorldFrame());
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      GDXTools.toEuclid(new Matrix4(), rigidBodyTransform);
      tempFramePose.set(rigidBodyTransform);
      tempFramePose.getOrientation().set(new RotationMatrix(latestFootstepYaw, 0.0, 0.0));
      tempFramePose.get(footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent());
      footstepBeingPlaced.getSelectablePose3DGizmo().getPoseGizmo().updateTransforms();
   }

   public void removeFootStep()
   {
      if(footstepBeingPlaced != null || footstepArrayList.size() > 0)
      {
         if (getFootstepBeingPlacedOrLastFootstepPlaced() != null && getFootstepBeingPlacedOrLastFootstepPlaced().getFootstepModelInstance() != null)
         {
            getFootstepBeingPlacedOrLastFootstepPlaced().getFootstepModelInstance().transform.val[Matrix4.M03] = Float.NaN;
         }
         baseUI.getPrimaryScene().removeRenderableAdapter((getFootstepBeingPlacedOrLastFootstepPlaced().getRenderableAdapter()));

         if (footstepBeingPlaced == null)
         {
            footstepIndex--;
            footstepArrayList.remove(footstepArrayList.size() - 1);
         }
         else
         {
            footstepBeingPlaced = null;
         }
      }
   }

   public RigidBodyTransform getLatestPlacedFootstepTransform(RobotSide robotSide)
   {
      if (footstepArrayList.size() > 0)
      {
         return footstepArrayList.get(footstepIndex).getSelectablePose3DGizmo().getPoseGizmo().getTransformToParent();
      }
      else
      {
         return syncedRobot.getReferenceFrames().getSoleFrame(robotSide).getTransformToWorldFrame();
      }
   }

   /*
   Returns future footstep currently being placed. If you are not placing a footstep currently, it will return last footstep from list.
   Does NOT return footsteps that you already walked on
    */
   public ImGuiGDXManuallyPlacedFootstep getFootstepBeingPlacedOrLastFootstepPlaced()
   {
      if (footstepBeingPlaced != null)
      {
         return footstepBeingPlaced;
      }
      else if (footstepArrayList.size() > 0)
      {
         return footstepArrayList.get(footstepArrayList.size() - 1);
      }
      return null;
   }

   public boolean isCurrentlyPlacingFootstep()
   {
      if (footstepBeingPlaced == null)
      {
         return false;
      }
      else
      {
         return true;
      }
   }
}
