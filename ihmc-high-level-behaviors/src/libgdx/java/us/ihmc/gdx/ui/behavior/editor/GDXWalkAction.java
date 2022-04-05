package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.input.ImGui3DViewInput;
import us.ihmc.gdx.ui.gizmo.GDXFootstepPlannerGoalGizmo;
import us.ihmc.gdx.ui.graphics.GDXFootstepGraphic;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.JSONTools;

import java.util.List;
import java.util.UUID;

public class GDXWalkAction implements GDXBehaviorAction
{
   private GDXFootstepPlanGraphic footstepPlanGraphic;
   private List<ReferenceFrame> referenceFrameLibrary;
   private final SideDependentList<GDXFootstepGraphic> goalFeet = new SideDependentList<>();
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>();
   private FootstepPlanningModule footstepPlanner;
   private ReferenceFrame parentFrame = ReferenceFrame.getWorldFrame();
   private final GDXFootstepPlannerGoalGizmo footstepPlannerGoalGizmo = new GDXFootstepPlannerGoalGizmo();
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean selected = new ImBoolean();
   private final ImInt referenceFrameIndex = new ImInt();
   private String[] referenceFrameNames;

   public void create(FocusBasedGDXCamera camera3D,
                      DRCRobotModel robotModel,
                      FootstepPlanningModule footstepPlanner,
                      List<ReferenceFrame> referenceFrameLibrary)
   {
      this.footstepPlanner = footstepPlanner;
      footstepPlanGraphic = new GDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      this.referenceFrameLibrary = referenceFrameLibrary;
      referenceFrameNames = new String[referenceFrameLibrary.size()];
      for (int i = 0; i < referenceFrameLibrary.size(); i++)
      {
         referenceFrameNames[i] = referenceFrameLibrary.get(i).getName();
      }

      footstepPlannerGoalGizmo.create(camera3D);
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      for (RobotSide side : RobotSide.values)
      {
         GDXFootstepGraphic goalFootGraphic = new GDXFootstepGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints(),
                                                                     side);
         goalFootGraphic.create();
         goalFeet.put(side, goalFootGraphic);
         goalFeetPoses.put(side, new FramePose3D());
      }
   }

   @Override
   public void process3DViewInput(ImGui3DViewInput input)
   {
      if (selected.get())
      {
         footstepPlannerGoalGizmo.process3DViewInput(input);
         for (RobotSide side : RobotSide.values)
         {
            FramePose3D goalFootPose = goalFeetPoses.get(side);
            goalFootPose.setToZero(footstepPlannerGoalGizmo.getReferenceFrame());
            goalFootPose.getPosition().addY(0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()));
            goalFootPose.changeFrame(ReferenceFrame.getWorldFrame());
            goalFeet.get(side).setPose(goalFootPose);
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (ImGui.combo(labels.get("Reference frame"), referenceFrameIndex, referenceFrameNames))
      {
         footstepPlannerGoalGizmo.setParentFrame(referenceFrameLibrary.get(referenceFrameIndex.get()));
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      footstepPlanGraphic.getRenderables(renderables, pool);
      if (selected.get())
         footstepPlannerGoalGizmo.getRenderables(renderables, pool);
      for (RobotSide side : RobotSide.values)
         goalFeet.get(side).getRenderables(renderables, pool);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
//      String referenceFrameName = jsonNode.get("referenceFrame").asText();
      JSONTools.toEuclid(jsonNode, footstepPlannerGoalGizmo.getTransform());
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      JSONTools.toJSON(jsonNode, footstepPlannerGoalGizmo.getTransform());
   }

   @Override
   public void destroy()
   {
      footstepPlanGraphic.destroy();
   }

   public void walk(ReferenceFrame referenceFrame, ROS2ControllerHelper helper, ROS2SyncedRobotModel syncedRobot)
   {
      double proximityToGoalToMaintainOrientation = 1.5;

      FramePose3D approachPointA = new FramePose3D(referenceFrame);
      approachPointA.set(footstepPlannerGoalGizmo.getTransform());
      approachPointA.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3DReadOnly midFeetUnderPelvisFramePose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetUnderPelvisFrame);
      double midFeetUnderPelvisYaw = midFeetUnderPelvisFramePose.getYaw();

      FrameVector3D walkingDirection = new FrameVector3D(ReferenceFrame.getWorldFrame());
      walkingDirection.set(approachPointA.getPosition());
      walkingDirection.sub(midFeetUnderPelvisFramePose.getPosition());
      walkingDirection.normalize();
      double pathToGoalYaw = Math.atan2(walkingDirection.getY(), walkingDirection.getX());

      double distanceToGoal = approachPointA.getPosition().distanceXY(midFeetUnderPelvisFramePose.getPosition());
      double desiredHeading = AngleTools.computeAngleDifferenceMinusPiToPi(midFeetUnderPelvisYaw, pathToGoalYaw);
      approachPointA.getOrientation().setToYawOrientation(midFeetUnderPelvisYaw);

      RobotSide stanceSide = RobotSide.LEFT;
      FramePose3D leftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(leftFootPose, rightFootPose);
      // TODO: Set start footholds!!
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), approachPointA);
      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      LogTools.info("Stance side: {}", stanceSide.name());
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
         }
         LogTools.info("Footstep planning failure...");
      }
      else
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(),
                                                                                                 "Walk Action Planned"));

         double swingDuration = 1.2;
         double transferDuration = 0.8;
         FootstepDataListMessage footstepDataListMessage
               = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlannerOutput.getFootstepPlan(),
                                                                             swingDuration,
                                                                             transferDuration);
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
         helper.publishToController(footstepDataListMessage);
      }
   }

   @Override
   public ImBoolean getSelected()
   {
      return selected;
   }

   @Override
   public String getNameForDisplay()
   {
      return "Walk Goal";
   }
}
