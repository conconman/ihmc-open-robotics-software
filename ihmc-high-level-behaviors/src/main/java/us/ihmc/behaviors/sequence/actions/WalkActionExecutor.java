package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

import java.util.UUID;

public class WalkActionExecutor extends BehaviorActionExecutor
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final WalkActionState state;
   private final WalkActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> commandedGoalFeetTransformToWorld = new SideDependentList<>(() -> new FramePose3D());
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private FootstepDataListMessage footstepDataListMessage;
   private final Timer executionTimer = new Timer();
   private final WalkingFootstepTracker footstepTracker;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private double nominalExecutionDuration;
   private final SideDependentList<BehaviorActionCompletionCalculator> completionCalculator = new SideDependentList<>(BehaviorActionCompletionCalculator::new);

   public WalkActionExecutor(BehaviorActionSequence sequence,
                             ROS2ControllerHelper ros2ControllerHelper,
                             ROS2SyncedRobotModel syncedRobot,
                             WalkingFootstepTracker footstepTracker,
                             FootstepPlanningModule footstepPlanner,
                             FootstepPlannerParametersBasics footstepPlannerParameters,
                             WalkingControllerParameters walkingControllerParameters)
   {
      super(sequence);

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepTracker = footstepTracker;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;

      state = new WalkActionState(syncedRobot, footstepPlannerParameters);
      definition = state.getDefinition();
   }

   @Override
   public void update()
   {
      super.update();

      if (state.getGoalFrame().isChildOfWorld())
      {
         for (RobotSide side : RobotSide.values)
         {
            goalFeetPoses.get(side).setIncludingFrame(state.getGoalFrame().getReferenceFrame(), definition.getGoalFootstepToGoalTransforms().get(side));
            goalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
         }
      }
   }

   /**
    * TODO: Footstep planning should happen on a thread so it doesn't hang up updates.
    */
   private void plan()
   {
      FramePose3D leftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlannerParameters.setFinalTurnProximity(1.0);

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setStartFootPoses(leftFootPose, rightFootPose);
      // TODO: Set start footholds!!
      for (RobotSide side : RobotSide.values)
      {
         footstepPlannerRequest.setGoalFootPose(side, goalFeetPoses.get(side));
         goalFeetPoses.get(side).get(commandedGoalFeetTransformToWorld.get(side));
      }

      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      double idealFootstepLength = 0.5;
      footstepPlanner.getFootstepPlannerParameters().setIdealFootstepLength(idealFootstepLength);
      footstepPlanner.getFootstepPlannerParameters().setMaximumStepReach(idealFootstepLength);
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlan.getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(FootstepPlannerLogger::deleteOldLogs, "FootstepPlanLogDeletion");

      if (footstepPlan.getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
         }
         LogTools.info("Footstep planning failure...");
         footstepDataListMessage = null;
      }
      else
      {
         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
         {
            if (i == 0 || i == footstepPlan.getNumberOfSteps() - 1)
               footstepPlan.getFootstep(i).setTransferDuration(definition.getTransferDuration() / 2.0);
            else
               footstepPlan.getFootstep(i).setTransferDuration(definition.getTransferDuration());

            footstepPlan.getFootstep(i).setSwingDuration(definition.getSwingDuration());
         }

         footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
                                                                                               definition.getSwingDuration(),
                                                                                               definition.getTransferDuration());
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }

      nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanner.getOutput().getFootstepPlan(),
                                                                                         definition.getSwingDuration(),
                                                                                         walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                         definition.getTransferDuration(),
                                                                                         walkingControllerParameters.getDefaultFinalTransferTime());
      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
      }
   }

   @Override
   public void triggerActionExecution()
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         plan();
         if (footstepDataListMessage != null)
         {
            ros2ControllerHelper.publishToController(footstepDataListMessage);
            executionTimer.reset();
         }
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (state.getGoalFrame().isChildOfWorld())
      {
         boolean isComplete = true;
         for (RobotSide side : RobotSide.values)
         {
            syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));

            isComplete &= completionCalculator.get(side)
                                              .isComplete(commandedGoalFeetTransformToWorld.get(side),
                                                          syncedFeetPoses.get(side),
                                                          POSITION_TOLERANCE,
                                                          ORIENTATION_TOLERANCE,
                                                          nominalExecutionDuration,
                                                          executionTimer,
                                                          BehaviorActionCompletionComponent.TRANSLATION,
                                                          BehaviorActionCompletionComponent.ORIENTATION);
         }
         int incompleteFootsteps = footstepTracker.getNumberOfIncompleteFootsteps();
         isComplete &= incompleteFootsteps == 0;

         state.setIsExecuting(!isComplete);

         executionStatusMessage.setActionIndex(state.getActionIndex());
         executionStatusMessage.setNominalExecutionDuration(nominalExecutionDuration);
         executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
         executionStatusMessage.setTotalNumberOfFootsteps(footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps());
         executionStatusMessage.setNumberOfIncompleteFootsteps(incompleteFootsteps);
         executionStatusMessage.setCurrentOrientationDistanceToGoal(
               completionCalculator.get(RobotSide.LEFT).getRotationError() + completionCalculator.get(RobotSide.RIGHT).getRotationError());
         executionStatusMessage.setCurrentPositionDistanceToGoal(
               completionCalculator.get(RobotSide.LEFT).getTranslationError() + completionCalculator.get(RobotSide.RIGHT).getTranslationError());
         executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         executionStatusMessage.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public WalkActionState getState()
   {
      return state;
   }

   @Override
   public WalkActionDefinition getDefinition()
   {
      return definition;
   }
}