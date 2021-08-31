package us.ihmc.behaviors.lookAndStep;

import java.util.UUID;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.footstepPlanner.FootstepPlanEtcetera;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.interfaces.RobotWalkRequester;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.behaviors.tools.interfaces.UIPublisher;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.LastCommandedFootsteps;

public class LookAndStepSteppingTask
{
   protected StatusLogger statusLogger;
   protected UIPublisher uiPublisher;
   protected LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   protected FootstepPlannerParametersReadOnly footstepPlannerParameters;
   protected SwingPlannerParametersReadOnly swingPlannerParameters;

   protected RobotWalkRequester robotWalkRequester;
   protected Runnable replanFootstepsOutput;

   protected FootstepPlan footstepPlan;
   protected ROS2SyncedRobotModel syncedRobot;
   protected TimerSnapshotWithExpiration robotDataReceptionTimerSnaphot;
   protected long previousStepMessageId = 0L;
   protected SideDependentList<PlannedFootstepReadOnly> lastCommandedFootsteps;

   public static class LookAndStepStepping extends LookAndStepSteppingTask
   {
      private ResettableExceptionHandlingExecutorService executor;
      private final TypedInput<FootstepPlan> footstepPlanInput = new TypedInput<>();
      private BehaviorTaskSuppressor suppressor;

      public void initialize(LookAndStepBehavior lookAndStep)
      {
         lastCommandedFootsteps = lookAndStep.lastCommandedFootsteps;
         statusLogger = lookAndStep.statusLogger;
         syncedRobot = lookAndStep.robotInterface.newSyncedRobot();
         lookAndStepParameters = lookAndStep.lookAndStepParameters;
         footstepPlannerParameters = lookAndStep.footstepPlannerParameters;
         swingPlannerParameters = lookAndStep.swingPlannerParameters;
         uiPublisher = lookAndStep.helper::publish;
         robotWalkRequester = lookAndStep.robotInterface::requestWalk;
         replanFootstepsOutput = () ->
         {
            if (!lookAndStep.isBeingReset.get())
            {
               lookAndStep.behaviorStateReference.set(LookAndStepBehavior.State.FOOTSTEP_PLANNING);
               lookAndStep.bodyPathLocalization.acceptSwingSleepComplete();
            }
         };

         executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
         footstepPlanInput.addCallback(data -> executor.clearQueueAndExecute(this::evaluateAndRun));

         suppressor = new BehaviorTaskSuppressor(statusLogger, "Robot motion");
         suppressor.addCondition("Not in robot motion state", () -> !lookAndStep.behaviorStateReference.get().equals(LookAndStepBehavior.State.STEPPING));
         suppressor.addCondition(() -> "Footstep plan not OK: numberOfSteps = " + (footstepPlan == null ? null : footstepPlan.getNumberOfSteps())
                                       + ". Planning again...",
                                 () -> !(footstepPlan != null && footstepPlan.getNumberOfSteps() > 0), replanFootstepsOutput);
         suppressor.addCondition("Robot disconnected", () -> !robotDataReceptionTimerSnaphot.isRunning());
         suppressor.addCondition("Robot not in walking state", () -> !lookAndStep.controllerStatusTracker.isInWalkingState());
      }

      public void reset()
      {
         executor.interruptAndReset();
         previousStepMessageId = 0L;
      }

      public void acceptFootstepPlan(FootstepPlan footstepPlan)
      {
         footstepPlanInput.set(footstepPlan);
      }

      private void evaluateAndRun()
      {
         footstepPlan = footstepPlanInput.getLatest();
         syncedRobot.update();
         robotDataReceptionTimerSnaphot = syncedRobot.getDataReceptionTimerSnapshot()
                                                     .withExpiration(lookAndStepParameters.getRobotConfigurationDataExpiration());

         if (suppressor.evaulateShouldAccept())
         {
            performTask();
         }
      }
   }

   protected void performTask()
   {
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         lastCommandedFootsteps.put(footstepPlan.getFootstep(i).getRobotSide(), footstepPlan.getFootstep(i));
      }

      statusLogger.warn("Requesting walk");
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);
      FootstepDataMessageConverter.appendPlanToMessage(footstepPlan, footstepDataListMessage);
      // TODO: Add combo to look and step UI to chose which steps to visualize
      uiPublisher.publishToUI(LastCommandedFootsteps, MinimalFootstep.convertFootstepDataListMessage(footstepDataListMessage));

      ExecutionMode executionMode = previousStepMessageId == 0L ? ExecutionMode.OVERRIDE : ExecutionMode.QUEUE;
      footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
      long messageId = UUID.randomUUID().getLeastSignificantBits();
      footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
      footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
      previousStepMessageId = messageId;
      TypedNotification<WalkingStatusMessage> walkingStatusNotification = robotWalkRequester.requestWalk(footstepDataListMessage);

      ThreadTools.startAsDaemon(() -> robotWalkingThread(walkingStatusNotification), "RobotWalking");
      sleepForPartOfSwingThread(lookAndStepParameters.getSwingDuration());
   }

   private void sleepForPartOfSwingThread(double swingDuration)
   {
      double percentSwingToWait = lookAndStepParameters.getPercentSwingToWait();
      double waitDuration = swingDuration * percentSwingToWait;
      statusLogger.info("Waiting {} s for {} % of swing...", waitDuration, percentSwingToWait);
      ThreadTools.sleepSeconds(waitDuration);
      statusLogger.info("{} % of swing complete!", percentSwingToWait);
      replanFootstepsOutput.run();
   }

   private void robotWalkingThread(TypedNotification<WalkingStatusMessage> walkingStatusNotification)
   {
      statusLogger.debug("Waiting for robot walking...");
      walkingStatusNotification.blockingPoll();
      statusLogger.debug("Robot walk complete.");
   }
}
