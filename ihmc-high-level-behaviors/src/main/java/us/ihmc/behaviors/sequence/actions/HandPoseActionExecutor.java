package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.*;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class HandPoseActionExecutor extends ActionNodeExecutor<HandPoseActionState, HandPoseActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final HandPoseActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators;
   private final Timer executionTimer = new Timer();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public HandPoseActionExecutor(long id,
                                 CRDTInfo crdtInfo,
                                 ROS2ControllerHelper ros2ControllerHelper,
                                 ReferenceFrameLibrary referenceFrameLibrary,
                                 DRCRobotModel robotModel,
                                 ROS2SyncedRobotModel syncedRobot,
                                 SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculators = handWrenchCalculators;

      state = new HandPoseActionState(id, crdtInfo, referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
   }

   @Override
   public void update()
   {
      super.update();

      state.setCanExecute(state.getPalmFrame().isChildOfWorld());

      if (state.getPalmFrame().isChildOfWorld() && state.getIsNextForExecution())
      {
         ChestOrientationActionExecutor concurrentChestOrientationAction = null;
         if (state.getIsToBeExecutedConcurrently() && getParent() instanceof ActionSequenceExecutor parentSequence)
         {
            int concurrentSetIndex = parentSequence.getState().getExecutionNextIndex();

            while (concurrentSetIndex <= parentSequence.getLastIndexOfConcurrentSetToExecute())
            {
               if (parentSequence.getExecutorChildren().get(concurrentSetIndex) instanceof ChestOrientationActionExecutor chestOrientationAction)
               {
                  concurrentChestOrientationAction = chestOrientationAction;
               }
               ++concurrentSetIndex;
            }
         }
         if (concurrentChestOrientationAction == null)
         {
            state.getGoalChestToWorldTransform().getValue().set(syncedRobot.getReferenceFrames().getChestFrame().getTransformToRoot());
         }
         else
         {
            concurrentChestOrientationAction.getState().update(); // Ensure state's frames are initialized
            state.getGoalChestToWorldTransform().getValue()
                 .set(concurrentChestOrientationAction.getState().getChestFrame().getReferenceFrame().getTransformToRoot());
         }
         state.getGoalChestFrame().update();

         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());
         armIKSolver.copySourceToWork();
         armIKSolver.update(state.getGoalChestFrame(), state.getPalmFrame().getReferenceFrame());
         armIKSolver.solve();

         // Send the solution back to the UI so the user knows what's gonna happen with the arm.
         state.setSolutionQuality(armIKSolver.getQuality());
         for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         {
            state.getJointAngles().getValue()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
         }
      }
   }

   @Override
   public void triggerActionExecution()
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());

         OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
         double[] jointAngles = new double[solutionOneDoFJoints.length];
         for (int i = 0; i < jointAngles.length; i++)
         {
            jointAngles[i] = solutionOneDoFJoints[i].getQ();
         }

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                     getDefinition().getTrajectoryDuration(),
                                                                                                     jointAngles);
         armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
         if (getDefinition().getJointSpaceControl())
         {
            ros2ControllerHelper.publishToController(armTrajectoryMessage);
         }
         else
         {
            FramePose3D frameHand = new FramePose3D(state.getPalmFrame().getReferenceFrame());
            frameHand.changeFrame(ReferenceFrame.getWorldFrame());
            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(getDefinition().getSide(),
                                                                                                           getDefinition().getTrajectoryDuration(),
                                                                                                           frameHand.getPosition(),
                                                                                                           frameHand.getOrientation(),
                                                                                                           ReferenceFrame.getWorldFrame());
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setXWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setYWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getAngularWeightMatrix().setZWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setXWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setYWeight(50.0);
            handTrajectoryMessage.getSe3Trajectory().getLinearWeightMatrix().setZWeight(50.0);
            handTrajectoryMessage.setForceExecution(true);

            HandHybridJointspaceTaskspaceTrajectoryMessage hybridHandMessage = HumanoidMessageTools.createHandHybridJointspaceTaskspaceTrajectoryMessage(
                  getDefinition().getSide(),
                  handTrajectoryMessage.getSe3Trajectory(),
                  armTrajectoryMessage.getJointspaceTrajectory());
            ros2ControllerHelper.publishToController(hybridHandMessage);
         }

         executionTimer.reset();

         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
         startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (state.getPalmFrame().isChildOfWorld())
      {
         desiredHandControlPose.setFromReferenceFrame(state.getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));

         boolean wasExecuting = state.getIsExecuting();
         // Left hand broke on Nadia and not in the robot model?
         state.setIsExecuting(!completionCalculator.isComplete(desiredHandControlPose,
                                                               syncedHandControlPose,
                                                               POSITION_TOLERANCE,
                                                               ORIENTATION_TOLERANCE,
                                                               getDefinition().getTrajectoryDuration(),
                                                               executionTimer,
                                                               BehaviorActionCompletionComponent.TRANSLATION,
                                                               BehaviorActionCompletionComponent.ORIENTATION));

         state.setNominalExecutionDuration(getDefinition().getTrajectoryDuration());
         state.setElapsedExecutionTime(executionTimer.getElapsedTime());
         state.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         state.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         state.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         state.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
         state.setHandWrenchMagnitudeLinear(handWrenchCalculators.get(getDefinition().getSide()).getLinearWrenchMagnitude(true));
         if (!state.getIsExecuting() && wasExecuting && !getDefinition().getJointSpaceControl() && !getDefinition().getHoldPoseInWorldLater())
         {
            disengageHoldPoseInWorld();
         }
      }
   }

   private void disengageHoldPoseInWorld()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                  getDefinition().getTrajectoryDuration(),
                                                                                                  jointAngles);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
   }

   @Override
   public HandPoseActionState getState()
   {
      return state;
   }
}
