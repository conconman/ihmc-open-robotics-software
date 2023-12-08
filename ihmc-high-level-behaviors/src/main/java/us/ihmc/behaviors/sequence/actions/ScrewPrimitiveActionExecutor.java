package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.ActionSequenceState;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.tools.ROS2HandWrenchCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionExecutor extends ActionNodeExecutor<ScrewPrimitiveActionState, ScrewPrimitiveActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators;
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();
   private final FramePose3D workPose = new FramePose3D();
   private final HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();

   public ScrewPrimitiveActionExecutor(long id,
                                       CRDTInfo crdtInfo,
                                       WorkspaceResourceDirectory saveFileDirectory,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       ReferenceFrameLibrary referenceFrameLibrary,
                                       DRCRobotModel robotModel,
                                       ROS2SyncedRobotModel syncedRobot,
                                       SideDependentList<ROS2HandWrenchCalculator> handWrenchCalculators)
   {
      super(new ScrewPrimitiveActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.handWrenchCalculators = handWrenchCalculators;
   }

   @Override
   public void update()
   {
      super.update();

      getState().setCanExecute(getState().getScrewFrame().isChildOfWorld());

      if (getState().getScrewFrame().isChildOfWorld())
      {
         if (getParent().getState() instanceof ActionSequenceState parent)
         {
            if (parent.getExecutionNextIndex() <= getState().getActionIndex())
            {
               ReferenceFrame initialHandFrame = null;

               if (getState().getIsNextForExecution())
               {
                  initialHandFrame = syncedRobot.getReferenceFrames().getHandFrame(getDefinition().getSide());
               }
               else
               {
                  HandPoseActionState previousHandPose = parent.findNextPreviousAction(HandPoseActionState.class,
                                                                                       getState().getActionIndex(),
                                                                                       getDefinition().getSide());
                  if (previousHandPose != null && previousHandPose.getPalmFrame().isChildOfWorld())
                  {
                     initialHandFrame = previousHandPose.getPalmFrame().getReferenceFrame();
                  }
               }

               if (initialHandFrame != null)
               {
                  RecyclingArrayList<Pose3D> trajectoryPoses = getState().getTrajectory().getValue();
                  trajectoryPoses.clear();
                  Pose3D firstPose = trajectoryPoses.add();
                  workPose.setToZero(initialHandFrame);
                  workPose.changeFrame(ReferenceFrame.getWorldFrame());
                  firstPose.set(workPose);

                  int segments = (int) Math.ceil(Math.abs(getDefinition().getRotation()) / 0.15 + Math.abs(getDefinition().getTranslation()) / 0.01);
                  double rotationPerSegment = getDefinition().getRotation() / segments;
                  double translationPerSegment = getDefinition().getTranslation() / segments;

                  for (int i = 0; i < segments; i++)
                  {
                     Pose3D lastPose = trajectoryPoses.getLast();
                     Pose3D nextPose = trajectoryPoses.add();

                     workPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), lastPose);
                     workPose.changeFrame(getState().getScrewFrame().getReferenceFrame());

                     workPose.prependRollRotation(rotationPerSegment);
                     workPose.prependTranslation(translationPerSegment, 0.0, 0.0);

                     workPose.changeFrame(ReferenceFrame.getWorldFrame());
                     nextPose.set(workPose);
                  }
               }
            }
         }
      }
   }

   @Override
   public void triggerActionExecution()
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
//         desiredHandControlPose.setFromReferenceFrame(getState().getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
         startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);

         handTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         handTrajectoryMessage.setForceExecution(true);

         SE3TrajectoryMessage se3TrajectoryMessage = handTrajectoryMessage.getSe3Trajectory();
         se3TrajectoryMessage.getAngularWeightMatrix().setXWeight(50.0);
         se3TrajectoryMessage.getAngularWeightMatrix().setYWeight(50.0);
         se3TrajectoryMessage.getAngularWeightMatrix().setZWeight(50.0);
         se3TrajectoryMessage.getLinearWeightMatrix().setXWeight(50.0);
         se3TrajectoryMessage.getLinearWeightMatrix().setYWeight(50.0);
         se3TrajectoryMessage.getLinearWeightMatrix().setZWeight(50.0);
         se3TrajectoryMessage.getAngularWeightMatrix().setXWeight(-1.0);
         se3TrajectoryMessage.getAngularWeightMatrix().setYWeight(-1.0);
         se3TrajectoryMessage.getAngularWeightMatrix().setZWeight(-1.0);
         se3TrajectoryMessage.getLinearWeightMatrix().setXWeight(-1.0);
         se3TrajectoryMessage.getLinearWeightMatrix().setYWeight(-1.0);
         se3TrajectoryMessage.getLinearWeightMatrix().setZWeight(-1.0);
         se3TrajectoryMessage.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

         se3TrajectoryMessage.getTaskspaceTrajectoryPoints().clear();

         double timeStep = 0.5;
         double time = timeStep;
         int bookendRamp = 5;
         Vector3D linearVelocity = new Vector3D();
         Vector3D angularVelocity = new Vector3D();
         Quaternion orientationDelta = new Quaternion();
         int numberOfPoints = getState().getTrajectory().getSize();

         for (int i = 1; i < numberOfPoints; i++)
         {
            Pose3DReadOnly pose3D = getState().getTrajectory().getValueReadOnly(i);

            double velocityScale = Math.min(Math.min(i, numberOfPoints - i), bookendRamp) / (double) bookendRamp;
            velocityScale = 1.0; // FIXME

            Pose3DReadOnly lastPose3D = getState().getTrajectory().getValueReadOnly(i - 1);
            linearVelocity.sub(pose3D.getPosition(), lastPose3D.getPosition());

            orientationDelta.difference(pose3D.getOrientation(), lastPose3D.getOrientation());
            orientationDelta.getRotationVector(angularVelocity);

            double linearDistance = linearVelocity.norm();
            double angularDistance = pose3D.getOrientation().distance(lastPose3D.getOrientation());

            linearVelocity.normalize();
            linearVelocity.scale(linearDistance / timeStep);
            angularVelocity.normalize();
            angularVelocity.scale(angularDistance / timeStep);

            SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
            se3TrajectoryPointMessage.getPosition().set(pose3D.getTranslation());
            se3TrajectoryPointMessage.getOrientation().set(pose3D.getOrientation());
            se3TrajectoryPointMessage.getLinearVelocity().set(linearVelocity);
            se3TrajectoryPointMessage.getAngularVelocity().set(angularVelocity);

            time += timeStep;
            se3TrajectoryPointMessage.setTime(time);
         }
         ros2ControllerHelper.publishToController(handTrajectoryMessage);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (getState().getScrewFrame().isChildOfWorld())
      {
//         desiredHandControlPose.setFromReferenceFrame(getState().getPalmFrame().getReferenceFrame());
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));


         // TODO: Completion criteria
//         getState().setIsExecuting();


         getState().setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         getState().setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         getState().setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         getState().setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         getState().setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         getState().setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }
}
