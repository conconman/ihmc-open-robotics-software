package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
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
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
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
   private final HandWrenchTrajectoryMessage handWrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
   private final Vector3D linearVelocity = new Vector3D();
   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D force = new Vector3D();
   private final Vector3D torque = new Vector3D();
   private final Quaternion localRotationQuaternion = new Quaternion();
   private final Vector3D worldRotationVector = new Vector3D();;
   private final Vector3D localRotationVectorEnd = new Vector3D();
   private final FramePoint3D frameRotationVectorEnd = new FramePoint3D();
   private final MutableReferenceFrame previousPoseFrame = new MutableReferenceFrame();
   private final MutableReferenceFrame nextPoseFrame = new MutableReferenceFrame();

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
         syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getDefinition().getSide()));
         startPositionDistanceToGoal = syncedHandControlPose.getTranslation().differenceNorm(desiredHandControlPose.getTranslation());
         startOrientationDistanceToGoal = syncedHandControlPose.getRotation().distance(desiredHandControlPose.getRotation(), true);

         handWrenchTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         handWrenchTrajectoryMessage.setForceExecution(true);

         handTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         handTrajectoryMessage.setForceExecution(true);

         SE3TrajectoryMessage se3TrajectoryMessage = handTrajectoryMessage.getSe3Trajectory();
         se3TrajectoryMessage.getAngularWeightMatrix().setXWeight(getDefinition().getAngularPositionWeight());
         se3TrajectoryMessage.getAngularWeightMatrix().setYWeight(getDefinition().getAngularPositionWeight());
         se3TrajectoryMessage.getAngularWeightMatrix().setZWeight(getDefinition().getAngularPositionWeight());
         se3TrajectoryMessage.getLinearWeightMatrix().setXWeight(getDefinition().getLinearPositionWeight());
         se3TrajectoryMessage.getLinearWeightMatrix().setYWeight(getDefinition().getLinearPositionWeight());
         se3TrajectoryMessage.getLinearWeightMatrix().setZWeight(getDefinition().getLinearPositionWeight());
         se3TrajectoryMessage.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         se3TrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

         WrenchTrajectoryMessage wrenchTrajectory = handWrenchTrajectoryMessage.getWrenchTrajectory();
         wrenchTrajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         wrenchTrajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

         wrenchTrajectory.getWrenchTrajectoryPoints().clear();

         se3TrajectoryMessage.getTaskspaceTrajectoryPoints().clear();

         int numberOfPoints = getState().getTrajectory().getSize();

         double time = 0.0;
         linearVelocity.setToZero();
         angularVelocity.setToZero();
         force.setToZero();
         torque.setToZero();

         for (int i = 0; i < numberOfPoints; i++)
         {
            // For the first point, they are the same -- the initial hand pose
            Pose3DReadOnly previousPose = getState().getTrajectory().getValueReadOnly(i == 0 ? i : i - 1);
            Pose3DReadOnly nextPose = getState().getTrajectory().getValueReadOnly(i);

            if (i > 0)
            {
               // Find whether linear or angular would take longer at max speed and slow the other down
               double linearDistance = nextPose.getPosition().distance(previousPose.getPosition());
               double angularDistance = nextPose.getOrientation().distance(previousPose.getOrientation());
               double timeForLinear = linearDistance / getDefinition().getMaxLinearVelocity();
               double timeForAngular = angularDistance / getDefinition().getMaxAngularVelocity();
               double longestTime = Math.max(timeForLinear, timeForAngular);
               time += longestTime;

               previousPoseFrame.getTransformToParent().set(previousPose);
               previousPoseFrame.getReferenceFrame().update();

               nextPoseFrame.getTransformToParent().set(nextPose);
               nextPoseFrame.getReferenceFrame().update();

               linearVelocity.sub(nextPose.getPosition(), previousPose.getPosition());
               linearVelocity.normalize();
               linearVelocity.scale(linearDistance / longestTime);

               localRotationQuaternion.difference(previousPose.getOrientation(), nextPose.getOrientation());
               localRotationQuaternion.getRotationVector(localRotationVectorEnd);
               frameRotationVectorEnd.setIncludingFrame(nextPoseFrame.getReferenceFrame(), localRotationVectorEnd);
               frameRotationVectorEnd.changeFrame(ReferenceFrame.getWorldFrame());
               worldRotationVector.sub(frameRotationVectorEnd, nextPose.getTranslation());

               angularVelocity.set(worldRotationVector);
               angularVelocity.normalize();
               angularVelocity.scale(angularDistance / longestTime);

               // TODO: Calculate force/torque as tracking error
               force.set(linearVelocity);
               force.normalize();
               force.scale(getDefinition().getMaxForce());

               torque.set(angularVelocity);
               torque.normalize();
               torque.scale(getDefinition().getMaxTorque());
            }

            SE3TrajectoryPointMessage se3TrajectoryPointMessage = se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add();
            se3TrajectoryPointMessage.getPosition().set(nextPose.getTranslation());
            se3TrajectoryPointMessage.getOrientation().set(nextPose.getOrientation());
            se3TrajectoryPointMessage.getLinearVelocity().set(linearVelocity);
            se3TrajectoryPointMessage.getAngularVelocity().set(angularVelocity);

            WrenchTrajectoryPointMessage wrenchTrajectoryPointMessage = wrenchTrajectory.getWrenchTrajectoryPoints().add();
            wrenchTrajectoryPointMessage.getWrench().getForce().set(force);
            wrenchTrajectoryPointMessage.getWrench().getTorque().set(torque);

            // First time is 0.0
            wrenchTrajectoryPointMessage.setTime(time);
            se3TrajectoryPointMessage.setTime(time);

            LogTools.info("Adding point time: %.2f  nextPose: %s %s  linearVel: %s  angularVel: %s  force: %s  torque %s"
                    .formatted(time,
                               nextPose.getPosition(),
                               new YawPitchRoll(nextPose.getOrientation()),
                               linearVelocity,
                               angularVelocity,
                               force,
                               torque));
         }
         ros2ControllerHelper.publishToController(handTrajectoryMessage);
//         ros2ControllerHelper.publishToController(handWrenchTrajectoryMessage);
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
