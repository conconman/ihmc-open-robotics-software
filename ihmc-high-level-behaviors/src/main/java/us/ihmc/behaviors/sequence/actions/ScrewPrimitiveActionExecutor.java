package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
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
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
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
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
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

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
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

         handHybridTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         handHybridTrajectoryMessage.setForceExecution(true);

         JointspaceTrajectoryMessage jointspaceTrajectoryMessage = handHybridTrajectoryMessage.getJointspaceTrajectoryMessage();
         jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().clear();

         SE3TrajectoryMessage taskspaceTrajectoryMessage = handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage();
         taskspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
         taskspaceTrajectoryMessage.getLinearWeightMatrix().setXWeight(getDefinition().getLinearPositionWeight());
         taskspaceTrajectoryMessage.getLinearWeightMatrix().setYWeight(getDefinition().getLinearPositionWeight());
         taskspaceTrajectoryMessage.getLinearWeightMatrix().setZWeight(getDefinition().getLinearPositionWeight());
         taskspaceTrajectoryMessage.getAngularWeightMatrix().setXWeight(getDefinition().getAngularPositionWeight());
         taskspaceTrajectoryMessage.getAngularWeightMatrix().setYWeight(getDefinition().getAngularPositionWeight());
         taskspaceTrajectoryMessage.getAngularWeightMatrix().setZWeight(getDefinition().getAngularPositionWeight());
         taskspaceTrajectoryMessage.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         taskspaceTrajectoryMessage.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().clear();

         handWrenchTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         handWrenchTrajectoryMessage.setForceExecution(true);

         WrenchTrajectoryMessage wrenchTrajectory = handWrenchTrajectoryMessage.getWrenchTrajectory();
         wrenchTrajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         wrenchTrajectory.getFrameInformation().setTrajectoryReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
         wrenchTrajectory.setUseCustomControlFrame(true);
         // TODO: Translate to body fixed frame
         wrenchTrajectory.getControlFramePose().set(getDefinition().getWrenchContactPoseInHandControlFrame().getValueReadOnly());
         wrenchTrajectory.getWrenchTrajectoryPoints().clear();

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

            double deltaTime = 0.0;
            if (i > 0)
            {
               // Find whether linear or angular would take longer at max speed and slow the other down
               double linearDistance = nextPose.getPosition().distance(previousPose.getPosition());
               double angularDistance = nextPose.getOrientation().distance(previousPose.getOrientation());
               double timeForLinear = linearDistance / getDefinition().getMaxLinearVelocity();
               double timeForAngular = angularDistance / getDefinition().getMaxAngularVelocity();
               deltaTime = Math.max(timeForLinear, timeForAngular); // Take longest
               time += deltaTime;

               previousPoseFrame.getTransformToParent().set(previousPose);
               previousPoseFrame.getReferenceFrame().update();

               nextPoseFrame.getTransformToParent().set(nextPose);
               nextPoseFrame.getReferenceFrame().update();

               linearVelocity.sub(nextPose.getPosition(), previousPose.getPosition());
               linearVelocity.normalize();
               linearVelocity.scale(linearDistance / deltaTime);

               localRotationQuaternion.difference(previousPose.getOrientation(), nextPose.getOrientation());
               localRotationQuaternion.getRotationVector(localRotationVectorEnd);
               frameRotationVectorEnd.setIncludingFrame(nextPoseFrame.getReferenceFrame(), localRotationVectorEnd);
               frameRotationVectorEnd.changeFrame(ReferenceFrame.getWorldFrame());
               worldRotationVector.sub(frameRotationVectorEnd, nextPose.getTranslation());

               angularVelocity.set(worldRotationVector);
               angularVelocity.normalize();
               angularVelocity.scale(angularDistance / deltaTime);

               // TODO: Calculate force/torque as tracking error
               force.set(linearVelocity);
               force.normalize();
               force.scale(getDefinition().getMaxForce());

               torque.set(angularVelocity);
               torque.normalize();
               torque.scale(getDefinition().getMaxTorque());
            }

            ArmIKSolver armIKSolver = armIKSolvers.get(getDefinition().getSide());
            if (i == 0)
               armIKSolver.copySourceToWork();
            armIKSolver.update(syncedRobot.getReferenceFrames().getChestFrame(), nextPoseFrame.getReferenceFrame());
            armIKSolver.solve();

            if (armIKSolver.getQuality() > ArmIKSolver.GOOD_QUALITY_MAX)
               LogTools.warn("Bad quality: {} (i == {})", armIKSolver.getQuality(), i);

            for (int j = 0; j < armIKSolver.getSolutionOneDoFJoints().length; j++)
            {
               OneDoFJointTrajectoryMessage oneDoFJointTrajectoryMessage = i == 0 ? jointspaceTrajectoryMessage.getJointTrajectoryMessages().add()
                                                                                  : jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(j);
               if (i == 0)
                  oneDoFJointTrajectoryMessage.getTrajectoryPoints().clear();
               oneDoFJointTrajectoryMessage.setWeight(-1.0); // Use default weight

               TrajectoryPoint1DMessage trajectoryPoint1DMessage = oneDoFJointTrajectoryMessage.getTrajectoryPoints().add();
               trajectoryPoint1DMessage.setTime(time);
               trajectoryPoint1DMessage.setPosition(armIKSolver.getSolutionOneDoFJoints()[j].getQ());
               if (i == 0)
               {
                  trajectoryPoint1DMessage.setVelocity(0.0);
               }
               else
               {
                  double previousQ = oneDoFJointTrajectoryMessage.getTrajectoryPoints().get(i - 1).getPosition();
                  double nextQ = trajectoryPoint1DMessage.getPosition();
                  trajectoryPoint1DMessage.setVelocity((nextQ - previousQ) / deltaTime);
               }
            }

            SE3TrajectoryPointMessage se3TrajectoryPointMessage = taskspaceTrajectoryMessage.getTaskspaceTrajectoryPoints().add();
            se3TrajectoryPointMessage.setTime(time);
            se3TrajectoryPointMessage.getPosition().set(nextPose.getTranslation());
            se3TrajectoryPointMessage.getOrientation().set(nextPose.getOrientation());
            se3TrajectoryPointMessage.getLinearVelocity().set(linearVelocity);
            se3TrajectoryPointMessage.getAngularVelocity().set(angularVelocity);

            WrenchTrajectoryPointMessage wrenchTrajectoryPointMessage = wrenchTrajectory.getWrenchTrajectoryPoints().add();
            wrenchTrajectoryPointMessage.setTime(time);
            wrenchTrajectoryPointMessage.getWrench().getForce().set(force);
            wrenchTrajectoryPointMessage.getWrench().getTorque().set(torque);

            LogTools.info("Adding point time: %.2f  nextPose: %s %s  linearVel: %s  angularVel: %s  force: %s  torque %s"
                    .formatted(time,
                               nextPose.getPosition(),
                               new YawPitchRoll(nextPose.getOrientation()),
                               linearVelocity,
                               angularVelocity,
                               force,
                               torque));
         }
         ArmTrajectoryMessage armTrajectoryMessage = new ArmTrajectoryMessage();
         armTrajectoryMessage.setRobotSide(getDefinition().getSide().toByte());
         armTrajectoryMessage.getJointspaceTrajectory().set(jointspaceTrajectoryMessage);
         LogTools.info("Sending Jointspace ArmTrajectoryMessage");
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
//         ros2ControllerHelper.publishToController(handHybridTrajectoryMessage);
         if (getDefinition().getMaxForce() > 0.0 || getDefinition().getMaxTorque() > 0.0)
            ros2ControllerHelper.publishToController(handWrenchTrajectoryMessage);
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
