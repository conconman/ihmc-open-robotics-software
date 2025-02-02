package us.ihmc.quadrupedCommunication.teleop;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import controller_msgs.msg.dds.RobotConfigurationData;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import quadruped_msgs.msg.dds.*;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedNetworkProcessor;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.atomic.AtomicReference;

public class RemoteQuadrupedTeleopManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final QuadrupedXGaitSettingsBasics xGaitSettings;
   private final ROS2Node ros2Node;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();

   private final IHMCROS2Publisher<HighLevelStateMessage> controllerStatePublisher;
   private final IHMCROS2Publisher<QuadrupedRequestedSteppingStateMessage> steppingStatePublisher;
   private final IHMCROS2Publisher<QuadrupedTimedStepListMessage> timedStepListPublisher;
   private final IHMCROS2Publisher<QuadrupedBodyOrientationMessage> bodyOrientationPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionsListControllerPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pauseWalkingMessagePublisher;
   private final IHMCROS2Publisher<AbortWalkingMessage> abortWalkingMessagePublisher;
   private final IHMCROS2Publisher<QuadrupedFootLoadBearingMessage> loadBearingMessagePublisher;
   private final IHMCROS2Publisher<QuadrupedBodyHeightMessage> bodyHeightPublisher;
   private final IHMCROS2Publisher<QuadrupedBodyTrajectoryMessage> bodyPosePublisher;

   private final IHMCROS2Publisher<QuadrupedTeleopDesiredVelocity> desiredVelocityPublisher;

   private final IHMCROS2Publisher<ToolboxStateMessage> stepTeleopStatePublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> pawPlannerStatePublisher;

   private final IHMCROS2Publisher<QuadrupedXGaitSettingsPacket> stepXGaitSettingsPublisher;
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionsListTeleopPublisher;
   private final IHMCROS2Publisher<QuadrupedBodyPathPlanMessage> bodyPathPublisher;

   private final IHMCROS2Publisher<QuadrupedXGaitSettingsPacket> plannerXGaitSettingsPublisher;
   private final IHMCROS2Publisher<PawStepPlanningRequestPacket> planningRequestPublisher;

   private final AtomicDouble timestamp = new AtomicDouble();
   private final QuadrupedRobotDataReceiver robotDataReceiver;
   private final QuadrupedNetworkProcessor networkProcessor;
   private final String robotName;

   public RemoteQuadrupedTeleopManager(String robotName, ROS2Node ros2Node, QuadrupedNetworkProcessor networkProcessor,
                                       FullQuadrupedRobotModel robotModel, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, YoRegistry parentRegistry)
   {
      this.robotName = robotName;
      this.ros2Node = ros2Node;
      this.xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      this.networkProcessor = networkProcessor;

      ROS2Topic controllerOutputTopic = ROS2Tools.getQuadrupedControllerOutputTopic(robotName);
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, HighLevelStateChangeStatusMessage.class, controllerOutputTopic,
                                           s -> controllerStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, QuadrupedSteppingStateChangeMessage.class, controllerOutputTopic,
                                           s -> steppingStateChangeMessage.set(s.takeNextData()));
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, controllerOutputTopic, s -> robotConfigurationData.set(s.takeNextData()));

      ROS2Topic controllerInputTopic = ROS2Tools.getQuadrupedControllerInputTopic(robotName);
      ROS2Topic stepTeleopInputTopic = ROS2Tools.STEP_TELEOP_TOOLBOX.withRobot(robotName)
                                                                        .withInput();
      ROS2Topic footstepPlannerInputTopic = ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotName)
                                                                             .withInput();

      controllerStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, HighLevelStateMessage.class, controllerInputTopic);
      steppingStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedRequestedSteppingStateMessage.class, controllerInputTopic);
      timedStepListPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedTimedStepListMessage.class, controllerInputTopic);
      bodyOrientationPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedBodyOrientationMessage.class, controllerInputTopic);
      planarRegionsListControllerPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, controllerInputTopic);
      pauseWalkingMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PauseWalkingMessage.class, controllerInputTopic);
      abortWalkingMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, AbortWalkingMessage.class, controllerInputTopic);
      loadBearingMessagePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedFootLoadBearingMessage.class, controllerInputTopic);
      bodyHeightPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedBodyHeightMessage.class, controllerInputTopic);
      bodyPosePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedBodyTrajectoryMessage.class, controllerInputTopic);

      desiredVelocityPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedTeleopDesiredVelocity.class, stepTeleopInputTopic);

      stepTeleopStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, stepTeleopInputTopic);
      pawPlannerStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, ToolboxStateMessage.class, footstepPlannerInputTopic);

      planarRegionsListTeleopPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PlanarRegionsListMessage.class, stepTeleopInputTopic);
      stepXGaitSettingsPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedXGaitSettingsPacket.class, stepTeleopInputTopic);
      bodyPathPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedBodyPathPlanMessage.class, stepTeleopInputTopic);

      plannerXGaitSettingsPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, QuadrupedXGaitSettingsPacket.class, footstepPlannerInputTopic);
      planningRequestPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, PawStepPlanningRequestPacket.class, footstepPlannerInputTopic);

      robotDataReceiver = new QuadrupedRobotDataReceiver(robotModel, null);

      parentRegistry.addChild(registry);
   }

   public String getRobotName()
   {
      return robotName;
   }

   public void publishTimedStepListToController(QuadrupedTimedStepListMessage message)
   {
      timedStepListPublisher.publish(message);
   }

   public void publishBodyOrientationMessage(QuadrupedBodyOrientationMessage message)
   {
      bodyOrientationPublisher.publish(message);
   }

   public void publishXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      stepXGaitSettingsPublisher.publish(xGaitSettings.getAsPacket());
      plannerXGaitSettingsPublisher.publish(xGaitSettings.getAsPacket());
   }

   public void publishPlanningRequest(PawStepPlanningRequestPacket packet)
   {
      pawPlannerStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
      planningRequestPublisher.publish(packet);
   }

   public void setDesiredVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityZ)
   {
      desiredVelocityPublisher.publish(QuadrupedMessageTools.createQuadrupedTeleopDesiredVelocity(desiredVelocityX, desiredVelocityY, desiredVelocityZ));
   }

   public void requestStandPrep()
   {
      controllerStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.STAND_PREP_STATE));
   }

   public void requestWalkingState()
   {
      controllerStatePublisher.publish(HumanoidMessageTools.createHighLevelStateMessage(HighLevelControllerName.STAND_TRANSITION_STATE));
   }

   public void requestPauseWalking(boolean pauseWalking)
   {
      pauseWalkingMessagePublisher.publish(HumanoidMessageTools.createPauseWalkingMessage(pauseWalking));
   }

   public void requestAbortWalking()
   {
      abortWalkingMessagePublisher.publish(new AbortWalkingMessage());
   }

   public void requestLoadBearing(RobotQuadrant quadrant)
   {
      loadBearingMessagePublisher.publish(QuadrupedMessageTools.createLoadBearingMessage(quadrant));
   }

   private void requestStopWalking()
   {
      abortWalkingMessagePublisher.publish(new AbortWalkingMessage());
   }

   public void requestXGait()
   {
      stepTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
   }

   public void requestStanding()
   {
      requestStopWalking();
      stepTeleopStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public void setDesiredBodyHeight(double desiredBodyHeight)
   {
      updateRobotModel();

      ReferenceFrame bodyHeightFrame = robotDataReceiver.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();
      FramePoint3D bodyHeight = new FramePoint3D(bodyHeightFrame, 0.0, 0.0, desiredBodyHeight);
      bodyHeight.changeFrame(ReferenceFrame.getWorldFrame());

      QuadrupedBodyHeightMessage bodyHeightMessage = QuadrupedMessageTools.createQuadrupedBodyHeightMessage(0.0, bodyHeight.getZ());
      bodyHeightMessage.setControlBodyHeight(true);
      bodyHeightMessage.setIsExpressedInAbsoluteTime(false);

      bodyHeightPublisher.publish(bodyHeightMessage);
   }

   public void setDesiredBodyOrientation(double desiredYaw, double desiredPitch, double desiredRoll, double time)
   {
      updateRobotModel();

      QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();
      SE3TrajectoryMessage se3Trajectory = bodyTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      se3Trajectory.getAngularSelectionMatrix().setXSelected(true);
      se3Trajectory.getAngularSelectionMatrix().setYSelected(true);
      se3Trajectory.getAngularSelectionMatrix().setZSelected(true);
      se3Trajectory.getLinearSelectionMatrix().setXSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setYSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setZSelected(false);
      SE3TrajectoryPointMessage trajectoryPointMessage = se3Trajectory.getTaskspaceTrajectoryPoints().add();
      trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
      trajectoryPointMessage.setTime(time + timestamp.get());

      bodyPosePublisher.publish(bodyTrajectoryMessage);
   }

   public void submitPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      planarRegionsListControllerPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
      planarRegionsListTeleopPublisher.publish(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));
   }

   public void submitBodyPathPlan(QuadrupedBodyPathPlanMessage message)
   {
      bodyPathPublisher.publish(message);
   }

   public void setEndDoubleSupportDuration(QuadrupedSpeed speed, double endPhaseShift, double endDoubleSupportDuration)
   {
      if (endPhaseShift < 90.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getPaceSlowTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         case MEDIUM:
            xGaitSettings.getPaceMediumTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         default:
            xGaitSettings.getPaceFastTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         }
      }
      else if (endPhaseShift < 180.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getAmbleSlowTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         case MEDIUM:
            xGaitSettings.getAmbleMediumTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         default:
            xGaitSettings.getAmbleFastTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         }
      }
      else
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getTrotSlowTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         case MEDIUM:
            xGaitSettings.getTrotMediumTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         default:
            xGaitSettings.getTrotFastTimings().setEndDoubleSupportDuration(endDoubleSupportDuration);
            break;
         }
      }      publishXGaitSettings(xGaitSettings);
   }

   public void setEndPhaseShift(double endPhaseShift)
   {
      xGaitSettings.setEndPhaseShift(endPhaseShift);
      publishXGaitSettings(xGaitSettings);
   }

   public void setQuadrupedSpeed(QuadrupedSpeed speed)
   {
      xGaitSettings.setQuadrupedSpeed(speed);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStanceWidth(double stanceWidth)
   {
      xGaitSettings.setStanceWidth(stanceWidth);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStanceLength(double stanceLength)
   {
      xGaitSettings.setStanceLength(stanceLength);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStepGroundClearance(double groundClearance)
   {
      xGaitSettings.setStepGroundClearance(groundClearance);
      publishXGaitSettings(xGaitSettings);
   }

   public void setStepDuration(QuadrupedSpeed speed, double endPhaseShift, double stepDuration)
   {
      if (endPhaseShift < 90.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getPaceSlowTimings().setStepDuration(stepDuration);
            break;
         case MEDIUM:
            xGaitSettings.getPaceMediumTimings().setStepDuration(stepDuration);
            break;
         default:
            xGaitSettings.getPaceFastTimings().setStepDuration(stepDuration);
            break;
         }
      }
      else if (endPhaseShift < 180.0)
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getAmbleSlowTimings().setStepDuration(stepDuration);
            break;
         case MEDIUM:
            xGaitSettings.getAmbleMediumTimings().setStepDuration(stepDuration);
            break;
         default:
            xGaitSettings.getAmbleFastTimings().setStepDuration(stepDuration);
            break;
         }
      }
      else
      {
         switch (speed)
         {
         case SLOW:
            xGaitSettings.getTrotSlowTimings().setStepDuration(stepDuration);
            break;
         case MEDIUM:
            xGaitSettings.getTrotMediumTimings().setStepDuration(stepDuration);
            break;
         default:
            xGaitSettings.getTrotFastTimings().setStepDuration(stepDuration);
            break;
         }
      }
      publishXGaitSettings(xGaitSettings);
   }

   private void updateRobotModel()
   {
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.getAndSet(null);
      if(robotConfigurationData != null)
      {
         robotDataReceiver.receivedPacket(robotConfigurationData);
         robotDataReceiver.updateRobotModel();
         timestamp.set(1e-9 * robotConfigurationData.getMonotonicTime());
      }
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shift)
   {
      networkProcessor.setShiftPlanBasedOnStepAdjustment(shift);
   }

   public void setXGaitSettings(QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      this.xGaitSettings.set(xGaitSettings);
      publishXGaitSettings(this.xGaitSettings);
   }

   public QuadrupedXGaitSettingsBasics getXGaitSettings()
   {
      return xGaitSettings;
   }

   public ROS2Node getROS2Node()
   {
      return ros2Node;
   }
}