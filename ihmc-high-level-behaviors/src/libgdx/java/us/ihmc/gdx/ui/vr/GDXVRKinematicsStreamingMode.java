package us.ihmc.gdx.ui.vr;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.KinematicsStreamingToolboxInputMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiPlot;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.graphics.GDXMultiBodyGraphic;
import us.ihmc.gdx.ui.graphics.GDXReferenceFrameGraphic;
import us.ihmc.gdx.ui.missionControl.RestartableMissionControlProcess;
import us.ihmc.gdx.ui.missionControl.processes.RestartableJavaProcess;
import us.ihmc.gdx.ui.visualizers.ImGuiFrequencyPlot;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.Throttler;

public class GDXVRKinematicsStreamingMode
{
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final RestartableJavaProcess kinematicsStreamingToolboxProcess;
   private GDXMultiBodyGraphic ghostRobotGraphic;
   private FullHumanoidRobotModel ghostFullRobotModel;
   private OneDoFJointBasics[] ghostOneDoFJointsExcludingHands;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean enabled = new ImBoolean(false);
   private IHMCROS2Input<KinematicsToolboxOutputStatus> status;
   private final double streamPeriod = UnitConversions.hertzToSeconds(10.0);
   private final Throttler toolboxInputStreamRateLimiter = new Throttler();
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final SideDependentList<OneDoFJointBasics> wristJoints = new SideDependentList<>();
   private final SideDependentList<ImGuiPlot> wristJointAnglePlots = new SideDependentList<>();
   private PausablePeriodicThread wakeUpThread;
   private final ImBoolean wakeUpThreadRunning = new ImBoolean(false);
   private GDXReferenceFrameGraphic headsetFrameGraphic;
   private final SideDependentList<ModifiableReferenceFrame> handDesiredControlFrames = new SideDependentList<>();
   private final SideDependentList<GDXReferenceFrameGraphic> controllerFrameGraphics = new SideDependentList<>();
   private final SideDependentList<GDXReferenceFrameGraphic> handControlFrameGraphics = new SideDependentList<>();
   private final ImBoolean showReferenceFrameGraphics = new ImBoolean(true);
   private boolean streamToController;
   private final Throttler messageThrottler = new Throttler();
   private final KinematicsRecordingReplaying kinematicsRecorder = new KinematicsRecordingReplaying(enabled);

   private final HandConfiguration[] handConfigurations = {HandConfiguration.OPEN, HandConfiguration.HALF_CLOSE, HandConfiguration.CRUSH};
   private int leftIndex = -1;
   private int rightIndex = -1;

   public GDXVRKinematicsStreamingMode(DRCRobotModel robotModel,
                                       ROS2ControllerHelper ros2ControllerHelper,
                                       RestartableJavaProcess kinematicsStreamingToolboxProcess)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.kinematicsStreamingToolboxProcess = kinematicsStreamingToolboxProcess;
   }

   public void create(GDXVRContext vrContext)
   {
      RobotDefinition ghostRobotDefinition = new RobotDefinition(robotModel.getRobotDefinition());
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.parse("0xDEE934").derive(0.0, 1.0, 1.0, 0.5));
      RobotDefinition.forEachRigidBodyDefinition(ghostRobotDefinition.getRootBodyDefinition(),
                                                 body -> body.getVisualDefinitions().forEach(visual -> visual.setMaterialDefinition(material)));

      ghostFullRobotModel = robotModel.createFullRobotModel();
      ghostOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(ghostFullRobotModel);
      ghostRobotGraphic = new GDXMultiBodyGraphic(robotModel.getSimpleRobotName() + " (IK Preview Ghost)");
      ghostRobotGraphic.loadRobotModelAndGraphics(ghostRobotDefinition, ghostFullRobotModel.getElevator());
      ghostRobotGraphic.setActive(true);
      ghostRobotGraphic.create();

      double length = 0.2;
      headsetFrameGraphic = new GDXReferenceFrameGraphic(length);
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.put(side, new GDXReferenceFrameGraphic(length));
         handControlFrameGraphics.put(side, new GDXReferenceFrameGraphic(length));
         RigidBodyTransform wristToHandControlTransform = robotModel.getUIParameters().getTransformWristToHand(side);
         ModifiableReferenceFrame handDesiredControlFrame = new ModifiableReferenceFrame(vrContext.getController(side).getXForwardZUpControllerFrame());
         //            handDesiredControlFrame.getTransformToParent().set(robotModel.getJointMap().getHandControlFrameToWristTransform(side));
         // Atlas
         //         {
         //            handDesiredControlFrame.getTransformToParent().getTranslation().setX(-0.1);
         //            handDesiredControlFrame.getTransformToParent()
         //                                   .getRotation()
         //                                   .setYawPitchRoll(side == RobotSide.RIGHT ? 0.0 : Math.toRadians(180.0), side.negateIfLeftSide(Math.toRadians(90.0)), 0.0);
         //         }
         // Nadia
         {
            if (side == RobotSide.LEFT)
            {
               handDesiredControlFrame.getTransformToParent().getRotation().setToYawOrientation(Math.PI);
               handDesiredControlFrame.getTransformToParent().getRotation().appendRollRotation(Math.PI / 2.0);
            }
            else
            {
               handDesiredControlFrame.getTransformToParent().getRotation().setToRollOrientation(Math.PI / 2.0);
            }
         }
         handDesiredControlFrame.getReferenceFrame().update();
         handDesiredControlFrames.put(side, handDesiredControlFrame);
         ArmJointName lastWristJoint = robotModel.getJointMap().getArmJointNames()[robotModel.getJointMap().getArmJointNames().length - 1];
         wristJoints.put(side, ghostFullRobotModel.getArmJoint(side, lastWristJoint));
         wristJointAnglePlots.put(side, new ImGuiPlot(labels.get(side + " Hand Joint Angle")));
      }

      status = ros2ControllerHelper.subscribe(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()));

      wakeUpThread = new PausablePeriodicThread(getClass().getSimpleName() + "WakeUpThread", 1.0, true, this::wakeUpToolbox);
   }

   public void processVRInput(GDXVRContext vrContext)
   {
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
      {
         InputDigitalActionData aButton = controller.getAButtonActionData();
         if (aButton.bChanged() && !aButton.bState())
         {
            streamToController = !streamToController;
         }

         // NOTE: Implement hand open close for controller trigger button.
         InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
         if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
         {
            HandConfiguration handConfiguration = nextHandConfiguration(RobotSide.LEFT);
            sendHandCommand(RobotSide.LEFT, handConfiguration);
         }

         // Check if left B button is pressed in order to trigger recording or replay of motion
         InputDigitalActionData bButton = controller.getBButtonActionData();
         kinematicsRecorder.processRecordReplayInput(bButton);
      });


      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
      {
         InputDigitalActionData aButton = controller.getAButtonActionData();
         if (aButton.bChanged() && !aButton.bState())
         {
            setEnabled(!enabled.get());
         }

         // NOTE: Implement hand open close for controller trigger button.
         InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
         if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
         {
            HandConfiguration handConfiguration = nextHandConfiguration(RobotSide.RIGHT);
            sendHandCommand(RobotSide.RIGHT, handConfiguration);
         }
      });

      for (RobotSide side : RobotSide.values)
      {
         vrContext.getController(side).runIfConnected(controller ->
         {
            float currentGripX = controller.getGripActionData().x();
//            if (currentGripX)
            {

            }
//            lastGripX
         });
      }

      if ((enabled.get() || kinematicsRecorder.isReplaying()) && toolboxInputStreamRateLimiter.run(streamPeriod))
      {
         KinematicsStreamingToolboxInputMessage toolboxInputMessage = new KinematicsStreamingToolboxInputMessage();
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side).runIfConnected(controller ->
            {
               KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
               message.setEndEffectorHashCode(ghostFullRobotModel.getHand(side).hashCode());
               tempFramePose.setToZero(handDesiredControlFrames.get(side).getReferenceFrame());
               tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               controllerFrameGraphics.get(side).setToReferenceFrame(controller.getXForwardZUpControllerFrame());
               handControlFrameGraphics.get(side).setToReferenceFrame(handDesiredControlFrames.get(side).getReferenceFrame());
               if (kinematicsRecorder.isReplaying())
                  kinematicsRecorder.replay(tempFramePose); //get values of tempFramePose from replay
               message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
               message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
               message.getControlFrameOrientationInEndEffector().setYawPitchRoll(0.0,
                                                                                 side.negateIfLeftSide(Math.PI / 2.0),
                                                                                 side.negateIfLeftSide(Math.PI / 2.0));
               toolboxInputMessage.getInputs().add().set(message);
               kinematicsRecorder.record(tempFramePose);
            });
         }


//         vrContext.getHeadset().runIfConnected(headset ->
//         {
//            KinematicsToolboxRigidBodyMessage message = new KinematicsToolboxRigidBodyMessage();
//            message.setEndEffectorHashCode(ghostFullRobotModel.getHead().hashCode());
//            tempFramePose.setToZero(headset.getXForwardZUpHeadsetFrame());
//            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
//            headsetFrameGraphic.setToReferenceFrame(headset.getXForwardZUpHeadsetFrame());
//            message.getDesiredPositionInWorld().set(tempFramePose.getPosition());
//            message.getDesiredOrientationInWorld().set(tempFramePose.getOrientation());
//            message.getControlFramePositionInEndEffector().set(0.1, 0.0, 0.0);
//            message.getControlFrameOrientationInEndEffector().setYawPitchRoll(Math.PI / 2.0, 0.0, Math.PI / 2.0);
//            boolean xSelected = false;
//            boolean ySelected = false;
//            boolean zSelected = true;
//            message.getLinearSelectionMatrix().set(MessageTools.createSelectionMatrix3DMessage(xSelected,
//                                                                                               ySelected,
//                                                                                               zSelected,
//                                                                                               ReferenceFrame.getWorldFrame()));
//            toolboxInputMessage.getInputs().add().set(message);
//         });
         if(enabled.get())
            toolboxInputMessage.setStreamToController(streamToController);
         else
            toolboxInputMessage.setStreamToController(kinematicsRecorder.isReplaying());
         toolboxInputMessage.setTimestamp(System.nanoTime());
         ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputCommandTopic(robotModel.getSimpleRobotName()), toolboxInputMessage);
         outputFrequencyPlot.recordEvent();
      }
   }

   public void update(boolean ikStreamingModeEnabled)
   {
      // Safety features!
      if (!ikStreamingModeEnabled)
         streamToController = false;
      if (!enabled.get())
         streamToController = false;

      if (status.getMessageNotification().poll())
      {
         KinematicsToolboxOutputStatus latestStatus = status.getMessageNotification().read();
         statusFrequencyPlot.recordEvent();
         if (latestStatus.getJointNameHash() == -1)
         {
            if (latestStatus.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_FAILURE_MISSING_RCD
             && messageThrottler.run(1.0))
               LogTools.warn("Status update: Toolbox failed initialization, missing RobotConfigurationData.");
            else if (latestStatus.getCurrentToolboxState() == KinematicsToolboxOutputStatus.CURRENT_TOOLBOX_STATE_INITIALIZE_SUCCESSFUL)
               LogTools.info("Status update: Toolbox initialized successfully.");
         }
         else
         {
            ghostFullRobotModel.getRootJoint().setJointPosition(latestStatus.getDesiredRootPosition());
            ghostFullRobotModel.getRootJoint().setJointOrientation(latestStatus.getDesiredRootOrientation());
            for (int i = 0; i < ghostOneDoFJointsExcludingHands.length; i++)
            {
               ghostOneDoFJointsExcludingHands[i].setQ(latestStatus.getDesiredJointAngles().get(i));
            }
            ghostFullRobotModel.getElevator().updateFramesRecursively();
         }
      }
      ghostRobotGraphic.update();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Toggle IK tracking enabled: Right A button");
      ImGui.text("Toggle stream to controller: Left A button");

      kinematicsStreamingToolboxProcess.renderImGuiWidgets();
      ghostRobotGraphic.renderImGuiWidgets();
      if (ImGui.checkbox(labels.get("Kinematics streaming"), enabled))
      {
         setEnabled(enabled.get());
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Reinitialize")))
      {
         reinitializeToolbox();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Wake up")))
      {
         wakeUpToolbox();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Sleep")))
      {
         sleepToolbox();
      }
      // add widgets for recording/replaying motion in VR
      ImGui.text("Start/Stop recording: Press Right B button");
      kinematicsRecorder.renderRecordWidgets(labels);
      ImGui.text("Start/Stop replay: Press Right B button (cannot stream/record if replay)");
      kinematicsRecorder.renderReplayWidgets(labels);
      if (ImGui.checkbox(labels.get("Wake up thread"), wakeUpThreadRunning))
      {
         wakeUpThread.setRunning(wakeUpThreadRunning.get());
      }
      ImGui.text("Streaming to controller: " + streamToController);
      ImGui.text("Output:");
      ImGui.sameLine();
      outputFrequencyPlot.renderImGuiWidgets();
      ImGui.text("Status:");
      ImGui.sameLine();
      statusFrequencyPlot.renderImGuiWidgets();
      for (RobotSide side : RobotSide.values)
      {
         wristJointAnglePlots.get(side).render(wristJoints.get(side).getQ());
      }

      ImGui.checkbox(labels.get("Show reference frames"), showReferenceFrameGraphics);
   }

   public void setEnabled(boolean enabled)
   {
      if (enabled != this.enabled.get())
         this.enabled.set(enabled);
      if (enabled)
      {
         wakeUpToolbox();
         kinematicsRecorder.setReplay(false); //check no concurrency replay and streaming
      }
   }

   private void reinitializeToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.REINITIALIZE.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
   }

   private void wakeUpToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
   }

   private void sleepToolbox()
   {
      ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
      toolboxStateMessage.setRequestedToolboxState(ToolboxState.SLEEP.toByte());
      ros2ControllerHelper.publish(KinematicsStreamingToolboxModule.getInputStateTopic(robotModel.getSimpleRobotName()), toolboxStateMessage);
   }

   public void getVirtualRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (status.hasReceivedFirstMessage())
         ghostRobotGraphic.getRenderables(renderables, pool);
      if (showReferenceFrameGraphics.get())
      {
//         headsetFrameGraphic.getRenderables(renderables, pool);
         for (RobotSide side : RobotSide.values)
         {
            controllerFrameGraphics.get(side).getRenderables(renderables, pool);
            handControlFrameGraphics.get(side).getRenderables(renderables, pool);
         }
      }
   }

   public void destroy()
   {
      ghostRobotGraphic.destroy();
      headsetFrameGraphic.dispose();
      for (RobotSide side : RobotSide.values)
      {
         controllerFrameGraphics.get(side).dispose();
      }
   }

   public void sendHandCommand(RobotSide robotSide, HandConfiguration desiredHandConfiguration)
   {
      ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                   HumanoidMessageTools.createHandDesiredConfigurationMessage(robotSide, desiredHandConfiguration));
   }

   public HandConfiguration nextHandConfiguration(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
      {
         leftIndex++;
         return handConfigurations[leftIndex % handConfigurations.length];
      }
      rightIndex++;
      return handConfigurations[rightIndex % handConfigurations.length];
   }

   public RestartableMissionControlProcess getKinematicsStreamingToolboxProcess()
   {
      return kinematicsStreamingToolboxProcess;
   }
}
