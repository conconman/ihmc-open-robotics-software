package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import imgui.internal.ImGui;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import static us.ihmc.avatar.sakeGripper.SakeHandParameters.CLOSED_FINGER_ANGLE;
import static us.ihmc.avatar.sakeGripper.SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS;

/**
 * This slider allows the user to control the Sake hand's finger positions,
 * and reflects the position of the fingers on the robot.
 *
 * When the slider is moved by the user, a GOTO command is sent to the Sake hand
 * causing the hand to move to the target position.
 *
 * The slider's position value is an approximate angle between the two fingertips,
 * 0 degrees when the fingers are touching and 210 degrees when the fingers are fully open.
 */
public class RDXSakeHandPositionSlider
{
   private static final double UPDATE_PERIOD = UnitConversions.hertzToSeconds(10.0);
   private static final double SEND_PERIOD = UnitConversions.hertzToSeconds(5.0);
   private static final double EPSILON = 1E-6;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ROS2SyncedRobotModel syncedRobot;
   private final IHMCROS2Input<SakeHandStatusMessage> handStatusMessage;
   private final CommunicationHelper communicationHelper;
   private final RobotSide handSide;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double valueFromRobot = Double.NaN;
   private final Throttler updateThrottler = new Throttler();
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandPositionSlider(ROS2SyncedRobotModel syncedRobot,
                                    CommunicationHelper communicationHelper,
                                    RobotSide handSide)
   {
      this.syncedRobot = syncedRobot;
      this.communicationHelper = communicationHelper;
      this.handSide = handSide;
      sliderName = handSide.getPascalCaseName() + " angle";

      handStatusMessage = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                 .withTypeName(SakeHandStatusMessage.class),
                                                        message -> message.getRobotSide() == handSide.toByte());
   }

   private void receiveRobotConfigurationData()
   {
      if (updateThrottler.run(UPDATE_PERIOD) && handStatusMessage.hasReceivedFirstMessage())
      {
         valueFromRobot = handStatusMessage.getLatest().getPostionRatio();
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD))
         {
            double positionRatio = sliderValue[0] / Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS);

            SakeHandDesiredCommandMessage message = new SakeHandDesiredCommandMessage();
            message.setRobotSide(handSide.toByte());
            message.setDesiredHandConfiguration((byte) SakeHandCommandOption.GOTO.getCommandNumber());
            message.setPostionRatio(positionRatio);
            message.setTorqueRatio(0.3);

            communicationHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
         }
      }
      else
      {
         sliderValue[0] = (float) (Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS) * valueFromRobot);
      }

      receiveRobotConfigurationData();
   }

   private boolean renderImGuiSliderAndReturnChanged()
   {
      float previousValue = sliderValue[0];
      ImGui.sliderAngle(labels.get(sliderName), sliderValue, 0.0f, (float) MAX_ANGLE_BETWEEN_FINGERS);
      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && !MathTools.epsilonEquals(currentValue, previousValue, EPSILON);
   }
}