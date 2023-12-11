package us.ihmc.rdx.ui.interactable;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import controller_msgs.msg.dds.SakeHandStatusMessage;
import imgui.flag.ImGuiCol;
import imgui.internal.ImGui;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sakeHandCommunication.ROS2SakeHandCommander;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

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
   private final ROS2SakeHandCommander handCommander;
   private final IHMCROS2Input<SakeHandStatusMessage> handStatusMessage;
   private final RobotSide handSide;
   private final String sliderName;
   private final float[] sliderValue = new float[1];
   private double measuredPosition = Double.NaN;
   private double desiredTorque = Double.NaN;
   private final Throttler updateThrottler = new Throttler();
   private final Throttler sendThrottler = new Throttler();

   public RDXSakeHandPositionSlider(CommunicationHelper communicationHelper, RobotSide handSide)
   {
      this.handSide = handSide;
      this.handCommander = ROS2SakeHandCommander.getSakeHandCommander(communicationHelper);
      sliderName = handSide.getPascalCaseName() + " angle";

      handStatusMessage = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                                 .withTypeName(SakeHandStatusMessage.class),
                                                        message -> message.getRobotSide() == handSide.toByte());
   }

   private void receiveRobotConfigurationData()
   {
      if (updateThrottler.run(UPDATE_PERIOD) && handStatusMessage.hasReceivedFirstMessage())
      {
         measuredPosition = handStatusMessage.getLatest().getNormalizedMeasuredPosition();
         desiredTorque = handStatusMessage.getLatest().getNormalizedDesiredTorque();
      }
   }

   public void renderImGuiWidgets()
   {
      if (renderImGuiSliderAndReturnChanged())
      {
         if (sendThrottler.run(SEND_PERIOD))
         {
            double desiredPosition = 1.0 - (sliderValue[0] / Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS));

            handCommander.setDesiredPosition(handSide, desiredPosition);
         }
      }
      else
      {
         sliderValue[0] = (float) (Math.toRadians(MAX_ANGLE_BETWEEN_FINGERS) * (1.0 - measuredPosition));
      }

      receiveRobotConfigurationData();
   }

   private boolean renderImGuiSliderAndReturnChanged()
   {
      float previousValue = sliderValue[0];

      if (desiredTorque < 0.2)  // render slider with warning that torque is too low
      {
         ImGui.pushStyleColor(ImGuiCol.SliderGrab, ImGuiTools.RED);
         ImGui.sliderAngle(labels.get(sliderName),
                           sliderValue,
                           0.0f,
                           (float) MAX_ANGLE_BETWEEN_FINGERS,
                           String.format("%.0f deg (Torque Too Low)", Math.toDegrees(sliderValue[0])));
         ImGui.popStyleColor();
      }
      else  // render normal slider
         ImGui.sliderAngle(labels.get(sliderName), sliderValue, 0.0f, (float) MAX_ANGLE_BETWEEN_FINGERS);

      float currentValue = sliderValue[0];
      return !Double.isNaN(sliderValue[0]) && !MathTools.epsilonEquals(currentValue, previousValue, EPSILON);
   }
}