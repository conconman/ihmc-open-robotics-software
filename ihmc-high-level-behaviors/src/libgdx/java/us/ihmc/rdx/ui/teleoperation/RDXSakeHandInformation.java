package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.msg.dds.HandSakeDesiredCommandMessage;
import controller_msgs.msg.dds.HandSakeStatusMessage;
import imgui.ImGui;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.rdx.imgui.ImGuiFlashingText;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.robotSide.RobotSide;

public class RDXSakeHandInformation
{
   private final RobotSide side;
   private final CommunicationHelper communicationHelper;
   private final IHMCROS2Input<HandSakeStatusMessage> statusInput;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiFlashingText calibrateStatusText = new ImGuiFlashingText(ImGuiTools.RED);
   private final ImGuiFlashingText needResetStatusText = new ImGuiFlashingText(ImGuiTools.RED);

   public RDXSakeHandInformation(RobotSide side, CommunicationHelper communicationHelper)
   {
      this.side = side;
      this.communicationHelper = communicationHelper;
      statusInput = communicationHelper.subscribe(ROS2Tools.getControllerOutputTopic(communicationHelper.getRobotName())
                                                           .withTypeName(HandSakeStatusMessage.class),
                                                  message -> message.getRobotSide() == side.toByte());
   }
}
