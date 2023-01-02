package us.ihmc.rdx.ui.behavior.editor.actions;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.ImGui;
import us.ihmc.behaviors.sequence.actions.ChestOrientationActionData;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXChestOrientationAction extends RDXBehaviorAction
{
   private final ChestOrientationActionData action = new ChestOrientationActionData();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper yawWidget = new ImDoubleWrapper(() -> action.getYawPitchRoll().getYaw(),
                                                                 yaw -> action.getYawPitchRoll().setYaw(yaw),
                                                                 imDouble -> ImGui.inputDouble(labels.get("Yaw"), imDouble));
   private final ImDoubleWrapper pitchWidget = new ImDoubleWrapper(() -> action.getYawPitchRoll().getPitch(),
                                                                   pitch -> action.getYawPitchRoll().setPitch(pitch),
                                                                   imDouble -> ImGui.inputDouble(labels.get("Pitch"), imDouble));
   private final ImDoubleWrapper rollWidget = new ImDoubleWrapper(() -> action.getYawPitchRoll().getRoll(),
                                                                  roll -> action.getYawPitchRoll().setRoll(roll),
                                                                  imDouble -> ImGui.inputDouble(labels.get("Roll"), imDouble));
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(action::getTrajectoryDuration,
                                                                                action::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));

   public RDXChestOrientationAction()
   {
      super("Chest Orientation");
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      yawWidget.renderImGuiWidget();
      ImGui.sameLine();
      pitchWidget.renderImGuiWidget();
      ImGui.sameLine();
      rollWidget.renderImGuiWidget();
      trajectoryDurationWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      action.saveToFile(jsonNode);
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      action.loadFromFile(jsonNode);
   }
}
