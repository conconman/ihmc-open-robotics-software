package us.ihmc.rdx.imgui;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.tools.Timer;

/**
 * Flashes text between black and a color of choice.
 */
public class ImGuiFlashingText
{
   private static final double PERIOD = 1.0;
   private final Timer timer = new Timer();
   private boolean flashState = false;
   private final int flashColor;

   public ImGuiFlashingText(int flashColor)
   {
      this.flashColor = flashColor;
   }

   public void renderText(String text, boolean flashing)
   {
      if (!timer.isRunning(PERIOD))
      {
         timer.reset();
         flashState = !flashState;
      }

      if (flashing && flashState)
      {
         ImGui.pushStyleColor(ImGuiCol.Text, flashColor);
      }
      ImGui.text(text);
      if (flashing && flashState)
      {
         ImGui.popStyleColor();
      }
   }
}
