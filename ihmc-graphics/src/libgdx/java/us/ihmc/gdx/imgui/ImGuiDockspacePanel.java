package us.ihmc.gdx.imgui;

import imgui.flag.ImGuiDockNodeFlags;
import imgui.flag.ImGuiStyleVar;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;

public class ImGuiDockspacePanel extends ImGuiPanelSizeHandler
{
   private final String name;
   private final ImBoolean isShowing = new ImBoolean(true);

   public ImGuiDockspacePanel(String name)
   {
      this.name = name;
   }

   public void renderPanel()
   {
      if (isShowing.get())
      {
         // Info here: https://github.com/ocornut/imgui/blob/docking/imgui_demo.cpp#L7408
         int flags2 = ImGuiDockNodeFlags.None;
         flags2 += ImGuiDockNodeFlags.PassthruCentralNode;
         //      flags2 += ImGuiDockNodeFlags.AutoHideTabBar;
         ImGui.pushStyleVar(ImGuiStyleVar.WindowPadding, 0.0f, 0.0f);
         handleSizeBeforeBegin();
         ImGui.begin(name, isShowing);
         handleSizeAfterBegin();
         ImGui.popStyleVar();
         int id = ImGui.getID(name);
         ImGui.dockSpace(id, 0, 0, flags2);
         ImGui.end();
      }
   }

   public void renderMenuItem()
   {
      ImGui.checkbox(name + "###DockspacePanel" + name, isShowing);
   }

   public ImBoolean getIsShowing()
   {
      return isShowing;
   }

   public String getName()
   {
      return name;
   }
}
