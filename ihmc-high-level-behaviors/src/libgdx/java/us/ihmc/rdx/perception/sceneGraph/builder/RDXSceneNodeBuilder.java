package us.ihmc.rdx.perception.sceneGraph.builder;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.perception.sceneGraph.ImGuiSceneNodeComboBox;
import us.ihmc.rdx.perception.sceneGraph.RDXSceneNodeInterface;

public abstract class RDXSceneNodeBuilder<T extends RDXSceneNodeInterface>
{
   protected final SceneGraph sceneGraph;
   protected SceneNode parent;
   protected final ImString name = new ImString();
   protected final ImGuiSceneNodeComboBox sceneNodeComboBox;

   public RDXSceneNodeBuilder(SceneGraph sceneGraph)
   {
      this.sceneGraph = sceneGraph;
      parent = sceneGraph.getRootNode();
      sceneNodeComboBox = new ImGuiSceneNodeComboBox(sceneGraph);
   }

   public SceneNode getParent()
   {
      return parent;
   }

   public void renderImGuiWidgets()
   {
      ImGui.inputText("Name", name);
      sceneNodeComboBox.render();
      parent = sceneNodeComboBox.getSelectedNode();
   }

   public String getRejectionTooltip()
   {
      if (name.isEmpty())
      {
         return "Name field empty";
      }

      if (sceneGraph.getNodeNameList().contains(name.get()))
      {
         return "Name already taken";
      }

      return "";
   }

   public abstract T build();
}
