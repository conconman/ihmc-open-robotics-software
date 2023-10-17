package us.ihmc.rdx.perception.sceneGraph;

import imgui.ImGui;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.rdx.imgui.ImGuiInputDoubleWrapper;
import us.ihmc.rdx.tools.RDXIconTexture;

public class RDXArUcoMarkerNode extends RDXDetectableSceneNode
{
   private static final RDXIconTexture ARUCO_ICON = new RDXIconTexture("icons/sceneNodeIcons/aruco.png");

   private final ArUcoMarkerNode arUcoMarkerNode;
   private final ImGuiInputDoubleWrapper alphaFilterValueSlider;

   public RDXArUcoMarkerNode(ArUcoMarkerNode arUcoMarkerNode)
   {
      super(arUcoMarkerNode);

      this.arUcoMarkerNode = arUcoMarkerNode;

      alphaFilterValueSlider = new ImGuiInputDoubleWrapper("Break frequency:",
                                                           "%.2f",
                                                           0.2,
                                                           5.0,
                                                           arUcoMarkerNode::getBreakFrequency,
                                                           arUcoMarkerNode::setBreakFrequency,
                                                           arUcoMarkerNode::freezeFromModification);
      alphaFilterValueSlider.setWidgetWidth(100.0f);
   }

   @Override
   public void renderImGuiControls(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      super.renderImGuiControls(modificationQueue, sceneGraph);

      ImGui.text("Marker ID: %d   Size: %.2f m".formatted(arUcoMarkerNode.getMarkerID(), arUcoMarkerNode.getMarkerSize()));
      alphaFilterValueSlider.render();
   }

   @Override
   public RDXIconTexture getIcon()
   {
      return ARUCO_ICON;
   }
}
