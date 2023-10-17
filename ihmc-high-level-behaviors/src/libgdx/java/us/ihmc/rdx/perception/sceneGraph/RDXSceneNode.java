package us.ihmc.rdx.perception.sceneGraph;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.flag.ImGuiTableColumnFlags;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.modification.SceneGraphClearSubtree;
import us.ihmc.perception.sceneGraph.modification.SceneGraphModificationQueue;
import us.ihmc.perception.sceneGraph.modification.SceneGraphNodeRemoval;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.RDXIconTexture;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

import java.util.Set;

public class RDXSceneNode
{
   private static final RDXIconTexture UNKNOWN_ICON = new RDXIconTexture("icons/sceneNodeIcons/unknown.png");

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private final SceneNode sceneNode;
   private final RDXReferenceFrameGraphic referenceFrameGraphic;
   private final String detailsText;

   public RDXSceneNode(SceneNode sceneNode)
   {
      this.sceneNode = sceneNode;
      referenceFrameGraphic = new RDXReferenceFrameGraphic(0.05, Color.BLUE);
      detailsText = "ID: %d, Type: %s".formatted(sceneNode.getID(), sceneNode.getClass().getSuperclass().getSimpleName());
   }

   public void update(SceneGraphModificationQueue modificationQueue)
   {
      referenceFrameGraphic.setToReferenceFrame(sceneNode.getNodeFrame());
   }

   public void renderImGuiPreview(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
//      if (ImGui.beginTable(labels.get("sceneNode"), 2))
//      {
//         ImGui.tableSetupColumn(labels.get("##sceneNodeName"), ImGuiTableColumnFlags.WidthStretch);
//         ImGui.tableSetupColumn(labels.get("##sceneNodeDelete"), ImGuiTableColumnFlags.WidthStretch);
//         ImGui.tableNextRow();
//         ImGui.tableSetColumnIndex(0);
//         ImGuiTools.textBold(sceneNode.getName());
//         ImGui.tableSetColumnIndex(1);
//
//         float buttonWidth = 60;
//         ImGui.setCursorPosX(ImGui.getCursorPosX() + ImGui.getContentRegionAvail().x - buttonWidth);
//         if (ImGui.button("Delete", buttonWidth, 0))
//         {
//
//         }
//         ImGui.endTable();
//      }
   }

   public void renderImGuiControls(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      ImGui.text(detailsText);

      if (sceneNode != sceneGraph.getRootNode()) // Don't allow removing root node
      {
         if (ImGui.button("Remove##" + sceneNode.getID()))
         {
            remove(modificationQueue, sceneGraph);
         }
      }
   }

   public RDXIconTexture getIcon()
   {
      return UNKNOWN_ICON;
   }

   public void renderIcon()
   {
      ImGui.image(getIcon().getTexture().getTextureObjectHandle(), 14f, 14f);
   }

   public void remove(SceneGraphModificationQueue modificationQueue, SceneGraph sceneGraph)
   {
      modificationQueue.accept(new SceneGraphClearSubtree(sceneNode));
      modificationQueue.accept(new SceneGraphNodeRemoval(sceneNode, sceneGraph));
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {
      if (sceneLevels.contains(RDXSceneLevel.VIRTUAL))
         referenceFrameGraphic.getRenderables(renderables, pool);
   }

   public SceneNode getSceneNode()
   {
      return sceneNode;
   }
}
