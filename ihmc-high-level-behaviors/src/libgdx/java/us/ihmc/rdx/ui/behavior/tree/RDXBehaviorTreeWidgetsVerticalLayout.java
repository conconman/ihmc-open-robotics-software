package us.ihmc.rdx.ui.behavior.tree;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeTopologyOperationQueue;
import us.ihmc.behaviors.behaviorTree.topology.BehaviorTreeNodeInsertionType;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.*;

public class RDXBehaviorTreeWidgetsVerticalLayout
{
   private final ImGuiExpandCollapseRenderer expandCollapseRenderer = new ImGuiExpandCollapseRenderer();
   private final RDXBehaviorTree tree;
   private final BehaviorTreeTopologyOperationQueue topologyOperationQueue;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private BehaviorTreeNodeInsertionType insertionType = null;
   private RDXBehaviorTreeNode<?, ?> modalPopupNode;
   private final TypedNotification<Runnable> queuePopupModal = new TypedNotification<>();

   public RDXBehaviorTreeWidgetsVerticalLayout(RDXBehaviorTree tree)
   {
      this.tree = tree;

      topologyOperationQueue = tree.getBehaviorTreeState().getTopologyChangeQueue();
   }

   public void renderImGuiWidgets(RDXBehaviorTreeNode<?, ?> node)
   {
      if (expandCollapseRenderer.render(node.getTreeWidgetExpanded()))
      {
         node.setTreeWidgetExpanded(!node.getTreeWidgetExpanded());
      }

      ImGui.sameLine();
      node.renderTreeViewIconArea();

      ImGui.sameLine();
      node.renderNodeDescription();

      if (ImGui.beginPopup(node.getNodePopupID()))
      {
         if (ImGui.menuItem("Rename..."))
         {
            node.enterEditDescriptionMode();
         }
         ImGui.separator();
         if (!node.isRootNode())
         {
            if (ImGui.menuItem("Insert Node Before..."))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_BEFORE));
            }
            if (ImGui.menuItem("Insert Node After..."))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_AFTER));
            }
         }
         if (node.getChildren().isEmpty())
         {
            if (ImGui.menuItem("Add Child Node..."))
            {
               queuePopupModal.set(() -> popNodeCreationModalDialog(node, BehaviorTreeNodeInsertionType.INSERT_AS_CHILD));
            }
         }
         if (!(node.isRootNode() && !node.getChildren().isEmpty()))
         {
            ImGui.separator();
         }

         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
         if (ImGui.menuItem("Delete Node"))
         {
            topologyOperationQueue.queueDestroySubtree(node);

            if (node.isRootNode()) // Root node
            {
               tree.setRootNode(null);
               tree.getBehaviorTreeState().freeze();
            }
         }
         ImGui.popStyleColor();

         ImGui.separator();
         if (ImGui.menuItem("Cancel"))
            ImGui.closeCurrentPopup();

         ImGui.endPopup();
      }

      // This doesn't work if done inside the other popup
      if (queuePopupModal.poll())
         queuePopupModal.read().run();

      renderNodeCreationModalDialog(node);

      if (node.getTreeWidgetExpanded())
      {
         float indentAmount = 10.0f;
         ImGui.indent(indentAmount);

         node.renderImGuiWidgets();

         for (RDXBehaviorTreeNode<?, ?> child : node.getChildren())
         {
            renderImGuiWidgets(child);
         }

         ImGui.unindent(indentAmount);
      }
   }

   private void popNodeCreationModalDialog(RDXBehaviorTreeNode<?, ?> node, BehaviorTreeNodeInsertionType insertionType)
   {
      this.modalPopupNode = node;
      this.insertionType = insertionType;
      ImGui.openPopup(node.getModalPopupID());
      LogTools.info("Opening popup {}", node.getModalPopupID());
   }

   private void renderNodeCreationModalDialog(RDXBehaviorTreeNode<?, ?> node)
   {
      if (ImGui.beginPopupModal(node.getModalPopupID()))
      {
         tree.getNodeCreationMenu().renderImGuiWidgets(modalPopupNode, insertionType);

         ImGui.separator();
         if (ImGui.button("Cancel"))
         {
            ImGui.closeCurrentPopup();
         }
         ImGui.endPopup();
      }
   }
}
