package us.ihmc.behaviors.behaviorTree;

import org.apache.commons.lang3.mutable.MutableLong;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeExtensionSubtreeRebuilder;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModification;
import us.ihmc.behaviors.behaviorTree.modification.BehaviorTreeModificationQueue;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * This is the state related functionality of a behavior tree,
 * which would live on the UI side and the robot side.
 *
 * The root node is going to be a single basic root node with no functionality
 * and it will never be replaced.
 */
public class BehaviorTreeState
{
   private final MutableLong nextID = new MutableLong(0);
   private final Queue<BehaviorTreeModification> queuedModifications = new LinkedList<>();
   private final BehaviorTreeNodeStateBuilder nodeStateBuilder;
   private final BehaviorTreeExtensionSubtreeRebuilder treeRebuilder;
   private final Supplier<BehaviorTreeNodeExtension> rootNodeSupplier;
   private boolean localTreeFrozen = false;

   public BehaviorTreeState(BehaviorTreeNodeStateBuilder nodeStateBuilder,
                            BehaviorTreeExtensionSubtreeRebuilder treeRebuilder,
                            Supplier<BehaviorTreeNodeExtension> rootNodeSupplier)
   {
      this.nodeStateBuilder = nodeStateBuilder;
      this.treeRebuilder = treeRebuilder;
      this.rootNodeSupplier = rootNodeSupplier;
   }

   public void update()
   {

   }

   public void modifyTree(Consumer<BehaviorTreeModificationQueue> modifier)
   {
      modifier.accept(queuedModifications::add);

      boolean modified = !queuedModifications.isEmpty();

      while (!queuedModifications.isEmpty())
      {
         BehaviorTreeModification modification = queuedModifications.poll();
         modification.performOperation();
      }

      if (modified)
         update();
   }

   public void checkTreeModified()
   {
      localTreeFrozen = false;
      checkTreeModified(rootNodeSupplier.get().getState());
   }

   private void checkTreeModified(BehaviorTreeNodeState localNode)
   {
      localTreeFrozen |= localNode.isFrozenFromModification();

      for (BehaviorTreeNodeState child : localNode.getChildren())
      {
         checkTreeModified(child);
      }
   }

   public MutableLong getNextID()
   {
      return nextID;
   }

   public BehaviorTreeNodeExtension getRootNode()
   {
      return rootNodeSupplier.get();
   }

   public BehaviorTreeNodeStateBuilder getNodeStateBuilder()
   {
      return nodeStateBuilder;
   }

   public BehaviorTreeExtensionSubtreeRebuilder getTreeRebuilder()
   {
      return treeRebuilder;
   }

   public boolean getLocalTreeFrozen()
   {
      return localTreeFrozen;
   }
}
