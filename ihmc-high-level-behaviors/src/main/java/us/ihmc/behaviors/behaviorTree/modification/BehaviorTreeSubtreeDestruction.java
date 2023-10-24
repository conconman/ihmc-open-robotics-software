package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

/**
 * Clearing the subtree and detroying the removed nodes.
 */
public class BehaviorTreeSubtreeDestruction<T extends BehaviorTreeNode<T>> implements BehaviorTreeModification<T>
{
   private final T subtreeToClear;

   public BehaviorTreeSubtreeDestruction(T subtreeToClear)
   {
      this.subtreeToClear = subtreeToClear;
   }

   @Override
   public void performOperation()
   {
      clearChildren(subtreeToClear);
   }

   private void clearChildren(T localNode)
   {
      for (T child : localNode.getChildren())
      {
         clearChildren(child);
      }

      localNode.getChildren().clear();

      if (localNode instanceof BehaviorTreeNodeExtension nodeExtension)
         nodeExtension.destroy();
   }
}

