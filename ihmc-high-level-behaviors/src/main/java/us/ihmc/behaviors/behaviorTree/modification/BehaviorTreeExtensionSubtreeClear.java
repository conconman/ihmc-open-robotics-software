package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeExtensionSubtreeClear<T extends BehaviorTreeNodeExtension<T, E, ?, ?>,
                                               E extends BehaviorTreeNode<E>>
      extends BehaviorTreeSubtreeClear<T>
      implements BehaviorTreeModification<T>
{
   private final BehaviorTreeSubtreeClear<E> extensionSubtreeClear;

   public BehaviorTreeExtensionSubtreeClear(T subtreeToClear)
   {
      super(subtreeToClear);

      if (subtreeToClear.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedSubtreeToClear)
      {
         // This will result in recuresively performing the modification on all extended types
         extensionSubtreeClear = new BehaviorTreeExtensionSubtreeClear(extendedSubtreeToClear);
      }
      else
      {
         extensionSubtreeClear = new BehaviorTreeSubtreeClear<>(subtreeToClear.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      super.performOperation();

      extensionSubtreeClear.performOperation();
   }
}

