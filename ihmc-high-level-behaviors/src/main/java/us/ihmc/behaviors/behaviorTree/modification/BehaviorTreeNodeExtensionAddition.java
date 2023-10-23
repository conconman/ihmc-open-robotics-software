package us.ihmc.behaviors.behaviorTree.modification;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeNode;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExtension;

public class BehaviorTreeNodeExtensionAddition<T extends BehaviorTreeNodeExtension<T, U>,
                                               U extends BehaviorTreeNode<U>>
      extends BehaviorTreeNodeAddition<T> implements BehaviorTreeModification<T>
{
   private final BehaviorTreeNodeAddition<U> extendedTypeAddition;

   public BehaviorTreeNodeExtensionAddition(T nodeToAdd, T parent)
   {
      super(nodeToAdd, parent);

      if (nodeToAdd.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedNodeToAdd
       && parent.getExtendedNode() instanceof BehaviorTreeNodeExtension extendedParent)
      {
         // This will result in recuresively performing the modification on all extended types
         extendedTypeAddition = new BehaviorTreeNodeExtensionAddition<>(extendedNodeToAdd, extendedParent);
      }
      else
      {
         extendedTypeAddition = new BehaviorTreeNodeAddition<>(nodeToAdd.getExtendedNode(), parent.getExtendedNode());
      }
   }

   @Override
   public void performOperation()
   {
      extendedTypeAddition.performOperation();

      super.performOperation();
   }
}