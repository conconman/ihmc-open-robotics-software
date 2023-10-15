package us.ihmc.behaviors.tools;

import behavior_msgs.msg.dds.BehaviorTreeNodeMessage;
import behavior_msgs.msg.dds.BehaviorTreeMessage;
import org.apache.commons.lang3.mutable.MutableInt;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.communication.packets.MessageTools;

public class BehaviorMessageTools
{
   /**
    * Pack a behavior tree into a ROS 2 message. We can have recursive fields in our
    * messages, so we pack the tree in depth-first order.
    *
    * TODO: This is going to have to be fixed to pack different node types in
    *   appropriate fields in the ROS 2 message.
    */
   public static void packBehaviorTreeMessage(BehaviorTreeNodeState treeNode, BehaviorTreeMessage behaviorTreeMessage)
   {
      BehaviorTreeNodeMessage nodeMessage = behaviorTreeMessage.getNodes().add();
      if (treeNode.getLastTickInstant()  != null)
         MessageTools.toMessage(treeNode.getLastTickInstant(), nodeMessage.getLastTickInstant());
      nodeMessage.setNodeName(treeNode.getName());
      if (treeNode.getStatus() != null)
         nodeMessage.setPreviousStatus((byte) treeNode.getStatus().ordinal());
      else
         nodeMessage.setPreviousStatus((byte) -1);

      if (treeNode instanceof BehaviorTreeControlFlowNode controlFlowTreeNode)
      {
         nodeMessage.setNumberOfChildren(controlFlowTreeNode.getChildren().size());
         nodeMessage.setHasBeenClocked(controlFlowTreeNode.getHasBeenClocked());

         for (BehaviorTreeNodeState child : controlFlowTreeNode.getChildren())
         {
            packBehaviorTreeMessage(child, behaviorTreeMessage);
         }
      }
      else
      {
         nodeMessage.setNumberOfChildren(0);
      }
   }

   /**
    * We unpack a tree from a list of nodes using recursion and the number of
    * children of that node. We assume the ordering as packed in {@link #packBehaviorTreeMessage}.
    */
   public static BehaviorTreeNodeState unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage)
   {
      return unpackBehaviorTreeMessage(behaviorTreeMessage, new MutableInt());
   }

   private static BehaviorTreeNodeState unpackBehaviorTreeMessage(BehaviorTreeMessage behaviorTreeMessage, MutableInt nodeIndex)
   {
      BehaviorTreeNodeMessage treeNodeMessage = behaviorTreeMessage.getNodes().get(nodeIndex.getAndIncrement());

      BehaviorTreeStatusNode behaviorTreeStatusNode = createBehaviorTreeNode(treeNodeMessage.getNodeTypeAsString());
      // The message will have 0s for a node that has not yet been ticked
      if (treeNodeMessage.getLastTickInstant().getSecondsSinceEpoch() != 0)
         behaviorTreeStatusNode.setLastTickInstant(MessageTools.toInstant(treeNodeMessage.getLastTickInstant()));
      behaviorTreeStatusNode.setName(treeNodeMessage.getNodeNameAsString());
      // Previous status will be -1 if the node has not been ticked yet
      if (treeNodeMessage.getPreviousStatus() >= 0)
         behaviorTreeStatusNode.setPreviousStatus(BehaviorTreeNodeStatus.fromByte(treeNodeMessage.getPreviousStatus()));
      behaviorTreeStatusNode.setHasBeenClocked(treeNodeMessage.getHasBeenClocked());

      for (int i = 0; i < treeNodeMessage.getNumberOfChildren(); i++)
      {
         behaviorTreeStatusNode.getChildren().add(unpackBehaviorTreeMessage(behaviorTreeMessage, nodeIndex));
      }

      return behaviorTreeStatusNode;
   }

   private static BehaviorTreeStatusNode createBehaviorTreeNode(String typeName)
   {
      BehaviorTreeStatusNode behaviorTreeStatusNode = new BehaviorTreeStatusNode();
      // TODO: We need to instantiate certain types of status nodes instead.
      //  They will need different functionality depending on their type.
//      if (typeName.equals(SequenceNode.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(SequenceNode.class);
//      else if (typeName.equals(FallbackNode.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(FallbackNode.class);
//      else if (typeName.equals(AsynchronousActionNode.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(AsynchronousActionNode.class);
//      else if (typeName.equals(BehaviorTreeNodeState.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(BehaviorTreeNodeState.class);
//      else if (typeName.equals(BehaviorTreeCondition.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(BehaviorTreeCondition.class);
//      else if (typeName.equals(OneShotAction.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(OneShotAction.class);
//      else if (typeName.equals(AlwaysSuccessfulAction.class.getSimpleName()))
//         behaviorTreeStatusNode.setType(AlwaysSuccessfulAction.class);
//      else
//         behaviorTreeStatusNode.setType(BehaviorTreeNodeState.class);
      return behaviorTreeStatusNode;
   }
}
