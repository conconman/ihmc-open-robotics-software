package us.ihmc.perception.sceneGraph.ros2;

import perception_msgs.msg.dds.PredefinedRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.PrimitiveRigidBodySceneNodeMessage;
import perception_msgs.msg.dds.SceneGraphMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.sceneGraph.DetectableSceneNode;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.perception.sceneGraph.arUco.ArUcoMarkerNode;
import us.ihmc.perception.sceneGraph.centerpose.CenterposeNode;
import us.ihmc.perception.sceneGraph.rigidBody.PredefinedRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.StaticRelativeSceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodySceneNode;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

public class ROS2SceneGraphTools
{
   public static SceneNode createNodeFromMessage(ROS2SceneGraphSubscriptionNode subscriptionNode, SceneGraph sceneGraph)
   {
      SceneNode sceneNode;

      byte nodeType = subscriptionNode.getType();
      long nodeID = subscriptionNode.getSceneNodeMessage().getId();
      String nodeName = subscriptionNode.getSceneNodeMessage().getNameAsString();

      if (nodeType == SceneGraphMessage.PREDEFINED_RIGID_BODY_NODE_TYPE || nodeType == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
      {
         PredefinedRigidBodySceneNodeMessage predefinedRigidBodySceneNodeMessage = subscriptionNode.getPredefinedRigidBodySceneNodeMessage();
         RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(predefinedRigidBodySceneNodeMessage.getInitialTransformToParent(), initialTransformToParent);
         RigidBodyTransform visualTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(predefinedRigidBodySceneNodeMessage.getVisualTransformToParent(), visualTransformToParent);

         if (nodeType == SceneGraphMessage.STATIC_RELATIVE_NODE_TYPE)
         {
            sceneNode = new StaticRelativeSceneNode(nodeID,
                                                    nodeName,
                                                    sceneGraph.getIDToNodeMap(),
                                                    predefinedRigidBodySceneNodeMessage.getInitialParentId(),
                                                    initialTransformToParent,
                                                    predefinedRigidBodySceneNodeMessage.getVisualModelFilePathAsString(),
                                                    visualTransformToParent,
                                                    subscriptionNode.getStaticRelativeSceneNodeMessage().getDistanceToDisableTracking());
         }
         else // PREDEFINED_RIGID_BODY_NODE_TYPE
         {
            sceneNode = new PredefinedRigidBodySceneNode(nodeID,
                                                         nodeName,
                                                         sceneGraph.getIDToNodeMap(),
                                                         predefinedRigidBodySceneNodeMessage.getInitialParentId(),
                                                         initialTransformToParent,
                                                         predefinedRigidBodySceneNodeMessage.getVisualModelFilePathAsString(),
                                                         visualTransformToParent);
         }
      }
      else if (nodeType == SceneGraphMessage.ARUCO_MARKER_NODE_TYPE)
      {
         sceneNode = new ArUcoMarkerNode(nodeID,
                                         nodeName,
                                         subscriptionNode.getArUcoMarkerNodeMessage().getMarkerId(),
                                         subscriptionNode.getArUcoMarkerNodeMessage().getMarkerSize());
      }
      else if (nodeType == SceneGraphMessage.CENTERPOSE_NODE_TYPE)
      {
         sceneNode = new CenterposeNode(nodeID,
                                        nodeName,
                                        subscriptionNode.getCenterposeNodeMessage().getObjectId(),
                                        subscriptionNode.getCenterposeNodeMessage().getBoundingBoxVertices(),
                                        subscriptionNode.getCenterposeNodeMessage().getBoundingBox2dVertices());
      }
      else if (nodeType == SceneGraphMessage.DETECTABLE_SCENE_NODE_TYPE)
      {
         sceneNode = new DetectableSceneNode(nodeID, nodeName);
      }
      else if (nodeType == SceneGraphMessage.PRIMITIVE_RIGID_BODY_NODE_TYPE)
      {
         PrimitiveRigidBodySceneNodeMessage primitiveRigidBodySceneNodeMessage = subscriptionNode.getPrimitiveRigidBodySceneNodeMessage();
         RigidBodyTransform initialTransformToParent = new RigidBodyTransform();
         MessageTools.toEuclid(primitiveRigidBodySceneNodeMessage.getInitialTransformToParent(), initialTransformToParent);
         sceneNode = new PrimitiveRigidBodySceneNode(nodeID,
                                                     nodeName,
                                                     sceneGraph.getIDToNodeMap(),
                                                     primitiveRigidBodySceneNodeMessage.getInitialParentId(),
                                                     initialTransformToParent,
                                                     PrimitiveRigidBodyShape.fromString(primitiveRigidBodySceneNodeMessage.getShapeAsString()));
      }
      else
      {
         sceneNode = new SceneNode(nodeID, nodeName);
      }

      sceneGraph.getIDToNodeMap().put(nodeID, sceneNode); // Make sure any new nodes are in the map // TODO: Probably remove not necessary

      return sceneNode;
   }
}
