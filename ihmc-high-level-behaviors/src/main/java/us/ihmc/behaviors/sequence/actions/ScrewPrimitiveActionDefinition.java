package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.behaviors.sequence.ActionNodeDefinition;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SidedObject;
import us.ihmc.tools.io.JSONTools;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionDefinition extends ActionNodeDefinition implements SidedObject
{
   private final CRDTUnidirectionalEnumField<RobotSide> side;
   private final CRDTUnidirectionalString objectFrameName;
   private final CRDTUnidirectionalRigidBodyTransform screwAxisTransformToObject;
   /** The magnitude of the rotation component */
   private final CRDTUnidirectionalDouble rotation;
   /** The magnitude of the translation component */
   private final CRDTUnidirectionalDouble translation;
   private final CRDTUnidirectionalDouble axialTorque;
   private final CRDTUnidirectionalDouble axialForce;
   // TODO: Remove?
   private final CRDTUnidirectionalBoolean holdPoseInWorldLater;

   public ScrewPrimitiveActionDefinition(CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(crdtInfo, saveFileDirectory);

      side = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.OPERATOR, crdtInfo, RobotSide.LEFT);
      holdPoseInWorldLater = new CRDTUnidirectionalBoolean(ROS2ActorDesignation.OPERATOR, crdtInfo, true);
      objectFrameName = new CRDTUnidirectionalString(ROS2ActorDesignation.OPERATOR, crdtInfo, ReferenceFrame.getWorldFrame().getName());
      screwAxisTransformToObject = new CRDTUnidirectionalRigidBodyTransform(ROS2ActorDesignation.OPERATOR, crdtInfo);
      rotation = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.0);
      translation = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 0.1);
      axialTorque = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 10.0);
      axialForce = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 10.0);
   }

   @Override
   public void saveToFile(ObjectNode jsonNode)
   {
      super.saveToFile(jsonNode);

      jsonNode.put("side", side.getValue().getLowerCaseName());
      jsonNode.put("objectFrame", objectFrameName.getValue());
      JSONTools.toJSON(jsonNode, screwAxisTransformToObject.getValueReadOnly());
      jsonNode.put("rotation", rotation.getValue());
      jsonNode.put("translation", translation.getValue());
      jsonNode.put("axialTorque", axialTorque.getValue());
      jsonNode.put("axialForce", axialForce.getValue());
      jsonNode.put("holdPoseInWorldLater", holdPoseInWorldLater.getValue());
   }

   @Override
   public void loadFromFile(JsonNode jsonNode)
   {
      super.loadFromFile(jsonNode);

      side.setValue(RobotSide.getSideFromString(jsonNode.get("side").asText()));
      objectFrameName.setValue(jsonNode.get("objectFrame").textValue());
      JSONTools.toEuclid(jsonNode, screwAxisTransformToObject.getValue());
      rotation.setValue(jsonNode.get("rotation").asDouble());
      translation.setValue(jsonNode.get("translation").asDouble());
      axialTorque.setValue(jsonNode.get("axialTorque").asDouble());
      axialForce.setValue(jsonNode.get("axialForce").asDouble());
      holdPoseInWorldLater.setValue(jsonNode.get("holdPoseInWorldLater").asBoolean());
   }

   public void toMessage(ScrewPrimitiveActionDefinitionMessage message)
   {
      super.toMessage(message.getDefinition());

      message.setRobotSide(side.toMessage().toByte());
      message.setObjectFrameName(objectFrameName.toMessage());
      screwAxisTransformToObject.toMessage(message.getScrewAxisTransformToObject());
      message.setRotation(rotation.toMessage());
      message.setTranslation(translation.toMessage());
      message.setAxialTorque(axialTorque.toMessage());
      message.setAxialForce(axialForce.toMessage());
      message.setHoldPoseInWorld(holdPoseInWorldLater.toMessage());
   }

   public void fromMessage(ScrewPrimitiveActionDefinitionMessage message)
   {
      super.fromMessage(message.getDefinition());

      side.fromMessage(RobotSide.fromByte(message.getRobotSide()));
      objectFrameName.fromMessage(message.getObjectFrameNameAsString());
      screwAxisTransformToObject.fromMessage(message.getScrewAxisTransformToObject());
      rotation.fromMessage(message.getRotation());
      translation.fromMessage(message.getTranslation());
      axialTorque.fromMessage(message.getAxialTorque());
      axialForce.fromMessage(message.getAxialForce());
      holdPoseInWorldLater.fromMessage(message.getHoldPoseInWorld());
   }

   @Override
   public RobotSide getSide()
   {
      return side.getValue();
   }

   public void setSide(RobotSide side)
   {
      this.side.setValue(side);
   }

   public String getObjectFrameName()
   {
      return objectFrameName.getValue();
   }

   public void setObjectFrameName(String objectFrameName)
   {
      this.objectFrameName.setValue(objectFrameName);
   }

   public CRDTUnidirectionalRigidBodyTransform getScrewAxisTransformToObject()
   {
      return screwAxisTransformToObject;
   }

   public double getRotation()
   {
      return rotation.getValue();
   }

   public void setRotation(double rotation)
   {
      this.rotation.setValue(rotation);
   }

   public double getTranslation()
   {
      return translation.getValue();
   }

   public void setTranslation(double translation)
   {
      this.translation.setValue(translation);
   }

   public double getAxialTorque()
   {
      return axialTorque.getValue();
   }

   public void setAxialTorque(double axialTorque)
   {
      this.axialTorque.setValue(axialTorque);
   }

   public double getAxialForce()
   {
      return axialForce.getValue();
   }

   public void setAxialForce(double axialForce)
   {
      this.axialForce.setValue(axialForce);
   }

   public boolean getHoldPoseInWorldLater()
   {
      return holdPoseInWorldLater.getValue();
   }

   public void setHoldPoseInWorldLater(boolean holdPoseInWorldLater)
   {
      this.holdPoseInWorldLater.setValue(holdPoseInWorldLater);
   }
}
