package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ScrewPrimitiveActionDefinitionMessage extends Packet<ScrewPrimitiveActionDefinitionMessage> implements Settable<ScrewPrimitiveActionDefinitionMessage>, EpsilonComparable<ScrewPrimitiveActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage definition_;
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte robot_side_ = (byte) 255;
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder object_frame_name_;
   /**
            * Transform that expresses the screw axis in the object frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage screw_axis_transform_to_object_;
   /**
            * The magnitude of the rotation component
            */
   public double rotation_;
   /**
            * The magnitude of the translation component
            */
   public double translation_;
   /**
            * The axial torque
            */
   public double axial_torque_;
   /**
            * The axial_force
            */
   public double axial_force_;
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean hold_pose_in_world_;

   public ScrewPrimitiveActionDefinitionMessage()
   {
      definition_ = new behavior_msgs.msg.dds.ActionNodeDefinitionMessage();
      object_frame_name_ = new java.lang.StringBuilder(255);
      screw_axis_transform_to_object_ = new controller_msgs.msg.dds.RigidBodyTransformMessage();
   }

   public ScrewPrimitiveActionDefinitionMessage(ScrewPrimitiveActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(ScrewPrimitiveActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
      robot_side_ = other.robot_side_;

      object_frame_name_.setLength(0);
      object_frame_name_.append(other.object_frame_name_);

      controller_msgs.msg.dds.RigidBodyTransformMessagePubSubType.staticCopy(other.screw_axis_transform_to_object_, screw_axis_transform_to_object_);
      rotation_ = other.rotation_;

      translation_ = other.translation_;

      axial_torque_ = other.axial_torque_;

      axial_force_ = other.axial_force_;

      hold_pose_in_world_ = other.hold_pose_in_world_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.ActionNodeDefinitionMessage getDefinition()
   {
      return definition_;
   }

   /**
            * Specifies the side of the robot that this message refers to.
            */
   public void setRobotSide(byte robot_side)
   {
      robot_side_ = robot_side;
   }
   /**
            * Specifies the side of the robot that this message refers to.
            */
   public byte getRobotSide()
   {
      return robot_side_;
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public void setObjectFrameName(java.lang.String object_frame_name)
   {
      object_frame_name_.setLength(0);
      object_frame_name_.append(object_frame_name);
   }

   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.String getObjectFrameNameAsString()
   {
      return getObjectFrameName().toString();
   }
   /**
            * Name of the frame the this action is expressed in
            */
   public java.lang.StringBuilder getObjectFrameName()
   {
      return object_frame_name_;
   }


   /**
            * Transform that expresses the screw axis in the object frame
            */
   public controller_msgs.msg.dds.RigidBodyTransformMessage getScrewAxisTransformToObject()
   {
      return screw_axis_transform_to_object_;
   }

   /**
            * The magnitude of the rotation component
            */
   public void setRotation(double rotation)
   {
      rotation_ = rotation;
   }
   /**
            * The magnitude of the rotation component
            */
   public double getRotation()
   {
      return rotation_;
   }

   /**
            * The magnitude of the translation component
            */
   public void setTranslation(double translation)
   {
      translation_ = translation;
   }
   /**
            * The magnitude of the translation component
            */
   public double getTranslation()
   {
      return translation_;
   }

   /**
            * The axial torque
            */
   public void setAxialTorque(double axial_torque)
   {
      axial_torque_ = axial_torque;
   }
   /**
            * The axial torque
            */
   public double getAxialTorque()
   {
      return axial_torque_;
   }

   /**
            * The axial_force
            */
   public void setAxialForce(double axial_force)
   {
      axial_force_ = axial_force;
   }
   /**
            * The axial_force
            */
   public double getAxialForce()
   {
      return axial_force_;
   }

   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public void setHoldPoseInWorld(boolean hold_pose_in_world)
   {
      hold_pose_in_world_ = hold_pose_in_world;
   }
   /**
            * Whether maintaining the rigid body controlled in world after the action is complete
            */
   public boolean getHoldPoseInWorld()
   {
      return hold_pose_in_world_;
   }


   public static Supplier<ScrewPrimitiveActionDefinitionMessagePubSubType> getPubSubType()
   {
      return ScrewPrimitiveActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ScrewPrimitiveActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ScrewPrimitiveActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.robot_side_, other.robot_side_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsStringBuilder(this.object_frame_name_, other.object_frame_name_, epsilon)) return false;

      if (!this.screw_axis_transform_to_object_.epsilonEquals(other.screw_axis_transform_to_object_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.rotation_, other.rotation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.translation_, other.translation_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.axial_torque_, other.axial_torque_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.axial_force_, other.axial_force_, epsilon)) return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.hold_pose_in_world_, other.hold_pose_in_world_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ScrewPrimitiveActionDefinitionMessage)) return false;

      ScrewPrimitiveActionDefinitionMessage otherMyClass = (ScrewPrimitiveActionDefinitionMessage) other;

      if (!this.definition_.equals(otherMyClass.definition_)) return false;
      if(this.robot_side_ != otherMyClass.robot_side_) return false;

      if (!us.ihmc.idl.IDLTools.equals(this.object_frame_name_, otherMyClass.object_frame_name_)) return false;

      if (!this.screw_axis_transform_to_object_.equals(otherMyClass.screw_axis_transform_to_object_)) return false;
      if(this.rotation_ != otherMyClass.rotation_) return false;

      if(this.translation_ != otherMyClass.translation_) return false;

      if(this.axial_torque_ != otherMyClass.axial_torque_) return false;

      if(this.axial_force_ != otherMyClass.axial_force_) return false;

      if(this.hold_pose_in_world_ != otherMyClass.hold_pose_in_world_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ScrewPrimitiveActionDefinitionMessage {");
      builder.append("definition=");
      builder.append(this.definition_);      builder.append(", ");
      builder.append("robot_side=");
      builder.append(this.robot_side_);      builder.append(", ");
      builder.append("object_frame_name=");
      builder.append(this.object_frame_name_);      builder.append(", ");
      builder.append("screw_axis_transform_to_object=");
      builder.append(this.screw_axis_transform_to_object_);      builder.append(", ");
      builder.append("rotation=");
      builder.append(this.rotation_);      builder.append(", ");
      builder.append("translation=");
      builder.append(this.translation_);      builder.append(", ");
      builder.append("axial_torque=");
      builder.append(this.axial_torque_);      builder.append(", ");
      builder.append("axial_force=");
      builder.append(this.axial_force_);      builder.append(", ");
      builder.append("hold_pose_in_world=");
      builder.append(this.hold_pose_in_world_);
      builder.append("}");
      return builder.toString();
   }
}
