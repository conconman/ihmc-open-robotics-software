package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the multi-contact feasibility module
       * This message describes how a keyframe is parameterized in terms of kinematic objectives
       */
public class MultiContactKinematicsSnapshot extends Packet<MultiContactKinematicsSnapshot> implements Settable<MultiContactKinematicsSnapshot>, EpsilonComparable<MultiContactKinematicsSnapshot>
{
   /**
            * The keyframe kinematics snapshot
            */
   public toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage kinematics_snapshot_;
   /**
            * The output of the IK toolbox
            */
   public toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus kinematics_solution_;
   /**
            * Which rigid bodies are in contact in kinematics_snapshot
            */
   public us.ihmc.idl.IDLSequence.Boolean  in_contact_;
   /**
            * The surface normals of the contacting rigid bodies
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  surface_normals_;

   public MultiContactKinematicsSnapshot()
   {
      kinematics_snapshot_ = new toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage();
      kinematics_solution_ = new toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus();
      in_contact_ = new us.ihmc.idl.IDLSequence.Boolean (20, "type_7");

      surface_normals_ = new us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D> (20, new geometry_msgs.msg.dds.Vector3PubSubType());

   }

   public MultiContactKinematicsSnapshot(MultiContactKinematicsSnapshot other)
   {
      this();
      set(other);
   }

   public void set(MultiContactKinematicsSnapshot other)
   {
      toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessagePubSubType.staticCopy(other.kinematics_snapshot_, kinematics_snapshot_);
      toolbox_msgs.msg.dds.KinematicsToolboxOutputStatusPubSubType.staticCopy(other.kinematics_solution_, kinematics_solution_);
      in_contact_.set(other.in_contact_);
      surface_normals_.set(other.surface_normals_);
   }


   /**
            * The keyframe kinematics snapshot
            */
   public toolbox_msgs.msg.dds.KinematicsToolboxInputCollectionMessage getKinematicsSnapshot()
   {
      return kinematics_snapshot_;
   }


   /**
            * The output of the IK toolbox
            */
   public toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus getKinematicsSolution()
   {
      return kinematics_solution_;
   }


   /**
            * Which rigid bodies are in contact in kinematics_snapshot
            */
   public us.ihmc.idl.IDLSequence.Boolean  getInContact()
   {
      return in_contact_;
   }


   /**
            * The surface normals of the contacting rigid bodies
            */
   public us.ihmc.idl.IDLSequence.Object<us.ihmc.euclid.tuple3D.Vector3D>  getSurfaceNormals()
   {
      return surface_normals_;
   }


   public static Supplier<MultiContactKinematicsSnapshotPubSubType> getPubSubType()
   {
      return MultiContactKinematicsSnapshotPubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactKinematicsSnapshotPubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactKinematicsSnapshot other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.kinematics_snapshot_.epsilonEquals(other.kinematics_snapshot_, epsilon)) return false;
      if (!this.kinematics_solution_.epsilonEquals(other.kinematics_solution_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBooleanSequence(this.in_contact_, other.in_contact_, epsilon)) return false;

      if (this.surface_normals_.size() != other.surface_normals_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.surface_normals_.size(); i++)
         {  if (!this.surface_normals_.get(i).epsilonEquals(other.surface_normals_.get(i), epsilon)) return false; }
      }


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactKinematicsSnapshot)) return false;

      MultiContactKinematicsSnapshot otherMyClass = (MultiContactKinematicsSnapshot) other;

      if (!this.kinematics_snapshot_.equals(otherMyClass.kinematics_snapshot_)) return false;
      if (!this.kinematics_solution_.equals(otherMyClass.kinematics_solution_)) return false;
      if (!this.in_contact_.equals(otherMyClass.in_contact_)) return false;
      if (!this.surface_normals_.equals(otherMyClass.surface_normals_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactKinematicsSnapshot {");
      builder.append("kinematics_snapshot=");
      builder.append(this.kinematics_snapshot_);      builder.append(", ");
      builder.append("kinematics_solution=");
      builder.append(this.kinematics_solution_);      builder.append(", ");
      builder.append("in_contact=");
      builder.append(this.in_contact_);      builder.append(", ");
      builder.append("surface_normals=");
      builder.append(this.surface_normals_);
      builder.append("}");
      return builder.toString();
   }
}
