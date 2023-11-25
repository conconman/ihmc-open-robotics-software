package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the multi-contact feasibility module
       * This message acts as an input to the module and describes the candidate robot state.
       */
public class MultiContactScripingInputMessage extends Packet<MultiContactScripingInputMessage> implements Settable<MultiContactScripingInputMessage>, EpsilonComparable<MultiContactScripingInputMessage>
{
   /**
            * The current keyframe snapshot
            */
   public toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot snapshot_;
   /**
            * The current keyframe snapshot (optional)
            */
   public toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot previous_snapshot_;
   /**
            * Whether previous_snapshot is populated
            */
   public boolean contains_previous_keyframe_;

   public MultiContactScripingInputMessage()
   {
      snapshot_ = new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot();
      previous_snapshot_ = new toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot();
   }

   public MultiContactScripingInputMessage(MultiContactScripingInputMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactScripingInputMessage other)
   {
      toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.staticCopy(other.snapshot_, snapshot_);
      toolbox_msgs.msg.dds.MultiContactKinematicsSnapshotPubSubType.staticCopy(other.previous_snapshot_, previous_snapshot_);
      contains_previous_keyframe_ = other.contains_previous_keyframe_;

   }


   /**
            * The current keyframe snapshot
            */
   public toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot getSnapshot()
   {
      return snapshot_;
   }


   /**
            * The current keyframe snapshot (optional)
            */
   public toolbox_msgs.msg.dds.MultiContactKinematicsSnapshot getPreviousSnapshot()
   {
      return previous_snapshot_;
   }

   /**
            * Whether previous_snapshot is populated
            */
   public void setContainsPreviousKeyframe(boolean contains_previous_keyframe)
   {
      contains_previous_keyframe_ = contains_previous_keyframe;
   }
   /**
            * Whether previous_snapshot is populated
            */
   public boolean getContainsPreviousKeyframe()
   {
      return contains_previous_keyframe_;
   }


   public static Supplier<MultiContactScripingInputMessagePubSubType> getPubSubType()
   {
      return MultiContactScripingInputMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactScripingInputMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactScripingInputMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.snapshot_.epsilonEquals(other.snapshot_, epsilon)) return false;
      if (!this.previous_snapshot_.epsilonEquals(other.previous_snapshot_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.contains_previous_keyframe_, other.contains_previous_keyframe_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactScripingInputMessage)) return false;

      MultiContactScripingInputMessage otherMyClass = (MultiContactScripingInputMessage) other;

      if (!this.snapshot_.equals(otherMyClass.snapshot_)) return false;
      if (!this.previous_snapshot_.equals(otherMyClass.previous_snapshot_)) return false;
      if(this.contains_previous_keyframe_ != otherMyClass.contains_previous_keyframe_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactScripingInputMessage {");
      builder.append("snapshot=");
      builder.append(this.snapshot_);      builder.append(", ");
      builder.append("previous_snapshot=");
      builder.append(this.previous_snapshot_);      builder.append(", ");
      builder.append("contains_previous_keyframe=");
      builder.append(this.contains_previous_keyframe_);
      builder.append("}");
      return builder.toString();
   }
}
