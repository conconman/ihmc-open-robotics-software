package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the multi-contact feasibility module
       * This message acts as an output of the module and contains the CoM region of the current keyframe.
       */
public class MultiContactCoMRegionMessage extends Packet<MultiContactCoMRegionMessage> implements Settable<MultiContactCoMRegionMessage>, EpsilonComparable<MultiContactCoMRegionMessage>
{
   /**
            * Feasible CoM region of the current keyframe
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  support_polygon_;

   public MultiContactCoMRegionMessage()
   {
      support_polygon_ = new us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage> (20, new ihmc_common_msgs.msg.dds.Point2DMessagePubSubType());

   }

   public MultiContactCoMRegionMessage(MultiContactCoMRegionMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactCoMRegionMessage other)
   {
      support_polygon_.set(other.support_polygon_);
   }


   /**
            * Feasible CoM region of the current keyframe
            */
   public us.ihmc.idl.IDLSequence.Object<ihmc_common_msgs.msg.dds.Point2DMessage>  getSupportPolygon()
   {
      return support_polygon_;
   }


   public static Supplier<MultiContactCoMRegionMessagePubSubType> getPubSubType()
   {
      return MultiContactCoMRegionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactCoMRegionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactCoMRegionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.support_polygon_.size() != other.support_polygon_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.support_polygon_.size(); i++)
         {  if (!this.support_polygon_.get(i).epsilonEquals(other.support_polygon_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactCoMRegionMessage)) return false;

      MultiContactCoMRegionMessage otherMyClass = (MultiContactCoMRegionMessage) other;

      if (!this.support_polygon_.equals(otherMyClass.support_polygon_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactCoMRegionMessage {");
      builder.append("support_polygon=");
      builder.append(this.support_polygon_);
      builder.append("}");
      return builder.toString();
   }
}
