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
   public controller_msgs.msg.dds.Polygon2DMessage com_feasibility_region_;
   /**
            * Feasible CoM region for contact removability
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Polygon2DMessage>  com_feasibility_preview_regions_;

   public MultiContactCoMRegionMessage()
   {
      com_feasibility_region_ = new controller_msgs.msg.dds.Polygon2DMessage();
      com_feasibility_preview_regions_ = new us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Polygon2DMessage> (15, new controller_msgs.msg.dds.Polygon2DMessagePubSubType());

   }

   public MultiContactCoMRegionMessage(MultiContactCoMRegionMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactCoMRegionMessage other)
   {
      controller_msgs.msg.dds.Polygon2DMessagePubSubType.staticCopy(other.com_feasibility_region_, com_feasibility_region_);
      com_feasibility_preview_regions_.set(other.com_feasibility_preview_regions_);
   }


   /**
            * Feasible CoM region of the current keyframe
            */
   public controller_msgs.msg.dds.Polygon2DMessage getComFeasibilityRegion()
   {
      return com_feasibility_region_;
   }


   /**
            * Feasible CoM region for contact removability
            */
   public us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.Polygon2DMessage>  getComFeasibilityPreviewRegions()
   {
      return com_feasibility_preview_regions_;
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

      if (!this.com_feasibility_region_.epsilonEquals(other.com_feasibility_region_, epsilon)) return false;
      if (this.com_feasibility_preview_regions_.size() != other.com_feasibility_preview_regions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.com_feasibility_preview_regions_.size(); i++)
         {  if (!this.com_feasibility_preview_regions_.get(i).epsilonEquals(other.com_feasibility_preview_regions_.get(i), epsilon)) return false; }
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

      if (!this.com_feasibility_region_.equals(otherMyClass.com_feasibility_region_)) return false;
      if (!this.com_feasibility_preview_regions_.equals(otherMyClass.com_feasibility_preview_regions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactCoMRegionMessage {");
      builder.append("com_feasibility_region=");
      builder.append(this.com_feasibility_region_);      builder.append(", ");
      builder.append("com_feasibility_preview_regions=");
      builder.append(this.com_feasibility_preview_regions_);
      builder.append("}");
      return builder.toString();
   }
}
