package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the multi-contact feasibility module
       * This message acts as an output of the module and contains a set of preview CoM regions if contacts are removed.
       */
public class MultiContactCoMRegionPreviewMessage extends Packet<MultiContactCoMRegionPreviewMessage> implements Settable<MultiContactCoMRegionPreviewMessage>, EpsilonComparable<MultiContactCoMRegionPreviewMessage>
{
   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.MultiContactCoMRegionMessage>  com_preview_regions_;

   public MultiContactCoMRegionPreviewMessage()
   {
      com_preview_regions_ = new us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.MultiContactCoMRegionMessage> (15, new toolbox_msgs.msg.dds.MultiContactCoMRegionMessagePubSubType());

   }

   public MultiContactCoMRegionPreviewMessage(MultiContactCoMRegionPreviewMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactCoMRegionPreviewMessage other)
   {
      com_preview_regions_.set(other.com_preview_regions_);
   }


   public us.ihmc.idl.IDLSequence.Object<toolbox_msgs.msg.dds.MultiContactCoMRegionMessage>  getComPreviewRegions()
   {
      return com_preview_regions_;
   }


   public static Supplier<MultiContactCoMRegionPreviewMessagePubSubType> getPubSubType()
   {
      return MultiContactCoMRegionPreviewMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactCoMRegionPreviewMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactCoMRegionPreviewMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (this.com_preview_regions_.size() != other.com_preview_regions_.size()) { return false; }
      else
      {
         for (int i = 0; i < this.com_preview_regions_.size(); i++)
         {  if (!this.com_preview_regions_.get(i).epsilonEquals(other.com_preview_regions_.get(i), epsilon)) return false; }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactCoMRegionPreviewMessage)) return false;

      MultiContactCoMRegionPreviewMessage otherMyClass = (MultiContactCoMRegionPreviewMessage) other;

      if (!this.com_preview_regions_.equals(otherMyClass.com_preview_regions_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactCoMRegionPreviewMessage {");
      builder.append("com_preview_regions=");
      builder.append(this.com_preview_regions_);
      builder.append("}");
      return builder.toString();
   }
}
