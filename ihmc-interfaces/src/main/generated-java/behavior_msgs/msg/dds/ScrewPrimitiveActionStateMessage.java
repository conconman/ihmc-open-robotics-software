package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class ScrewPrimitiveActionStateMessage extends Packet<ScrewPrimitiveActionStateMessage> implements Settable<ScrewPrimitiveActionStateMessage>, EpsilonComparable<ScrewPrimitiveActionStateMessage>
{
   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage state_;
   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage definition_;

   public ScrewPrimitiveActionStateMessage()
   {
      state_ = new behavior_msgs.msg.dds.ActionNodeStateMessage();
      definition_ = new behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage();
   }

   public ScrewPrimitiveActionStateMessage(ScrewPrimitiveActionStateMessage other)
   {
      this();
      set(other);
   }

   public void set(ScrewPrimitiveActionStateMessage other)
   {
      behavior_msgs.msg.dds.ActionNodeStateMessagePubSubType.staticCopy(other.state_, state_);
      behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessagePubSubType.staticCopy(other.definition_, definition_);
   }


   /**
            * Parent state fields
            */
   public behavior_msgs.msg.dds.ActionNodeStateMessage getState()
   {
      return state_;
   }


   /**
            * Definition
            */
   public behavior_msgs.msg.dds.ScrewPrimitiveActionDefinitionMessage getDefinition()
   {
      return definition_;
   }


   public static Supplier<ScrewPrimitiveActionStateMessagePubSubType> getPubSubType()
   {
      return ScrewPrimitiveActionStateMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return ScrewPrimitiveActionStateMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(ScrewPrimitiveActionStateMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.state_.epsilonEquals(other.state_, epsilon)) return false;
      if (!this.definition_.epsilonEquals(other.definition_, epsilon)) return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof ScrewPrimitiveActionStateMessage)) return false;

      ScrewPrimitiveActionStateMessage otherMyClass = (ScrewPrimitiveActionStateMessage) other;

      if (!this.state_.equals(otherMyClass.state_)) return false;
      if (!this.definition_.equals(otherMyClass.definition_)) return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ScrewPrimitiveActionStateMessage {");
      builder.append("state=");
      builder.append(this.state_);      builder.append(", ");
      builder.append("definition=");
      builder.append(this.definition_);
      builder.append("}");
      return builder.toString();
   }
}
