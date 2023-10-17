package behavior_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

public class WaitDurationActionDefinitionMessage extends Packet<WaitDurationActionDefinitionMessage> implements Settable<WaitDurationActionDefinitionMessage>, EpsilonComparable<WaitDurationActionDefinitionMessage>
{
   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage action_definition_;
   /**
            * Wait duration
            */
   public double wait_duration_;

   public WaitDurationActionDefinitionMessage()
   {
      action_definition_ = new behavior_msgs.msg.dds.BehaviorActionDefinitionMessage();
   }

   public WaitDurationActionDefinitionMessage(WaitDurationActionDefinitionMessage other)
   {
      this();
      set(other);
   }

   public void set(WaitDurationActionDefinitionMessage other)
   {
      behavior_msgs.msg.dds.BehaviorActionDefinitionMessagePubSubType.staticCopy(other.action_definition_, action_definition_);
      wait_duration_ = other.wait_duration_;

   }


   /**
            * Parent definition fields
            */
   public behavior_msgs.msg.dds.BehaviorActionDefinitionMessage getActionDefinition()
   {
      return action_definition_;
   }

   /**
            * Wait duration
            */
   public void setWaitDuration(double wait_duration)
   {
      wait_duration_ = wait_duration;
   }
   /**
            * Wait duration
            */
   public double getWaitDuration()
   {
      return wait_duration_;
   }


   public static Supplier<WaitDurationActionDefinitionMessagePubSubType> getPubSubType()
   {
      return WaitDurationActionDefinitionMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return WaitDurationActionDefinitionMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(WaitDurationActionDefinitionMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!this.action_definition_.epsilonEquals(other.action_definition_, epsilon)) return false;
      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wait_duration_, other.wait_duration_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof WaitDurationActionDefinitionMessage)) return false;

      WaitDurationActionDefinitionMessage otherMyClass = (WaitDurationActionDefinitionMessage) other;

      if (!this.action_definition_.equals(otherMyClass.action_definition_)) return false;
      if(this.wait_duration_ != otherMyClass.wait_duration_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("WaitDurationActionDefinitionMessage {");
      builder.append("action_definition=");
      builder.append(this.action_definition_);      builder.append(", ");
      builder.append("wait_duration=");
      builder.append(this.wait_duration_);
      builder.append("}");
      return builder.toString();
   }
}