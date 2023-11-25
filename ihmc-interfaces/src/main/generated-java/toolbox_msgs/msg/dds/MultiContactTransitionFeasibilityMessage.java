package toolbox_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import java.util.function.Supplier;
import us.ihmc.pubsub.TopicDataType;

/**
       * This message is part of the multi-contact feasibility module
       * This message acts as an output of the module and specifies if the candidate keyframe transition is feasible.
       */
public class MultiContactTransitionFeasibilityMessage extends Packet<MultiContactTransitionFeasibilityMessage> implements Settable<MultiContactTransitionFeasibilityMessage>, EpsilonComparable<MultiContactTransitionFeasibilityMessage>
{
   /**
            * Whether the current keyframe transition is feasible
            */
   public boolean is_transition_feasible_;

   public MultiContactTransitionFeasibilityMessage()
   {
   }

   public MultiContactTransitionFeasibilityMessage(MultiContactTransitionFeasibilityMessage other)
   {
      this();
      set(other);
   }

   public void set(MultiContactTransitionFeasibilityMessage other)
   {
      is_transition_feasible_ = other.is_transition_feasible_;

   }

   /**
            * Whether the current keyframe transition is feasible
            */
   public void setIsTransitionFeasible(boolean is_transition_feasible)
   {
      is_transition_feasible_ = is_transition_feasible;
   }
   /**
            * Whether the current keyframe transition is feasible
            */
   public boolean getIsTransitionFeasible()
   {
      return is_transition_feasible_;
   }


   public static Supplier<MultiContactTransitionFeasibilityMessagePubSubType> getPubSubType()
   {
      return MultiContactTransitionFeasibilityMessagePubSubType::new;
   }

   @Override
   public Supplier<TopicDataType> getPubSubTypePacket()
   {
      return MultiContactTransitionFeasibilityMessagePubSubType::new;
   }

   @Override
   public boolean epsilonEquals(MultiContactTransitionFeasibilityMessage other, double epsilon)
   {
      if(other == null) return false;
      if(other == this) return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsBoolean(this.is_transition_feasible_, other.is_transition_feasible_, epsilon)) return false;


      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if(other == null) return false;
      if(other == this) return true;
      if(!(other instanceof MultiContactTransitionFeasibilityMessage)) return false;

      MultiContactTransitionFeasibilityMessage otherMyClass = (MultiContactTransitionFeasibilityMessage) other;

      if(this.is_transition_feasible_ != otherMyClass.is_transition_feasible_) return false;


      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("MultiContactTransitionFeasibilityMessage {");
      builder.append("is_transition_feasible=");
      builder.append(this.is_transition_feasible_);
      builder.append("}");
      return builder.toString();
   }
}
