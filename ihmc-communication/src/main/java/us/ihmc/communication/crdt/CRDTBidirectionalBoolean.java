package us.ihmc.communication.crdt;

/**
 * Represents a data field that can be modified by both the
 * robot and the operator.
 *
 * Warning: With this type, the data should not be continuously modified
 *   tick after tick, as that will mean the value is essentially never
 *   synced properly to the other side.
 */
public class CRDTBidirectionalBoolean
{
   private final Confirmable confirmable;

   private boolean value;

   public CRDTBidirectionalBoolean(Confirmable confirmable, boolean initialValue)
   {
      this.confirmable = confirmable;

      value = initialValue;
   }

   public boolean booleanValue()
   {
      return value;
   }

   public void setValue(boolean value)
   {
      if (this.value != value) // Don't want to do anything in the case nothing changed
      {
         this.value = value;
         confirmable.freeze();
      }
   }

   public void fromMessage(boolean value)
   {
      if (!confirmable.isFrozen())
      {
         this.value = value;
      }
   }

   public boolean toMessage()
   {
      return value;
   }
}
