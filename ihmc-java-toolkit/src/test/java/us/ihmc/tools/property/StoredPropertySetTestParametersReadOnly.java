package us.ihmc.tools.property;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.tools.property.StoredPropertySetTestParameters.*;

public interface StoredPropertySetTestParametersReadOnly extends StoredPropertySetReadOnly
{
   default boolean getTheFirstBooleanProperty()
   {
      return get(theFirstBooleanProperty);
   }

   default double getTheFirstDoubleProperty()
   {
      return get(theFirstDoubleProperty);
   }

   default int getTheFirstIntegerProperty()
   {
      return get(theFirstIntegerProperty);
   }
}
