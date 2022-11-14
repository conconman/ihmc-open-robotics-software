package us.ihmc.perception.parameters;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface PointCloudSegmentationParametersBasics extends PointCloudSegmentationParametersReadOnly, StoredPropertySetBasics
{
   /**
    * The segmentation divisor divides the message up into parts. A message with 2048
    * scan lines with a divisor of 8 will result in eight 256 scan line messages being
    * sent out.
    */
   default void setSegmentationDivisor(int segmentationDivisor)
   {
      set(PointCloudSegmentationParameters.segmentationDivisor, segmentationDivisor);
   }

   /**
    * The whole scan send frequency is how fast the whole scan gets published. So the
    * message frequency will be this times the segmentation divisor. So for a whole
    * scan frequency of 4 and divisor of 8, messages will get sent at 32 Hz.
    */
   default void setWholeScanSendFrequency(double wholeScanSendFrequency)
   {
      set(PointCloudSegmentationParameters.wholeScanSendFrequency, wholeScanSendFrequency);
   }
}
